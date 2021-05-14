/*
 * Apple M1 SoC and Mac Mini board emulation
 * 
 * Copyright (c) 2021 Iris Johnson <iris@modwiz.com>
 * 
 * SPDX-License-Identifier: MIT
 */

#include "qemu/osdep.h"
#include "exec/address-spaces.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties-system.h"
#include "cpu.h"
#include "chardev/char-fe.h"
#include "hw/arm/boot.h"
#include "hw/sysbus.h"
#include "hw/misc/unimp.h"
#include "hw/loader.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/or-irq.h"
#include "exec/hwaddr.h"
#include "sysemu/sysemu.h"
#include "exec/exec-all.h"
#include "sysemu/reset.h"
#include "hw/misc/unimp.h"
#include "arm-powerctl.h"
#include "hw/arm/apple-m1.h"

#include <err.h>

// Notes on this file:
// - This file doesn't actually represent the M1 very well, it's more a platform designed and focused around m1n1
// - The ARM IRQ pin is directly connected to the UART, which is helpful if you want to test the simplest 
//   interrupts possible
// - The structure of this file is based on the other board files I could find and me making my best guess
//   unfortunately that doesn't get you very far, and there's some not so great stuff here too just to tie
//   it all together to get an interesting demo
// - The "M1" core types are sort of emulated, but actual specific features like
//   the CP registers for fast IPIs and several of the other registers are not
//   implemented and this will need to be fixed so code that expects a real M1
//   doesn't get random exceptions
//
// One idea for this is to do what's suggested below and literally pull in a device tree (converted to
// not apple format) then use that to layout all the devices, creating stub regions of unimplemented registers
// until they're created. Unfortunately it seems like nothing else in qemu does this, not even the most modern
// code. So there's no framework, so that seems like it would be it's own project to prototype. 
// So for now we hardcode and make a few M1 helpers

#define RAMLIMIT_GB     8192
#define RAMLIMIT_BYTES  (RAMLIMIT_GB * GiB)

#define VRAM_ZONE_SIZE (500 * MiB)
#define ADT_MAX_SIZE 0x1000

/* 
 * TODO: this way of mapping memory is probably not ideal, (it's hard to add
 * multiple regions of ram split by IO, we should instead construct this from
 * a dtb conversion of an ADT from a real Mac Mini
 */
enum {
    M1_MEM,         // The main memory for the M1
    M1_UART,        // The virtual uart (on real hardware this is on the type-C and multiplexed via USB-PD selection)
    M1_AIC,         // The Apple interrupt controller
    M1_WDT,         // The M1 Watchdog timer. This is probably actually part of the AIC
    M1_ARGS,        // A bit of ROM to hold the bootargs
    M1_PMGR,        // Peripheral used to set cores to on (and theoretically off) along with giving them a reset vector address to start at
};
// This is used in several other ARM platforms to help collect the addresses and give them meaningful names
// during realization. These aren't correlated to the M1 hardware at all though it's just easier to set
// them up like this during dev (the UART is the correct address for where it is when m1n1 loads though)
// In reality this should be constructed from a hardware device tree provided from an M1 Mac
static const struct MemMapEntry memmap[] = {
    //[M1_SPINTABLE] =        {     0x10000,                0x100},
    [M1_UART] =             { 0x235200000,              0x10000},
    [M1_AIC] =              { 0x23B100000,              0x10000},
    [M1_WDT] =              { 0x23D2B0000,                0x100},
    /* TODO: replace this with a real addr from hardware */
    [M1_PMGR] =             { 0x23D300000,                0x100},
    [M1_MEM] =              { 0x800000000,       RAMLIMIT_BYTES},
};

/* Helper function to convert cpu_index to ARMCPU* pointer from complex */
static ARMCPU* m1_get_cpu_by_index(AppleM1State *s, int cpu_index)
{
    if (cpu_index < APPLE_M1_TOTAL_CPUs) {
        if (cpu_index >= APPLE_M1_ICESTORM_CPUS) {
            /* Firestorm core */
            return &s->firestorm_cores[cpu_index-APPLE_M1_ICESTORM_CPUS];
        } else {
            /* Icestorm core */
            return &s->icestorm_cores[cpu_index];
        }
    }
    return NULL;
}

/* 
 * Simple power management/startup unit emulation to match the M1, this could
 * probably be replicated without using an actual IO peripheral by just using
 * modified startup code, but that can't set the reset vectors so we use this
 */
static uint64_t m1_pmgr_read(void *opaque, hwaddr offset, unsigned size)
{
    AppleM1State *s = APPLE_M1(opaque);
    /* TODO this code assumes you're using 32 bit reads except for the RVBAR addr */
    switch (offset) {
    case 0x4:
    case 0x8:
    case 0xC:
        /* Ignore all these reads (maybe we'll have to report what was written) */
        break;
    case 0x10 ... 0x50:
    {
        /* There's no specific offset for setting the reset vectors so stick them here */
        uint32_t cpuid = (offset - 0x10)/8;
        assert(cpuid < APPLE_M1_TOTAL_CPUs);
        uint64_t rvbar = m1_get_cpu_by_index(s, cpuid)->rvbar;
        /* Access it little endian in 32-bit halves */
        /* extract64 needed for 64-bit input but output value is truncted */
        return extract64(rvbar, (offset & 4) ? 32 : 0, 32);
    }   
    default:
        qemu_log_mask(LOG_UNIMP, "M1 PMGR: Unhandled read of @ %#"PRIx64"\n", offset);
        break;
    }
    return 0;
}

static void m1_pmgr_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    AppleM1State *s = APPLE_M1(opaque);
    /* TODO This code assumes you're using 32 bit writes except for the RVBAR addr */
    uint32_t value = val;
    switch (offset) {
    case 0x4: 
        /* M1N1 describes this as a system level startup bit we just ignore */
        break;
    case 0x8:
        /* TODO: combine the two */
        /* This is where we startu  p one of the efficiency cores */
        //qemu_log("Efficiency core startup: %#x\n", value);
        for (int i = ctz32(value); i < APPLE_M1_ICESTORM_CPUS; i++) {
            if (value & (1<<i)) {
                ARMCPU *cpu = &s->icestorm_cores[i];
                if (cpu->power_state == PSCI_OFF) {
                    qemu_log("Starting ICESTORM core #%u\n", i);
                    arm_set_cpu_on_and_reset(cpu->mp_affinity);
                }
                //qemu_log("Power state was: %#x\n", s->icestorm_cores[i].power_state);
                //qemu_log("RVBAR was: %#lx\n", s->icestorm_cores[i].rvbar);
            }
        }
        break;
    case 0xC:
        /* This is where we startup one of the performance cores */
        qemu_log("Performance core startup: %#x\n", value);
        for (int i = ctz32(value); i < APPLE_M1_FIRESTORM_CPUS; i++) {
            if (value & (1<<i)) {
                ARMCPU *cpu = &s->firestorm_cores[i];
                if (cpu->power_state == PSCI_OFF) {
                    qemu_log("Starting FIRESTORM core #%u\n", i);
                    arm_set_cpu_on_and_reset(cpu->mp_affinity);
                }
                //qemu_log("Power state was: %#x\n", s->firestorm_cores[i].power_state);
                //qemu_log("RVBAR was: %#lx\n", s->firestorm_cores[i].rvbar);
            }
        }
        break;
    case 0x10 ... 0x50:
    {
        /* There's no specific offset for setting the reset vectors so stick them here */
        uint32_t cpuid = (offset - 0x10)/8;
        assert(cpuid < APPLE_M1_TOTAL_CPUs);
        ARMCPU *cpu = m1_get_cpu_by_index(s, cpuid);
        /* Deposit only the bits being set since it's a 64-bit field with 32-bit
         * access */
        cpu->rvbar = deposit64(cpu->rvbar, (offset & 4) ? 32 : 0, 32, value);
        break;
    }
    default:
        qemu_log_mask(LOG_UNIMP, "M1 PMGR: Unhandled write of %#x to %#" PRIx64 "\n", value, offset);
        break;
    }
}

static const MemoryRegionOps m1_pmgr_ops = {
    .read = m1_pmgr_read,
    .write = m1_pmgr_write,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void apple_m1_init(Object *obj) {
    AppleM1State *s = APPLE_M1(obj);

    /* TODO: Better object names */
    /* TODO Replace this with properties */
    s->num_icestorm = APPLE_M1_ICESTORM_CPUS;
    s->num_firestorm = APPLE_M1_FIRESTORM_CPUS;
    // Initialize CPU cores
    for (int i = 0; i < s->num_icestorm; i++) {
        object_initialize_child(obj, "icestorm[*]", &s->icestorm_cores[i],
                                ARM_CPU_TYPE_NAME("apple-icestorm"));
    }
    
    for (int i = 0; i < s->num_firestorm; i++) {
        object_initialize_child(obj, "firestorm[*]", &s->firestorm_cores[i],
                                ARM_CPU_TYPE_NAME("apple-firestorm"));
    }
    
    /* Initialize framebuffer */
    object_initialize_child(obj, "fb", &s->fb, TYPE_M1_FB);
    // Initialize interrupt controller
    object_initialize_child(obj, "aic", &s->aic, TYPE_APPLE_AIC);
}

/* Reload a RAM memory region with static content. */
static void reload_memory_region(struct m1_rel_region* rel_region) {
    size_t size = memory_region_size(&rel_region->region);

    if (size == 0) {
        return;
    }

    void* ptr = memory_region_get_ram_ptr(&rel_region->region);
    memcpy(ptr, rel_region->data, size);
}

/* Get the absolute address of a memory region */
static hwaddr get_absolute_addr(MemoryRegion* region)
{
    /* FIXME: There is probably a better way to do this */
    hwaddr addr = 0;

    while (region != NULL) {
        addr += region->addr;
        region = region->container;
    }

    return addr;
}

static void apple_m1_finalize(Object *obj) {
    AppleM1State *s = APPLE_M1(obj);

    free(s->ram_adt_mr.data);
    s->ram_adt_mr.data = NULL;

    free(s->ram_firmware_mr.data);
    s->ram_firmware_mr.data = NULL;

    free(s->ram_initrd_mr.data);
    s->ram_initrd_mr.data = NULL;

    free(s->ram_dtb_mr.data);
    s->ram_dtb_mr.data = NULL;

    free(s->ram_bootargs_mr.data);
    s->ram_bootargs_mr.data = NULL;
}

/* Reset handler: put firmware, ADT boot-args in RAM and set registers */
static void handle_m1_reset(void *opaque)
{
    ARMCPU *cpu = ARM_CPU(opaque);
    // FIXME: It's possible this should be replaced by a rom region that does this
    // while executing on the cpu
    
    AppleM1State *m1 = APPLE_M1(OBJECT(cpu)->parent);

    reload_memory_region(&m1->ram_adt_mr);
    reload_memory_region(&m1->ram_firmware_mr);
    reload_memory_region(&m1->ram_kernel_mr);
    reload_memory_region(&m1->ram_initrd_mr);
    reload_memory_region(&m1->ram_dtb_mr);
    reload_memory_region(&m1->ram_bootargs_mr);

    hwaddr boot_args_addr = get_absolute_addr(&m1->ram_bootargs_mr.region);

    /* FIXME: Parse the firmware to determine its entry point */
    hwaddr entry_point = get_absolute_addr(&m1->ram_firmware_mr.region)
        + 0x4800;

    // INFO: This way of doing things is actually done on several other machines mostly
    // not ARM ones though, but using the reset vector like this is fair, most of them
    // load a bootrom bios into a fixed location, setup the registers to a loaded main binary
    // image (or just let the bootrom work) and then set the PC to the entrypoint
    cpu_reset(CPU(cpu));
    cpu->env.xregs[0] = boot_args_addr;
    cpu_set_pc(CPU(cpu), entry_point);
    // FIXME: This seems like a crazy way to initialize this but there's no easy
    // way to do it at the CPU level because CPU reset will reset this to 0.
    // There might be a better way if I look deeper though but this works okay for now
    cpu->env.cp15.hcr_el2 = 0x30488000000;
}   

/*
 * TODO Cleanup and maybe move this to a device per core complex (linked to
 * ARMCPU pointers stored in a top-level)
 */
static void m1_realize_cpu(ARMCPU *cpu, uint64_t mp_affinity, bool start_powered_off)
{
    Object *obj = OBJECT(cpu);
    DeviceState *dev = DEVICE(cpu);
    // TODO: Do we need to set CBAR?
    /* Disable CPU */
    qdev_prop_set_bit(dev, "start-powered-off", start_powered_off);
    /* Set MPIDR */
    qdev_prop_set_uint64(dev, "mp-affinity", mp_affinity);
    /* Realize the CPU (defines IRQs) */
    qdev_realize(dev, NULL, &error_abort);
    
    /* Connect a generic ARM timer to FIQ for the interrupts we need */
    /* TODO: This OR IRQ ends up being more trouble than our own wrapper... */
    DeviceState *fiq_or = qdev_new(TYPE_OR_IRQ);
    object_property_add_child(obj, "fiq-or", OBJECT(fiq_or));
    
    /* NOTE: if you use less than 16 it can't store state properly */
    qdev_prop_set_uint16(fiq_or, "num-lines", 16);
    qdev_realize_and_unref(fiq_or, NULL, &error_fatal);
    
    /* Connect the output of the OR'd FIQ events to the FIQ input of the CPU */
    qdev_connect_gpio_out(fiq_or, 0, qdev_get_gpio_in(dev, ARM_CPU_FIQ));
    
    /* Connect all the timer output pins from the core to the FIQ OR, not sure
     * if this is true of hardware, fast IPIs also take this path
     */
    /* TODO: Is looping like this correct */
    for (int i = 0; i < NUM_GTIMERS; i++) {
        /* Connect output timer IRQ to input of the FIQ or */
        qdev_connect_gpio_out(dev, i, qdev_get_gpio_in(fiq_or, i));
    }
}

static void apple_m1_realize(DeviceState *dev, Error **errp)
{
    /* TODO: Cleanup needed before merge */
    AppleM1State *s = APPLE_M1(dev);

    /*
     * NOTE: Initialization according to my understanding of hardware is as
     * follows.
     * 
     * Default reset vectors (RVBAR) are at zero/rom, on boot, the first core
     * is an Icestorm core (which is MPIDR 0 for Aff1 and Aff2), this might be
     * something that boot rom sets up or it might be part of the hardware
     * definition. Regardless, it ends up loading iBoot which completes loading
     * the kernel eventually.
     * 
     * After startup the RVBARs are locked and the cores
     * are no-longer able to have their reset vectors changed until some type
     * of SoC-level reset occurs (unknown exactly what or if unlock is possible).
     * 
     * So Icestorm core 0 is the one that jumps into the kernel and allows the
     * other cores to be started via the PMGR registers in the SoC.
     */
    /* TODO: QEMU has some built-in wrappers for clusters and core types
     * those might be usable here to make this a bit cleaner so it can be
     * a single loop (or we can just move the common initialization somewhere
     * else and keep the two loops), having a single array of CPUs would be
     * helpful in other places like the AIC code where we want to initialize
     * a per-cpu state for each core and just need to iterate through the whole
     * list (and we don't want to use QEMU's general list since I think it's
     * good to not make QEMU rely on the assumption that CPUs will always be
     * of the same type, at least not more than it does already)
     */
    for (int i = 0; i < APPLE_M1_ICESTORM_CPUS; i++) {
        // TODO: Use wrapper that takes cluster and core id (I think the 0x101
        // is actually 2 values, 0x1 and 0x1 again, differentiating cluster
        // and something else)
        /* All except core 0 start off */
        m1_realize_cpu(&s->icestorm_cores[i], i, i > 0);
    }
    // Now do the firestorm cores
    for (int i = 0; i < APPLE_M1_FIRESTORM_CPUS; i++) {
        // TODO: Use wrapper that takes cluster and core id (I think the 0x101
        // is actually 2 values, 0x1 and 0x1 again, differentiating cluster
        // and something else)
        /* All firestorms start off */
        m1_realize_cpu(&s->firestorm_cores[i], (0x101UL<<8)|i, true);
    }

    /* Create Video RAM region */
    memory_region_init_ram(&s->ram_vram_mr, OBJECT(s), "vram",
        VRAM_ZONE_SIZE, &error_fatal);

    /* Add link for vram */
    object_property_add_const_link(OBJECT(&s->fb), "vram", OBJECT(&s->ram_vram_mr));
    
    /* Now we're safe to realize the device */
    /* TODO: If the framebuffer never gets MMIO it should not be sysbus */
    sysbus_realize(SYS_BUS_DEVICE(&s->fb), &error_abort);

    // XXX: This code needs to be made more generic before it could sanely be attached here
    //      also it seems serial_hd(x) is bad practice for getting the chardev, but I can't see
    //      another method that wouldn't require adding a chardev prop to the machine itself
    // XXX: UART is directly connected to the IRQ input lol, this lets us test the interrupt handling easy though
    //      replace this when AIC is supported or if we temporarily hack a GIC on there
    // XXX: This replicates the creation code with some constants that should be exposed in a non-existant .h
    //      we replicated it to allow setting it as a child before relization completes
    DeviceState *uart = qdev_new("exynos4210.uart");
    object_property_add_child(OBJECT(s), "uart", OBJECT(uart));
    qdev_prop_set_chr(uart, "chardev", serial_hd(0));
    qdev_prop_set_uint32(uart, "rx-size", 4096*32); // Give the serial port an insane buffer to speed up code loads
    sysbus_realize_and_unref(SYS_BUS_DEVICE(uart), &error_abort);
    // XXX: Currently the FIFO reset state is changed so that it's ready for FIFO usage before m1n1 starts
    // the correct solution is probably to have m1n1 specifically enable the FIFO or add a pre-m1n1 boot stub
    // to initialize the registers to the values needed by iboot (an iboot stub)
    sysbus_mmio_map(SYS_BUS_DEVICE(uart), 0, memmap[M1_UART].base);
    // currently the UART is connected directly to firestorm core 0's interrupts... odd.
    // first_cpu is sometimes used but is that guranteed to be anything specific? Until we find out use
    // firstorm_cores[0]
    // FIXME: This was connecting IRQ directly to the uart but that causes annoying issues when m1n1 enables IRQs
    // during startup. For now just disable this. The AIC->m1n1 is already setup with full masking so for real
    // hardware this isn't a concern.
    // XXX: pending AIC
    //sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0, qdev_get_gpio_in(DEVICE(&s->firestorm_cores[0]), ARM_CPU_IRQ));
    
    sysbus_realize(SYS_BUS_DEVICE(&s->aic), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->aic), 0, memmap[M1_AIC].base);
    
    /* TODO: CPU order (at least for linux is 0-4 icestorm, 0-4 firestorm
     *       we will want to connect all of them as CPUs 0-8
     */
    for (int i = 0; i < APPLE_M1_TOTAL_CPUs; i++) {
        dev = DEVICE(m1_get_cpu_by_index(s, i));
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->aic), i,
                           qdev_get_gpio_in(dev, ARM_CPU_IRQ));  
    }
    /* Connect UART to AIC */
    /* FIXME: connect to every IRQ bc something weird with linux */
    /*
    DeviceState *splitq = qdev_new("split-irq");
    object_property_add_child(OBJECT(s), "split-irq", OBJECT(splitq));
    qdev_prop_set_uint16(splitq, "num-lines", AIC_NUM_IRQ);
    qdev_realize_and_unref(splitq, NULL, &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0, qdev_get_gpio_in(splitq, 0));
    for (int i = 0; i < AIC_NUM_IRQ; i++) {
        qdev_connect_gpio_out(splitq, i, qdev_get_gpio_in(DEVICE(&s->aic), i));
    } */
    sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0, qdev_get_gpio_in(DEVICE(&s->aic), 605));
    
    /* TODO: Connect input IRQs from hardware to this */
    //create_unimplemented_device("AIC", memmap[M1_AIC].base, memmap[M1_AIC].size); 
    // TODO: Implement
    create_unimplemented_device("Watchdog", memmap[M1_WDT].base, memmap[M1_WDT].size);
    
    // FIXME: Do this properly
    // Add a reset hook for the first core that will pull in the correct x0 reg for boot_args
    qemu_register_reset(handle_m1_reset, &s->icestorm_cores[0]);
    /* TODO: add a reset handler that handles resetting secondaries */
    
    /* Add the power manager peripheral */
    /* TODO: This needs to be real device added to the AppleM1State struct */
    
    MemoryRegion *pmgr_mr = g_new0(MemoryRegion, 1);
    memory_region_init_io(pmgr_mr, OBJECT(s), &m1_pmgr_ops, DEVICE(s), "m1-pmgr",
                          memmap[M1_PMGR].size);
    memory_region_add_subregion(get_system_memory(), memmap[M1_PMGR].base,
                                pmgr_mr);
}

static void apple_m1_class_init(ObjectClass *oc, void *data) { 
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = apple_m1_realize;

    // The M1 SoC is mostly a glued-to-machine component it has no meaning in other contexts
    // additionally our use of serial_hd() apparently is not good according to other comments
    dc->user_creatable = false;
}

static const TypeInfo apple_m1_type_info = {
    .name = TYPE_APPLE_M1,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(AppleM1State),
    .instance_init = apple_m1_init,
    .instance_finalize = apple_m1_finalize,
    .class_init = apple_m1_class_init,
};

static void apple_m1_register_types(void)
{
    type_register_static(&apple_m1_type_info);
}

type_init(apple_m1_register_types);

/*
 * Define the data and size of the device tree reloadable memory region.
 * The memory mappings must be done beforehand.
 */
static void setup_adt(AppleM1State *m1) {
    // This was generated using an external tool by taking a dts base, compiling to fdt
    // then converting to adt format. Full ADT support will be needed if we want to boot
    // OS X though so that we can feed it all the nodes (a modified real ADT dump could 
    // be used also potentially)
    uint8_t databuf[] = {0x3, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x6e, 0x61, 0x6d, 0x65, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0xc, 0x0, 0x0, 0x0, 0x64, 0x65, 0x76, 0x69, 0x63, 0x65, 
               0x2d, 0x74, 0x72, 0x65, 0x65, 0x0, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x11, 0x0, 0x0, 0x0, 0x51, 0x45, 0x4d, 0x55, 0x20, 0x4d, 
               0x31, 0x20, 0x4d, 0x61, 0x63, 0x20, 0x4d, 0x69, 0x6e, 0x69, 0x0, 
               0x0, 0x0, 0x0, 0x74, 0x61, 0x72, 0x67, 0x65, 0x74, 0x2d, 0x74, 
               0x79, 0x70, 0x65, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 
               0x0, 0x0, 0x0, 0x55, 0x6e, 0x6b, 0x6e, 0x6f, 0x77, 0x6e, 0x0, 
               0x5, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6e, 0x61, 0x6d, 0x65, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x7, 0x0, 0x0, 0x0, 0x63, 0x68, 0x6f, 0x73, 0x65, 0x6e, 
               0x0, 0x0, 0x62, 0x6f, 0x61, 0x72, 0x64, 0x2d, 0x69, 0x64, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x0, 0x0, 
               0x0, 0xad, 0xde, 0x0, 0x0, 0x63, 0x68, 0x69, 0x70, 0x2d, 0x69, 
               0x64, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x4, 0x0, 0x0, 0x0, 0xfe, 0xca, 0x0, 0x0, 0x64, 0x72, 0x61, 
               0x6d, 0x2d, 0x62, 0x61, 0x73, 0x65, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 
               0x0, 0x0, 0x0, 0x0, 0x64, 0x72, 0x61, 0x6d, 0x2d, 0x73, 0x69, 
               0x7a, 0x65, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0};
    size_t bufsize = sizeof(databuf);

    if (bufsize > ADT_MAX_SIZE) {
        error_report("ADT size (%zu) exceeds max size! "
            "Increase ADT_MAX_SIZE.\n", bufsize);
        exit(1);
    }

    char* adt_data = g_malloc(bufsize);
    memcpy(adt_data, databuf, bufsize);

    m1->ram_adt_mr.data = adt_data;
    memory_region_ram_resize(&m1->ram_adt_mr.region, bufsize, &error_fatal);
}

#define BOOTARGS_REVISION 0xDEAD
#define BOOTARGS_FB_DEPTH 30

struct m1_boot_args {
    uint16_t revision;
    uint16_t pad0;
    uint32_t pad1;
    uint64_t virtual_load_base; // TODO: document if this is a load base or memory base
    uint64_t physical_load_base;
    uint64_t memory_size;
    uint64_t loaded_end;        // End address of loaded binary (safe mem starts here)
    uint64_t fb_addr;           // Framebuffer address in memory
    uint64_t pad2;
    uint64_t fb_stride;         // Stride (bytes per line)
    uint64_t fb_width;
    uint64_t fb_height;
    uint64_t fb_depth;          // TODO: Extract what real hardware puts here
    uint64_t pad3;
    uint64_t device_tree;
    uint32_t device_tree_size;
    char cmdline[608];
    uint64_t flags;
    uint64_t mem_size_act;
};

/*
 * Define the data of the boot-arg reloadable memory region.
 * The memory mappings (including the actual ADT size) must be done
 * beforehand.
 */
static void setup_boot_args(AppleM1State *m1, uint64_t total_ram_size) {
    struct m1_boot_args *boot_args = g_malloc0(sizeof(struct m1_boot_args));
    uint64_t free_ram_start = get_absolute_addr(&m1->ram_free_mr);
    uint64_t free_ram_size = memory_region_size(&m1->ram_free_mr);

    boot_args->revision = cpu_to_le16(0xdead);
    boot_args->virtual_load_base = cpu_to_le64(memmap[M1_MEM].base);
    boot_args->physical_load_base = cpu_to_le64(memmap[M1_MEM].base);
    boot_args->memory_size = cpu_to_le64(free_ram_size);
    boot_args->loaded_end = cpu_to_le64(free_ram_start);

    uint64_t video_ram_start = get_absolute_addr(&m1->ram_vram_mr);
    boot_args->fb_addr = cpu_to_le64(video_ram_start);
    /* TODO Clean this up */
    boot_args->fb_stride = cpu_to_le64(m1->fb.width * 4);
    boot_args->fb_width = cpu_to_le64(m1->fb.width);
    boot_args->fb_height = cpu_to_le64(m1->fb.height);
    boot_args->fb_depth = cpu_to_le64(BOOTARGS_FB_DEPTH);

    uint64_t adt_start = get_absolute_addr(&m1->ram_adt_mr.region);
    uint64_t adt_size = memory_region_size(&m1->ram_adt_mr.region);

    boot_args->device_tree = cpu_to_le64(adt_start);
    boot_args->device_tree_size = cpu_to_le64(adt_size);

    char buf[] ="no cmdline lol";
    memset(boot_args->cmdline, 0, sizeof(boot_args->cmdline));
    memcpy(boot_args->cmdline, buf, sizeof(buf)); // FIXME: This is stupid
    boot_args->cmdline[sizeof(buf)] = 0x0;

    boot_args->mem_size_act = cpu_to_le64(total_ram_size);

    m1->ram_bootargs_mr.data = boot_args;
}

/*
 * Initialize a reloadable region from a file.
 * If path is NULL, a zero-sized region is initialized.
 */
static void init_rel_region_from_file(Object *owner, const char *name,
    struct m1_rel_region *rel_region, const char *path)
{
    if (path == NULL) {
        memory_region_init(&rel_region->region, owner, name, 0);
        rel_region->data = NULL;
        return;
    }

    int fd = open(path, O_RDONLY | O_BINARY);
    if (fd < 0) {
        err(EXIT_FAILURE, "Error loading %s region: open(%s)", name, path);
    }

    int64_t size = lseek(fd, 0, SEEK_END);
    if (size < 0) {
        err(EXIT_FAILURE, "Error loading %s region: lseek(%s)", name, path);
    }

    rel_region->data = g_malloc((uint64_t)size);

    if (lseek(fd, 0, SEEK_SET) < 0) {
        err(EXIT_FAILURE, "Error loading %s region: lseek(%s)", name, path);
    }

    int64_t result = read(fd, rel_region->data, (uint64_t)size);
    if (result < 0) {
        err(EXIT_FAILURE, "Error loading %s region: read(%s)", name, path);
    }

    if (result != size) {
        errx(EXIT_FAILURE, "Error loading %s region: short read from %s", name,
            path);
    }

    close(fd);

    memory_region_init_ram(&rel_region->region, owner, name, (uint64_t)size,
        &error_fatal);
}

/* Map a region to RAM */
static void map_ram_region(MemoryRegion* ram_region, MemoryRegion *to_map,
    uint64_t alignment, hwaddr *offset)
{
    hwaddr cur_offset = QEMU_ALIGN_UP(*offset, alignment);
    const char* name = memory_region_name(to_map);
    uint64_t size = memory_region_size(to_map);

    if (size == 0) {
        return;
    }

    printf("Mapping %s (size 0x%" PRIx64 ") at RAM offset 0x%" PRIx64 "\n",
        name, size, cur_offset);
    memory_region_add_subregion(ram_region, cur_offset, to_map);

    *offset = cur_offset + memory_region_size(to_map);
}

static void m1_mac_init(MachineState *machine)
{
    AppleM1State *m1;

    m1 = APPLE_M1(qdev_new(TYPE_APPLE_M1));

    Object* m1_obj = OBJECT(m1);

    object_property_add_child(OBJECT(machine), "soc", OBJECT(m1));
    qdev_realize_and_unref(DEVICE(m1), NULL, &error_abort);

    /* This is for loading M1N1 which acts as a sort of replacement firmware */
    /* We will want to do some sort of stub firmware loader first before M1N1
     * to setup the registers and boot args it expects, but for now we abuse
     * the reset handler and use ROM regions
     * 
     * NOTE: we cannot really use the helper functions from arm/boot.c because
     * loading elfs offers no control over the load address (it just uses the
     * physical values which are offset from 0 for m1n1), we also want to start
     * M1N1 with certain register and memory regions setup like iBoot would with
     * info about the hardware (Apple's form of DTB)
     * 
     * We want to support this boot process in addition to a semi-standard
     * "just load the kernel" method where we can directly load the kernel,
     * however to do that we need to patch a fdt so that it has the dynamic
     * info like CPU spintable addresses and framebuffer address. Since Linux
     * can be loaded directly we can just do this the standard way, but then
     * the code isn't really emulating an Apple M1 based Mac anymore, just a
     * ARM64 Mac with some of the M1's peripherals, which is a lot easier to
     * implement but not likely a good starting point if you want to also
     * support emulation of OS X which is indeed an important goal here.
     * 
     * For now we have a compromise. M1N1 is loaded as "firmware" which
     * essentially means we just grab the mach-o (which is intended to be loaded
     * as a flat image with a fixed entry point) and dump it into memory, setup
     * the required boot arguments (because only QEMU knows the values for
     * those) and jump in, with some helper code to dump a kernel, initrd and
     * dtb into memory so that proxy code can start it without copying it over
     * the serial port. When I pull in the latest M1N1 changes again this can
     * likely be changed to append the image like a concatenated M1N1 (but done
     * dynamically in QEMU) so the proxy part wouldn't be needed.
     * 
     * In the future we may want to support directly starting linux without
     * a bootloader if M1N1 isn't provided, this will likely be required for
     * upstream merge and offers helpful features like not having to fuss with
     * ADT stuff since a standard FDT can be easily constructed using QEMU
     * helpers.
     * 
     * TL;DR There's a plan for loading, for now it's almost the simplest thing
     * but not quite.
     */
    
    /* Initialize the reloadable regions */

    /* ADT (memory region only; the ADT data is determined after mapping) */
    memory_region_init_resizeable_ram(&m1->ram_adt_mr.region, m1_obj, "adt",
        ADT_MAX_SIZE, ADT_MAX_SIZE, NULL, &error_fatal);

    /* Firmware */
    init_rel_region_from_file(m1_obj, "firmware", &m1->ram_firmware_mr,
        machine->firmware);

    /* Kernel */
    init_rel_region_from_file(m1_obj, "kernel", &m1->ram_kernel_mr,
        machine->kernel_filename);

    /* Initrd */
    init_rel_region_from_file(m1_obj, "initrd", &m1->ram_initrd_mr,
        machine->initrd_filename);

    /* Linux device tree */
    init_rel_region_from_file(m1_obj, "dtb", &m1->ram_dtb_mr,
        machine->dtb);

    /* boot-args (memory region only; ADT data is determined after mapping) */
    memory_region_init_ram(&m1->ram_bootargs_mr.region, m1_obj, "boot-args",
        sizeof(struct m1_boot_args), &error_fatal);

    /* The video RAM region is initialized during realization */

    /* Map the RAM and its regions */
    memory_region_add_subregion(get_system_memory(), memmap[M1_MEM].base,
        machine->ram);

    /* FIXME: This respects the order of these components on a real M1,
     * but there are some gaps between them */
    hwaddr offset = 0;
    map_ram_region(machine->ram, &m1->ram_adt_mr.region, 8, &offset);

    /* Align firmware to a 16K page */
    map_ram_region(machine->ram, &m1->ram_firmware_mr.region, 16384, &offset);

    /* Align kernel to a 2MB addr */
    map_ram_region(machine->ram, &m1->ram_kernel_mr.region, 2 * 1024 * 1024,
        &offset);

    map_ram_region(machine->ram, &m1->ram_dtb_mr.region, 1, &offset);

    /* Align initrd to 64K like M1N1 does for some reason */
    map_ram_region(machine->ram, &m1->ram_initrd_mr.region, 65536, &offset);

    /* Put some space between the payload and the boot args */
    /* FIXME: Find the right offset */
    offset += 0x100000;

    map_ram_region(machine->ram, &m1->ram_bootargs_mr.region, 8, &offset);

    /* Map the VRAM at the end of the RAM, leaving some free space */
    uint64_t total_ram_size = memory_region_size(machine->ram);
    if ((offset + VRAM_ZONE_SIZE) > total_ram_size) {
        error_report("Insufficient RAM size.\n");
        exit(1);
    }

    memory_region_init_ram(&m1->ram_free_mr, m1_obj, "free-space",
        total_ram_size - VRAM_ZONE_SIZE - offset, &error_fatal);

    map_ram_region(machine->ram, &m1->ram_free_mr, 1, &offset);
    map_ram_region(machine->ram, &m1->ram_vram_mr, 1, &offset);

    /* Initialize the ADT and the boot args */
    setup_adt(m1);
    setup_boot_args(m1, total_ram_size);
}

static void m1_mac_machine_init(MachineClass *mc)
{
    /* TODO: Clean this up a bit */
    mc->init = m1_mac_init;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;
    mc->default_ram_id = "ram";
    mc->default_ram_size = 8 * GiB;
    mc->desc = "Apple M1 Mac Mini";
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a57");
    mc->block_default_type = IF_SD;
    mc->min_cpus = 8;
    mc->max_cpus = 8;
    mc->default_cpus = 8;
}

// TODO: Change this when docs can be changed
DEFINE_MACHINE("apple-m1", m1_mac_machine_init)
