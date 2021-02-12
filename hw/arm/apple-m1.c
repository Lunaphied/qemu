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
    M1_ROM,         // We setup a block of ROM to keep the initial boot code in
    M1_ARGS,        // A bit of ROM to hold the bootargs
    M1_FB,          // The M1N1 code expects a framebuffer, for now this just means to dump ram in this region
    M1_PMGR,        // Peripheral used to set cores to on (and theoretically off) along with giving them a reset vector address to start at
};
// This is used in several other ARM platforms to help collect the addresses and give them meaningful names
// during realization. These aren't correlated to the M1 hardware at all though it's just easier to set
// them up like this during dev (the UART is the correct address for where it is when m1n1 loads though)
// In reality this should be constructed from a hardware device tree provided from an M1 Mac
static const struct MemMapEntry memmap[] = {
    [M1_ROM] =              {         0x0,              0x10000},
    //[M1_SPINTABLE] =        {     0x10000,                0x100},
    [M1_UART] =             { 0x235200000,              0x10000},
    [M1_AIC] =              { 0x23B100000,              0x10000},
    [M1_WDT] =              { 0x23D2B0000,                0x100},
    /* TODO: replace this with a real addr from hardware */
    [M1_PMGR] =             { 0x23D300000,                0x100},
    [M1_FB] =               { 0x300000000,                8*GiB},
    [M1_MEM] =              { 0x800000000,       RAMLIMIT_BYTES},
};

// TODO: Get rid of this and just handle returning the size from the boot args
// and device create creation code, also those should be stuffed in ram that's
// initialized at boot instead of into ROM, ROM should be tiny and used for
// initializing stuff before jumping in
static hwaddr device_tree_end = 0;

/*
 * TODO: get rid of this? (or split the boot code into it's own file maybe
 *       a lot of this process for boot is similar to the unique process
 *       that the x86 microvm does (specifically the PVH load))
 */

/* 
 * This stores the first address after the loaded kernel (which might be M1N1
 * alone or a collection of M1N1+linux+device tree+initrd)
 */
static hwaddr safe_ram_start = 0;

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
        qemu_log_mask(LOG_UNIMP, "M1 PMGR: Unhandled read of @ %#lx\n", offset);
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
        qemu_log_mask(LOG_UNIMP, "M1 PMGR: Unhandled write of %#x to %#lx\n", value, offset);
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
    
    // Initialize framebuffer
    object_initialize_child(obj, "framebuffer", &s->fb,
		    		TYPE_APPLE_M1_FB);
    
    // Initialize interrupt controller
    object_initialize_child(obj, "apple-aic", &s->aic, TYPE_APPLE_AIC);
}

// Apple M1 reset needs us to put the correct x0 register to point to our boot args
static void handle_m1_reset(void *opaque)
{
    ARMCPU *cpu = ARM_CPU(opaque);
    // TODO: Change all this logic into a blob that gets dropped into
    //       the rom area
    // TODO: More sanely seperate device_tree_end and boot args
    // FIXME: It's possible this should be replaced by a rom region that does this
    // while executing on the cpu
    
    // INFO: This way of doing things is actually done on several other machines mostly
    // not ARM ones though, but using the reset vector like this is fair, most of them
    // load a bootrom bios into a fixed location, setup the registers to a loaded main binary
    // image (or just let the bootrom work) and then set the PC to the entrypoint
    cpu_reset(CPU(cpu));
    cpu->env.xregs[0] = device_tree_end;
    cpu_set_pc(CPU(cpu), memmap[M1_MEM].base+0x4800);
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
    for (int i = 0; i < APPLE_M1_FIRESTORM_CPUS; i++) {
        // TODO: Use wrapper that takes cluster and core id (I think the 0x101
        // is actually 2 values, 0x1 and 0x1 again, differentiating cluster
        // and something else)
        /* All firestorms start off */
        m1_realize_cpu(&s->firestorm_cores[i], (0x101UL<<8)|i, true);
    }
    // Now do the icestorm cores
    for (int i = 0; i < APPLE_M1_ICESTORM_CPUS; i++) {
        // TODO: Use wrapper that takes cluster and core id (I think the 0x101
        // is actually 2 values, 0x1 and 0x1 again, differentiating cluster
        // and something else)
        /* All except core 0 start off */
        m1_realize_cpu(&s->icestorm_cores[i], i, i > 0);
    }

    // Framebuffer init
    // TODO: Set framebuffer memory area size as a property and connect it to the memory map here
    // (This is all temporary until the mapping system is understood enough to have framebuffer be
    // just another part of normal RAM with a default mapping) (which I also don't know how to setup
    // properly in qemu but VGA PCI stuff might be similar)
    AppleM1FBState *fb = &s->fb;
    sysbus_realize(SYS_BUS_DEVICE(fb), &error_abort);
    sysbus_mmio_map(SYS_BUS_DEVICE(fb), 0, memmap[M1_FB].base);

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
    .class_init = apple_m1_class_init,
};

static void apple_m1_register_types(void)
{
    type_register_static(&apple_m1_type_info);
}

type_init(apple_m1_register_types);

// This builds an apple style device tree then inserts it into memory somewhere, 
// TODO: Make this a lot nicer (model after the virt platform fdt builders)
// it should be dynamic (ideally we would take an existing device tree from
// hardware and then create the other devices using the information so contained
static void create_apple_dt(MachineState *machine) {
    struct node_header {
        uint32_t prop_count;
        uint32_t child_count;
    };
    struct node_property {
        char name[32];
        uint32_t size;
        // after this is a blob of size data (plus padding to align, which is annoying to express)
    };
    // TODO: Put more contents in here and do this correctly
    // FIXME: size this buffer more properly
    uint32_t bufsize = 0x1000;
    //uint8_t *databuf = g_malloc0(bufsize);
#if 0
    // Add initial root header?
    struct node_header header = {0x0, 0x1};
    memcpy(databuf, &header, sizeof(header));
    databuf += sizeof(header);

    // Add chosen header?
    header = {0x0, 0x0};
    memcpy(databuf, &header, sizeof(header));
    databuf += sizeof(header);
    struct node_header chosen = {2, 0};
    struct node_property prop_temp;
    // Add a parent property for the chosen tree?
    snprintf(prop_temp.name, 32, "chosen");
    prop_temp.size = 0;
#endif
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
    bufsize = sizeof(databuf);
    printf("Mapping device tree at: %lx\n", memmap[M1_ROM].base);
    rom_add_blob_fixed("device-tree", databuf, bufsize, memmap[M1_ROM].base);
    //g_free(databuf);
    // XXX: Make this use the actual size?
    device_tree_end = QEMU_ALIGN_UP(bufsize + memmap[M1_ROM].base, 64);
}

#define BOOTARGS_REVISION 0xDEAD
#define BOOTARGS_FB_WIDTH 800
#define BOOTARGS_FB_HEIGHT 600
#define BOOTARGS_FB_STRIDE (BOOTARGS_FB_WIDTH*4)
#define BOOTARGS_FB_DEPTH 30

// This builds and then inserts a boot args structure like m1n1 and OS X expect
// it then puts it into memory (I've moved it to 0x10 so that it's not set to
// a null pointer)
/* 
 * NOTE: Must be called after the data is loaded into memory since it relies
 * on safe_ram_start being defined
 */
static void create_boot_args(MachineState *machine) {
    struct {
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
        uint64_t unknown;
    } bootargs;
    bootargs.revision = 0xdead;
    bootargs.virtual_load_base = memmap[M1_MEM].base;
    bootargs.physical_load_base = memmap[M1_MEM].base;
    bootargs.memory_size = machine->ram_size;
    bootargs.loaded_end = safe_ram_start;
    bootargs.fb_addr = memmap[M1_FB].base;
    bootargs.fb_stride = BOOTARGS_FB_STRIDE;
    bootargs.fb_width = BOOTARGS_FB_WIDTH;
    bootargs.fb_height = BOOTARGS_FB_HEIGHT;
    bootargs.fb_depth = BOOTARGS_FB_DEPTH;

    // FIXME: There's got to be a better way of doing it than this. Maybe a property or something? Maybe not?
    bootargs.device_tree = memmap[M1_ROM].base;
    bootargs.device_tree_size = device_tree_end - memmap[M1_ROM].base;

    char buf[] ="no cmdline lol";
    memset(bootargs.cmdline, 0, sizeof(bootargs.cmdline));
    memcpy(bootargs.cmdline, buf, sizeof(buf)); // FIXME: This is stupid
    bootargs.cmdline[sizeof(buf)] = 0x0;

    printf("Mapping boot args tree at: %lx\n", device_tree_end);
    rom_add_blob_fixed("boot-args", &bootargs, sizeof(bootargs), device_tree_end);
}

static void m1_mac_init(MachineState *machine)
{
    AppleM1State *m1;

    m1 = APPLE_M1(qdev_new(TYPE_APPLE_M1));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(m1));
    qdev_realize_and_unref(DEVICE(m1), NULL, &error_abort);

    // This initializes the memory region map
    MemoryRegion *sysmem = get_system_memory();

    /* 
     * I think it's okay that this doesn't get freed later since many machines
     * create unfreed memory in their init, since it seems machines aren't meant
     * to be created multiple times ever, it's also the only way to create
     * anonymous MemoryRegion blocks which is helpful when you have lots of them
     * and don't need to keep the pointers other than for potential freeing
     */
    /* Create a memory region for the boot arguments (boot_args, ADT, startup code) */
    MemoryRegion *boot_args = g_new0(MemoryRegion, 1);
    memory_region_init_rom(boot_args, NULL, "boot-args", memmap[M1_ROM].size,
                           &error_abort);
    /* Add the ROM region to the system memory map */
    memory_region_add_subregion(sysmem, memmap[M1_ROM].base, boot_args);
    /* Add the RAM created for us to the main system memory space */
    memory_region_add_subregion(sysmem, memmap[M1_MEM].base, machine->ram);

    /* This is for loading M1N1 which acts as a sort of replacement firmware */
    /* We will want to do some sort of stub firmware loader first before M1N1
     * to setup the registers and boot args it expects, but for now we abuse
     * the reset handler and use ROM regions (we want to replace these with RAM
     * that reloads on CPU reset or something)
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
    safe_ram_start = memmap[M1_MEM].base;
    
    /* Keeps base and size so we can write a boot_param.config for qemu loader */
    hwaddr kernel_base = 0;
    int kernel_size = 0;
    hwaddr dtb_base = 0;
    int dtb_size = 0;
    hwaddr initrd_base = 0;
    int initrd_size = 0;
    
    /* TODO move these to specific helper functions, model after ppc/riscv boot
     * helper code which *does* work well with platforms that don't want to
     * have reset handled (unlike the ARM booter functions) */
    if (machine->firmware) {
        /* Align firmware to a 4K page */
        safe_ram_start = QEMU_ALIGN_UP(safe_ram_start, 4096);
        //printf("Found firmware: %s\n", machine->firmware);
        int load_size = load_image_targphys(machine->firmware, safe_ram_start,
                                        memmap[M1_MEM].size);
        if (kernel_size < 0) {
            error_report("Failed to load firmware from %s", machine->firmware);
            exit(1);
        }
        printf("Firmware loaded@%0lx size=%0x\n", safe_ram_start, load_size);
        safe_ram_start += load_size;
    }
    
    if (machine->kernel_filename) {
        /* Align kernel to a 2MB addr */
        safe_ram_start = QEMU_ALIGN_UP(safe_ram_start, 2 * 1024 * 1024);
        kernel_base = safe_ram_start;
        printf("Found kernel: %s\n", machine->kernel_filename);
        kernel_size = load_image_targphys(machine->kernel_filename, kernel_base,
                                        memmap[M1_MEM].size);
        if (kernel_size < 0) {
            error_report("Failed to load kernel from %s", machine->kernel_filename);
            exit(1);
        }
        printf("Kernel loaded@%0lx size=%0x\n", kernel_base, kernel_size);
        safe_ram_start += kernel_size;
    }
    
    if (machine->dtb) {
        printf("Found dtb: %s\n", machine->dtb);
        dtb_base = safe_ram_start;
        dtb_size = load_image_targphys(machine->dtb, dtb_base, 
                                       memmap[M1_MEM].size);
        if (dtb_size < 0) {
            error_report("Failed to load dtb from %s", machine->dtb);
            exit(1);
        }
        printf("DTB loaded@%0lx size=%0x\n", dtb_base, dtb_size);
        safe_ram_start += dtb_size;
    }
    
    if (machine->initrd_filename) {
        /* Align initrd to 64K like M1N1 does for some reason */
        safe_ram_start = QEMU_ALIGN_UP(safe_ram_start, 65536);
        initrd_base = safe_ram_start;
        printf("Found initrd: %s\n", machine->initrd_filename);
        initrd_size = load_image_targphys(machine->initrd_filename, initrd_base,
                                        memmap[M1_MEM].size);
        if (initrd_size < 0) {
            error_report("Failed to load initrd from %s", machine->initrd_filename);
            exit(1);
        }
        printf("Initrd loaded@%0lx size=%0x\n", initrd_base, initrd_size);
        safe_ram_start += initrd_size;
    }
    /* TODO: do this in a better way or not at all it's helpful for dev tho */
    FILE *fconfig = fopen("boot_params.config", "w");
    if (fconfig != NULL) {
        if (kernel_base) {
            fprintf(fconfig, "kernel\t0x%lx\t0x%x\n", kernel_base, kernel_size);
        }
        if (dtb_base) {
            fprintf(fconfig, "dtb\t0x%lx\t0x%x\n", dtb_base, dtb_size);
        }
        if (initrd_base) {
            fprintf(fconfig, "initrd\t0x%lx\t0x%x\n", initrd_base, initrd_size);
        }
        fclose(fconfig);
    }
    /* 
     * NOTE: These come after because we need to know the size of the loaded
     * data to safely load things
     */
    // Add apple flavored device tree
    create_apple_dt(machine);
    // Add boot args
    create_boot_args(machine);
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
