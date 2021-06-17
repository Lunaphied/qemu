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

// TODO: Replace this with an argument or add fw_cfg stuff so we can use builtin ramfb code
// NOTE: In our code we dedicate this region to the framebuffer (since we don't emulate the
// entire graphics complex yet and want to be able to display stuff still. The framebuffer
// region on real hardware is just some arbitrary region of memory, (there's more info known
// now or soon I think about how it's selected but it can definitely move) we will need to
// somehow carve out sections to be VRAM dynamically rather than allocating one on demand.
// I think the shadow/write-coalesing memory code is designed to make this work well since
// I think we already make VRAM normal memory and not MMIO but it will need to not expect
// itself to get a unique region since I don't think I can "move" the address of regions
// on demand (sadly we lose the nice memory map info that qemu would have since memory
// will just be shown as "ram" instead of split up into these neat little regions but
// we can just pull that from the kernel itself anyway)
#define VRAM_ZONE_SIZE (500 * MiB)

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
// FIXME: Do not merge until this garbage is replaced by dynamic load from either an Apple or Linux style
// device tree or some other configurable file.
// NOTE: I think ultimately the best solution here will be to create a data structure that 
// holds these tied to the machine state or the SoC state and then fill them in from some
// configuration file
static const struct MemMapEntry memmap[] = {
    [M1_UART] =             { 0x235200000,              0x10000},
    [M1_AIC] =              { 0x23B100000,              0x10000},
    [M1_WDT] =              { 0x23D2B0000,                0x100},
    /* TODO: replace this with a real addr from hardware */
    [M1_PMGR] =             { 0x23D300000,                0x100},
    /* NOTE: This contains VRAM as well given the UMA config of the M1 */
    [M1_MEM] =              { 0x800000000,       RAMLIMIT_BYTES},
};

/* Helper function to convert cpu_index to ARMCPU* pointer from complex */
/* TODO: This feels messy and horrible..., maybe they can be stored together
 * and then downcast into the correct type once we seperate those?
 */
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
// TODO: Move this to it's own damn file
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
        qemu_log_mask(LOG_UNIMP, "M1 PMGR: Unhandled read of @ %#" HWADDR_PRIx "\n", offset);
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
        qemu_log_mask(LOG_UNIMP, "M1 PMGR: Unhandled write of %#x to %#" HWADDR_PRIx "\n", value, offset);
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

// Apple M1 reset needs us to put the correct x0 register to point to our boot args
static void handle_m1_reset(void *opaque)
{
    AppleM1State *s = APPLE_M1(opaque);
    /* TODO: This needs to hold EVERYTHING having to do with the reset ideally
     * so all the processor state should probably get reset along with maybe
     * peripherals depending on hw behavior.
     * 
     * For now just reset the first icestorm core, not sure if we need to stop the
     * other cores since I haven't tested much since I made multicore work
     */
    ARMCPU *cpu = ARM_CPU(&s->icestorm_cores[0]);
    cpu_reset(CPU(cpu));
    /* Entry point code expects x0 to contain the address of the boot arguments */
    cpu->env.xregs[0] = s->boot_args_base;
    /* TODO: is cpu_set_pc better than trying to set directly via ARMCPU? */
    /* Set the entry point for post-reset as was determined from firmware */
    printf("Entry addr was: 0x%" HWADDR_PRIx "\n", s->entry_addr);
    cpu_set_pc(CPU(cpu), s->entry_addr);

    /* Set the initial register state to match the hardware.
     * This is a valid way to do it (used in other modules) but must be done
     * with care since it bypasses the checks done by the register handler. */

    /* Set the correct hypervisor configuration */
    cpu->env.cp15.hcr_el2 = HCR_API | HCR_APK | HCR_E2H | HCR_RW | HCR_TGE;

    /* Rebuild qemu's hidden flags after changing register state */
    arm_rebuild_hflags(&cpu->env);
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
     * (Update: It appears that RVBAR or an effectively similar function is
     * tied to the PMGR unit. Poking it with an address causes a core to start 
     * there and seems to lock it's reset address there?, icestorm core 0 is initial
     * bring up core, so it probably gets initialized internally somehow)
     * 
     * So Icestorm core 0 is the one that jumps into the kernel and allows the
     * other cores to be started via the PMGR registers in the SoC.
     */
    /* 
     * TODO: QEMU has some built-in wrappers for clusters and core types
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

    /* 
     * Realize framebuffer, VRAM's region is setup for us during the machine init
     * we just setup the link and realize the framebuffer here.
     */
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
    sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0, qdev_get_gpio_in(DEVICE(&s->aic), 605));
    
    /* TODO: Connect input IRQs from hardware to this */
    //create_unimplemented_device("AIC", memmap[M1_AIC].base, memmap[M1_AIC].size); 
    // TODO: Implement
    create_unimplemented_device("Watchdog", memmap[M1_WDT].base, memmap[M1_WDT].size);
    
    // FIXME: Do this properly
    // Add a reset hook, this emulates the processor (and some firmware for now)
    // for now all it does is reset the first CPU core to the base address
    qemu_register_reset(handle_m1_reset, s);
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

/* 
 * Helper method to insert a flat image from a file into memory with a given alignment
 * Offset is updated after load to the next unused address
 */
static int load_image_aligned(const char *filename, hwaddr* offset, uint64_t alignment, uint64_t max_sz) {
    if (!filename || !offset) {
        return -1;
    }
    
    hwaddr aligned_offset = QEMU_ALIGN_UP(*offset, alignment);
    int r = load_image_targphys(filename, aligned_offset, max_sz);
    if (r < 0) {
        return r;
    }

    printf("Loaded file %s (size 0x%x) at RAM offset 0x%" HWADDR_PRIx "\n", filename, r, aligned_offset);

    /* Update offset to be next address after written */
    *offset = aligned_offset + r;
    return r;
}

/*
 * Helper method to insert a blob from memory in an aligned fashion similar to above code
 */
static int load_blob_aligned(const char *name, const void *blob, hwaddr* offset, uint64_t alignment, uint64_t size) {
    if (!name || !offset) {
        return -1;
    }

    hwaddr aligned_offset = QEMU_ALIGN_UP(*offset, alignment);
    /* This is mostly doing the same thing as rom_add_blob_fixed but we need read_only to be false */
    /* It returns a NULL MemoryRegion* when we don't use fw_cfg */
    (void) rom_add_blob(name, blob, size, size, aligned_offset, NULL, NULL, NULL, NULL, false);

    printf("Loaded blob %s (size 0x%" PRIx64 ") at RAM offset 0x%" HWADDR_PRIx "\n", name, size, aligned_offset);

    /* Update offset to be next address after written */
    *offset = aligned_offset + size;
    return size;
}

// This builds an apple style device tree then inserts it into memory somewhere, 
// TODO: Make this a lot nicer (model after the virt platform fdt builders)
// it should be dynamic (ideally we would take an existing device tree from
// hardware and then create the other devices using the information so contained
// This takes an address which will hold the address after the aligned buffer after the call
// the return value is the size of the buffer, we can use these to reconstruct the aligned_base
static int init_apple_dt(hwaddr *adt_base) {
    // TODO: Use the new ADT code to construct this somewhat dynamically at least
    // TODO: After the above is done we should load this from a file or something
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
    return load_blob_aligned("device-tree", &databuf, adt_base, 8, sizeof(databuf));
}

/* XNU uses this format and iBoot provides it, so we must also provide it */
/* This has been rearranged to flatten out into a single level struct */
struct xnu_boot_args {
    uint16_t revision;              /* XNU says this is the revision of the boot_args */
    uint16_t version;               /* Then calls this the "version", how is that different... */
    uint32_t pad0;                  /* Padding required to avoid packing */
    uint64_t virtual_base;          /* Virtual base of memory (what this really tells us is how to compensate for ASLR slide) */
    uint64_t physical_base;         /* Physical base of memory */
    uint64_t memory_size;           /* Physical memory size before VRAM and SEP carveouts */
    uint64_t top_of_kernel_data;    /* End address of loaded binary (safe mem starts here) */
    uint64_t vram_base;             /* Initial framebuffer vram address */
    uint64_t pad1;                  /* Padding required to avoid packing */ 
    uint64_t fb_stride;             /* Stride (bytes per row) for vram */
    uint64_t fb_width;
    uint64_t fb_height;
    uint64_t fb_depth;              // TODO: Extract what real hardware puts here
    uint64_t pad2;                  /* Padding required to avoid packing */
    uint64_t adt_base;              /* ADT address in memory */
    uint32_t adt_size;              /* Overall ADT size */
    char cmdline[608];              /* Kernel cmdline args, m1n1 will pass onwards to linux iirc */
    uint64_t flags;                 /* I'm assuming some kind of boot flags not covered by cmdline */
    uint64_t memory_size_actual;    /* This is the real size of physical memory (max physical addr + 1) */
};

/* This fills in the boot_args struct and registers it into memory via the rom system
 * which should automatically copy it into place upon reset.
 * NOTE: This code expects device_tree_(base|size) and safe_ram_start to be initialized
 * already since that's what it's going to fill in.
 */
static void initialize_boot_args(AppleM1State *s, hwaddr phys_mem_base, hwaddr mem_size,
                                 hwaddr mem_size_actual, hwaddr top_of_kernel_data,
                                 hwaddr vram_base, hwaddr adt_base, hwaddr adt_size) {
    /* First we construct a bootargs struct, then we use the rom system to insert it */
    /* The memory region must already be initialized, since that's what's expected here */
    struct xnu_boot_args boot_args = {0, };

    /* Revsion is arbitrary and set to something to catch issues */
    boot_args.revision = cpu_to_le16(0xdead);

    /* We don't do an ASLR slide but we could to try and catch issues in M1N1/Firmware */
    /* Virtual base is relied on being connected to what m1n1 says it should be */
    boot_args.virtual_base = cpu_to_le64(0xfffffe0010000000);
    boot_args.physical_base = cpu_to_le64(phys_mem_base);

    /* This memory size is post carveout */
    boot_args.memory_size = cpu_to_le64(mem_size);
    boot_args.top_of_kernel_data = cpu_to_le64(top_of_kernel_data);

    /* TODO: VRAM size is assumed to fit the fb_stride*fb_height bytes we should validate */
    boot_args.vram_base = cpu_to_le64(vram_base);
    boot_args.fb_stride = cpu_to_le64(s->fb.width*4);
    boot_args.fb_width = cpu_to_le64(s->fb.width);
    boot_args.fb_height = cpu_to_le64(s->fb.height);
    /* TODO: Find out what this is in the real world*/
    boot_args.fb_depth = cpu_to_le64(30);

    boot_args.adt_base = cpu_to_le64(adt_base-phys_mem_base+0xfffffe0010000000);
    boot_args.adt_size = cpu_to_le64(adt_size);

    /* For now do this, later we should probably pass it a real command line */
    const char* buf = "no cmdline lol";
    snprintf(boot_args.cmdline, sizeof(boot_args.cmdline), "%s", buf);

    /* XXX: Don't know if this is right yet */
    boot_args.memory_size_actual = vram_base + VRAM_ZONE_SIZE - phys_mem_base;
    /* 
     * Now insert it into memory at the correct address, this will be reloaded on
     * reset since it's registered as a rom region
     */
    int size = load_blob_aligned("boot-args", &boot_args, &s->boot_args_base, sizeof(uint64_t), sizeof(boot_args));
    /* 
     * The above modifies the provided offset, it also potentially adds alignment padding we must
     * fix the written address to be the unpadded base by backtracking from the end by the size 
     */
    s->boot_args_base -= size;
}

static void m1_mac_init(MachineState *machine)
{
    /*
     * Allocate the M1 SoC state so we can fill in bits of it
     * However we purposefully do not link it to the machine and realize it until later
     * on when things like the VRAM memory region have been filled in.
     */
    AppleM1State *m1;
    m1 = APPLE_M1(qdev_new(TYPE_APPLE_M1));

    // This initializes the memory region map
    MemoryRegion *sysmem = get_system_memory();

    /* TODO: Memory region mapping should probably be split into being somewhere else */
    /* Grab the base address of physical memory in the address space */
    hwaddr phys_base = memmap[M1_MEM].base;
    /* Compute the final valid physical memory address */
    hwaddr phys_end = memmap[M1_MEM].base+machine->ram_size;

    /* Check to make sure there's enough memory for VRAM */
    if (VRAM_ZONE_SIZE >= phys_end) {
        error_report("Insufficent RAM size (VRAM would not fit).\n");
        exit(1);
    }

    /* VRAM is stolen from the final bit of ram */
    hwaddr vram_base = phys_end - VRAM_ZONE_SIZE;

    /* Map the pre-allocated machine memory into the system memory */
    memory_region_add_subregion(sysmem, phys_base, machine->ram);

    /* Map the VRAM region to the correct address */
    /* NOTE: We tried to do this as an lias and it simply didn't work, the FB memory helper
     * code gets extermely upset by that, (memory_region_find returns NULL), even if it did
     * work we'd be left with VGA writeback mode being set on the entire system memory
     * which is probably bad. I wanted to do this with the alias to better represent the
     * real hardware and avoid allocating shadowed memory, but there are other workarounds
     * and it probably doesn't even matter since unused regions will probably be swapped out
     * and/or never faulted in since they're unaccessed. Furthermore we could emulate the
     * ram-fb code which directly accesses the memory and I think doesn't bother with the
     * helpers although that might be bad performance wise. Either way the alias approach is
     * totally broken
     */
    memory_region_init_ram(&m1->ram_vram_mr, OBJECT(m1), "vram", VRAM_ZONE_SIZE, &error_abort);

    /* The VRAM is system accessable too so add it to the main system map shadowing normal RAM */
    /* We actually add it to the RAM region itself for various reasons */
    memory_region_add_subregion_overlap(machine->ram, vram_base - phys_base, &m1->ram_vram_mr, 0);

    /* Now we can initialize the M1 SoC class itself since the vram region is setup */
    /* TODO: Move this to the end of this function since it probably logically goes there */
    object_property_add_child(OBJECT(machine), "soc", OBJECT(m1));
    qdev_realize_and_unref(DEVICE(m1), NULL, &error_abort);

    /* This is for loading M1N1 which acts as a sort of replacement firmware */
    /* 
     * NOTE: we cannot really use the helper functions from arm/boot.c because
     * loading elfs offers no control over the load address (it just uses the
     * physical values which are offset from 0 for m1n1), we also want to start
     * M1N1 with certain register and memory regions setup like iBoot would with
     * info about the hardware (Apple's form of DTB).
     * 
     * (Update: The above is sort of true, the helpers in arm/boot.c aren't well
     * suited for our uses here, but the generic rom loading stuff for loading
     * elf's could probably be modified to fit our purposes. The elf code allows
     * specifying a translate function which we could use to patch the address
     * problem)
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
     * essentially means we just grab the mach-o (which is allowed to be loaded
     * as a flat image with a fixed entry point) and dump it into memory, setup
     * the required boot arguments (because only QEMU knows the values for
     * those) and jump in, with some helper code to dump a kernel, initrd and
     * dtb into memory so that proxy code can start it without copying it over
     * the serial port. 
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
    hwaddr offset = phys_base;
    printf("phys_base: 0x%" HWADDR_PRIx "\n", phys_base);
    hwaddr remaining_mem = machine->ram_size - VRAM_ZONE_SIZE;

    /*
     * We put the ADT right in front of the "firmware" since m1n1 doesn't expect itself
     * to be the first firmware, I think this is best changed to be handled internal
     * with some code to have M1N1 construct an ADT using fw_cfg values or something
     */
    /* FIXME: We prealign because otherwise alignment might mean we don't know the base */
    hwaddr adt_base = QEMU_ALIGN_UP(offset, 8);
    int adt_size = init_apple_dt(&offset);
    
    /* TODO move these to specific helper functions, model after ppc/riscv boot
     * helper code which *does* work well with platforms that don't want to
     * have reset handled (unlike the ARM booter functions) */
    if (machine->firmware) {
        /* Align firmware to a 16K page */
        int result = load_image_aligned(machine->firmware, &offset, 65536, remaining_mem);
        if (result < 0) {
            error_report("Failed to load firmware from %s", machine->firmware);
            exit(1);
        }
    	// HACK: Total hack to hardcode entry offset of m1n1 for post reset
        m1->entry_addr = offset - result + 0x4800;
        printf("Entry addr computed was: 0x%" HWADDR_PRIx "\n", m1->entry_addr);
        remaining_mem -= result;
    }
    
    if (machine->kernel_filename) {
        /* Align kernel to 2 MiB */
        int result = load_image_aligned(machine->kernel_filename, &offset, 2*1024*1024, remaining_mem);
        if (result < 0) {
            error_report("Failed to load kernel from %s", machine->kernel_filename);
            exit(1);
        }
        remaining_mem -= result;
    }

    if (machine->dtb) {
        /* Align dtb to a 16K page */
        int result = load_image_aligned(machine->dtb, &offset, 65536, remaining_mem);
        if (result < 0) {
            error_report("Failed to load dtb from %s", machine->dtb);
            exit(1);
        }
        remaining_mem -= result;
    }

    if (machine->initrd_filename) {
        /* Align initrd_filename to 64K, I believe linux gets mad otherwise */
        int result = load_image_aligned(machine->initrd_filename, &offset, 65536, remaining_mem);
        if (result < 0) {
            error_report("Failed to load initrd from %s", machine->initrd_filename);
            exit(1);
        }
        remaining_mem -= result;
    }
    /* FIXME: Another prealignment */
    /* This padding is required because if we load m1n1 flat instead of properly, the space for the bss region isn't computed properly */
    offset = ROUND_UP(offset+0x100000, 8);
    m1->boot_args_base = offset;
    initialize_boot_args(m1, phys_base, remaining_mem, phys_end - phys_base, offset,
                         vram_base, adt_base, adt_size);

    /* NOTE: This can be removed once our loading code either emulates Mach-O enough to
     * not confuse M1N1 when trying to get m1n1 to auto-boot the packed together image
     * or when M1N1 itself is modified to be more friendly to QEMU usage such that
     * it handles being loaded as a cat'd binary even when loaded flat
     */
    /* TODO: do this in a better way or not at all it's helpful for dev tho */
    /* 
     * TODO: Fix this next patch because accesser functions need an additional param to 
     * write back aligned base to make this work easily
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
    */
}

static void m1_mac_machine_init(MachineClass *mc)
{
    /* TODO: These constants and things should be specified more realistically
     * since they're just kinda pulled out of nowhere right now.
     */
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
// TODO: What am I trying to change!?
DEFINE_MACHINE("apple-m1", m1_mac_machine_init)
