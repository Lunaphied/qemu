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
#include "exec/hwaddr.h"
#include "sysemu/sysemu.h"
#include "exec/exec-all.h"
#include "hw/arm/apple-m1-soc.h"

// Notes on this file:
// - This file doesn't actually represent the M1 very well, it's more a platform designed and focused around m1n1
// - The ARM IRQ pin is directly connected to the UART, which is helpful if you want to test the simplest 
//   interrupts possible
// - The structure of this file is based on the other board files I could find and me making my best guess
//   unfortunately that doesn't get you very far, and there's some not so great stuff here too just to tie
//   it all together to get an interesting demo
// - The "M1" is emulated as if it's an SoC containing multiple Cortex-A72 cores, that's not true in reality,
//   Apple has custom cores that have some special functionality that might be worth emulating, but now we have
//   to edit the actual aarch64 emualtion logic (and we don't know the changes in full yet other than registers).
//   I suspect with all Apple's custom registers and changes to existing registers, massive changes would be 
//   required for true system emulation, this is something worth considering but not here, not right now.
//
// One idea for this is to do what's suggested below and literally pull in a device tree (converted to
// not apple format) then use that to layout all the devices, creating stub regions of unimplemented registers
// until they're created. Unfortunately it seems like nothing else in qemu does this, not even the most modern
// code. So there's no framework, so that seems like it would be it's own project to prototype. 
// So for now we hardcode and make a few M1 helpers

// This is used in several other ARM platforms to help collect the addresses and give them meaningful names
// during realization. These aren't correlated to the M1 hardware at all though it's just easier to set
// them up like this during dev (the UART is the correct address for where it is when m1n1 loads though)
// In reality this should be constructed from a hardware device tree provided from an M1 Mac
static const struct MemMapEntry memmap[] = {
    [BOOT_ARGS] =           {         0x0,     0x10000},
    [VIRT_MEM] =            {     0x10000,       8*GiB},
    [VIRT_UART] =           { 0x235200000,     0x10000},
};

static void apple_m1_soc_init(Object *obj) {
    AppleM1SoCState *s = APPLE_M1_SOC(obj);

    // not sure if this is the best way to create a CPU core, but we definitely want to use the A72 which
    // seems to be the fanciest ARM core that qemu supports at the moment (which is probably the most like
    // the M1's cores)
    object_initialize_child(obj, "maincpu", &s->maincore,
                            ARM_CPU_TYPE_NAME("cortex-a72"));
}

static void apple_m1_soc_realize(DeviceState *dev, Error **errp)
{
    AppleM1SoCState *s = APPLE_M1_SOC(dev);
    //MemoryRegion *system_mem = get_system_memory();

    // Yes there's only one CPU right now. This makes testing easier and I don't have an emulated interrupt
    // controller so I think SMP stuff would be weird (probably not impossible since SEV exists though)
    // But in truth we'd want to model the way the smp bringup works on actual hardware so lets not waste time
    // on it now
    DeviceState* cpuobj = DEVICE(&s->maincore);

    // The apple M1 has no EL3 support but does have EL2, so these must be set for compatibility
    qdev_prop_set_bit(cpuobj, "has_el3", false);
    qdev_prop_set_bit(cpuobj, "has_el2", true);

    qdev_realize(DEVICE(cpuobj), NULL, &error_abort);

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
    SysBusDevice *bus = SYS_BUS_DEVICE(uart);
    sysbus_realize_and_unref(bus, &error_abort);
    sysbus_mmio_map(bus, 0, memmap[VIRT_UART].base);
    sysbus_connect_irq(bus, 0, qdev_get_gpio_in(cpuobj, ARM_CPU_IRQ));
    // exynos4210_uart_create(memmap[VIRT_UART].base, 16, 0, serial_hd(0),
    //                       qdev_get_gpio_in(DEVICE(cpuobj), ARM_CPU_IRQ));
}

static void apple_m1_soc_class_init(ObjectClass *oc, void *data) { 
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = apple_m1_soc_realize;

    // The M1 SoC is mostly a glued-to-machine component it has no meaning in other contexts
    // additionally our use of serial_hd() apparently is not good according to other comments
    dc->user_creatable = false;
}

static const TypeInfo apple_m1_soc_type_info = {
    .name = TYPE_APPLE_M1_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(AppleM1SoCState),
    .instance_init = apple_m1_soc_init,
    .class_init = apple_m1_soc_class_init,
};

static void apple_m1_soc_register_types(void)
{
    type_register_static(&apple_m1_soc_type_info);
}

type_init(apple_m1_soc_register_types);

static struct arm_boot_info m1_boot_info;

static void apple_m1_init(MachineState *machine)
{
    AppleM1SoCState *m1;

    m1 = APPLE_M1_SOC(qdev_new(TYPE_APPLE_M1_SOC));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(m1));
    qdev_realize_and_unref(DEVICE(m1), NULL, &error_abort);

    // This initializes the memory region map
    MemoryRegion *sysmem = get_system_memory();

    // doesn't this leak?
    MemoryRegion *boot_args = g_new0(MemoryRegion, 1);
    memory_region_init_rom(boot_args, NULL, "boot-args", memmap[BOOT_ARGS].size, &error_abort);
    memory_region_add_subregion(sysmem, memmap[BOOT_ARGS].base, boot_args);
    memory_region_add_subregion(sysmem, memmap[VIRT_MEM].base, machine->ram);

    // Used to add bootargs, yes I know this memory leaks but it doesn't matter
    // at the moment, oddly adding the blob doesn't actually add it into the memory map
    // which is annoying
    void* boot_args_data = g_new0(uint8_t, 0x300);
    // set the revision to something noticable
    *(uint16_t*) ((uint8_t*) boot_args_data + 0) = 0xdead;
    // Set virtual and physical base to the same value since virtual load wasn't performed
    *(uint64_t*) ((uint8_t*) boot_args_data + 8) = memmap[VIRT_MEM].base;
    *(uint64_t*) ((uint8_t*) boot_args_data + 16) = memmap[VIRT_MEM].base;    
    // set the memory size to the correct value
    *(uint64_t*) ((uint8_t*) boot_args_data + 24) = memmap[VIRT_MEM].size;
    // Set the end of kernel address to like some value far after m1n1
    // XXX: This especially of all things will need fixing
    *(uint64_t*) ((uint8_t*) boot_args_data + 32) = memmap[VIRT_MEM].base+0x40000;
    // Setup the data to be written into ROM before the CPU boots
    rom_add_blob_fixed("boot-args", boot_args_data, 0x300, memmap[BOOT_ARGS].base);
    // Free our temp data, (the previous step copies it elsewhere)
    g_free(boot_args_data);

    m1_boot_info.loader_start = memmap[VIRT_MEM].base;
    m1_boot_info.ram_size = machine->ram_size;

    arm_load_kernel(ARM_CPU(first_cpu), machine, &m1_boot_info);
    printf("Entry after load: %lx\n", m1_boot_info.entry);
}

static void apple_m1_machine_init(MachineClass *mc)
{
    mc->init = apple_m1_init;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;
    mc->default_ram_id = "ram";
    mc->default_ram_size = 8 * GiB;
    mc->desc = "Apple M1 Mac Mini";
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a57");
    mc->block_default_type = IF_SD;
}

DEFINE_MACHINE("apple-m1", apple_m1_machine_init)
