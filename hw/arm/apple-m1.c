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

// FROM exynos4210.h
DeviceState *exynos4210_uart_create(hwaddr addr,
                                    int fifo_size,
                                    int channel,
                                    Chardev *chr,
                                    qemu_irq irq);

static const struct MemMapEntry memmap[] = {
    [VIRT_MEM] =            {     0x10000,       8*GiB},
    [VIRT_UART] =           { 0x235200000,     0x10000},
};

static void apple_m1_soc_init(Object *obj) {
    AppleM1SoCState *s = APPLE_M1_SOC(obj);

    object_initialize_child(obj, "maincpu", &s->maincore,
                            ARM_CPU_TYPE_NAME("cortex-a72"));
}

static void apple_m1_soc_realize(DeviceState *dev, Error **errp)
{
    AppleM1SoCState *s = APPLE_M1_SOC(dev);
    //MemoryRegion *system_mem = get_system_memory();

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

    MemoryRegion *sysmem = get_system_memory();
    memory_region_add_subregion(sysmem, memmap[VIRT_MEM].base, machine->ram);

    m1_boot_info.loader_start = memmap[VIRT_MEM].base;
    m1_boot_info.ram_size = machine->ram_size;
    // HACK to try and get m1n1 loaded at a different address    
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
