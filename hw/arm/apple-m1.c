#include "qemu/osdep.h"
#include "exec/address-spaces.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/qdev-core.h"
#include "cpu.h"
#include "hw/sysbus.h"
#include "hw/char/serial.h"
#include "hw/misc/unimp.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/loader.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "hw/arm/apple-m1-soc.h"

static void apple_m1_soc_init(Object *obj) {
    AppleM1SoCState *s = APPLE_M1_SOC(obj);

    object_initialize_child(obj, "maincpu", &s->maincore,
                            ARM_CPU_TYPE_NAME("cortex-a72"));
}

static void apple_m1_soc_realize(DeviceState *dev, Error **errp)
{
    AppleM1SoCState *s = APPLE_M1_SOC(dev);

    qdev_realize(DEVICE(&s->maincore), NULL, &error_abort);
}

static void apple_m1_soc_class_init(ObjectClass *oc, void *data) { 
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = apple_m1_soc_realize;
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

#if 1
static void macmini_init(MachineState *machine)
{
    AppleM1SoCState *m1;

    m1 = APPLE_M1_SOC(object_new(TYPE_APPLE_M1_SOC));
    //object_property_add_child(OBJECT(machine), "soc", OBJECT(m1));
    object_unref(OBJECT(m1));

    qdev_realize(DEVICE(m1), NULL, &error_abort);

}

static void macmini_machine_init(MachineClass *mc)
{
    mc->init = macmini_init;
    mc->desc = "Apple M1 Mac Mini";
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a57");
    mc->block_default_type = IF_SD;
    mc->default_ram_size = 1 * GiB;
    mc->default_ram_id = "macmini.ram";
}

DEFINE_MACHINE("macmini-m1", macmini_machine_init)
#endif
