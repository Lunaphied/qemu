#ifndef HW_ARM_APPLE_M1_SOC_H
#define HW_ARM_APPLE_M1_SOC_H

#include "qom/object.h"
#include "target/arm/cpu.h"

#define TYPE_APPLE_M1_SOC "apple-m1-soc"

OBJECT_DECLARE_SIMPLE_TYPE(AppleM1SoCState, APPLE_M1_SOC)

struct AppleM1SoCState {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    ARMCPU maincore;
};

#endif // HW_ARM_APPLE_M1_SOC_H
