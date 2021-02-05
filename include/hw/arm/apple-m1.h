/*
 * Apple M1 SoC and Mac Mini board emualtion
 * 
 * Copyright (c) 2021 Iris Johnson <iris@modwiz.com>
 * 
 * SPDX-License-Identifier: MIT
 */

#ifndef HW_ARM_APPLE_M1_H
#define HW_ARM_APPLE_M1_H

#include "qom/object.h"
#include "target/arm/cpu.h"
#include "hw/display/apple-m1-fb.h"

// The M1 has a fixed number of cores, model that
#define APPLE_M1_FIRESTORM_CPUS 4
#define APPLE_M1_ICESTORM_CPUS 4

// Renamed to just apple-m1 since that seems to be the SoC name
#define TYPE_APPLE_M1 "apple-m1"

OBJECT_DECLARE_SIMPLE_TYPE(AppleM1State, APPLE_M1)

struct AppleM1State {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    /* Cores */
    ARMCPU firestorm_cores[APPLE_M1_FIRESTORM_CPUS];
    ARMCPU icestorm_cores[APPLE_M1_ICESTORM_CPUS];

    /* SoC devices */
    AppleM1FBState fb;
};

#endif // HW_ARM_APPLE_M1_H
