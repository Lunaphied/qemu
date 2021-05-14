/*
 * Apple M1 SoC and Mac Mini board emulation
 * 
 * Copyright (c) 2021 Iris Johnson <iris@modwiz.com>
 * 
 * SPDX-License-Identifier: MIT
 */

#ifndef HW_ARM_APPLE_M1_H
#define HW_ARM_APPLE_M1_H

#include "qom/object.h"
#include "target/arm/cpu.h"
#include "hw/display/m1_fb.h"
#include "hw/intc/apple-aic.h"

// The M1 has a fixed number of cores, model that
/* TODO: This seems more messy as time goes on... */
#define APPLE_M1_FIRESTORM_CPUS 4
#define APPLE_M1_ICESTORM_CPUS  4
#define APPLE_M1_TOTAL_CPUs     8

// Renamed to just apple-m1 since that seems to be the SoC name
#define TYPE_APPLE_M1 "apple-m1"

OBJECT_DECLARE_SIMPLE_TYPE(AppleM1State, APPLE_M1)

// A RAM memory region that is reloaded from static data on reset
struct m1_rel_region {
    MemoryRegion region;
    void* data;
};


struct AppleM1State {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    /* Properties */
    uint32_t num_firestorm;
    uint32_t num_icestorm;
    
    /* Cores */
    /* Icestorm  0..num_icestorm-1 */
    /* Firestorm num_icestorm..num_firestorm-1 */
    /* TODO Replace this with the proper concept of core clusters */
    ARMCPU icestorm_cores[APPLE_M1_ICESTORM_CPUS];
    ARMCPU firestorm_cores[APPLE_M1_FIRESTORM_CPUS];

    /* SoC devices */
    M1FBState fb;
    AppleAICState aic;
    
    /* SoC memory regions */
    struct m1_rel_region ram_adt_mr;
    struct m1_rel_region ram_firmware_mr;
    struct m1_rel_region ram_kernel_mr;
    struct m1_rel_region ram_initrd_mr;
    struct m1_rel_region ram_dtb_mr;
    struct m1_rel_region ram_bootargs_mr;
    MemoryRegion ram_free_mr;
    MemoryRegion ram_vram_mr;
};

#endif // HW_ARM_APPLE_M1_H
