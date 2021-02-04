#ifndef HW_ARM_APPLE_M1_SOC_H 
#define HW_ARM_APPLE_M1_SOC_H

#include "qom/object.h"
#include "target/arm/cpu.h"
#include "hw/display/apple-m1-fb.h"

enum {
    VIRT_MEM,   // A block of ram
    VIRT_UART,  // The virtual uart (on real hardware this is on the type-C and multiplexed via USB-PD selection)
    BOOT_ARGS,  // We setup a block of ROM to make pretend boot arguments to match what the real hardware does
    VIRT_FB,    // The m1n1 code expects a framebuffer, for now this just means to dump ram in this region
};

// The M1 has a fixed number of cores, model that
#define APPLE_M1_FIRESTORM_CPUS 4
#define APPLE_M1_ICESTORM_CPUS 4

// Renamed to just apple-m1 since that seems to be the most generic SoC name
#define TYPE_AAPL_M1 "apple-m1"

OBJECT_DECLARE_SIMPLE_TYPE(AaplM1State, AAPL_M1)

struct AaplM1State {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    /* Cores */
    ARMCPU firestorm_cores[APPLE_M1_FIRESTORM_CPUS];
    ARMCPU icestorm_cores[APPLE_M1_ICESTORM_CPUS];

    /* SoC devices */
    AppleM1FBState fb;
};

#endif // HW_ARM_APPLE_M1_SOC_H
