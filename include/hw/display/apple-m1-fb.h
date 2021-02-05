/*
 * Apple M1 SoC Framebuffer Emulation
 * 
 * Copyright (c) 2021 Iris Johnson <iris@modwiz.com>
 * 
 * SPDX-License-Identifier: MIT
 */
#ifndef HW_FB_APPLE_M1_FB_H
#define HW_FB_APPLE_M1_FB_H

#include "hw/sysbus.h"
#include "ui/console.h"
#include "qom/object.h"

#define TYPE_APPLE_M1_FB "apple-m1-fb"
OBJECT_DECLARE_SIMPLE_TYPE(AppleM1FBState, APPLE_M1_FB);

struct AppleM1FBState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion vram;
    MemoryRegionSection vram_section;
    QemuConsole *console;
};

#endif /* HW_FB_APPLE_M1_FB_H */
