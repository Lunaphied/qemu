/*
 * Apple M1 SoC Framebuffer Emulation
 * 
 * Copyright (c) 2021 Iris Johnson <iris@modwiz.com>
 * 
 * SPDX-License-Identifier: MIT
 */
#ifndef APPLE_M1_FB_H
#define APPLE_M1_FB_H

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

/**
 * apple_m1_fb_30_to_24: return a color converted from 10 bit XRGB to 8 bit XRGB
 * @color: Packed 30bit XRGB color from framebuffer
 */
static inline uint32_t apple_m1_fb_30_to_24(uint32_t color)
{
    /* These drop the lower 2 bits since QEMU doesn't support 10 bit color surfaces */
    uint32_t r = (color >> 22) & 0xFF;
    uint32_t g = (color >> 12) & 0xFF;
    uint32_t b = (color >> 2) & 0xFF;
    return (r << 16) | (g << 8) | b;
}

#endif
