#include "qemu/osdep.h"
#include "qom/object.h"
#include "ui/console.h"
#include "qapi/error.h"
#include "framebuffer.h"
#include "hw/display/apple-m1-fb.h"

static void apple_m1_fb_draw_row(void *opaque, uint8_t *dest, const uint8_t *src,
                                 int width, int pitch)
{
    /* FIXME: This is not host endian safe I don't think */
    for (int i = 0; i < width*pitch; i+=4) {
            uint32_t color = src[i];
            color |= src[i+1] << 8;
            color |= src[i+2] << 16;
            color |= src[i+3] << 24;
            color = apple_m1_fb_30_to_24(color);
            dest[i] = color & 0xFF;
            dest[i+1] = (color >> 8) & 0xFF;
            dest[i+2] = (color >> 16) & 0xFF;
            dest[i+3] = (color >> 24) & 0xFF;
            //dest[i] = 0xFF;
            //dest[i+3] = 0xFF;
            //dest[i] = src[i];
            //dest[i+1] = src[i+1];
            //dest[i+2] = src[i+2];
            //dest[i+3] = src[i+3];
    }
}

static void apple_m1_fb_update(void *opaque)
{
    AppleM1FBState *s = APPLE_M1_FB(opaque);
    DisplaySurface *surface = qemu_console_surface(s->console);

    /* Used as first row */
    int first = 0;
    /* Unused but updated by helper */
    int last;

    /* Other code sets an invalidate handler that pokes a flag in the ops
     * and only does a update memory section when that flag is set, why is
     * that done?
     */
    // Maybe this only needs to be done once i.e. if vram region is
    // changed by moving the framebuffer in memory?
    framebuffer_update_memory_section(&s->vram_section, &s->vram, 0,
                                      600, 800*4);
    framebuffer_update_display(surface, &s->vram_section, 800, 600,
                               800*4, 800*4, 4, 0, apple_m1_fb_draw_row,
                               s, &first, &last);
    dpy_gfx_update(s->console, 0, 0, 800, 600);
}

static void apple_m1_fb_invalidate(void *opaque)
{
        printf("FB invalidate called\n");
}

static const GraphicHwOps apple_m1_fb_ops = {
    .invalidate = apple_m1_fb_invalidate,
    .gfx_update = apple_m1_fb_update,
};

static void apple_m1_fb_realize(DeviceState *dev, Error **errp)
{
    AppleM1FBState *s = APPLE_M1_FB(dev);
    
    memory_region_init_ram(&s->vram, OBJECT(dev), "vram", 800*600*4,
                           &error_abort);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->vram);

    s->console = graphic_console_init(dev, 0, &apple_m1_fb_ops, s);
    qemu_console_resize(s->console, 800, 600);
}

static void apple_m1_fb_class_init(ObjectClass *oc, void *data) { 
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = apple_m1_fb_realize;
}

static const TypeInfo apple_m1_fb_type_info = {
    .name = TYPE_APPLE_M1_FB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AppleM1FBState),
    .class_init = apple_m1_fb_class_init,
};

static void apple_m1_fb_register_types(void)
{
    type_register_static(&apple_m1_fb_type_info);
}

type_init(apple_m1_fb_register_types);
