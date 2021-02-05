/*
 * Apple M1 Interrupt Controller
 * 
 * Copyright (c) 2021 Iris Johnson <iris@modwiz.com>
 * 
 * SPDX-License-Identifier: MIT
 */

/*
 * For a full description of the AIC interrupt controller read the
 * irq-apple-aic.c driver in the Linux kernel, the code is unrelated
 * but the peripheral is documented full there.
 * 
 * Below is extracted from https://github.com/AsahiLinux/docs/wiki/HW:AIC
 * 
 * Basic memory layout is:
 * 0x0000 ... 0x2000 - Global configuration
 * 0x2000 ... 0x3000 - Interrupts, ACKS, IPIs
 * 0x3000 ... 0x4000 - Target distribution (1 per reg, CPU mask in reg)
 * 
 * 0x0004            - Info register (lower bits store NR irqs)
 * 
 * 0x4000            - Software generated set bits
 * 0x4080            - Software generated clear bits
 * 0x4100            - IRQ mask set bits
 * 0x4180            - IRQ mask clear bits
 * 0x4200            - HW IRQ line status maybe?
 * 
 * 0x8020            - System timer (CNTPCT_EL0) low 32
 * 0x8028            - System timer (CNTPCT_EL0) high 32
 * 
 * 0x2000            - Current CPU id?
 * 0x2004            - IRQ reason
 * 0x2008            - IPI flag set
 * 0x200C            - IPI flag clear
 * 0x2024            - IPI mask set
 * 0x2028            - IPI mask clear
 */

#include "qemu/osdep.h"
#include "hw/intc/apple-aic.h"

/* TODO: Add tracing */

/* TODO: Register defines */
#define AIC_INFO         0x0004
#define AIC_CURRENT_CORE 0x2000
#define AIC_IRQ_REASON   0x2004

/* HW to CPU distribution map */
#define AIC_DIST         0x3000
#define AIC_DIST_SIZE    0x1000
#define AIC_DIST_END     (AIC_DIST+AIC_DIST_SIZE-1)

/* All the HW IRQ ranges are this big */
/* FIXME: Technically this should be AIC_NUM_IRQ/32 */
/* TODO: Name is awkward */
#define AIC_IRQ_REG_SIZE    0x0080

/* SW initiated HW IRQs */
#define AIC_SW_SET       0x4000
#define AIC_SW_SET_END   (AIC_SW_SET+AIC_IRQ_REG_SIZE-1)
#define AIC_SW_CLEAR     0x4080
#define AIC_SW_CLEAR_END (AIC_SW_CLEAR+AIC_IRQ_REG_SIZE-1)

/* HW IRQ mask set */
#define AIC_MASK_SET     0x4100
#define AIC_MASK_SET_END (AIC_MASK_SET+AIC_IRQ_REG_SIZE-1)

/* HW IRQ mask clear */
#define AIC_MASK_CLEAR      0x4180
#define AIC_MASK_CLEAR_END  (AIC_MASK_CLEAR+AIC_IRQ_REG_SIZE-1)

/* TODO: Understand IPIs better, the linux driver implies features
 * that aren't exposed in the actual driver or the Github wiki page
 * entry, but doesn't use them either so there's no clarity of how
 * the self/other IPI is distinguished when there's only one mask
 * there's also defines for a register out at 0x5000 that seem to
 * let you set IPIs as well? Is that the other IPI functionality?
 */

/* IPI flag set and clear (sends the IPI) */
#define AIC_IPI_FLAG_SET    0x2008
#define AIC_IPI_FLAG_CLEAR  0x200C

/* IPI mask */
#define AIC_IPI_MASK_SET    0x2024
#define AIC_IPI_MASK_CLEAR  0x2028

/* IRQ reasons (reported by reason register) */
#define AIC_IRQ_HW  (1<<0)
#define AIC_IRQ_IPI (1<<2)

/* TODO MMIO for real, each type of IO region should probably be split */
static uint64_t aic_mem_read(void *opaque, hwaddr offset, unsigned size)
{
    AppleAICState *s = APPLE_AIC(opaque);
    switch (offset) {
    case AIC_INFO: /* AIC_INFO */
        return AIC_NUM_IRQ;
    case AIC_CURRENT_CORE: /* AIC_CURRENT_CORE */
        /* TODO: Link the CPUs back here so we can return who we are? */
        return 0;
    case AIC_IRQ_REASON: /* AIC_IRQ_REASON */
        /* Reading this also handles acking an IRQ */
        
        /* TODO: Keep track of reasons such that we can tie them back
         * when this is read, (we need the current CPU doing the read
         * to do this
         */
        return AIC_IRQ_HW;
    case AIC_DIST ... AIC_DIST_END: /* AIC_DIST */
        return s->irq_dist[offset - AIC_DIST];
    case AIC_SW_SET ... AIC_SW_SET_END: /* AIC_SW_SET */
        /* TODO: This would have to get an IRQ's status I think */
        /* or we would have to shadow it */
        return 0;
    case AIC_SW_CLEAR ... AIC_SW_CLEAR_END: /* AIC_SW_CLEAR */
        /* TODO: Same as AIC_SW_SET */
        return 0;
    case AIC_MASK_SET ... AIC_MASK_SET_END: /* AIC_MASK_SET */
        return s->irq_mask[(offset - AIC_MASK_SET)/32];
    case AIC_MASK_CLEAR ... AIC_MASK_CLEAR_END: /* AIC_MASK_CLEAR */
        return s->irq_mask[(offset - AIC_MASK_CLEAR)/32];
    case AIC_IPI_FLAG_SET:
    case AIC_IPI_FLAG_CLEAR:
        return s->ipi_pending;
    case AIC_IPI_MASK_SET: 
    case AIC_IPI_MASK_CLEAR:
        return s->ipi_mask;
    default:
        printf("aic_mem_read: Unhandled read from @%0lx of size=%d\n",
               offset, size);
    }
    return 0;
}

static void aic_mem_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    AppleAICState *s = APPLE_AIC(opaque);
    switch (offset) {
    case AIC_DIST ... AIC_DIST_END: /* AIC_DIST */
        s->irq_dist[offset - 0x3000] = val & 0xFFFFFFFF;
        break;
    case AIC_SW_SET ... AIC_SW_SET_END: /* AIC_SW_SET */
        /* TODO: This would have to raise the IRQ line */
        break;
    case AIC_SW_CLEAR ... AIC_SW_CLEAR_END: /* AIC_SW_CLEAR */
        /* TODO: And this would have to lower it if HW isn't
         * driving it (so we need an OR effectively)
         */
        break;
    case AIC_MASK_SET ... AIC_MASK_SET_END: /* AIC_MASK_SET */
        s->irq_mask[(offset - AIC_MASK_SET)/32] |= val & 0xFFFFFFFF;
        break;
    case AIC_MASK_CLEAR ... AIC_MASK_CLEAR_END: /* AIC_MASK_CLEAR */
        s->irq_mask[(offset - AIC_MASK_CLEAR)/32] &= ~(val & 0xFFFFFFFF);
        break;
    case AIC_IPI_FLAG_SET:
        s->ipi_pending |= (val & 0xFFFFFFFF);
        break;
    case AIC_IPI_FLAG_CLEAR:
        s->ipi_pending &= ~(val & 0xFFFFFFFF);
        break;
    case AIC_IPI_MASK_SET: 
        s->ipi_mask |= (val & 0xFFFFFFFF);
        break;
    case AIC_IPI_MASK_CLEAR:
        s->ipi_mask &= ~(val & 0xFFFFFFFF);
        break;
    default:
        printf("aic_mem_write: Unhandled write of %0lx to @%0lx of size=%d\n",
               val, offset, size);
    }
}

static const MemoryRegionOps aic_io_ops = {
    .read = aic_mem_read,
    .write = aic_mem_write,
    .impl.min_access_size = 4, /* We only want to think about 32 bits */
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8, /* I don't see why not */
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* Handle incoming HW interrupts changes here */
static void aic_irq_handler(void *opaque, int n, int level)
{
    /* TODO: Do this right? */
    AppleAICState *s = APPLE_AIC(opaque);
    /* TODO: handle aborting if n>AIC_NUM_IRQ */
    /* Update pending */
    if (n < AIC_NUM_IRQ) {
        /* The logic below is too confusing, wrap in macros or
         * an inline function (inline functions seem cleaner */
        s->irq_pending[n/32] |= n & 0x1F;
    }
    /* TODO: most things have an update_irq method to process irqs
     * after a change is handled specifically */
}

static void apple_aic_realize(DeviceState *dev, Error **errp)
{
    AppleAICState *s = APPLE_AIC(dev);
    
    /* Clear state */
    /* TODO: /32 is really not ideal, use some sort of BITS wrapper */
    for (int i = 0; i < (AIC_NUM_IRQ/32); i++) {
        /* IRQs are cleared by default */
        s->irq_pending[i] = 0;
        /* Masks are set by default */
        s->irq_mask[i] = ~0;
    }
    
    memory_region_init_io(&s->mmio_region, OBJECT(s), &aic_io_ops, s,
                          "aic-mmio", 0x10000);
    /* Init input HW interrupts */
    qdev_init_gpio_in(dev, aic_irq_handler, AIC_NUM_IRQ);
    
    SysBusDevice *busdev = SYS_BUS_DEVICE(s);
    sysbus_init_mmio(busdev, &s->mmio_region);
    
    /* TODO: Make this based on num cpus */
    for (int i = 0; i < 8; i++) {
        sysbus_init_irq(busdev, &s->irq_out[i]);
    }
}

static void apple_aic_class_init(ObjectClass *oc, void *data) {
    DeviceClass *dc = DEVICE_CLASS(oc);
    
    dc->realize = apple_aic_realize;
}

/* TODO: VM State stuff seems important for interrupt controllers */

static const TypeInfo apple_aic_type_info = {
    .name = TYPE_APPLE_AIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AppleAICState),
    .class_init = apple_aic_class_init,
};

static void apple_aic_register_types(void)
{
    type_register_static(&apple_aic_type_info);
}

type_init(apple_aic_register_types);
