/*
 * Apple M1 Interrupt Controller
 * 
 * Copyright (c) 2021 Iris Johnson <iris@modwiz.com>
 * 
 * SPDX-License-Identifier: MIT
 */

/*
 * FIXME: Currently CPU 
 * QEMU interface:
 *   + unnamed GPIO inputs: 0..AIC_NUM_IRQ
 *   + sysbus IRQs:
 *     - IRQ for CPU 0
 *     - IRQ for CPU 1
 *     - ...
 *   + sysbus MMIO regions: 
 *     - general AIC_MMIO_SIZE region, handles non-specific data
 * See the .c file for more info about the MMIO space layout
 */

#ifndef HW_INTC_APPLE_AIC_H
#define HW_INTC_APPLE_AIC_H

#include "hw/sysbus.h"

/* TODO: This is a hack */
/* NOTE: Explanation of the hack, we use 32bit chunk based 
 * bitmaps to compactly store masks and pending IRQs for
 * hardware, however QEMU's internal code uses longs but
 * the hardware expect accesses in 4-byte words and working
 * in those units makes things easier. The hardware only
 * supports 32 cpus as a max as a result.
 * 
 * The size of the bitmaps in irq_mask and irq_pending are
 * mostly only that size to match the hardware, 
 */
#define AIC_MAX_CPUS 32

/* TODO: Find out real address space of AIC */
#define AIC_MMIO_SIZE 0x10000
/* TODO: Find out the real per-cpu address space size */
#define AIC_PER_CPU_MMIO_SIZE 0x4

#define AIC_NUM_IRQ 896
#define AIC_NUM_IPI 2

#define TYPE_APPLE_AIC "apple-aic"

OBJECT_DECLARE_SIMPLE_TYPE(AppleAICState, APPLE_AIC)

struct AppleAICState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/ 
    /* FIXME: correct data types */
    /* Number of HW irq inputs */
    int num_irq;
    int num_cpu;
    MemoryRegion mmio_region;
    /* TODO: Make this allocated based on num-cpus */
    qemu_irq irq_out[AIC_MAX_CPUS];
    /* TODO: Does this belong in the AIC struct? also round up 
     * the IRQ number instead of hardcoding */
    /* TODO: Use macros to do this conversion */
    uint32_t irq_mask[AIC_NUM_IRQ/32];
    uint32_t irq_pending[AIC_NUM_IRQ/32];
    /* TODO: Again use proper definitions for these */
    /* FIXME: if this ever needs to be more than 32 cpus this breaks */
    uint32_t ipi_mask;
    uint32_t ipi_pending;
    /* TODO: This might need to be a hashtabe to not take up a huge amount
     * of memory just for IRQ distribution storage? */
    uint32_t irq_dist[AIC_NUM_IRQ];
    /* We need this because IRQs are handled by state change handlers and
     * do not normally store a present value, this captures the incoming
     * IRQs and stores their current vu...wait*/
};

#endif /* HW_INTC_APPLE_AIC_H */
