/*
 * Apple M1 Interrupt Controller
 * 
 * Copyright (c) 2021 Iris Johnson <iris@modwiz.com>
 * 
 * SPDX-License-Identifier: MIT
 */

/*
 * QEMU interface:
 *   + unnamed GPIO inputs: 0..AIC_NUM_IRQ
 *   + sysbus IRQs:
 *     - IRQ for CPU 0
 *     - IRQ for CPU 1
 *     - ...
 *   + sysbus MMIO 0 region: AIC_MMIO_SIZE region
 * 
 * See the .c file for more info about the MMIO space layout
 */

#ifndef HW_INTC_APPLE_AIC_H
#define HW_INTC_APPLE_AIC_H

#include "hw/sysbus.h"

/* TODO: This is a hack */
#define AIC_MAX_CPUS 32
/* TODO: Find out real address space of AIC */
#define AIC_MMIO_SIZE 0x10000

#define AIC_NUM_IRQ 896
#define AIC_NUM_IPI 2

#define TYPE_APPLE_AIC "apple-aic"

OBJECT_DECLARE_SIMPLE_TYPE(AppleAICState, APPLE_AIC)

struct AppleAICState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/ 
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
};

#endif /* HW_INTC_APPLE_AIC_H */
