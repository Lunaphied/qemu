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
 *   + sysbus MMIO regions: 
 *     0 - general AIC_MMIO_SIZE region, connect to sysbus
 * 
 * QOM properties:
 *   + "soc" - Link back to the parent SoC
 * Note that 
 * Intended use is for users of this device to map the general memory region
 * at the base address for the AIC, then overlap map specific CPU interface
 * offsets. The reason we overlap map the other regions on top is that the
 * common mapped region is embedded inside the memory map of the overall
 * AIC and splitting it into multiple regions isn't ideal.
 * 
 * apple-m1.c has an example of this usage.
 * 
 * See the .c file for more info about the MMIO space layout
 */

#ifndef HW_INTC_APPLE_AIC_H
#define HW_INTC_APPLE_AIC_H

#include "hw/sysbus.h"
#include "apple_aic_bitmap.h"

/*
 * The AIC uses a 32-bit field for it's distribution bitmap so only 31 output
 * IRQs at a time can ever exist, furthermore the IPI_FLAG (stored as ipi_pending)
 * reserves the top bit for "self" IPI which triggers this core with an IPI that
 * has a different result from reading the IRQ_REASON register.
 */
#define AIC_MAX_CPUS 31

/* TODO: Find out real address space of AIC */
#define AIC_MMIO_SIZE 0x10000

/* TODO: Find out the real per-cpu address space size */
#define AIC_PER_CPU_MMIO_SIZE 0x4

/* TODO: Make these into max values instead of hardcoded to
 * current limitations (note that AIC_NUM_IRQ can only go up to
 * 1024 before overflowing the MMIO spaces on this model
 */
#define AIC_NUM_IRQ 896
#define AIC_NUM_IPI 2

/* 
 * This is the number of vaild uint32_t entries in the irq map
 */
#define AIC_IRQ_REG_COUNT DIV_ROUND_UP(AIC_NUM_IRQ, 32)


#define TYPE_APPLE_AIC "apple-aic"
OBJECT_DECLARE_SIMPLE_TYPE(AppleAICState, APPLE_AIC)

/*
 * Each core has a copy of it's per-core memory region mapped to a per-core
 * local alias. Unfortunately this (seemingly common) dependency pattern
 * is not easily expressed in QEMU's views, since it requires creating a unique
 * container for each CPU's memory region with the main system bus view mapped
 * in at a lower priority so the per-cpu regions are able to overlap.
 * 
 * This quickly becomes a mess with many cores, and is overall confusing since
 * each core is mostly the same, plus this mapping has to happen externally
 * somewhat, since the container must contain the rest of memory/peripherals
 * too. You can't even make it easy by adding them as sysbus regions because
 * sysbus_mmio_map will map them into the common system bus, which in this case
 * is not what you want. 
 * 
 * Thinking about this too much leads in circles, some peripherals like the 
 * A9GTimer and the ARM MPTimer have the same problem, but they solve it in
 * an unfortunate way that involves assuming that the thread local current_cpu
 * variable is valid, that all the types are ARMCPU compatible, etc.
 * 
 * This is not really supposed to be done, while much of qemu device emulation
 * uses this to get a reference to a cpu index to compute per-cpu offsets for
 * cpu private regions, there's a very clear comment in part of the address
 * handling code that makes a note about potentially trying to invalidate
 * current_cpu before non-ram access is attempted to catch buggy code. (only the
 * write handler has that comment but the point still stands), this means that
 * indeed the correct thing to do is to not punt this to the peripheral but to
 * properly expose each per_cpu region as a seperate mmio region, then let the
 * SoC code handle initializing each core with it's own view of memory.
 * 
 * FIXME before merge
 * The solution for now is that we want things to work so we'll use this hack,
 * however ideally we want code to stop doing that, one potential fix that should
 * work is passing an address space reference for an access. This will be handled
 * in another set of accessor methods that will fallback to the existing code,
 * by using the address space, this idea needs a lot of development and some
 * collaboration from the mailing list and even a partial implementation would still
 * be more work than I think this kind of structure should be. So for now we use
 * the hack.
 */
struct AICPerCoreState {
    /* 
     * This is stored to when interrupts are recalculated
     * which occurs when any of the IRQ config is changed,
     * along with when outside interrupts come in, finally
     * this is also updated by reading the register which
     * is what masks the incoming IRQs/IPIs, IPIs also
     * have to be "ACKed" before they will be deasserted
     */
    /* IRQ type and #, IPI type and source (self/other) */
    uint32_t irq_reason;
    
    /* IPI pending/mask is per-core */
    uint32_t ipi_mask;
    /* 
     * This holds pending IPIs for this core with the bit index
     * representing the core sending the IPI
     */
    uint32_t ipi_pending;
    
    /* Output IRQ line */
    /* XXX do we want to handle FIQ here too? */
    qemu_irq irq;
    
    /* 
     * Keep track of existing state so we only send state transitions
     * This is just used as a bool
     */
    uint32_t irq_status;
};

typedef struct AICPerCoreState AICPerCoreState;

struct AppleAICState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion mmio_mr;
    
    /* Number of HW irq inputs */
    uint32_t num_irq;
    uint32_t num_cpu;
    
    /* 
     * NOTE: These are declared as compiled time constants
     * so they are not actually dynamically resizable yet
     * True bitmaps like QEMU's header uses are not general enough
     * for our uses, this is a difficiency of QEMU's common api bitmap
     * code, which doesn't handle the common case of 32-bit registers
     * packed into a bitmap. Extracting and setting values from set of
     * registers that doesn't match the size of the interface registers
     * is painful and requires more code. Instead we do our own 
     * implementation... again
     */
    /* HW IRQ mask and pending */
    AIC_DECLARE_BITMAP(irq_mask, AIC_NUM_IRQ);
    AIC_DECLARE_BITMAP(irq_pending, AIC_NUM_IRQ);
    
    /* Currently set SW triggered IRQs */
    AIC_DECLARE_BITMAP(sw_pending, AIC_NUM_IRQ);
    
    /* TODO: This might need to be a hashtabe to not take up a huge amount
     * of memory just for IRQ distribution storage? */
    uint32_t irq_dist[AIC_NUM_IRQ];

    /* IPIs and IRQ lines are stored in a per-cpu structure */
    AICPerCoreState core_state[AIC_MAX_CPUS];
};

#endif /* HW_INTC_APPLE_AIC_H */
