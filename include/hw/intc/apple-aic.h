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

/*
 * This is defined because if we ever try to use more than
 * this number of CPUs the AIC code isn't designed to handle
 * it and it won't work (CPU distribution uses 32bit sized
 * bitmaps
 */
#define AIC_MAX_CPUS 32

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
 * Mini local version of the bitmap/bitop functions working in 32-bit
 * values to match the hardware. Other code in QEMU does similar and
 * the way the bitmap API works should probably be changes to handle
 * this problem
 */

/*
 * nbits must be a compile time constant
 */
#define AIC_DECLARE_BITMAP(name, nbits) \
    uint32_t name[DIV_ROUND_UP(nbits, 32)]

#define AIC_BIT_MASK(nr) (1U << ((nr) % 32))
#define AIC_BIT_WORD(nr) ((nr) / 32)

/*
 * Returns a pointer accessing the 32-bit word containing
 * the specified bit
 */
static inline uint32_t* aic_bit_ptr(int nr, uint32_t *addr)
{
    return addr + AIC_BIT_WORD(nr);
}

/*
 * Sets the specified bit
 */
static inline void aic_set_bit(int nr, uint32_t *addr)
{
    uint32_t mask = AIC_BIT_MASK(nr);
    uint32_t *p = aic_bit_ptr(nr, addr);
    
    *p |= mask;
}

/*
 * Clears the specified bit
 */
static inline void aic_clear_bit(int nr, uint32_t *addr)
{
    uint32_t mask = AIC_BIT_MASK(nr);
    uint32_t *p = aic_bit_ptr(nr, addr);
    
    *p &= ~mask;
}

/*
 * Returns the current status of a specified bit
 */
static inline bool aic_test_bit(int nr, uint32_t *addr)
{
    return addr[AIC_BIT_WORD(nr)] & AIC_BIT_MASK(nr);
}

/*
 * Sets the bit to the specified value
 */
static inline void aic_edit_bit(int nr, uint32_t *addr, bool val)
{
    if (val) {
        aic_set_bit(nr, addr);
    } else {
        aic_clear_bit(nr, addr);
    }
}

/*
 * Clears the provided bitmap
 */
static inline void aic_zero_bits(uint32_t *addr, int nr)
{
    for (int i = 0; i < DIV_ROUND_UP(nr, 32); i++) {
        addr[i] = 0;
    }
}


/*
 * Fills the provided bitmap
 */
static inline void aic_fill_bits(uint32_t *addr, int nr)
{
    for (int i = 0; i < DIV_ROUND_UP(nr, 32); i++) {
        addr[i] = ~0;
    }
}

#define TYPE_APPLE_AIC "apple-aic"
OBJECT_DECLARE_SIMPLE_TYPE(AppleAICState, APPLE_AIC)


struct AppleAICState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    /* TODO: Split this into one main dist region and many per-cpu
     * version
     */
    MemoryRegion mmio_region;
    /* Number of HW irq inputs */
    uint32_t num_irq;
    uint32_t num_cpu;
    /* TODO: Make this allocated based on num-cpus */
    qemu_irq irq_out[AIC_MAX_CPUS];
    
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
    /* Mask and pending for IPIs */
    uint32_t ipi_mask;
    uint32_t ipi_pending;
    
    /* TODO: This might need to be a hashtabe to not take up a huge amount
     * of memory just for IRQ distribution storage? */
    uint32_t irq_dist[AIC_NUM_IRQ];
};

#endif /* HW_INTC_APPLE_AIC_H */
