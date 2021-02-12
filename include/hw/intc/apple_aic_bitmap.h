/*
 * Apple M1 Interrupt Controller bitmap helpers
 * 
 * Copyright (c) 2021 Iris Johnson <iris@modwiz.com>
 * 
 * SPDX-License-Identifier: MIT
 */

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
 * Converts an offset value in the MMIO to an index in the
 * array
 */
static inline uint32_t aic_offset_to_reg(hwaddr offset, hwaddr base)
{
    return (offset-base)/4;
}

/*
 * Returns a pointer accessing the 32-bit word containing
 * the specified bit
 */
static inline uint32_t* aic_bit_ptr(int nr, uint32_t *addr)
{
    return addr + AIC_BIT_WORD(nr);
}

/*
 * Returns a pointer accessing the 32-bit word containing
 * the specified register, this is intended as a helper function
 * for the reader/writer MMIO handlers
 */
static inline uint32_t* aic_offset_ptr(hwaddr offset, hwaddr base,
                                       uint32_t *addr)
{
    return addr + aic_offset_to_reg(offset, base);
}

/*
 * Helper to update a register, returns true of there was a change
 * in value, this is helpful since it tells the accessers when to
 * update the IRQ status
 */
static inline bool aic_update_reg(hwaddr offset, hwaddr base, uint32_t *addr,
                                  uint32_t value)
{
    uint32_t *p = aic_offset_ptr(offset, base, addr);
    if (*p != value) {
        *p = value;
        return true;
    }
    return false;
}

/*
 * Helper to update a register by setting bits, returns true if a change
 * occured
 */
static inline bool aic_set_reg(hwaddr offset, hwaddr base, uint32_t *addr,
                               uint32_t value)
{
    uint32_t *p = aic_offset_ptr(offset, base, addr);
    uint32_t old_value = *p;
    uint32_t new_value = old_value | value;
    if (old_value != new_value) {
        *p = new_value;
        return true;
    }
    return false;
}

/*
 * Helper to update a register by setting bits, returns true if a change
 * occured
 */
static inline bool aic_clear_reg(hwaddr offset, hwaddr base, uint32_t *addr,
                               uint32_t value)
{
    uint32_t *p = aic_offset_ptr(offset, base, addr);
    uint32_t old_value = *p;
    uint32_t new_value = old_value & ~value;
    if (old_value != new_value) {
        *p = new_value;
        return true;
    }
    return false;
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
