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
 * 0x2008            - IPI send
 * 0x200C            - IPI ack
 * 0x2024            - IPI mask set
 * 0x2028            - IPI mask clear
 * 
 * 0x5000            - per CPU view of 0x2004
 * Right now for development I've placed the "views" into other CPU's versions
 * of these registers at 0x5000, so far I'm just putting the reason
 * registers there. The normal reason will be handled by aliases that
 * redirect that region to the per-cpu region. This is going to need to
 * be more complete in the future and use the real offsets but for now
 * this will be enough.
 * TODO: above is not actually done, right now we just use a hack
 * since for single core the interrupt executer will always be called
 * as the same CPU. We *cannot* get the CPU from qemu (at least in my
 * opinion) as that will create a situation where we assume the current
 * CPU is always an ARM cpu or compattible, which isn't ideal for a peripherial
 * instead it will just error if a CPU hasn't been hooked up properly.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
/* HACK this is only here for getting current core */
#include "hw/core/cpu.h" 
#include "cpu.h"
#include "qemu/timer.h"
#include "hw/intc/apple-aic.h"
#include "trace.h"

/* TODO: Add tracing */

/* TODO: Move register defs to seperate header */
#define AIC_INFO         0x0004

/* HW to CPU distribution map */
#define AIC_DIST         0x3000
#define AIC_DIST_SIZE    (AIC_NUM_IRQ*4)
#define AIC_DIST_END     (AIC_DIST+AIC_DIST_SIZE-1)

/* Packed IRQ bitmaps have this number of bytes */
#define AIC_IRQ_REG_SIZE 0x80

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

/* HW IRQ state */
#define AIC_HW_STATE        0x4200
#define AIC_HW_STATE_END    (AIC_HW_STATE+AIC_IRQ_REG_SIZE-1)

/* Per-core cpu region stuff */
#define AIC_CURRENT_CORE 0x2000
#define AIC_IRQ_REASON   0x2004

/* IPI send and ack are almost like set/clear for IPI but not quite */
#define AIC_IPI_SEND    0x2008
#define AIC_IPI_ACK     0x200c

/* IPI mask */
#define AIC_IPI_MASK_SET    0x2024
#define AIC_IPI_MASK_CLEAR  0x2028

/* Timer passthrough (TODO this probably isn't actually passthrough) */
#define AIC_TIMER_LOW       0x8020
#define AIC_TIMER_HIGH      0x8028

/* IRQ event types (reported by reason register) */
/* TODO: Rename to include event */
#define AIC_IRQ_HW  (1<<0)
#define AIC_IRQ_IPI (1<<2)

/* IPI types (used in-place of IRQ # when reason is read) */
/* TODO rename as part of above */
#define AIC_IPI_TYPE_OTHER 1
#define AIC_IPI_TYPE_SELF  2

/* IPI bit positions */
/* TODO Rename */
#define AIC_IPI_BIT_OTHER   (1<<0U)
#define AIC_IPI_BIT_SELF    (1<<31U)

/* TODO: Masks and shifts for IRQ reason */

/* Helpers to avoid mystery constants */
#define AIC_INVALID_IRQ (-1)
#define AIC_IRQ_HIGH    1
#define AIC_IRQ_LOW     0

/* TODO: prefix these with is_* */
/*
 * Reports if the current HW IRQ line number is a valid number
 * Takes a state argument to support eventual use of a property
 * for the IRQ number argument
 */
static inline bool aic_irq_valid(AppleAICState *s, int n)
{
    /* IRQs go from 0..num_irq-1 */
    if ((n < 0) || (n >= s->num_irq)) {
        return false;
    }
    return true;
}

/*
 * TODO: this seems to be the only possible way to get the 
 * CPU timer (even with a reference!) we just get the 
 * freq and use the clock to emulate 
 */
static inline uint64_t aic_emulate_timer(ARMCPU *cpu)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / gt_cntfrq_period_ns(cpu);
}

/* 
 * Handles raising/lowering an IRQ output only if the value changed
 * this might make tracing annoying though
 */
static inline void aic_set_irq(AppleAICState *s, int cpu, int level)
{
    if (cpu > AIC_MAX_CPUS) {
        return;
    }
    /* TODO Assert that level is 1 or 0? */
    
    /* Determine if level has changed and update with new value */
    int last_level = s->core_state[cpu].irq_status;
    int changed = last_level != level;
    s->core_state[cpu].irq_status = level;
    
    /* If the value has changed we propegate to the IRQ handlers */
    if (changed) {
        /* TODO: Add tracing */
        qemu_set_irq(s->core_state[cpu].irq, level);
    }
}
/* 
 * Forward declare, later we should probably just swap
 * the order TODO
 */
static void aic_update_irq(AppleAICState *s);

/* Handle updating and apply a single 32-bit register of IRQ */
static void aic_update_irq_reg(AppleAICState *s, int reg)
{
    /* Must be valid, TODO maybe should assert? */
    if (reg > AIC_IRQ_REG_COUNT) {
        return;
    }
    /*
     * Right now without a conception of what else might be
     * driving the IRQ line, we cannot do much meaningful
     * processing simply handling it in a per-reg update.
     * 
     * Theoretically we could set IRQ lines and only do
     * updates for cleared ones, but that means this only
     * does anything meaningful for fully set IRQ state
     * registers, which is highly unlikely to say the least.
     * 
     * Having an idea of if other interrupts are driving
     * the IRQ line for this CPU would change that but that
     * could get messy, we would keep a counter for each output
     * IRQ line (one per CPU then) and increment it when
     * an IRQ is raised on that line, and decrement it when
     * an IRQ stops asserting. However that gets complex fast
     * since we have to make sure to update it after masking
     * and overall there's a potential for error. Instead lets
     * just hope iterating is fast enough
     */
    aic_update_irq(s);
}

/* Handle updating and initiating IRQs after applying masks */
static void aic_update_irq(AppleAICState *s)
{
    /* 
     * NOTE: This code is like this because it's a slightly more
     * optimal approach visually than literally testing every bit
     * through the accessor functions. However it might actually
     * be less optimal because it might confuse the compiler more
     */
    /* 
     * Holds the value of the lines for each output IRQ line after
     * processing all the potential pending IRQs, if it's different
     * than the last state of that line, we raise or lower that line
     */
    uint32_t result_lines = 0;
    
    /* Update the IRQ lines */
    for (int i = 0; i < AIC_IRQ_REG_COUNT; i++) {
        /* Both HW and SW triggered HW are OR'd into one */
        uint32_t pending = s->irq_pending[i] | s->sw_pending[i];
        /* Apply the mask */
        pending &= ~s->irq_mask[i];
        if (pending) {
            //printf("Found unmasked pending: %0x\n", pending);
            /* Base number for IRQs stored in this register */
            int irq_base = i * 32;
            /* Starting from the first bit set, process all pending
             * IRQs and check their masks, no processing is done if
             * zero so ctz32 reports the number of trailing
             * zeros so first set bit is 1<<ctz32(x)
             */
            int first_set = ctz32(pending);
            /*
             * Yes this tests first_set a second time but it's much
             * cleaner than duplicating the code.
             */
            for (int j = first_set; j < 32; j++) {
                /* Only process actually set IRQs */
                if (!(pending & (1<<j))) {
                    continue;
                }
                uint32_t dist = s->irq_dist[irq_base+j];
                //printf("Distribution for IRQ %0x is %0x\n", irq_base+j, dist);
                /* 
                 * Only process the lines if some of them might be unset 
                 * (this clears all the values already set in result_lines)
                 * Already set lines alerady have a higher priority (lower #)
                 * IRQ waiting to be raised until after all lines are processed
                 * TODO: should we just raise immediately?
                 */
                dist &= ~result_lines;
                /*
                 * Now that we have masked the previously set lines
                 * we can just OR in the new ones and process the
                 * new lines to set the active IRQ numbers
                 */
                result_lines |= dist;
                /* Try to distribute the first set interrupt */
                if (dist) {
                    for (int k = ctz32(dist); k < 32; k++) {
                        /* 
                         * We only accelerate which bit we start
                         * with, have to check the rest of the bits
                         * too
                         */
                        if (!(dist & (1<<k))) {
                            continue;
                        }
                        /* Store the IRQ number that is responsible
                         * for raising the line, just store directly
                         * in reason.
                         */
                        s->core_state[k].irq_reason = irq_base+j;
                    }
                }
            }
        }
        /* 
         * TODO: If all the lines are set then we don't need to iterate
         * anymore (but this is only useful if we can ignore the lines
         * not being used by CPUs)
         */
    }
    /* TODO Remove for merge */
    if (result_lines != 0) {
        //printf("AIC: Computed IRQs after search: %0x\n", result_lines);
    }
    
    /* This can't really be faster since we need to apply every bit */
    for (int i = 0; i < s->num_cpu; i++) {
        /* TODO: order is unknown between IPIs and IRQs but we apply
         * hw IRQs first
         */
        if (result_lines & (1<<i)) {
            /* Add in the IRQ_HW type since we know it's hardware now */
            s->core_state[i].irq_reason |= AIC_IRQ_HW<<16;
            aic_set_irq(s, i, AIC_IRQ_HIGH);
        } else {
            /* 
            * IPIs are computed here since IPIs can obviously be sent
            * to each CPU individually, the only two bits that matter are
            * the top bit (bit 31) which is for "OTHER" type IPIs and the
            * bottom bit (bit 0) which is used for "SELF" type IPIs.
            * 
            * NOTE: we assume HW IRQs preempt IPIs at the moment.
            * an important aspect of this is that writes to trigger other
            * set a bit representing the CPU index to trigger, but the
            * only state that results is a single "OTHER" bit being set
            * meaning that only one OTHER IPI can be in flight at a time.
            * 
            * At least as we implement it. It's possible the hardware records
            * senders and keeps the other bit set until an ACK for each
            * trigger is sent but that seems rather unlikely and isn't
            * a terribly hard change should it be true.
            */
            uint32_t valid_ipis = s->core_state[i].ipi_pending & 
                                  ~s->core_state[i].ipi_mask;
//             if ((i == 0) && s->core_state[i].ipi_pending) {
//                 qemu_log("Valid IPIs were 0x%x\n", valid_ipis);
//                 qemu_log("Pending IPIs were 0x%x\n", s->core_state[i].ipi_pending);
//                 qemu_log("Masked  IPIs were 0x%x\n", s->core_state[i].ipi_mask);
//             }
            if (valid_ipis) {
                /* This sets the type and clears the HW IRQ */
                s->core_state[i].irq_reason = (AIC_IRQ_IPI<<16);
                /* Right now other IPIs pre-empt self IPIs */
                if (valid_ipis & AIC_IPI_BIT_OTHER) {
                    s->core_state[i].irq_reason |= AIC_IPI_TYPE_OTHER;
                    //qemu_log("Sent IPI type Other to 0x%x\n", i);
                } else if (valid_ipis & AIC_IPI_BIT_SELF) {
                    s->core_state[i].irq_reason |= AIC_IPI_TYPE_SELF;
                    //qemu_log("Sent IPI type Self to 0x%x\n", i);
                } else {
                    /* TODO: Replace this with proper assert */
                    error_report("Invalid IPI set 0x%x", valid_ipis);
                    abort();
                }
                /* Now we have to raise the IRQ line for this processor */
                aic_set_irq(s, i, AIC_IRQ_HIGH);
            }  else {
                /* TODO: move this out of nesting */
                /* Nothing is raising the IRQ line */
                /* Clear reason */
                s->core_state[i].irq_reason = 0;
                /* Set the line low */
                aic_set_irq(s, i, AIC_IRQ_LOW);
            }
        }
    }
}

/* 
 * This is marked as read because that's what it handles, but it
 * really processes an "update" since reads modify state
 */
static uint32_t aic_read_reason(AppleAICState *s, int cpu_index) {
    /* TODO Clean up */
    if (cpu_index > AIC_MAX_CPUS) {
        error_report("CPU calling us had an index of 0x%x which was too high",
                     cpu_index);
        abort();
    }
    
    /* Get the current reason */
    uint32_t reason = s->core_state[cpu_index].irq_reason;
    
    if (!reason) {
        /* 
         * If nothing is pending this is unspecified but just return 0
         * and skip processing.
         * 
         * (NOTE linux driver actually seems to assume that returning 0
         * is a valid way to indiciate no more IRQs pending so maybe it's
         * not unspecified)
         */
        return 0;
    }
    
    /* TODO: Use defines for this */
    /*
     * Now we have to mask the reason since the IRQ handler has now processed
     * this IRQ event and must tell the AIC it's ready to process another
     */
    uint32_t reason_type = reason >> 16;
    uint32_t reason_irq = reason & 0xFFFF;
    if (reason_type == AIC_IRQ_HW) {
        /* Mask the provided IRQ number */
        aic_set_bit(reason_irq, s->irq_mask);
    } else {
        /* Mask the specified IPI (both types are local masks) */
        if (reason_irq == AIC_IPI_TYPE_OTHER) {
            s->core_state[cpu_index].ipi_mask |= AIC_IPI_BIT_OTHER;
        } else if (reason_irq == AIC_IPI_TYPE_SELF) {
            s->core_state[cpu_index].ipi_mask |= AIC_IPI_BIT_SELF;
        } else {
            /* TODO: Real assert */
            error_report("Invalid IPI reason (check aic_update_irq): 0x%x",
                         reason_irq);
            abort();
        }
    }
    /* Trigger an update in case a new IRQ is triggered here (updates reason) */
    aic_update_irq(s);
    
    /* Return the original reason, next pass will get new reason */
    return reason;
}

/* 
 * Handles converting the incoming IPI bit to a set bit in the 
 * per CPU structure
 * We pass in caller CPU index because self IPIs use that
 */
static void aic_send_ipi(AppleAICState *s, int cpu, uint32_t value) {
    /* Loop through looking for set IPI bits */
    for (int i = ctz32(value); i < AIC_MAX_CPUS; i++) {
        if (value & (1<<i)) {
            /* This will set a pending other IPI */
            s->core_state[i].ipi_pending |= AIC_IPI_BIT_OTHER;
            //qemu_log("IPI triggered from cpu 0x%x to 0x%d\n", cpu, i);
        }
    }
    if (value & AIC_IPI_BIT_SELF) {
        /* This sets a pending self IPI */
        s->core_state[cpu].ipi_pending |= AIC_IPI_BIT_SELF;
        //qemu_log("SELF IPI triggered from cpu 0x%x\n", cpu);
    }
    //qemu_log("Input IPI value was 0x%x\n", value);
    //qemu_log("Self bit was 0x%x\n", AIC_IPI_BIT_SELF);
    //qemu_log("Other bit was 0x%x\n", AIC_IPI_BIT_OTHER);
    
    /* 
     * TODO is there any performance gains we can get by not scanning the
     * whole set every time we update IPIs or a single IRQ?
     */
    /* Handle the update locally */
    aic_update_irq(s);
}

/* 
 * FIXME This whole concept is broken by design, we need to use aliases
 * and per-cpu memory regions that handle the dispatch for us properly
 * for now we want things to work so use this (other code uses this too)
 * and for us this is the cleanest structure without overcomplicating the
 * SoC side of things.
 * 
 * This method should be only used in the read/write mem methods since
 * everything else should have this detail abstracted out as a parameter
 * for when the proper code is written.
 */
static CPUState* get_current_cpu(void) {
    /* FIXME this is broken by design */
    CPUState *cpu = CPU(current_cpu);
    return cpu;
}

/* TODO MMIO for real, each type of IO region should probably be split */
static uint64_t aic_mem_read(void *opaque, hwaddr offset, unsigned size)
{
    AppleAICState *s = APPLE_AIC(opaque);
    /* FIXME: read method FIXME */
    CPUState *cpu = get_current_cpu();
    /* 
     * FIXME also broken since this depends on initialization order and
     * isn't set specifically, would be nice to know preferred patterns
     * (i.e. should this be set manually then used? Is it internal?
     * who knows!)
     */
    int cpu_index = cpu->cpu_index;
    switch (offset) {
    case AIC_INFO: /* AIC_INFO */
        return AIC_NUM_IRQ;
    case AIC_CURRENT_CORE: /* AIC_CURRENT_CORE */
        /* TODO: Where is this value defined? */
        /* HACK */
        return cpu_index;
    case AIC_IRQ_REASON: /* AIC_IRQ_REASON */
        return aic_read_reason(s, cpu_index);
    case AIC_DIST ... AIC_DIST_END: /* AIC_DIST */
        return s->irq_dist[offset - AIC_DIST];
    case AIC_SW_SET ... AIC_SW_SET_END: /* AIC_SW_SET */
        return *aic_offset_ptr(offset, AIC_SW_SET, s->sw_pending);
    case AIC_SW_CLEAR ... AIC_SW_CLEAR_END: /* AIC_SW_CLEAR */
        return *aic_offset_ptr(offset, AIC_SW_CLEAR, s->sw_pending);
    case AIC_MASK_SET ... AIC_MASK_SET_END: /* AIC_MASK_SET */
        return *aic_offset_ptr(offset, AIC_MASK_SET, s->irq_mask);
    case AIC_MASK_CLEAR ... AIC_MASK_CLEAR_END: /* AIC_MASK_CLEAR */
        return *aic_offset_ptr(offset, AIC_MASK_CLEAR, s->irq_mask);
    case AIC_IPI_SEND:      /* AIC_IPI_FLAG_SET */
    case AIC_IPI_ACK:       /* AIC_IPI_FLAG_CLEAR */
        /* TODO: know if reading ACK actually returns this */
        return s->core_state[cpu_index].ipi_pending;
    case AIC_IPI_MASK_SET:      /* AIC_IPI_MASK_SET */
    case AIC_IPI_MASK_CLEAR:    /* AIC_IPI_MASK_CLEAR */
        return s->core_state[cpu_index].ipi_mask;
    case AIC_HW_STATE ... AIC_HW_STATE_END: /* State of input HW lines */
        /* Get the current pending value which is what I think this
         * does on hardware
         */
        /* FIXME: bounds checking? */
        //printf("accessing irq_pending[%ld] = %d\n", (offset-AIC_HW_STATE)/4, s->irq_pending[(offset - AIC_HW_STATE)/4]);
        return *aic_offset_ptr(offset, AIC_HW_STATE, s->irq_pending);
    case AIC_TIMER_LOW: /* Lower 32 bits of the CNTPCT_EL0 timer */
        /* TODO: this is bad and broken, we need per-cpu state to do this */
        return aic_emulate_timer(ARM_CPU(cpu)) & 0xFFFFFFFF;
    case AIC_TIMER_HIGH: /* Upper 32 bits of the CNTPCT_EL0 timer */
        /* TODO: this is bad and broken, we need per-cpu state to do this */
        return (aic_emulate_timer(ARM_CPU(cpu)) >> 32) & 0xFFFFFFFF;
    default:
        printf("aic_mem_read: Unhandled read from @%0" HWADDR_PRIx " of "
               "size=%d\n", offset, size);
    }
    return 0;
}

static void aic_mem_write(void *opaque, hwaddr offset, uint64_t val,
                         unsigned size)
{
    AppleAICState *s = APPLE_AIC(opaque);
    /* FIXME: read method FIXME */
    CPUState *cpu = get_current_cpu();
    /* 
     * FIXME also broken since this depends on initialization order and
     * isn't set specifically, would be nice to know preferred patterns
     * (i.e. should this be set manually then used? Is it internal?
     * who knows!)
     */
    int cpu_index = cpu->cpu_index;
    
    uint32_t value = val;
    bool changed = false;
    switch (offset) {
    case AIC_DIST ... AIC_DIST_END: /* AIC_DIST */
        //printf("Writing IRQ dist %u\n", aic_offset_to_reg(offset, AIC_DIST));
        aic_update_reg(offset, AIC_DIST, s->irq_dist, value);
        /* TODO: should this call aic_update_irq? */
        break;
    case AIC_SW_SET ... AIC_SW_SET_END: /* AIC_SW_SET */
        changed = aic_set_reg(offset, AIC_SW_SET, s->sw_pending, value);
        if (changed) {
            aic_update_irq_reg(s, aic_offset_to_reg(offset, AIC_SW_SET));
        }
        break;
    case AIC_SW_CLEAR ... AIC_SW_CLEAR_END: /* AIC_SW_CLEAR */
        changed = aic_clear_reg(offset, AIC_SW_CLEAR, s->sw_pending, value);
        if (changed) {
            aic_update_irq_reg(s, aic_offset_to_reg(offset, AIC_SW_CLEAR));
        }
        break;
    case AIC_MASK_SET ... AIC_MASK_SET_END: /* AIC_MASK_SET */
        changed = aic_set_reg(offset, AIC_MASK_SET, s->irq_mask, value);
        if (changed) {
            aic_update_irq_reg(s, aic_offset_to_reg(offset, AIC_MASK_SET));
        }
        break;
    case AIC_MASK_CLEAR ... AIC_MASK_CLEAR_END: /* AIC_MASK_CLEAR */
        changed = aic_clear_reg(offset, AIC_MASK_CLEAR, s->irq_mask, value);
        if (changed) {
            aic_update_irq_reg(s, aic_offset_to_reg(offset, AIC_MASK_CLEAR));
        }
        break;
    case AIC_IRQ_REASON:
        /* Ignore writes */
        break;
    case AIC_IPI_SEND:
        aic_send_ipi(s, cpu_index, value);
        break;
    case AIC_IPI_ACK:
        s->core_state[cpu_index].ipi_pending &= ~value;
        // TODO: only handle changes/IPIs
        aic_update_irq(s);
        break;
    case AIC_IPI_MASK_SET: 
        s->core_state[cpu_index].ipi_mask |= value;
        // TODO: only handle changes/IPIs
        aic_update_irq(s);
        break;
    case AIC_IPI_MASK_CLEAR:
        s->core_state[cpu_index].ipi_mask &= ~value;
        // TODO: only handle changes/IPIs
        aic_update_irq(s);
        break;
    default:
        printf("aic_mem_write: Unhandled write of %0" PRIx64
               " to @%0" HWADDR_PRIx " of size=%d\n", val, offset, size);
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
    if (aic_irq_valid(s, n)) {
        /* The logic below is too confusing, wrap in macros or
         * an inline function (inline functions seem cleaner */
        if (level) {
            aic_set_bit(n, s->irq_pending);
            //printf("irq_pending[%d] |= %d\n", n/32, n & 0x1F);
        } else {
            aic_clear_bit(n, s->irq_pending);
            //printf("irq_pending[%d] &= ~(%0x)\n", n/32, 1<<(n & 0x1F));
        }
    }
    aic_update_irq(s);
}

static void apple_aic_realize(DeviceState *dev, Error **errp)
{
    AppleAICState *s = APPLE_AIC(dev);
    
    /* Initialize properties */
    /* TODO: make these top level properties instead */
    s->num_irq = AIC_NUM_IRQ;
    s->num_cpu = AIC_MAX_CPUS;
    
    /* Clear state */
    /* NOTE: These use compile time constants to get better
     * generated code even if it is less generic
     */
    aic_zero_bits(s->sw_pending, AIC_NUM_IRQ);
    aic_zero_bits(s->irq_pending, AIC_NUM_IRQ);
    aic_fill_bits(s->irq_mask, AIC_NUM_IRQ);
    
    /* Use memset to clear all the per-CPU data */
    /* TODO: Allocate dynamically? */
    memset(&s->core_state, 0, sizeof(s->core_state));
    for (int i = 0; i < s->num_cpu; i++) {
        s->core_state[i].ipi_mask = ~0U;
    }
    
    memory_region_init_io(&s->mmio_mr, OBJECT(s), &aic_io_ops, s, "aic-mmio",
                          AIC_MMIO_SIZE);
    
    /* Init input HW interrupts */
    qdev_init_gpio_in(dev, aic_irq_handler, s->num_irq);
    
    SysBusDevice *busdev = SYS_BUS_DEVICE(s);
    sysbus_init_mmio(busdev, &s->mmio_mr);
    
    for (int i = 0; i < s->num_cpu; i++) {
        sysbus_init_irq(busdev, &s->core_state[i].irq);
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
