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
#include "hw/core/cpu.h" /* HACK this is only here for getting current core */
#include "cpu.h"
#include "qemu/timer.h"
#include "hw/intc/apple-aic.h"

/* TODO: Add tracing */

/* TODO: Move register defs to seperate header */
#define AIC_INFO         0x0004
#define AIC_CURRENT_CORE 0x2000
#define AIC_IRQ_REASON   0x2004

/* HW to CPU distribution map */
#define AIC_DIST         0x3000
#define AIC_DIST_SIZE    (AIC_NUM_IRQ*4)
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

/* HW IRQ state */
#define AIC_HW_STATE        0x4200
#define AIC_HW_STATE_END    (AIC_HW_STATE+AIC_IRQ_REG_SIZE-1)

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

/* Timer passthrough */
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

/* TODO: Masks and shifts for IRQ reason */

/* HACK: bad bad bad */
/* Explanation: The AIC as a peripheral has a concept of the
 * current core for accesses there's a "WHOAMI" register that
 * likely reports the value stored in MPIDR, there's a current
 * CPU view of the state (or at least IPIs), then at an offset
 * you can access each specific CPU's view of the registers
 * (at least for IPIs), this is how you send a "self" IPI vs.
 * an "other" IPI. The reason tells you the difference when
 * you get called, but you can trigger a "self" IPI by writing
 * to the self IPI bit using a view into another core's registers
 *
 * The problem is that we don't get passed in a CPU reference
 * when someone accesses the memory. And if access to the
 * peripheral depends on the CPU calling, it's not really on
 * a system bus so much as it is an internal core peripehral
 * with an outside bus interface too. This seems fairly tricky
 * to implement in QEMU, the GICv3 seems to handle this by
 * having a split state, with the GIC initializing per-cpu
 * states that reference their CPU and then registering things
 * like MMIO with a reference to the PER CPU state so we know
 * who's accessing us. Lets do similar but not as hacky, this
 * means letting the board/SoC code provide us with CPU
 * references to use (and their corresponding numbers)
 * TODO: understand if the above solution is good enough
 * TODO: find out how linux maps IPI/dist masks to CPU numbers
 *       might just be based on the info from the device tree
 */
/* HACK update: Slightly improved, but it's based on current_cpu which
 * relies on only ever having ARM_CPU compatible CPUs emulated and also
 * is probably unreliable in general (the same hack is currently used for
 * timers until both issues get fixed)
 */
static uint32_t current_reason[AIC_MAX_CPUS] = {0,};

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
    uint32_t masked = s->irq_out_status & AIC_BIT_MASK(cpu);
    if (level && !masked) {
        /* IRQ just was raised so trigger an output IRQ change */
        s->irq_out_status |= AIC_BIT_MASK(cpu);
        //printf("Raising IRQ for CPU@%d\n", cpu);
        qemu_irq_raise(s->irq_out[cpu]);
    } else if (!level && masked) {
        /* IRQ just was lowered so trigger an output IRQ change */
        s->irq_out_status &= ~AIC_BIT_MASK(cpu);
        qemu_irq_lower(s->irq_out[cpu]);
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
    /* HACK need to store as part of per-cpu data */
    uint32_t result_irqs[AIC_MAX_CPUS] = { 0, };
    
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
                         * for raising the line
                         */
                        result_irqs[k] = irq_base+j;
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
    if (result_lines != 0) {
        //printf("AIC: Computed IRQs after search: %0x\n", result_lines);
    }
    
    /* 
     * Now compute IPIs which are simply the IPI pending after masking
     * TODO: This actually only computes IPIs using the "other" mode
     * where we use the aliased IPI register to poke a bit that indicates
     * to set the IPI on the specific processor, kinda strange to
     * distinguish self vs. other, I wonder how OS X is coded
     */
    uint32_t valid_ipis = s->ipi_pending & ~s->ipi_mask;
    
    /* This can't really be faster since we need to apply every bit */
    for (int i = 0; i < s->num_cpu; i++) {
        /* TODO: order is unknown between IPIs and IRQs but we apply
         * hw IRQs first
         */
        if (result_lines & (1<<i)) {
            /* HACK */
            current_reason[i] = (AIC_IRQ_HW<<16)|result_irqs[i];
            aic_set_irq(s, i, AIC_IRQ_HIGH);
        } else if (valid_ipis & (1<<i)) {
            /* HACK */
            /* Since we don't actually implement self IPI's yet always use
             * "other" as the type
             */
            current_reason[i] = (AIC_IRQ_IPI<<16)|AIC_IPI_TYPE_OTHER;
            //printf("Raising IPI for CPU@%0x\n",i);
            aic_set_irq(s, i, AIC_IRQ_HIGH);
        } else {
            /* Nothing is raising the IRQ line */
            aic_set_irq(s, i, AIC_IRQ_LOW);
        }
    }
}

/* TODO MMIO for real, each type of IO region should probably be split */
static uint64_t aic_mem_read(void *opaque, hwaddr offset, unsigned size)
{
    AppleAICState *s = APPLE_AIC(opaque);
    ARMCPU *cpu = ARM_CPU(current_cpu);
    switch (offset) {
    case AIC_INFO: /* AIC_INFO */
        return AIC_NUM_IRQ;
    case AIC_CURRENT_CORE: /* AIC_CURRENT_CORE */
        /* TODO: Link the CPUs back here so we can return who we are? */
        return 0;
    case AIC_IRQ_REASON: /* AIC_IRQ_REASON */
    {
        /* Reading this also handles acking an IRQ (and masking it) */
        /* TODO: move this to handler it's too big for the switch */
        /* Must have a current reason to decode */
        int cpu_index = cpu->mp_affinity & 0xFF;
        /* TODO clean this mess up */
        if ((cpu->mp_affinity >> 8) == 0x101) {
            /* Firestorm core offset */
            cpu_index += 4;
        }
        uint32_t reason = current_reason[cpu_index];
        if (reason != 0) {
            uint32_t reason_type = reason >> 16;
            uint32_t reason_irq = reason & 0xFFFF;
            //printf("Got reason of type=%0x, irq=%0x\n", reason_type, reason_irq);
            switch (reason_type) {
                case AIC_IRQ_IPI:
                    s->ipi_mask |= (1U)<<cpu_index;
                    break;
                case AIC_IRQ_HW:
                    aic_set_bit(reason_irq, s->irq_mask);
                    break;
            }
        }
        /* TODO: Should we do an AIC update IRQ here since IRQ
         * was acked?*/
        /* NOTE: probably yes, since multiple IRQs could theoretically
         * be queued at once on hardware there could be multiple valid
         * IRQs occuring with unique event codes, reading it again
         * would get the next highest priority event. On QEMU everything
         * is synchronous (at least for now?) such that no code gets 
         * to set IRQs simultaneously with other code (and even if it did)
         * there would probably be a total order? (this is probably
         * nonsense whatever).
         * 
         * The result is it simply probably doesn't matter on qemu except 
         * in the case that there was an existing IRQ, we reenabled IRQs
         * to allow ourselves to be preempted by a higher priority IRQ
         * and then the hardware sends us another IRQ and we want to
         * go handle that. I'm fairly sure Linux might actually do this
         * but we need to understand what the real AIC does in that case
         * to understand what makes sense to implement here. For now
         * we don't support the reason being decoupled
         */
        /*
         * Above comment is old and I didn't read it to change it properly
         * we call IRQ update after this beacuse after a higher priority
         * IRQ is acked (reason is read) the existing IRQ is acked and then
         * the next read will return the next set IRQ if IRQs are still
         * pending. 
         * NOTE: we assume here that hardware dynamically generates reason
         * from the highest priority unmasked IRQ rather than say queuing
         * up incoming IRQ events. The latter makes little sense to me
         * at the moment but it's possible so noting it here.
         */
        current_reason[cpu_index] = 0;
        /* 
         * TODO: make this not sound like nonsense
         * NOTE: update must occur after we clear the reason
         * since it will update it given that we have now
         * masked the IRQ we have just asked by reading the reason
         */
        aic_update_irq(s);
        return reason;
    }
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
    case AIC_IPI_FLAG_SET:      /* AIC_IPI_FLAG_SET */
    case AIC_IPI_FLAG_CLEAR:    /* AIC_IPI_FLAG_CLEAR */
        return s->ipi_pending;
    case AIC_IPI_MASK_SET:      /* AIC_IPI_MASK_SET */
    case AIC_IPI_MASK_CLEAR:    /* AIC_IPI_MASK_CLEAR */
        return s->ipi_mask;
    case AIC_HW_STATE ... AIC_HW_STATE_END: /* State of input HW lines */
        /* Get the current pending value which is what I think this
         * does on hardware
         */
        /* FIXME: bounds checking? */
        //printf("accessing irq_pending[%ld] = %d\n", (offset-AIC_HW_STATE)/4, s->irq_pending[(offset - AIC_HW_STATE)/4]);
        return *aic_offset_ptr(offset, AIC_HW_STATE, s->irq_pending);
    case AIC_TIMER_LOW: /* Lower 32 bits of the CNTPCT_EL0 timer */
        /* TODO: this is bad and broken, we need per-cpu state to do this */
        return aic_emulate_timer(cpu) & 0xFFFFFFFF;
    case AIC_TIMER_HIGH: /* Upper 32 bits of the CNTPCT_EL0 timer */
        /* TODO: this is bad and broken, we need per-cpu state to do this */
        return (aic_emulate_timer(cpu) >> 32) & 0xFFFFFFFF;
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
    case AIC_IPI_FLAG_SET:
        s->ipi_pending |= value;
        // TODO: only handle changes/IPIs
        aic_update_irq(s);
        break;
    case AIC_IPI_FLAG_CLEAR:
        s->ipi_pending &= ~value;
        // TODO: only handle changes/IPIs
        aic_update_irq(s);
        break;
    case AIC_IPI_MASK_SET: 
        s->ipi_mask |= value;
        // TODO: only handle changes/IPIs
        aic_update_irq(s);
        break;
    case AIC_IPI_MASK_CLEAR:
        s->ipi_mask &= ~value;
        // TODO: only handle changes/IPIs
        aic_update_irq(s);
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
    
    /* NOTE: IPIs are capped to at most having 32 CPUs
     * accessed by a single AIC */
    s->ipi_pending = 0;
    s->ipi_mask = ~0;
    
    /* We start will all lines low */
    s->irq_out_status = 0;
    
    memory_region_init_io(&s->mmio_region, OBJECT(s), &aic_io_ops, s,
                          "aic-mmio", AIC_MMIO_SIZE);
    /* Init input HW interrupts */
    qdev_init_gpio_in(dev, aic_irq_handler, s->num_irq);
    
    SysBusDevice *busdev = SYS_BUS_DEVICE(s);
    sysbus_init_mmio(busdev, &s->mmio_region);
    
    for (int i = 0; i < s->num_cpu; i++) {
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
