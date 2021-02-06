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

/* IRQ reasons (reported by reason register) */
#define AIC_IRQ_HW  (1<<0)
#define AIC_IRQ_IPI (1<<2)
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
static uint32_t current_reason;

#define AIC_INVALID_IRQ (-1)

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

/* Handle updating and initiating IRQs after applying masks */
static void aic_update_irq(AppleAICState *s)
{
#if 0
    /* TODO: This way of handling things is common but does
     * other code pass in the IRQ in question?
     */
    
    /* We currently have to check ALL HW irqs that might be pending
     * since all the HW IRQs for a given CPU are OR'd together according
     * to the distributor. There's probably a cleaner interface to do
     * this with more performance than checking every single IRQ but
     * for now that's what we do.
     * 
     * TODO: check the above and use bit manipulation macros
     */
    /* FIXME: stop relying on the compiler to make this fast? */
    /* FIXME: a better way of doing this? depends on num-cpus <= 32 */
    /* Pending mask of all OR'd potential IRQs, we use this to set the
     * IRQs at the end so that we know that no IRQs are raised when
     * setting. A different way of handling incoming IRQs would probably
     * be able to make this not so clunky
     */
    /* FIXME: this is all wrong, reason also contains the IRQ that we are
     * handling, so this whole mess is wrong (it's also SUPER slow) */
    uint32_t hw_irqs = 0;
    /* HACK: Highest priority HW IRQ */
    int hw_irq[AIC_MAX_CPUS];
    for (int i = 0; i < AIC_MAX_CPUS; i++) {
        hw_irq[i] = INT_MAX;
    }
    for (int i = 0; i < s->num_irq/32; i++) {
        //printf("irq reg: %d\n", i);
        /* Compute the masked value */
        uint32_t valid_hw = s->irq_pending[i] & ~(s->irq_mask[i]);
        /* We only have potential to set the IRQ lines high if the IRQs
         * are still raised, since this controller is level triggered
         */
        if (valid_hw) {
            /* Distribute the IRQs */
            for (int j = 0; j < 32; j++) {
                //printf("testing IRQs %d-%d\n", i*32, i*32+31);
                //printf("values set are: %0x\n", valid_hw);
                int irq_num = (i*32)+j;
                //printf("irq_num = %d\n", irq_num);
                if (valid_hw & (1<<j)) {
                    uint32_t valid_cpus = s->irq_dist[irq_num];
                    /* Set the lines for potential CPU irqs for unmasked, raised IRQs */
                    hw_irqs |= valid_cpus;
                    for (int k = 0; k < s->num_cpu; k++) {
                        if (valid_cpus & (1<<k)) {
                            /* Lower numbered interrupts have higher priority */
                            if (irq_num < hw_irq[k]) {
                                hw_irq[k] = irq_num;
                            }
                        }
                    }
                    //printf("Distributing IRQ#%d to CPUs@%0x\n", irq_num, valid_cpus);
                }
            }
        }
    }
    /* HACK: Mask only the HW IRQs being handled */
    for (int i = 0; i < s->num_cpu; i++) {
        if (hw_irqs & (1<<i)) {
            /* Mask the IRQ */
            s->irq_mask[hw_irq[i]/23] |= 1<< (hw_irq[i] & 0x1F);
        }
    }
    uint32_t valid_ipis = s->ipi_pending & ~(s->ipi_mask);
    if (valid_ipis) {
        //printf("valid IPIs were: %0x\n", valid_ipis);
    }
    
    /* TODO: Figure out priority between IPIs and HW IRQs */
    
    /* Update the IRQ lines */
    for (int i = 0; i < s->num_cpu; i++) {
        if (hw_irqs & (1<<i)) {
            printf("Raising IRQ for CPU@%d\n", i);
            /* HACK */
            current_reason = (AIC_IRQ_HW<<16)|hw_irq[i];
            qemu_irq_raise(s->irq_out[i]);
        } else if (valid_ipis & (1<<i)) {
            /* Mask it first */
            s->ipi_mask |= 1<<i;
            /* HACK */
            current_reason = (AIC_IRQ_IPI<<16);
            //printf("Raising IPI for CPU@%0x\n",i);
            qemu_irq_raise(s->irq_out[i]);
        } else {
            /* Nothing is raising the IRQ line */
            qemu_irq_lower(s->irq_out[i]);
        }
    }
#endif
    /* BIG BIG HACK */
    uint32_t is_pending = aic_test_bit(605, s->irq_pending);
    uint32_t mask = aic_test_bit(605, s->irq_mask);
    uint32_t masked = is_pending & ~(mask);
    static int raise_refcount = 0;
    if (masked && (raise_refcount == 0)) {
        raise_refcount++;
        //printf("Raising IRQ %d\n", 605);
        aic_clear_bit(605, s->irq_mask);
        current_reason = (AIC_IRQ_HW<<16)|(605);
        qemu_irq_raise(s->irq_out[0]);
    } else if (!masked && (raise_refcount > 0)) {
        raise_refcount = 0;
        //printf("Lowering IRQ %d\n", 605);
        qemu_irq_lower(s->irq_out[0]);
    }
}

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
    {
        /* Reading this also handles acking an IRQ */
        
        /* TODO: Keep track of reasons such that we can tie them back
         * when this is read, (we need the current CPU doing the read
         * to do this
         */
        uint32_t old_reason = current_reason;
        current_reason = 0;
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
        return old_reason;
    }
    case AIC_DIST ... AIC_DIST_END: /* AIC_DIST */
        return s->irq_dist[offset - AIC_DIST];
    case AIC_SW_SET ... AIC_SW_SET_END: /* AIC_SW_SET */
        return s->sw_pending[(offset-AIC_SW_SET)/4];
    case AIC_SW_CLEAR ... AIC_SW_CLEAR_END: /* AIC_SW_CLEAR */
        return s->sw_pending[(offset-AIC_SW_CLEAR)/4];
    case AIC_MASK_SET ... AIC_MASK_SET_END: /* AIC_MASK_SET */
        return s->irq_mask[(offset-AIC_MASK_SET)/4];
    case AIC_MASK_CLEAR ... AIC_MASK_CLEAR_END: /* AIC_MASK_CLEAR */
        return s->irq_mask[(offset-AIC_MASK_CLEAR)/4];
    case AIC_IPI_FLAG_SET:
    case AIC_IPI_FLAG_CLEAR:
        return s->ipi_pending;
    case AIC_IPI_MASK_SET: 
    case AIC_IPI_MASK_CLEAR:
        return s->ipi_mask;
    case AIC_HW_STATE ... AIC_HW_STATE_END: /* State of input HW lines */
        /* Get the current pending value which is what I think this
         * does on hardware
         */
        /* FIXME: bounds checking? */
        //printf("accessing irq_pending[%ld] = %d\n", (offset-AIC_HW_STATE)/4, s->irq_pending[(offset - AIC_HW_STATE)/4]);
        return s->irq_pending[(offset-AIC_HW_STATE)/4];
        //return *aic_bit_ptr((offset-AIC_HW_STATE)*32, s->irq_pending);
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
    uint32_t value = val & 0xFFFFFFFF;
    switch (offset) {
    case AIC_DIST ... AIC_DIST_END: /* AIC_DIST */
        s->irq_dist[offset - 0x3000] = value;
        break;
    case AIC_SW_SET ... AIC_SW_SET_END: /* AIC_SW_SET */
        s->sw_pending[(offset-AIC_SW_SET)/4] |= value;
        // TODO: proper irq input
        aic_update_irq(s);
        break;
    case AIC_SW_CLEAR ... AIC_SW_CLEAR_END: /* AIC_SW_CLEAR */
        s->sw_pending[(offset-AIC_SW_CLEAR)/4] &= ~value;
        // TODO: proper irq input
        aic_update_irq(s);
        break;
    case AIC_MASK_SET ... AIC_MASK_SET_END: /* AIC_MASK_SET */
        s->irq_mask[(offset-AIC_MASK_SET)/4] |= value;
        //printf("Setting mask IRQS %ld-%ld\n", (offset - AIC_MASK_SET)/4*32,(offset - AIC_MASK_SET)/4*32+31);
        // TODO: only handle changes
        aic_update_irq(s);
        break;
    case AIC_MASK_CLEAR ... AIC_MASK_CLEAR_END: /* AIC_MASK_CLEAR */
        s->irq_mask[(offset-AIC_MASK_CLEAR)/4] &= ~value;
        //printf("Clearing mask IRQS %ld-%ld\n", (offset - AIC_MASK_CLEAR)/4*32,(offset - AIC_MASK_CLEAR)/4*32+31);
        // TODO: only handle changes
        aic_update_irq(s);
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
