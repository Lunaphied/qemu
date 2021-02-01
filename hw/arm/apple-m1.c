#include "qemu/osdep.h"
#include "exec/address-spaces.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties-system.h"
#include "cpu.h"
#include "chardev/char-fe.h"
#include "hw/arm/boot.h"
#include "hw/sysbus.h"
#include "hw/misc/unimp.h"
#include "hw/loader.h"
#include "hw/boards.h"
#include "exec/hwaddr.h"
#include "sysemu/sysemu.h"
#include "exec/exec-all.h"
#include "sysemu/reset.h"
#include "hw/arm/apple-m1-soc.h"
#include "hw/misc/unimp.h"

// Notes on this file:
// - This file doesn't actually represent the M1 very well, it's more a platform designed and focused around m1n1
// - The ARM IRQ pin is directly connected to the UART, which is helpful if you want to test the simplest 
//   interrupts possible
// - The structure of this file is based on the other board files I could find and me making my best guess
//   unfortunately that doesn't get you very far, and there's some not so great stuff here too just to tie
//   it all together to get an interesting demo
// - The "M1" is emulated as if it's an SoC containing multiple Cortex-A72 cores, that's not true in reality,
//   Apple has custom cores that have some special functionality that might be worth emulating, right now I've
//   edited the cortex-a72/etc. definitions for the aarch64 code to have stub Apple registers, along with a
//   non-matching CPU type, this is enough to get through m1n1's writes to these registers although they have
//   no actual functionality. I suspect most of these we can just have do nothing because it looks ilke they
//   are just ways to poke at unemulated microarchitectural state, although m1n1's code does suggest some of
//   them might affect things like interrupts which might actually require emulation
//
// One idea for this is to do what's suggested below and literally pull in a device tree (converted to
// not apple format) then use that to layout all the devices, creating stub regions of unimplemented registers
// until they're created. Unfortunately it seems like nothing else in qemu does this, not even the most modern
// code. So there's no framework, so that seems like it would be it's own project to prototype. 
// So for now we hardcode and make a few M1 helpers

// This is used in several other ARM platforms to help collect the addresses and give them meaningful names
// during realization. These aren't correlated to the M1 hardware at all though it's just easier to set
// them up like this during dev (the UART is the correct address for where it is when m1n1 loads though)
// In reality this should be constructed from a hardware device tree provided from an M1 Mac
static const struct MemMapEntry memmap[] = {
    [BOOT_ARGS] =           {        0x10,     0x10000},
    [VIRT_UART] =           { 0x235200000,     0x10000},
    [VIRT_FB] =             { 0x300000000,       8*GiB},
    [VIRT_MEM] =            { 0x800000000,       8*GiB},
};

// XXX: Hack to make sure we insert the boot args after the device tree
static hwaddr device_tree_end;

static void apple_m1_soc_init(Object *obj) {
    AppleM1SoCState *s = APPLE_M1_SOC(obj);

    // XXX: Change this to use M1 specific core types
    for (int i = 0; i < APPLE_M1_FIRESTORM_CPUS; i++) {
        object_initialize_child(obj, "firestorm[*]", &s->firestorm_cores[i],
                                ARM_CPU_TYPE_NAME("cortex-a72"));
    }
    // XXX: Change this to use M1 specific core types
    for (int i = 0; i < APPLE_M1_ICESTORM_CPUS; i++) {
        object_initialize_child(obj, "icestorm[*]", &s->icestorm_cores[i],
                                ARM_CPU_TYPE_NAME("cortex-a72"));
    }
    object_initialize_child(obj, "framebuffer", &s->fb,
		    		TYPE_APPLE_M1_FB);
}

// Apple M1 reset needs us to put the correct x0 register to point to our boot args
static void handle_m1_reset(void *opaque)
{
    ARMCPU *cpu = ARM_CPU(opaque);
    // TODO: More sanely seperate device_tree_end and boot args
    // FIXME: It's possible this should be replaced by a rom region that does this
    // while executing on the cpu
    
    // INFO: This way of doing things is actually done on several other machines mostly
    // not ARM ones though, but using the reset vector like this is fair, most of them
    // load a bootrom bios into a fixed location, setup the registers to a loaded main binary
    // image (or just let the bootrom work) and then set the PC to the entrypoint
    cpu_reset(CPU(cpu));
    cpu->env.xregs[0] = device_tree_end;
    cpu_set_pc(CPU(cpu), memmap[VIRT_MEM].base+0x4800);
    // FIXME: This seems like a crazy way to initialize this but there's no easy
    // way to do it at the CPU level because CPU reset will reset this to 0.
    // There might be a better way if I look deeper though but this works okay for now
    cpu->env.cp15.hcr_el2 = 0x30488000000;
}   

static void apple_m1_soc_realize(DeviceState *dev, Error **errp)
{
    AppleM1SoCState *s = APPLE_M1_SOC(dev);
    //Object *obj;
    //MemoryRegion *system_mem = get_system_memory();

    // Initialize all the cores
    // TODO: Firestorm core 0 is the lucky one that gets to start powered up right now, however in actual hardware
    //       all cores would have gone into a WFE type loop or setup waiting for an IPI and then WFI'd
    // XXX: Comment above is wrong I think, it seems like actual hardware does a RISC-V like race through the same
    //      code but one of them is specifically chosen and marked as such in the device tree and then the others
    //      know to go into holding patterns? No clue, figure out later
    for (int i = 0; i < APPLE_M1_FIRESTORM_CPUS; i++) {
        // TODO: Somehow set the MP id's properly
        // TODO: Do we need to set CBAR?
        // TODO: How about RBAR? or whatever it is
        // TODO: Interrupt mapping
        // TODO: qdev_prop_set_xyz exists for everything except links so code seems to mix both?
        // Disable CPU 
        object_property_set_bool(OBJECT(&s->firestorm_cores[i]), 
                                 "start-powered-off",
                                 i > 0, &error_abort);
        // Set CPU features
        // TODO: Get rid of this when M1 core type exists and that disables the feature support
	// Moved EL3 disable to core definition
        // object_property_set_bool(OBJECT(&s->firestorm_cores[i]), "has_el3", false, &error_abort);
        object_property_set_bool(OBJECT(&s->firestorm_cores[i]), "has_el2", true, &error_abort);

        qdev_realize(DEVICE(&s->firestorm_cores[i]), NULL, &error_abort);
    }
    printf("HCR EL2: %lx\n", s->firestorm_cores[0].env.cp15.hcr_el2);
    // Now do the icestorm cores
    for (int i = 0; i < APPLE_M1_ICESTORM_CPUS; i++) {
        // TODO: Somehow set the MP id's properly
        // TODO: Do we need to set CBAR?
        // TODO: How about RBAR? or whatever it is
        // TODO: Interrupt mapping
        // TODO: qdev_prop_set_xyz exists for everything except links so code seems to mix both?
        // Disable CPU 
        object_property_set_bool(OBJECT(&s->icestorm_cores[i]), 
                                 "start-powered-off",
                                 true, &error_abort);
        // Set CPU features
        // TODO: Get rid of this when M1 core type exists and that disables the feature support
	// Moved EL3 disable to core definition
        // object_property_set_bool(OBJECT(&s->icestorm_cores[i]), "has_el3", false, &error_abort);
        object_property_set_bool(OBJECT(&s->icestorm_cores[i]), "has_el2", true, &error_abort);

        qdev_realize(DEVICE(&s->icestorm_cores[i]), NULL, &error_abort);
    }

    // Framebuffer init
    // TODO: Set framebuffer memory area size as a property and connect it to the memory map here
    // (This is all temporary until the mapping system is understood enough to have framebuffer be
    // just another part of normal RAM with a default mapping) (which I also don't know how to setup
    // properly in qemu but VGA PCI stuff might be similar)
    AppleM1FBState *fb = &s->fb;
    sysbus_realize(SYS_BUS_DEVICE(fb), &error_abort);
    sysbus_mmio_map(SYS_BUS_DEVICE(fb), 0, memmap[VIRT_FB].base);

    // XXX: This code needs to be made more generic before it could sanely be attached here
    //      also it seems serial_hd(x) is bad practice for getting the chardev, but I can't see
    //      another method that wouldn't require adding a chardev prop to the machine itself
    // XXX: UART is directly connected to the IRQ input lol, this lets us test the interrupt handling easy though
    //      replace this when AIC is supported or if we temporarily hack a GIC on there
    // XXX: This replicates the creation code with some constants that should be exposed in a non-existant .h
    //      we replicated it to allow setting it as a child before relization completes
    DeviceState *uart = qdev_new("exynos4210.uart");
    object_property_add_child(OBJECT(s), "uart", OBJECT(uart));
    qdev_prop_set_chr(uart, "chardev", serial_hd(0));
    qdev_prop_set_uint32(uart, "rx-size", 0x10000); // Give the serial port an insane buffer to speed up code loads
    sysbus_realize_and_unref(SYS_BUS_DEVICE(uart), &error_abort);
    // XXX: Currently the FIFO reset state is changed so that it's ready for FIFO usage before m1n1 starts
    // the correct solution is probably to have m1n1 specifically enable the FIFO or add a pre-m1n1 boot stub
    // to initialize the registers to the values needed by iboot (an iboot stub)
    sysbus_mmio_map(SYS_BUS_DEVICE(uart), 0, memmap[VIRT_UART].base);
    
    // currently the UART is connected directly to firestorm core 0's interrupts... odd.
    // first_cpu is sometimes used but is that guranteed to be anything specific? Until we find out use
    // firstorm_cores[0]
    // FIXME: This was connecting IRQ directly to the uart but that causes annoying issues when m1n1 enables IRQs
    // during startup. For now just disable this. The AIC->m1n1 is already setup with full masking so for real
    // hardware this isn't a concern.
    // XXX: pending AIC
    //sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0, qdev_get_gpio_in(DEVICE(&s->firestorm_cores[0]), ARM_CPU_IRQ));
    // FIXME: Do them all and connect to AIC or whatever
    // Connect a generic ARM timer to FIQ for the interrupts we need (It's HYP because linux uses the hypervisor but I think all the timers should be connected to FIQ for the M1)
    qdev_connect_gpio_out(DEVICE(&s->firestorm_cores[0]), GTIMER_HYP,
		          qdev_get_gpio_in(DEVICE(&s->firestorm_cores[0]), ARM_CPU_FIQ));
    // This almost certainly the watchdog but it's easier to stub it here
    create_unimplemented_device("watchdog?", 0x23B102000, 0x10);
    // This is probably the AIC stub it here for linux writes to not error
    create_unimplemented_device("AIC?", 0x23B103000, 0x10000); // given how close this is to the AIC I bet they're the same peripheral internally
    // exynos4210_uart_create(memmap[VIRT_UART].base, 16, 0, serial_hd(0),
    //                       qdev_get_gpio_in(DEVICE(cpuobj), ARM_CPU_IRQ));

    // FIXME: Do this properly
    // Add a reset hook for the first core that will pull in the correct x0 reg for boot_args
    qemu_register_reset(handle_m1_reset, &s->firestorm_cores[0]);
}

static void apple_m1_soc_class_init(ObjectClass *oc, void *data) { 
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = apple_m1_soc_realize;

    // The M1 SoC is mostly a glued-to-machine component it has no meaning in other contexts
    // additionally our use of serial_hd() apparently is not good according to other comments
    dc->user_creatable = false;
}

static const TypeInfo apple_m1_soc_type_info = {
    .name = TYPE_APPLE_M1_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(AppleM1SoCState),
    .instance_init = apple_m1_soc_init,
    .class_init = apple_m1_soc_class_init,
};

static void apple_m1_soc_register_types(void)
{
    type_register_static(&apple_m1_soc_type_info);
}

type_init(apple_m1_soc_register_types);

static struct arm_boot_info m1_boot_info;


// This builds an apple style device tree then inserts it into memory somewhere, 
// TODO: Make this a lot nicer (model after the virt platform fdt builders)
// it should be dynamic (ideally we would take an existing device tree from
// hardware and then create the other devices using the information so contained
static void create_apple_dt(void) {
    struct node_header {
        uint32_t prop_count;
        uint32_t child_count;
    };
    struct node_property {
        char name[32];
        uint32_t size;
        // after this is a blob of size data (plus padding to align, which is annoying to express)
    };
    // TODO: Put more contents in here and do this correctly
    // FIXME: size this buffer more properly
    uint32_t bufsize = 0x1000;
    //uint8_t *databuf = g_malloc0(bufsize);
#if 0
    // Add initial root header?
    struct node_header header = {0x0, 0x1};
    memcpy(databuf, &header, sizeof(header));
    databuf += sizeof(header);

    // Add chosen header?
    header = {0x0, 0x0};
    memcpy(databuf, &header, sizeof(header));
    databuf += sizeof(header);
    struct node_header chosen = {2, 0};
    struct node_property prop_temp;
    // Add a parent property for the chosen tree?
    snprintf(prop_temp.name, 32, "chosen");
    prop_temp.size = 0;
#endif
    // This was generated using an external tool by taking a dts base, compiling to fdt
    // then converting to adt format. Full ADT support will be needed if we want to boot
    // OS X though so that we can feed it all the nodes (a modified real ADT dump could 
    // be used also potentially)
    uint8_t databuf[] = {0x3, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x6e, 0x61, 0x6d, 0x65, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0xc, 0x0, 0x0, 0x0, 0x64, 0x65, 0x76, 0x69, 0x63, 0x65, 
               0x2d, 0x74, 0x72, 0x65, 0x65, 0x0, 0x6d, 0x6f, 0x64, 0x65, 0x6c, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x11, 0x0, 0x0, 0x0, 0x51, 0x45, 0x4d, 0x55, 0x20, 0x4d, 
               0x31, 0x20, 0x4d, 0x61, 0x63, 0x20, 0x4d, 0x69, 0x6e, 0x69, 0x0, 
               0x0, 0x0, 0x0, 0x74, 0x61, 0x72, 0x67, 0x65, 0x74, 0x2d, 0x74, 
               0x79, 0x70, 0x65, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 
               0x0, 0x0, 0x0, 0x55, 0x6e, 0x6b, 0x6e, 0x6f, 0x77, 0x6e, 0x0, 
               0x5, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6e, 0x61, 0x6d, 0x65, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x7, 0x0, 0x0, 0x0, 0x63, 0x68, 0x6f, 0x73, 0x65, 0x6e, 
               0x0, 0x0, 0x62, 0x6f, 0x61, 0x72, 0x64, 0x2d, 0x69, 0x64, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0x0, 0x0, 
               0x0, 0xad, 0xde, 0x0, 0x0, 0x63, 0x68, 0x69, 0x70, 0x2d, 0x69, 
               0x64, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x4, 0x0, 0x0, 0x0, 0xfe, 0xca, 0x0, 0x0, 0x64, 0x72, 0x61, 
               0x6d, 0x2d, 0x62, 0x61, 0x73, 0x65, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 
               0x0, 0x0, 0x0, 0x0, 0x64, 0x72, 0x61, 0x6d, 0x2d, 0x73, 0x69, 
               0x7a, 0x65, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 
               0x8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0};
    bufsize = sizeof(databuf);
    printf("Mapping device tree at: %lx\n", memmap[BOOT_ARGS].base);
    rom_add_blob_fixed("device-tree", databuf, bufsize, memmap[BOOT_ARGS].base);
    //g_free(databuf);
    // XXX: Make this use the actual size?
    device_tree_end = QEMU_ALIGN_UP(bufsize + memmap[BOOT_ARGS].base, 64);
}

#define BOOTARGS_REVISION 0xDEAD
#define BOOTARGS_FB_WIDTH 800
#define BOOTARGS_FB_HEIGHT 600
#define BOOTARGS_FB_STRIDE (BOOTARGS_FB_WIDTH*4)
#define BOOTARGS_FB_DEPTH 30

// This builds and then inserts a boot args structure like m1n1 and OS X expect
// it then puts it into memory (I've moved it to 0x10 so that it's not set to
// a null pointer)
static void create_boot_args(void) {
    struct {
        uint16_t revision;
        uint16_t pad0;
        uint32_t pad1;
        uint64_t virtual_load_base; // TODO: document if this is a load base or memory base
        uint64_t physical_load_base;
        uint64_t memory_size;
        uint64_t loaded_end;        // End address of loaded binary (safe mem starts here)
        uint64_t fb_addr;           // Framebuffer address in memory
        uint64_t pad2;
        uint64_t fb_stride;         // Stride (bytes per line)
        uint64_t fb_width;
        uint64_t fb_height;
        uint64_t fb_depth;          // TODO: Extract what real hardware puts here
        uint64_t pad3;
        uint64_t device_tree;
        uint32_t device_tree_size;
        char cmdline[608];
        uint64_t flags;
        uint64_t unknown;
    } bootargs;
    bootargs.revision = 0xdead;
    bootargs.virtual_load_base = memmap[VIRT_MEM].base;
    bootargs.physical_load_base = memmap[VIRT_MEM].base;
    bootargs.memory_size = memmap[VIRT_MEM].size;
    bootargs.loaded_end = memmap[VIRT_MEM].base + 0x100000;  // FIXME: I just chose a large value big enough for current m1n1 not safe
    bootargs.fb_addr = memmap[VIRT_FB].base;
    bootargs.fb_stride = BOOTARGS_FB_STRIDE;
    bootargs.fb_width = BOOTARGS_FB_WIDTH;
    bootargs.fb_height = BOOTARGS_FB_HEIGHT;
    bootargs.fb_depth = BOOTARGS_FB_DEPTH;

    // FIXME: There's got to be a better way of doing it than this. Maybe a property or something? Maybe not?
    bootargs.device_tree = memmap[BOOT_ARGS].base;
    bootargs.device_tree_size = device_tree_end - memmap[BOOT_ARGS].base;

    char buf[] ="no cmdline lol";
    memset(bootargs.cmdline, 0, sizeof(bootargs.cmdline));
    memcpy(bootargs.cmdline, buf, sizeof(buf)); // FIXME: This is stupid
    bootargs.cmdline[sizeof(buf)] = 0x0;

    printf("Mapping boot args tree at: %lx\n", device_tree_end);
    rom_add_blob_fixed("boot-args", &bootargs, sizeof(bootargs), device_tree_end);
}

static void apple_m1_init(MachineState *machine)
{
    AppleM1SoCState *m1;

    m1 = APPLE_M1_SOC(qdev_new(TYPE_APPLE_M1_SOC));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(m1));
    //CPU(&m1->maincore)->memory = machine->ram;
    qdev_realize_and_unref(DEVICE(m1), NULL, &error_abort);

    // This initializes the memory region map
    MemoryRegion *sysmem = get_system_memory();

    // doesn't this leak?
    MemoryRegion *boot_args = g_new0(MemoryRegion, 1);
    memory_region_init_rom(boot_args, NULL, "boot-args", memmap[BOOT_ARGS].size, &error_abort);
    memory_region_add_subregion(sysmem, memmap[BOOT_ARGS].base, boot_args);

    // XXX: FB stuff was implemented and moved into soc this isn't needed anymore as far as I know
    // Again this should probably go in the SoC state struct or the machine struct (probably the SoC since it's all)
    // technically part of the M1 itself unlike most things (raspi SoC is again a good reference)
    // MemoryRegion *fb_mem = g_new0(MemoryRegion, 1);
    //memory_region_init_ram(fb_mem, NULL, "framebuffer", memmap[VIRT_FB].size, &error_abort);
    //memory_region_add_subregion(sysmem, memmap[VIRT_FB].base, fb_mem);
    memory_region_add_subregion(sysmem, memmap[VIRT_MEM].base, machine->ram);

    // Add apple flavored device tree
    create_apple_dt();
    // Add boot args
    create_boot_args();

    // XXX: These are unused but we leave temporarily 
    m1_boot_info.loader_start = memmap[VIRT_MEM].base;
    m1_boot_info.ram_size = machine->ram_size;

    /* This is for loading M1N1 which acts as a sort of replacement firmware */
    /* We will want to do some sort of stub firmware loader first before M1N1
     * to setup the registers and boot args it expects, but for now we abuse
     * the reset handler and use ROM regions (we want to replace these with RAM
     * that reloads on CPU reset or something)
     */
    if (machine->firmware) {
        printf("Found firmware: %s\n", machine->firmware);
        if (load_image_targphys(machine->firmware, memmap[VIRT_MEM].base,
                                memmap[VIRT_MEM].size) < 0) {
            error_report("Failed to load firmware from %s", machine->firmware);
            exit(1);
        }
    }


    // TODO: Replace this with a stubbed bootloader that loads the registers peroply
    // and loads a kernel blob into memory and then jumps into that kernel blob
    // we can probably implement enough macho parsing to grab the entry point

    // XXX: This only works with a raw m1n1 elf because of patches to the laod elf that will break other users
    //      should be replaced with a very simple loader for mach-o's or a seperate firmware bootloader
    //      support for the firmware bootloader path would need to be added I think to do this right?
    //      (TODO: look at the raspbery pi code which needs to do something similar)
    // arm_load_kernel(ARM_CPU(first_cpu), machine, &m1_boot_info);
    // If this value isn't at least 0x10000, something is wrong
    //printf("Entry after load: %lx\n", m1_boot_info.entry);
}

static void apple_m1_machine_init(MachineClass *mc)
{
    mc->init = apple_m1_init;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;
    mc->default_ram_id = "ram";
    mc->default_ram_size = 8 * GiB;
    mc->desc = "Apple M1 Mac Mini";
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a57");
    mc->block_default_type = IF_SD;
    mc->min_cpus = 8;
    mc->max_cpus = 8;
    mc->default_cpus = 8;
}

DEFINE_MACHINE("apple-m1", apple_m1_machine_init)
