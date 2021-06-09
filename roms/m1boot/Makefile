all: m1boot.bin

%.bin: %.elf
	llvm-objcopy -j.text -O binary $^ $@

%.elf: %.S
	clang --target=aarch64-linux-gnu -march=armv8.2-a -Wall \
 		-fuse-ld=lld -Wl,-Ttext=0 -nostartfiles -nodefaultlibs \
 		$^ -o $@

clean:
	rm -f *.bin *.elf
