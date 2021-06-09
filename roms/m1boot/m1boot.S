.text
.globl _start
_start:
	// Set up HCR_EL2
	ldr x0, =0x30488000000
	msr hcr_el2, x0
	
	// Load arg and jump to entry
	ldr x0, arg
	ldr x1, entry
	br x1
.pool
arg:
	.dword 0 // Replaced at runtime with the value to load into x0
entry:
	.dword 0 // Replaced at runtime with the entry point address
