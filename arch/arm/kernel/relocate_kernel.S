/*
 * relocate_kernel.S - put the kernel image in place to boot
 */

#include <linux/linkage.h>
#include <asm/kexec.h>

	.align	3	/* not needed for this code, but keeps fncpy() happy */

ENTRY(relocate_new_kernel)

	ldr	r0,kexec_indirection_page
	ldr	r1,kexec_start_address

	/*
	 * If there is no indirection page (we are doing crashdumps)
	 * skip any relocation.
	 */
	cmp	r0, #0
	beq	2f

0:	/* top, read another word for the indirection page */
	ldr	r3, [r0],#4

	/* Is it a destination page. Put destination address to r4 */
	tst	r3,#1,0
	beq	1f
	bic	r4,r3,#1
	b	0b
1:
	/* Is it an indirection page */
	tst	r3,#2,0
	beq	1f
	bic	r0,r3,#2
	b	0b
1:

	/* are we done ? */
	tst	r3,#4,0
	beq	1f
	b	2f

1:
	/* is it source ? */
	tst	r3,#8,0
	beq	0b
	bic r3,r3,#8
	mov r6,#1024

9:
	mov r2, r6
10:
	ldmia r3!, {r5-r12}
	stmia r4!, {r5-r12}
	subs r2, r2, #8
	bne 10b
	b 0b

2:
	/* Jump to relocated kernel */
	mov lr,r1
	mov r0,#0
	ldr r1,kexec_mach_type
	ldr r2,kexec_boot_atags
 ARM(	mov pc, lr	)
 THUMB(	bx lr		)

	.align

	.globl kexec_start_address
kexec_start_address:
	.long	0x0

	.globl kexec_indirection_page
kexec_indirection_page:
	.long	0x0

	.globl kexec_mach_type
kexec_mach_type:
	.long	0x0

	/* phy addr of the atags for the new kernel */
	.globl kexec_boot_atags
kexec_boot_atags:
	.long	0x0

ENDPROC(relocate_new_kernel)

relocate_new_kernel_end:

	.globl relocate_new_kernel_size
relocate_new_kernel_size:
	.long relocate_new_kernel_end - relocate_new_kernel


