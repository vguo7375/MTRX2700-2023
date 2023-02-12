.syntax unified
.thumb

.global assembly_function




.data
@ define variables
recursions: .byte 0x03

.text
@ define text



@ this is the entry function called from the c file
assembly_function:

@	BL recursive_fn
	BL sub_1

forever_loop:
	B forever_loop





sub_1:

	STR LR,[SP,#-4]! @ save the link return address (4 bytes, 32 bits)
	STR IP,[SP,#-4]! @ save the Stack Frame Pointer

	LDR R0, =recursions
	LDRB R1, [R0]
	SUBS R1, #0x01
	STRB R1, [R0]

	BEQ finished_sub_1

	BL sub_1

finished_sub_1:

	LDR IP,[SP],#4 @ restore the Stack Frame Pointer from before the previous call
	LDR PC,[SP],#4 @ restore the link return address (4 bytes, 32 bits)
				   @ note: this replaces the program counter PC with the
				   @ link return from before this function was called





@	STR LR,[SP,#-4]! Save Return Address
@	STR IP,[SP,#-4]! Save Stack Frame Pointer
@	STR R0,[SP,#-4]! Save General Register
@	STR R1,[SP,#-4]! Save General Register
@	MOV IP,SP Set Up New Stack Frame
@	...
@	LDR R1,[SP],#4 Restore General Register
@	LDR R0,[SP],#4 Restore General Register
@	LDR IP,[SP],#4 Restore Stack Frame
@	LDR PC,[SP],#4 Return












recursive_fn:

	LDR R0, =recursions
	LDRB R1, [R0]
	SUBS R1, #0x01
	STRB R1, [R0]

	BEQ finished_recursion

	BL recursive_fn

finished_recursion:

	BX LR
