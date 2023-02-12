.syntax unified
.thumb

.global assembly_function


.data
@ define variables
byte_array: .byte 0, 1, 2, 3, 4, 5, 6

.text
@ define code



@ this is the entry function called from the c file
assembly_function:

	LDR R0, =0x1234
	LDR R1, =0x0001

	LDR R2, =byte_array
	LDR R4, =0x00

forever_loop:

	LDRB R3, [R2, R4]

	ADD R0, R1
	ADD R4, 0x01


	B forever_loop
