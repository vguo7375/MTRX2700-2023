.syntax unified
.thumb

.global assembly_function



.equ GPIOA, 0x48000000	@ base register for GPIOA (pa15 is the TIM2, CH1 output)
.equ GPIOx_AFRH, 0x24 @ offset for setting the alternate pin function




.data
@ define variables


.text
@ define text



@ this is the entry function called from the c file
assembly_function:



forever_loop:
	B forever_loop
