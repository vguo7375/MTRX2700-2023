.syntax unified
.thumb

.global assembly_function


@ base register for resetting and clock settings
.equ RCC, 0x40021000
.equ AHBENR, 0x14	@ register for enabling clocks
.equ APB1ENR, 0x1C
.equ APB2ENR, 0x18
.equ AFRH, 0x24
.equ RCC_CR, 0x00 @ control clock register
.equ RCC_CFGR, 0x04 @ configure clock register

@ register addresses and offsets for UART4
.equ UART4, 0x40004C00 @ from peripheral register memory boundary in the big manual
.equ USART_CR1, 0x00
.equ USART_BRR, 0x0C
.equ USART_ISR, 0x1C @ UART status register offset
.equ USART_ICR, 0x20 @ UART clear flags for errors
.equ UART4EN, 19  @ specific bit to enable UART4
.equ UART_TE, 3	@ transmit enable bit
.equ UART_RE, 2	@ receive enable bit
.equ UART_UE, 0	@ enable bit for the whole UART
.equ UART_ORE, 3 @ Overrun flag
.equ UART_ORECF, 3 @ Overrun clear flag


@ uart4 is on GPIOC
.equ GPIOA, 0x48000000	@ base register for GPIOA (pa0 is the button)
.equ GPIOC, 0x48000800	@ base register for GPIOA (pa0 is the button)
.equ GPIOE, 0x48001000	@ base register for GPIOE (pe8-15 are the LEDs)

.equ GPIO_MODER, 0x00	@ set the mode for the GPIO
.equ GPIO_OSPEEDR, 0x08	@ set the speed for the GPIO

@ transmitting data
.equ UART_TXE, 7	@ a new byte is ready to read
.equ USART_TDR, 0x28	@ a new byte is ready to read

.equ UART_RXNE, 5	@ a new byte is ready to read
.equ USART_RDR, 0x24	@ a new byte is ready to read
.equ USART_RQR, 0x18
.equ UART_RXFRQ, 3	@ a new byte is ready to read

@ setting the clock speed higher using the PLL clock option
.equ HSEBYP, 18	@ bypass the external clock
.equ HSEON, 16 @ set to use the external clock
.equ HSERDY, 17 @ wait for this to indicate HSE is ready
.equ PLLON, 24 @ set the PLL clock source
.equ PLLRDY, 25 @ wait for this to indicate PLL is ready
.equ PLLEN, 16 @ enable the PLL clock
.equ PLLSRC, 16
.equ USBPRE, 22 @ with PLL active, this must be set for the USB

.data
@ define variables


.align
incoming_buffer: .byte 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
incoming_counter: .byte 10



.text
@ define text


@ this is the entry function called from the c file
assembly_function:

@ run the functions to perform the config of the ports
	BL initialise_power
	BL change_clock_speed
	BL enable_peripheral_clocks
	BL enable_uart4

@ initialise the buffer and counter
	LDR R6, =incoming_buffer
	LDR R7, =incoming_counter
	MOV R8, #0x00

@ continue reading forever (NOTE: eventually it will run out of memory as we don't have a big buffer
loop_forever:
	LDR R1, [R0, USART_ISR] @ load the status of the UART4

	TST R1, 1 << UART_ORE  @ 'AND' the current status with the bit mask that we are interested in
						   @ NOTE, the ANDS is used so that if the result is '0' the z register flag is set

	BNE clear_ORE

	TST R1, 1 << UART_RXNE @ 'AND' the current status with the bit mask that we are interested in
							  @ NOTE, the ANDS is used so that if the result is '0' the z register flag is set

	BEQ loop_forever @ loop back to check status again if the flag indicates there is no byte waiting

	LDRB R3, [R0, USART_RDR] @ load the lowest byte (RDR bits [0:7] for an 8 bit read)
	STR R3, [R6, R8]
	ADD R8, #1

	LDR R1, [R0, USART_RQR] @ load the status of the UART4
	ORR R1, 1 << UART_RXFRQ
	STR R1, [R0, USART_RQR]

	BGT loop_forever



clear_ORE:

	LDR R1, [R0, USART_ICR] @ load the status of the UART4
	ORR R1, 1 << UART_ORECF	@ Clear the overrun flag (see page 897)
	STR R1, [R0, USART_ICR] @ load the status of the UART4
	B loop_forever



@ function to enable the clocks for the peripherals we are using (A, C and E)
enable_peripheral_clocks:
	LDR R0, =RCC  @ load the address of the RCC address boundary (for enabling the IO clock)
	LDR R1, [R0, #AHBENR]  @ load the current value of the peripheral clock registers
	ORR R1, 1 << 21 | 1 << 19 | 1 << 17  @ 21st bit is enable GPIOE clock, 19 is GPIOC, 17 is GPIOA clock
	STR R1, [R0, #AHBENR]  @ store the modified register back to the submodule
	BX LR @ return

@ function to enable the UART4 - this requires setting the alternate functions for the UART4 pins
@ BAUD rate needs to change depending on whether it is 8MHz (external clock) or 24MHz (our PLL setting)
enable_uart4:
	LDR R0, =GPIOC
	MOV R1, 0x55	@ set the alternate function for the UART4 pins (PC10 and PC11)
	STRB R1, [R0, AFRH + 1]

	LDR R1, =0x00A00000 @ Mask for pins PC10 and PC11 to use the alternate function
	STR R1, [R0, GPIO_MODER]

	LDR R1, =0x00F00000 @ Set the speed for PC10 and PC11 to use high speed
	STR R1, [R0, GPIO_OSPEEDR]

	@ UART4EN is bit number 19, we need to turn the clock on for this
	LDR R0, =RCC @ the base address for the register to turn clocks on/off
	LDR R1, [R0, #APB1ENR] @ load the original value from the enable register
	ORR R1, 1 << UART4EN  @ apply the bit mask to the previous values of the enable UART4
	STR R1, [R0, #APB1ENR] @ store the modified enable register values back to RCC

	@ this is the baud rate
	MOV R1, #0xD0 @ from our earlier calculations (for 24MHz), store this in register R1
@	MOV R1, #0x46 @ from our earlier calculations (for 8MHz), store this in register R1
	LDR R0, =UART4 @ the base address for the register to turn clocks on/off
	STRH R1, [R0, #USART_BRR] @ store this value directly in the first half word (16 bits) of
							  	 @ the baud rate register

	@ we want to set a few things here, lets define their bit positions to make it more readable
	LDR R0, =UART4 @ the base address for the register to set up UART4
	LDR R1, [R0, #USART_CR1] @ load the original value from the enable register
	ORR R1, 1 << UART_TE | 1 << UART_RE | 1 << UART_UE @ make a bit mask with a '1' for the bits to enable,
													   @ apply the bit mask to the previous values of the enable register

	STR R1, [R0, #USART_CR1] @ store the modified enable register values back to RCC

	BX LR @ return

















@ set the PLL (clocks are described in page 125 of the large manual)
change_clock_speed:
@ step 1, set clock to HSE (the external clock)
	@ enable HSE (and wait for complete)
	LDR R0, =RCC @ the base address for the register to turn clocks on/off
	LDR R1, [R0, #RCC_CR] @ load the original value from the enable register
	LDR R2, =1 << HSEBYP | 1 << HSEON @ make a bit mask with a '1' in the 0th bit position
	ORR R1, R2 @ apply the bit mask to the previous values of the enable register
	STR R1, [R0, #RCC_CR] @ store the modified enable register values back to RCC

wait_for_HSERDY:
	LDR R1, [R0, #RCC_CR] @ load the original value from the enable register
	TST R1, 1 << HSERDY @ Test the HSERDY bit (check if it is 1)
	BEQ wait_for_HSERDY

@ step 2, now the clock is HSE, we are allowed to switch to PLL
	@ clock is set to External clock (external crystal) - 8MHz, can enable the PLL now
	LDR R1, [R0, #RCC_CFGR] @ load the original value from the enable register
	LDR R2, =1 << 20 | 1 << PLLSRC
	ORR R1, R2  @ set PLLSRC (use PLL) and PLLMUL to 0100 - bit 20 is 1 (set speed as 6x faster)
				@ see page 140 of the large manual for options
				@ NOTE: cannot go faster than 72MHz)
	STR R1, [R0, #RCC_CFGR] @ store the modified enable register values back to RCC

	@ enable PLL (and wait for complete)
	LDR R0, =RCC @ the base address for the register to turn clocks on/off
	LDR R1, [R0, #RCC_CR] @ load the original value from the enable register
	ORR R1, 1 << PLLON @ apply the bit mask to turn on the PLL
	STR R1, [R0, #RCC_CR] @ store the modified enable register values back to RCC

wait_for_PLLRDY:
	LDR R1, [R0, #RCC_CR] @ load the original value from the enable register
	TST R1, 1 << PLLRDY @ Test the HSERDY bit (check if it is 1)
	BEQ wait_for_PLLRDY

@ step 3, PLL is ready, switch over the system clock to PLL
	LDR R0, =RCC  @ load the address of the RCC address boundary (for enabling the IO clock)
	LDR R1, [R0, #RCC_CFGR]  @ load the current value of the peripheral clock registers
	MOV R2, 1 << 10 | 1 << 1  @ some more settings - bit 1 (SW = 10)  - PLL set as system clock
									   @ bit 10 (HCLK=100) divided by 2 (clock is faster, need to prescale for peripherals)
	ORR R1, R2	@ Set the values of these two clocks (turn them on)
	STR R1, [R0, #RCC_CFGR]  @ store the modified register back to the submodule

	LDR R1, [R0, #RCC_CFGR]  @ load the current value of the peripheral clock registers
	ORR R1, 1 << USBPRE	@ Set the USB prescaler (when PLL is on for the USB)
	STR R1, [R0, #RCC_CFGR]  @ store the modified register back to the submodule

	BX LR @ return


@ initialise the power systems on the microcontroller
@ PWREN (enable power to the clock), SYSCFGEN system clock enable
.equ PWREN, 28
.equ SYSCFGEN, 0
initialise_power:

	@ enable clock power
	LDR R0, =RCC @ the base address for the register to turn clocks on/off
	LDR R1, [R0, #APB1ENR] @ load the original value from the enable register
	ORR R1, 1 << PWREN @ apply the bit mask for power enable
	STR R1, [R0, #APB1ENR] @ store the modified enable register values back to RCC

	@ enable clock config
	LDR R1, [R0, #APB2ENR] @ load the original value from the enable register
	ORR R1, 1 << SYSCFGEN @ apply the bit mask to allow clock configuration
	STR R1, [R0, #APB2ENR] @ store the modified enable register values back to RCC
	BX LR @ return
