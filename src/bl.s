.thumb
.syntax unified

.section .text
.global  g_pfnVectors
.global Reset_Handler

.equ RCC_AHB1ENR, 0x40023830
.equ GPIOA_MODER, 0x40020000
.equ GPIOA_ODR, 0x40020014
.equ LED_PIN, 5  /* PA5 */

.section  .isr_vector,"a",%progbits
.type  g_pfnVectors, %object
.size  g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word  _estack
  .word  Reset_Handler

.section .text.Reset_Handler
.weak Reset_Handler
.type Reset_Handler, %function
Reset_Handler:

    /* jump to application: set MSP to app's vector table */
    /* commenting this part out should have LD2 (green right side LED) stay on */
    ldr r13, =0x08004000     /* boot MSP */
    ldr r0, =0x080048a5      /* reset handler address (verified in gdb disas)*/  
    bx r0     /* boot reset handler */

    /* Enable GPIOA clock */
    ldr r0, =RCC_AHB1ENR  /* Load RCC_AHB1ENR address into r0 */
    ldr r1, [r0]           /* Read RCC_AHB1ENR value into r1 */
    mov r2, #1
    orr r1, r1, r2  /* Enable GPIOA clock (bit 0) */
    str r1, [r0]           /* Write back to address in r0 which is of RCC_AHB1ENR */

    /* Configure PA5 as output */
    ldr r0, =GPIOA_MODER   /* Load GPIOA_MODER address into r0 */
    ldr r1, [r0]           /* Read GPIOA_MODER value into r1 */
    /* we want to OR (1 << LED_PIN*2) with MODER[*/
    mov r2, #1
    lsl r2, r2, #10 
    orr r1, r1, r2         /* Set GPIO pin based on LED_PIN */
    str r1, [r0]           /* Write back to GPIOA_MODER */

    /* Blink LED */
    ldr r0, =GPIOA_ODR     /* Load GPIOA_ODR address */
    mov r1, #(1 << LED_PIN) /* Prepare bit mask for PA5 */

    /* r0 contains the address GPIOA's ODR, which controls pin states*/
    str r1, [r0]           /* Turn on LED (set PA5) */
    