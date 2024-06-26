.thumb
.syntax unified

.section .text
.global  g_pfnVectors
.global Reset_Handler

.equ RCC_AHB1ENR, 0x40023830
.equ GPIOA_MODER, 0x40020000
.equ GPIOA_ODR, 0x40020014
.equ LED_PIN, 5  /* PA5 */

.equ FLASH_BASE_ADDR, 0x40023C00
.equ FLASH_KEYR_OFFSET,    0x04
.equ FLASH_KEY1, 0x45670123
.equ FLASH_KEY2, 0xCDEF89AB
.equ FLASH_CR_OFFSET, 0x10
.equ FLASH_CR_LOCK_BIT, 0x80000000

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

    bl Unlock_Flash
    bl Lock_Flash
    bl Unlock_Flash
    bl Lock_Flash

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
    
Unlock_Flash:
    /* Load the base address of the FLASH registers */
    ldr     r0, =FLASH_BASE_ADDR
    /* Load the address of the FLASH_KEYR register */ 
    add     r0, r0, #FLASH_KEYR_OFFSET
    
    /* Write the first key to the FLASH_KEYR */ 
    LDR     r1, =FLASH_KEY1
    STR     r1, [r0]
    
    /* Write the second key to the FLASH_KEYR */
    LDR     r1, =FLASH_KEY2
    STR     r1, [r0]
    
    /* Return from the function */
    bx      lr

Lock_Flash:
    /* Load the base address of the FLASH registers */
    ldr     r0, =FLASH_BASE_ADDR
    /* Load the address of the FLASH_CR register */
    add     r0, r0, #FLASH_CR_OFFSET
    
    /* Read the current value of the FLASH_CR register */
    ldr     r1, [r0]
    /* Set the LOCK bit */
    orr     r1, r1, #FLASH_CR_LOCK_BIT
    /* Write the modified value back to the FLASH_CR register */
    str     r1, [r0]
    
    /* Return from the function */
    bx      lr
