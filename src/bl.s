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
.equ FLASH_KEYR_ADDR, 0x40023C04    /* Offset 0x04 */
.equ FLASH_CR_ADDR, 0x40023C10      /* Offset 0x10 */
.equ FLASH_SR_ADDR, 0x40023C0C      /* Offset 0x0C */

.equ FLASH_KEY1, 0x45670123
.equ FLASH_KEY2, 0xCDEF89AB

.equ FLASH_CR_LOCK, 0x80000000
.equ FLASH_CR_STRT, 0x00010000
.equ FLASH_CR_SER, 0x00000002
.equ FLASH_CR_PG, 0x01

.equ FLASH_SR_BSY, 0x00010000

.equ FLASH_CR_PSIZE_MASK, 0x00000300
.equ FLASH_CR_PSIZE_32, 0x200

.equ BOOT_SECTOR_ADDRESS, 0x8004000
.equ UPDATE_SECTOR_ADDRESS, 0x8040000
.equ UPDATE_SECTOR_SIZE, 0x3E800

/* For continuing updates after power loss */
.equ UPDATE_FLAG_ADDRESS, 0x08003FF0  /* Address to store the update flag, if need to continue update */
.equ UPDATE_SECTOR_ADDRESS_FLAG, 0x08003FF4  /* Address to store the current update sector address */
.equ BOOT_SECTOR_ADDRESS_FLAG, 0x08003FF8  /* Address to store the current boot sector address */
.equ UPDATE_PROGRESS_FLAG, 0x08003FFC  /* Address to store how much data has been copied so far */

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

    /* Check if continue update flag is set */
    ldr r0, =UPDATE_FLAG_ADDRESS
    ldr r1, [r0]
    cmp r1, #0xFFFFFFFF
    bne Continue_Update

    /* If continue update flag not set: erase boot */
    bl Unlock_Flash

    bl Erase_Boot

After_Erase: 

    /* set PSIZE before programming flash */
    ldr     r0, =FLASH_CR_ADDR
    ldr     r1, [r0]
    bic r1, r1, #FLASH_CR_PSIZE_MASK    /* Clear previous PSIZE bits */
    orr r1, r1, #FLASH_CR_PSIZE_32      /* Set PSIZE to 32-bit */
    str r1, [r0]

    bl Set_Initial_Update_Progress

    bl Lock_Flash

After_Update:

    /* Clear update flag */
    ldr r0, =UPDATE_FLAG_ADDRESS
    mov r1, #0xFFFFFFFF
    ldr r2, =FLASH_CR_ADDR /* Input: r0 = address, r1 = value */

    /* Clear update progress flag */
    ldr r0, =UPDATE_PROGRESS_FLAG
    mov r1, #0xFFFFFFFF
    ldr r2, =FLASH_CR_ADDR /* Input: r0 = address, r1 = value */


    /* Jump to application: set MSP to app's vector table */
    /* commenting this part out should have LD2 (green right side LED) stay on */
    ldr r13, =0x08004000     /* boot MSP */
    ldr r0, =0x080048a5      /* reset handler address (verified in gdb disas)*/  
    bx r0     /* boot reset handler */

    // If jump failed:
    // this will turn on the green LED, indicating the app wasn't jumped to

    /* Enable GPIOA clock */
    ldr r0, =RCC_AHB1ENR  /* Load RCC_AHB1ENR address into r0 */
    ldr r1, [r0]           /* Read RCC_AHB1ENR value into r1 */
    mov r2, #1
    orr r1, r1, r2  /* Enable GPIOA clock (bit 0) */
    str r1, [r0]           /* Write back to address in r0 which is of RCC_AHB1ENR */

    /* Configure PA5 as output */
    ldr r0, =GPIOA_MODER   /* Load GPIOA_MODER address into r0 */
    ldr r1, [r0]           /* Read GPIOA_MODER value into r1 */
    /* we want to OR (1 << LED_PIN*2) with MODER*/
    mov r2, #1
    lsl r2, r2, #10 
    orr r1, r1, r2         /* Set GPIO pin based on LED_PIN */
    str r1, [r0]           /* Write back to GPIOA_MODER */

    /* turn on LED */
    ldr r0, =GPIOA_ODR     /* Load GPIOA_ODR address */
    mov r1, #(1 << LED_PIN) /* Prepare bit mask for PA5 */

    /* r0 contains the address GPIOA's ODR, which controls pin states*/
    str r1, [r0]           /* Turn on LED (set PA5) */

/*
* Flash unlocking and locking
*/
    
Unlock_Flash:
    /* Load the address of the FLASH_KEYR register */ 
    ldr     r0, =FLASH_KEYR_ADDR
    
    /* Write the first key to the FLASH_KEYR */ 
    LDR     r1, =FLASH_KEY1
    STR     r1, [r0]
    
    /* Write the second key to the FLASH_KEYR */
    LDR     r1, =FLASH_KEY2
    STR     r1, [r0]
    
    /* Return from the function */
    bx      lr

Lock_Flash:
    /* Load the address of the FLASH_CR register */
    ldr     r0, =FLASH_CR_ADDR
    
    /* Read the current value of the FLASH_CR register */
    ldr     r1, [r0]
    /* Set the LOCK bit */
    orr     r1, r1, #FLASH_CR_LOCK
    /* Write the modified value back to the FLASH_CR register */
    str     r1, [r0]
    
    /* Return from the function */
    bx      lr

/*
* Erasing the old image
*/

Erase_Boot:
    /* erase Sectors 1, 2, 3, 4, 5 */
    mov r0, 0x8     /* Sector 1 (aligned to bits 6:3) */
    bl Erase_Sector
    mov r0, 0x10     /* Sector 2 (aligned to bits 6:3) */
    bl Erase_Sector
    mov r0, 0x18     /* Sector 3 (aligned to bits 6:3) */
    bl Erase_Sector
    mov r0, 0x20     /* Sector 4 (aligned to bits 6:3) */
    bl Erase_Sector
    mov r0, 0x28     /* Sector 5 (aligned to bits 6:3) */
    bl Erase_Sector
    bl After_Erase

Erase_Sector:
    /* Input: r0 = sector number */
    /* Load the address of the FLASH_CR register */
    ldr r1, =FLASH_CR_ADDR

    /* Set the SER bit and the sector number in FLASH_CR */
    ldr r2, [r1]
    orr r2, r2, #FLASH_CR_SER /* set SER bit */
    bic r2, r2, #(0xF << 3)  /* Clear sector number bits */
    orr r2, r2, r0   /* Set sector number */
    str r2, [r1]
    
    /* Set the START bit */
    orr r2, r2, #FLASH_CR_STRT
    str r2, [r1]

Wait_Busy_Flash:
    /* Wait for the BSY bit to be cleared in FLASH_SR */
    ldr r1, =FLASH_SR_ADDR
    ldr r2, [r1]            /* Read the value of FLASH_SR */
    tst r2, #FLASH_SR_BSY   /* Test the BSY bit in FLASH_SR */
    bne Wait_Busy_Flash
    bx lr

/*
* Writing the update to boot sector
*/

Continue_Update:
    /* set PSIZE before programming flash */
    ldr r0, =FLASH_CR_ADDR
    ldr r1, [r0]
    bic r1, r1, #FLASH_CR_PSIZE_MASK    /* Clear previous PSIZE bits */
    orr r1, r1, #FLASH_CR_PSIZE_32      /* Set PSIZE to 32-bit */
    str r1, [r0]

   /* Check if update progress is stored */
    ldr r0, =UPDATE_PROGRESS_FLAG
    ldr r2, [r0]
    cmp r2, #0xFFFFFFFF
    beq Set_Initial_Update_Progress

    /* Resume from the stored progress */
    ldr r3, =UPDATE_SECTOR_ADDRESS
    add r3, r3, r2
    ldr r4, =BOOT_SECTOR_ADDRESS
    add r4, r4, r2
    ldr r5, =UPDATE_SECTOR_SIZE
    sub r5, r5, r2
    b Check_and_Set_Update_Progress   

Set_Initial_Update_Progress:
    ldr r3, =UPDATE_SECTOR_ADDRESS
    ldr r4, =BOOT_SECTOR_ADDRESS
    ldr r5, =UPDATE_SECTOR_SIZE
    mov r2, #0

Check_and_Set_Update_Progress:
    cmp r5, #0      /* If update sector size copied */
    beq After_Update

    /* Store the current update progress */
    ldr r0, =UPDATE_PROGRESS_FLAG
    str r2, [r0]

    ldr r1, [r3]    /* Read value of update sector address */

Wait_Busy_Copy:
    /* Check BSY bit in FLASH_SR */
    ldr r0, =FLASH_SR_ADDR
    ldr r6, [r0]
    tst r6, #FLASH_SR_BSY
    bne Wait_Busy_Copy

Write_To_Flash:
    /* Set PG bit in FLASH_CR */
    ldr r0, =FLASH_CR_ADDR
    ldr r6, [r0]
    orr r6, r6, #FLASH_CR_PG
    str r6, [r0]

    /* Write the word to the boot sector */
    /* r1 is the value at current update sector address */
    str r1, [r4]

    /* Loop */
    add r3, r3, #4
    add r4, r4, #4
    add r2, r2, #4
    sub r5, r5, #4
    b Check_and_Set_Update_Progress

