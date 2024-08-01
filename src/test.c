#include <stdint.h>
#include <stdio.h>

#define FLASH_KEYR_ADDR    ((volatile uint32_t*) 0x40023C04)
#define FLASH_CR_ADDR      ((volatile uint32_t*) 0x40023C10)
#define FLASH_CR_LOCK      0x80000000
#define BOOT_SECTOR_END      ((volatile uint32_t*) 0x0800BFFF)  // 128KB Flash, 64KB Bootloader

extern void Unlock_Flash(void);
extern void Lock_Flash(void);
extern void Erase_Boot(void);

void test_Unlock_Flash() 
{

    Unlock_Flash();

    uint32_t flash_cr = *FLASH_CR_ADDR;
    if ((flash_cr & FLASH_CR_LOCK) == 0) 
    {
        printf("Unlock_Flash: PASS\n");
    } else 
    {
        printf("Unlock_Flash: FAIL\n");
    }
}

void test_Lock_Flash() 
{
    Lock_Flash();

    uint32_t flash_cr = *FLASH_CR_ADDR;
    if ((flash_cr & FLASH_CR_LOCK) != 0) 
    {
        printf("Lock_Flash: PASS\n");
    } else 
    {
        printf("Lock_Flash: FAIL\n");
    }
}

void test_Erase_Boot() {
    Erase_Boot();

    volatile uint32_t* address;
    for (address = BOOT_SECTOR_START; address <= BOOT_SECTOR_END; address++) 
    {
        if (*address != 0xFFFFFFFF) 
        {
            printf("Erase_Boot: FAIL\n");
            return;
        }
    }
    printf("Erase_Boot: PASS\n");
}

int main() 
{

    test_Unlock_Flash();
    test_Lock_Flash();

    return 0;
}
