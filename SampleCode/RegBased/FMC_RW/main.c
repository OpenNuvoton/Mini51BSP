/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/10/06 11:43a $
 * @brief    Show FMC read flash IDs, erase, read, and write functions.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"
#include "uart.h"
#include "fmc.h"

#define APROM_TEST_BASE             0x3000
#define DATA_FLASH_TEST_BASE        0x3000
#define DATA_FLASH_TEST_END         0x4000
#define TEST_PATTERN                0x5A5A5A5A


void SYS_Init(void)
{
    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCON &= ~CLK_PWRCON_XTLCLK_EN_Msk;
    CLK->PWRCON |= (0x1 << CLK_PWRCON_XTLCLK_EN_Pos); // XTAL12M (HXT) Enabled

    /* Waiting for 12MHz clock ready */
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL_STB_Msk));

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_XTAL;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART_EN_Msk; // UART Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART_S_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

}

void UART_Init()
{
    /* Set P0,P1 multi-function pins for UART RXD and TXD  */
    SYS->P0_MFP = 0x303;
    UART->LCR = 0x3;
    UART->BAUD = 0x30000066;
}


int32_t fill_data_pattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        /* FMC_Write(u32Addr, u32Pattern) */
        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADR = u32Addr;
        FMC->ISPDAT = u32Pattern;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
    }
    return 0;
}


int32_t  verify_data(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;
    uint32_t    u32data;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        /* u32data = FMC_Read(u32Addr); */
        FMC->ISPCMD = FMC_ISPCMD_READ;
        FMC->ISPADR = u32Addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
        u32data = FMC->ISPDAT;

        if (u32data != u32Pattern)
        {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32data, u32Pattern);
            return -1;
        }
    }
    return 0;
}


int32_t  flash_test(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);

        /* Erase page -  FMC_Erase(u32Addr); */
        FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
        FMC->ISPADR = u32Addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

        // Verify if page contents are all 0xFFFFFFFF
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }

        // Write test pattern to fill the whole page
        if (fill_data_pattern(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("Failed to write page 0x%x!\n", u32Addr);
            return -1;
        }

        // Verify if page contents are all equal to test pattern
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("\nData verify failed!\n ");
            return -1;
        }

        /* Erase page -  FMC_Erase(u32Addr); */
        FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
        FMC->ISPADR = u32Addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

        // Verify if page contents are all 0xFFFFFFFF
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }
    }
    printf("\r    Flash Test Passed.          \n");
    return 0;
}


int main()
{
    /* Disable register write-protection function */
    SYS->RegLockAddr = 0x59;
    SYS->RegLockAddr = 0x16;
    SYS->RegLockAddr = 0x88;

    SYS_Init();
    UART_Init();

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|         MINI51 FMC Sample Code         |\n");
    printf("+----------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC->ISPCON |=  FMC_ISPCON_ISPEN_Msk;

    /* Read BS */
    printf("  Boot Mode ............................. ");
    if (FMC->ISPCON & FMC_ISPCON_BS_Msk)
    {
        printf("[LDROM]\n");
        printf("  WARNING: The driver sample code must execute in AP mode!\n");
        goto lexit;
    }
    else
        printf("[APROM]\n");

    printf("\n\nLDROM test =>\n");
    FMC->ISPCON |= FMC_ISPCON_LDUEN_Msk;
    if (flash_test(FMC_LDROM_BASE, FMC_LDROM_END, TEST_PATTERN) < 0)
    {
        printf("\n\nLDROM test failed!\n");
        goto lexit;
    }
    FMC->ISPCON &= ~FMC_ISPCON_LDUEN_Msk;

lexit:

    /* Disable FMC ISP function */
    FMC->ISPCON &=  ~FMC_ISPCON_ISPEN_Msk;

    /* Lock protected registers */
    SYS->RegLockAddr = 0;

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
