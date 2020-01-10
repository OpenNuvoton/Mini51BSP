/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  2.0.0
 *
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "targetdev.h"
#include "spi_transfer.h"
#include "isp_user.h"

void TIMER0_Init(void)
{
    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_TMR0_EN_Msk;
    /* Select IP clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0_S_Msk)) | CLK_CLKSEL1_TMR0_S_XTAL;
    // Set timer frequency to 3HZ
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 3);
    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    // Start Timer 3
    TIMER_Start(TIMER0);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    while (SYS->RegLockAddr != 1)
    {
        SYS->RegLockAddr = 0x59;
        SYS->RegLockAddr = 0x16;
        SYS->RegLockAddr = 0x88;
    }

    /* Enable internal 22.1184MHz */
    CLK->PWRCON |= (CLK_PWRCON_IRC22M_EN_Msk | CLK_PWRCON_XTLCLK_EN_Msk);

    /* Waiting for clock ready */
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC22M_STB_Msk));

    /* Enable IP clock */
    CLK_EnableModuleClock(SPI_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Setup SPI multi-function pin */
    SYS->P0_MFP |= SYS_MFP_P04_SPISS | SYS_MFP_P05_MOSI | SYS_MFP_P06_MISO | SYS_MFP_P07_SPICLK;

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

int main(void)
{
    uint32_t cmd_buff[16];
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    SPI_Init();
    GPIO_Init();
    TIMER0_Init();

    while (1)
    {
        if (bSpiDataReady == 1)
        {
            goto _ISP;
        }

        if (TIMER0->TISR & TIMER_TISR_TIF_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    while (1)
    {
        if (bSpiDataReady == 1)
        {
            memcpy(cmd_buff, spi_rcvbuf, 64);
            bSpiDataReady = 0;
            ParseCmd((unsigned char *)cmd_buff, 64);
        }
    }

_APROM:
    outpw(&SYS->RSTSRC, 3);//clear bit
    outpw(&FMC->ISPCON, inpw(&FMC->ISPCON) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}
