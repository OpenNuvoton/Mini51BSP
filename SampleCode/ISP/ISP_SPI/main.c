/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  2.0.0
 *
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "ISP_USER.h"
#include "targetdev.h"

uint32_t Pclk0;
uint32_t Pclk1;

#define TEST_COUNT 16

uint32_t u32DataCount;
uint32_t *_response_buff;
uint32_t spi_rcvbuf[TEST_COUNT];

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

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a slave, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
    /* Set IP clock divider. SPI clock rate = 22MHz */
    SPI_Open(SPI, SPI_SLAVE, SPI_MODE_0, 32, 22000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI, SPI_SS, SPI_SS_ACTIVE_LOW);
}

int main(void)
{
    SYS_Init();
    SPI_Init();
//    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
//    FMC->ISPCON |= (FMC_ISPCON_ISPEN_Msk | FMC_ISPCON_APUEN_Msk);
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

    /* Get APROM size, data flash size and address */
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

_ISP:
    u32DataCount = 0;

    SysTick->CTRL = 0UL;

    /* Check data count */
    while(u32DataCount < TEST_COUNT)
    {
        /* Write to TX register */
        SPI_WRITE_TX(SPI, _response_buff[u32DataCount]);
        /* Trigger SPI data transfer */
        SPI_TRIGGER(SPI);
        /* Check SPI1 busy status */
        while(SPI_IS_BUSY(SPI))
        {
            if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                goto _ISP;
            }
        }

        /* Read RX register */
        spi_rcvbuf[u32DataCount] = SPI_READ_RX(SPI);
        u32DataCount++;

        SysTick->LOAD = 1000 * CyclesPerUs;
        SysTick->VAL  = (0x00);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    }

    /* Disable SysTick counter */
    SysTick->CTRL = 0UL;

    if((u32DataCount == TEST_COUNT) && ((spi_rcvbuf[0] & 0xFFFFFF00) == 0x53504900))
    {
        spi_rcvbuf[0] &= 0x000000FF;
        ParseCmd((unsigned char *)spi_rcvbuf, 64);
    }

    goto _ISP;

}
