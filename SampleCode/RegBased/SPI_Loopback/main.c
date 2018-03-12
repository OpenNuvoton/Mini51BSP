/******************************************************************************
 * @file     main.c
 * @version  V0.10
 * $Revision: 8 $
 * $Date: 15/10/06 11:44a $
 * @brief    Demonstrate SPI function by connect MOSI (P0.5) with MISO (P0.6).
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

#define TEST_COUNT  64

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];

void SYS_Init(void);
void UART_Init(void);
void SPI_Init(void);
void SpiLoopbackTest(void);

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                       SPI Driver Sample Code                         |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf(" Please Connect:\n");
    printf(" P0.5 MOSI (Pin 34)<--> P0.6 MISO (Pin 33)\n");

    SpiLoopbackTest();

    return 0;
}

void SYS_Init(void)
{
    int32_t i32TimeOutCnt;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    while(SYS->RegLockAddr != SYS_RegLockAddr_RegUnLock_Msk)
    {
        SYS->RegLockAddr = 0x59;
        SYS->RegLockAddr = 0x16;
        SYS->RegLockAddr = 0x88;
    }

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK->PWRCON |= CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    i32TimeOutCnt = __HSI / 200; /* About 5ms */
    while((CLK->CLKSTATUS & (CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk)) !=
            (CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk))
    {
        if(i32TimeOutCnt-- <= 0)
            break;
    }

    /* Switch HCLK clock source to XTL, STCLK to XTL */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_XTAL | CLK_CLKSEL0_HCLK_S_XTAL;

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_XTAL;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP = SYS_MFP_P01_RXD | SYS_MFP_P00_TXD;

    /* Setup SPI multi-function pin */
    SYS->P0_MFP |= SYS_MFP_P04_SPISS | SYS_MFP_P05_MOSI | SYS_MFP_P06_MISO | SYS_MFP_P07_SPICLK;

    /* Lock protected registers */
    SYS->RegLockAddr = 0;

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART_RST_Msk;

    /* Configure UART and set UART Baudrate */
    UART->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk | (((__XTAL + (115200/2)) / 115200)-2);
    UART->LCR = 0x3 | (0x0 << UART_LCR_PBE_Pos) | (0x0 << UART_LCR_NSB_Pos) ;
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    CLK->APBCLK |= CLK_APBCLK_SPI_EN_Msk;
    SPI->CNTRL = SPI_MASTER | SPI_MODE_0;
    SPI->DIVIDER = (((12000000 / 2000000) + 1) >> 1) - 1;

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI->SSR |= (SPI_SS | SPI_SS_ACTIVE_LOW) | SPI_SSR_AUTOSS_Msk;
}

void SpiLoopbackTest(void)
{
    uint32_t u32DataCount, u32TestCount, u32Err;

    printf("\nSPI Loopback test ");

    /* Clear Tx register of SPI to avoid send non-zero data to Master. Just for safe. */
    SPI->TX =  0;

    u32Err = 0;
    for(u32TestCount=0; u32TestCount<100; u32TestCount++)
    {
        /* set the source data and clear the destination buffer */
        for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++)
        {
            g_au32SourceData[u32DataCount] = u32DataCount;
            g_au32DestinationData[u32DataCount] = 0;
        }

        u32DataCount=0;

        if((u32TestCount&0x1FF) == 0)
        {
            putchar('.');
        }

        while(1)
        {
            /* Set data to TX buffer */
            SPI->TX = g_au32SourceData[u32DataCount];

            /* SPI Go */
            SPI->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

            /* Wait SPI is free */
            while(SPI->CNTRL & SPI_CNTRL_GO_BUSY_Msk);

            /* Read Data */
            g_au32DestinationData[u32DataCount] = SPI->RX;
            u32DataCount++;
            if(u32DataCount > TEST_COUNT)    break;
        }

        /*  Check the received data */
        for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++)
        {
            if(g_au32DestinationData[u32DataCount]!=g_au32SourceData[u32DataCount])
                u32Err = 1;
        }

        if(u32Err)
            break;
    }

    if(u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    return ;
}


