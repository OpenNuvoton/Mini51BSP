/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/10/12 8:12p $
 * @brief    Demonstrate how to communicate with an off-chip SPI slave device.
 *           This sample code needs to work with SPI_SlaveMode.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

#define TEST_COUNT 16

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;
volatile uint8_t g_u8Done;

void SYS_Init(void)
{
    int32_t i32TimeOutCnt;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    while(SYS->RegLockAddr != SYS_RegLockAddr_RegUnLock_Msk) {
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
            (CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk)) {
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
    /* Set IP clock divider. SPI clock rate = 1MHz */
    CLK->APBCLK |= CLK_APBCLK_SPI_EN_Msk;
    SPI->CNTRL = SPI_MASTER | SPI_MODE_0;
    SPI->DIVIDER = (((12000000 / 1000000) + 1) >> 1) - 1;

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI->SSR |= (SPI_SS | SPI_SS_ACTIVE_LOW) | SPI_SSR_AUTOSS_Msk;
}

void SPI_IRQHandler(void)
{
    SPI_CLR_UNIT_TRANS_INT_FLAG(SPI0);
    g_u8Done = 1;
}

int main(void)
{
    uint32_t u32DataCount, i;

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

    printf("Configure SPI as a master.\n");
    printf("SPI clock rate: %d Hz\n", SPI_GetBusClock(SPI0));

    for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++) {
        g_au32SourceData[u32DataCount] = 0x00550000 + u32DataCount;
        g_au32DestinationData[u32DataCount] = 0;
    }

    printf("Before starting the data transfer, make sure the slave device is ready. Press any key to start the transfer.\n");
    getchar();
    printf("\n");

    SPI0->CNTRL |= SPI_CNTRL_IE_Msk;
    NVIC_EnableIRQ(SPI_IRQn);

    for(i=0; i<TEST_COUNT; i++) {
        g_u8Done = 0;
        SPI0->TX = g_au32SourceData[i];
        SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

        while(!g_u8Done);
        g_au32DestinationData[i] = SPI_READ_RX(SPI0);
    }

    printf("Received data:\n");
    for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++) {
        printf("%d:\t0x%08X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }

    printf("The data transfer was done.\n");

    while(1);
}

