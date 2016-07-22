/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/11/05 11:31a $
 * @brief    Mini51 SPI Driver Sample Code
 *           This is a SPI master mode demo and need to be tested with a slave device.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

#define TEST_COUNT 16

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount = 0;
volatile uint32_t g_u32RxDataCount = 0;
volatile uint8_t g_u8Done = 0;

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK_EnableXtalRC(CLK_PWRCON_HXT|CLK_PWRCON_HIRC_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_XTAL,CLK_CLKDIV_HCLK(1));

    /* STCLK to XTL STCLK to XTL */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLK_S_XTAL);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART_MODULE);
    CLK_EnableModuleClock(SPI_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART_MODULE,CLK_CLKSEL1_UART_S_XTAL,CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_RXD | SYS_MFP_P00_TXD);

    /* Setup SPI multi-function pin */
    SYS->P0_MFP |= SYS_MFP_P04_SPISS | SYS_MFP_P05_MOSI | SYS_MFP_P06_MISO | SYS_MFP_P07_SPICLK;

    /* Lock protected registers */
    SYS_LockReg();

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART, 115200);
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
    /* Set IP clock divider. SPI clock rate = 1MHz */
    SPI_Open(SPI, SPI_MASTER, SPI_MODE_0, 32, 1000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI, SPI_SS, SPI_SS_ACTIVE_LOW);
}

void SPI_IRQHandler(void)
{
    uint32_t temp;

    while( (SPI_GET_TX_FIFO_FULL_FLAG(SPI0)==0) && (g_u32TxDataCount<TEST_COUNT) ) {
        SPI_WRITE_TX(SPI0, g_au32SourceData[g_u32TxDataCount++]);
    }

    while((SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0))==0) {
        temp = SPI_READ_RX(SPI0);
        g_au32DestinationData[g_u32RxDataCount++] = temp;
    }

    if(g_u32TxDataCount>=TEST_COUNT) {
        SPI_DisableInt(SPI0, SPI_FIFO_TX_INTEN_MASK); /* Disable TX FIFO threshold interrupt */
        g_u8Done = 1;
    }
}

int main(void)
{
    uint32_t u32DataCount;

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
    //getchar();
    printf("\n");

    SPI_EnableInt(SPI0, SPI_FIFO_TX_INTEN_MASK | SPI_FIFO_RX_INTEN_MASK);
    NVIC_EnableIRQ(SPI_IRQn);
    SPI_EnableFIFO(SPI0, 2, 1);

    /* Wait for transfer done */
    while(!g_u8Done);

    printf("Received data:\n");
    for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++) {
        printf("%d:\t0x%08X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }

    printf("The data transfer was done.\n");

    while(1);
}

