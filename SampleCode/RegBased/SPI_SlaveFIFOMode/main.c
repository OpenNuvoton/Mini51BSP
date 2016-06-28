/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/11/05 11:33a $
 * @brief    Mini51 SPI Driver Sample Code
 *           This is a SPI slave mode demo and need to be tested with a master device.
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
    /* Configure as a slave, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    CLK->APBCLK |= CLK_APBCLK_SPI_EN_Msk;
    SPI->CNTRL = SPI_SLAVE | SPI_MODE_0;
    SPI->DIVIDER = (((12000000 / 2000000) + 1) >> 1) - 1;

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI->SSR |= (SPI_SS | SPI_SS_ACTIVE_LOW | SPI_SSR_SS_LTRIG_Msk);
}

void SPI_IRQHandler(void)
{
    uint32_t temp;

    while( !(SPI->STATUS & SPI_STATUS_TX_FULL_Msk) && (g_u32TxDataCount<TEST_COUNT) ) {
        SPI->TX = g_au32SourceData[g_u32TxDataCount++];
    }
	
	while(!(SPI->STATUS & SPI_STATUS_RX_EMPTY_Msk)) {
        temp = SPI->RX;
        g_au32DestinationData[g_u32RxDataCount++] = temp;
    }
	
    if(g_u32TxDataCount>=TEST_COUNT) {
		SPI->FIFO_CTL &= ~SPI_FIFO_CTL_TX_INTEN_Msk; /* Disable TX FIFO threshold interrupt */         
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

    printf("Configure SPI as a slave.\n");
    printf("SPI clock rate: %d Hz\n", SPI_GetBusClock(SPI0));
    
    for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++) {
        g_au32SourceData[u32DataCount] = 0x00550000 + u32DataCount;
        g_au32DestinationData[u32DataCount] = 0;
    }    
   
    SPI->CNTRL |= SPI_CNTRL_FIFO_Msk;
    SPI->FIFO_CTL |= (SPI_FIFO_CTL_RX_INTEN_Msk | SPI_FIFO_CTL_TX_INTEN_Msk);
    SPI->FIFO_CTL = (SPI->FIFO_CTL & ~(SPI_FIFO_CTL_TX_THRESHOLD_Msk | SPI_FIFO_CTL_RX_THRESHOLD_Msk) |
                     (2 << SPI_FIFO_CTL_TX_THRESHOLD_Pos) |
                     (1 << SPI_FIFO_CTL_RX_THRESHOLD_Pos));
    NVIC_EnableIRQ(SPI_IRQn);
	
	while(!g_u8Done);
    
    printf("Received data:\n");
    for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++) {
        printf("%d:\t0x%08X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }
        
    printf("The data transfer was done.\n");

    while(1);
}
