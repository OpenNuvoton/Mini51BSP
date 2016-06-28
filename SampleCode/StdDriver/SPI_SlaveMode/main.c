/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/10/13 2:15p $
 * @brief    Demonstrate how to communicate with an off-chip SPI master device. 
 *           This sample code needs to work with SPI_MasterMode.
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
    /* Configure as a slave, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI, SPI_SLAVE, SPI_MODE_0, 32, 2000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI, SPI_SS, SPI_SS_ACTIVE_LOW);
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

    printf("Configure SPI as a slave.\n");
    printf("SPI clock rate: %d Hz\n", SPI_GetBusClock(SPI0));
    
    for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++) {
        g_au32SourceData[u32DataCount] = 0x00550000 + u32DataCount;
        g_au32DestinationData[u32DataCount] = 0;
    }    
   
    SPI_EnableInt(SPI0, SPI_IE_MASK);
    NVIC_EnableIRQ(SPI_IRQn);
	
	for(i=0;i<TEST_COUNT;i++) {
		g_u8Done = 0;
		SPI_WRITE_TX(SPI0, g_au32SourceData[i]);
		SPI_TRIGGER(SPI0);
		
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
