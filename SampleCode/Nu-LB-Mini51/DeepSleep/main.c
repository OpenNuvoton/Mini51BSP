/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/10/06 11:22a $
 * @brief    This sample code demonstrates how to let system enter and exit
 *           deep sleep mode with external interrupt.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"
#include "lcd_driver.h"


volatile int8_t gi8Key = 1;
char g_strBuf[32] = {0};

void EINT0_IRQHandler(void)
{
    /* Clear P3.2 interrupt flag */
    P3->ISRC = 1 << 2;

    if(P32)
    {
        /* P3.2 is from low to high */
        gi8Key = 1;
    }
    else
    {
        /* P3.2 is from high to low */
        gi8Key = 0;
    }
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable external 12MHz XTAL, 10kHz */
    CLK->PWRCON &= ~CLK_PWRCON_XTLCLK_EN_Msk;
    CLK->PWRCON |= (0x1 <<     CLK_PWRCON_XTLCLK_EN_Pos); // XTAL12M (HXT) Enabled

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_XTAL;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_SPI_EN_Msk;
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART_S_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD  */
    SYS->P0_MFP &= ~(SYS_MFP_P00_Msk | SYS_MFP_P01_Msk);
    SYS->P0_MFP |= (SYS_MFP_P00_TXD | SYS_MFP_P01_RXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART, 115200);

}


void GPIO_Init(void)
{
    /* Enable debounce function of P3.2 (EINT0) */
    P3->DBEN = (1 << 2);

    /* Set debounce time. it is about 6.4 ms */
    GPIO->DBNCECON = GPIO_DBNCECON_DBCLKSRC_IRC10K | GPIO_DBNCECON_DBCLKSEL_64;

    /* Enable P3.2 to be EINT0 */
    GPIO_EnableInt(P3, 2, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(EINT0_IRQn);
}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    uint32_t u32Cnt;

    SYS_UnlockReg();
    SYS->P5_MFP = (SYS->P5_MFP & 0x00FFFCFC) | 0x03;  /* P5.1 -> XTAL2, P5.0 -> XTAL1 */
    CLK->PWRCON = CLK_PWRCON_XTL12M | 4 | 8 ;

    /* Init system, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Init SPI and LCD */
    LCD_Init();
    LCD_EnableBackLight();
    LCD_ClearScreen();

    printf("CPU @ %d Hz\n", SystemCoreClock);

    LCD_Print(0, "DeepSleep");

    /*Initialize external interrupt*/
    GPIO_Init();

    /*
        P3.2 is used as EINT0 for deep sleep (power down) control.
        Press P3.2 will toggle power down/wakeup state to show how to enter power down.
    */
    LCD_Print(1, "Press INT ");

    while (1)
    {
        char strClearVal[15] = "Count:         ";
        /* Enter power when key change from low to high */
        u32Cnt = 0;
        while (gi8Key == 1)
        {
            sprintf(g_strBuf, "Count:%d", u32Cnt++);
            LCD_Print(3, strClearVal);
            LCD_Print(3, g_strBuf);
        }
        while(gi8Key == 0)
        {
            sprintf(g_strBuf, "Count:%d", u32Cnt++);
            LCD_Print(3, strClearVal);
            LCD_Print(3, g_strBuf);
        }

        LCD_Print(2, "Deep Sleeping...");

        /* Unlock protected registers */
        SYS_UnlockReg();

        /* We need to disable debounce function before power down, otherwise, there would be twice interrupt when
           wakeup */
        P3->DBEN = 0;

        /* enter power down */;
        CLK_PowerDown();

        /* Re-enable debounce function if necessary */
        P3->DBEN = (1 << 2);

        LCD_Print(2, "Working...      ");
        printf("\nWorking... \n");

        /* Make sure the key is return to high before next key action */
        while (gi8Key == 0);

        gi8Key = 0;
    }
}

