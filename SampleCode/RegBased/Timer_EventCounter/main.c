
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/10/02 1:40p $
 * @brief    Use pin P3.4 to demonstrates timer event counter function.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

void TMR0_IRQHandler(void)
{
    printf("Count 1000 falling events! Test complete\n");
    TIMER0->TISR = TIMER_TISR_TIF_Msk;

}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    while(SYS->RegLockAddr != 1)
    {
        SYS->RegLockAddr = 0x59;
        SYS->RegLockAddr = 0x16;
        SYS->RegLockAddr = 0x88;
    }

    SYS->P5_MFP = SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2;
    /* Enable HIRC */
    CLK->PWRCON = CLK_PWRCON_OSC22M_EN_Msk | CLK_PWRCON_HXT;

    /* Waiting for clock ready */
    while((CLK->CLKSTATUS & (CLK_CLKSTATUS_OSC22M_STB_Msk | CLK_CLKSTATUS_XTL_STB_Msk)) !=
            (CLK_CLKSTATUS_OSC22M_STB_Msk | CLK_CLKSTATUS_XTL_STB_Msk));
    /* Enable UART and Timer 0 clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_TMR0_EN_Msk;
    /* Select Timer 0 clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0_S_Msk) | CLK_CLKSEL1_TMR0_S_XTAL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P3 multi function pin for Timer 0 counter pin */
    SYS->P3_MFP = SYS_MFP_P34_T0;

    /* Lock protected registers */
    SYS->RegLockAddr = 0;
}

void UART_Init(void)
{
    // Set UART to 8 bit character length, 1 stop bit, and no parity
    UART0->LCR = UART_LCR_WLS_Msk;
    // 22.1184 MHz reference clock input, for 115200 bps
    // 22118400 / 115200 = 192. Using mode 2 to calculate baudrate, 192 - 2 = 190 = 0xBE
    UART0->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk | (0xBE);

}

int main(void)
{
    int volatile i;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Init();

    printf("\nThis sample code use timer 0 to count P1.0 input event\n");
    printf("Please connect P1.0 to P3.4, press any key to continue\n");
    getchar();

    P1->DOUT |= 1;                     // Set init state to high
    P1->PMD = (P0->PMD & ~0x3) | 0x1;  // Set to output mode

    // Enable interrupt, event counter mode. Timer operate in one shot mode
    TIMER0->TCSR = TIMER_TCSR_CEN_Msk | TIMER_ONESHOT_MODE | TIMER_TCSR_IE_Msk | TIMER_TCSR_CTB_Msk;

    // Cpunt 1000 toggles
    TIMER0->TCMPR = 1000;

    // Counter increase on falling edge
    TIMER0->TEXCON = TIMER_COUNTER_FALLING_EDGE;


    // Enable timer interrupt
    NVIC_EnableIRQ(TMR0_IRQn);


    for(i = 0; i < 1000; i++)
    {
        P1->DOUT &= ~1; // low
        P1->DOUT |= 1;  // high
    }

    while(1);

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


