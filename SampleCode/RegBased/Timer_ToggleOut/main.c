/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/24 7:03p $
 * @brief    Demonstrate the timer 0 toggle out function on pin P3.4.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    while(SYS->RegLockAddr != 1) {
        SYS->RegLockAddr = 0x59;
        SYS->RegLockAddr = 0x16;
        SYS->RegLockAddr = 0x88;
    }

    /* Enable internal 22.1184MHz */
    CLK->PWRCON = CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for clock ready */
    while((CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk) !=
            CLK_CLKSTATUS_OSC22M_STB_Msk);
    /* Enable UART and Timer 0 clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_TMR0_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;


    /* Set P3 multi-function pins for Timer toggle output pin */
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
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Init();

    printf("\nThis sample code use timer to toggle P3.4 every 10ms \n");

    // To generate interrupt every second, we set prescale = 1, so timer clock is 22.1184MHz / (1 + 1)
    // And set compare value to 22118400 / 200, timer will time out every 1/100 second, and output 50Hz waveform
    TIMER0->TCMPR = (22118400 / 200);
    // Enable timer counter , set timer operating in toggle mode, and set prescale to 1
    TIMER0->TCSR = TIMER_TCSR_CEN_Msk | TIMER_TOGGLE_MODE | (1);

    while(1);

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


