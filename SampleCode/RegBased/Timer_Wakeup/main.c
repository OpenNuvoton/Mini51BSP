/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/24 6:58p $
 * @brief    Use timer to wake up system from Power-down mode periodically.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


void TMR0_IRQHandler(void)
{

    // clear timer interrupt flag and wake up flag
    TIMER0->TISR = TIMER_TISR_TIF_Msk | TIMER_TISR_TWF_Msk;

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

    /* Enable HIRC */
    CLK->PWRCON = CLK_PWRCON_OSC22M_EN_Msk | CLK_PWRCON_OSC10K_EN_Msk;

    /* Waiting for clock ready */
    while((CLK->CLKSTATUS & (CLK_CLKSTATUS_OSC22M_STB_Msk | CLK_CLKSTATUS_OSC10K_STB_Msk)) != (CLK_CLKSTATUS_OSC22M_STB_Msk | CLK_CLKSTATUS_OSC10K_STB_Msk));

    /* Enable UART and Timer 0 clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_TMR0_EN_Msk;

    /* Select Timer clock source from LIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0_S_Msk) | CLK_CLKSEL1_TMR0_S_IRC10K;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

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

    printf("\nThis sample code use timer to wake up system every 1 second \n");

    // To generate interrupt every second, we set prescale = 1, so timer clock is 10k / (1 + 1)
    // And set compare value to 10000 / 2, timer will generate time out interrupt every 1 second
    TIMER0->TCMPR = (10000 / 2);
    // Enable timer counter , enable timer wake up, enable timer interrupt, set timer operating in periodic mode, and set prescale to 1
    TIMER0->TCSR = TIMER_TCSR_CEN_Msk | TIMER_TCSR_WAKE_EN_Msk | TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE | (1);
    // Enable timer interrupt
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Unlock protected registers to control PWRCTL */
    while(SYS->RegLockAddr != 1)
    {
        SYS->RegLockAddr = 0x59;
        SYS->RegLockAddr = 0x16;
        SYS->RegLockAddr = 0x88;
    }

    while(1)
    {
        printf("Sleep 1 second\n");
        // Wait 'til UART FIFO empty to get a cleaner console out
        while(!(UART0->FSR & UART_FSR_TE_FLAG_Msk));
        // Enable sleep deep mode
        SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;
        // Enable power down mode
        CLK->PWRCON |= CLK_PWRCON_PWR_DOWN_EN_Msk;
        // Power down
        __WFI();
    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


