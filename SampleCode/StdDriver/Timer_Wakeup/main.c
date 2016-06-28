
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/24 4:26p $
 * @brief    Use timer to wake up system from Power-down mode periodically.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

void TMR0_IRQHandler(void)
{
    // Clear wake up flag
    TIMER_ClearWakeupFlag(TIMER0);
    // Clear interrupt flag
    TIMER_ClearIntFlag(TIMER0);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);


    /* Enable external 12MHz XTAL (UART), LIRC (for timer) and HIRC */
    CLK->PWRCON = CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk | CLK_PWRCON_IRC10K_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk | CLK_CLKSTATUS_IRC10K_STB_Msk);

    /* Enable UART and Timer 0 clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_TMR0_EN_Msk;

    /* Select UART clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_S_Msk) | CLK_CLKSEL1_UART_S_XTAL;

    /* Select Timer clock source from LIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0_S_Msk) | CLK_CLKSEL1_TMR0_S_IRC10K;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Lock protected registers */
    SYS_LockReg();
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
    UART_Open(UART0, 115200);

    printf("\nThis sample code use timer to wake up system every 1 second \n");

    /*Initial Timer0 to periodic mode with 1Hz */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);

    /* Enable timer wake up system */
    TIMER_EnableWakeup(TIMER0);
    /* Enable Timer0 interrupt */
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    /* Unlock protected registers to let CPU enter power down mode */
    SYS_UnlockReg();

    while(1) {
        printf("Sleep 1 second\n");
        // Wait 'til UART FIFO empty to get a cleaner console out
        while(!UART_IS_TX_EMPTY(UART0));
        CLK_PowerDown();
    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


