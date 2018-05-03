/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/10/06 1:26p $
 * @brief    Use the timer pin P3.2 to demonstrate timer free counting
 *           mode function. Also display the measured input
 *           frequency to UART console.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


void TMR0_IRQHandler(void)
{
    // printf takes long time and affect the freq. calculation, we only print out once a while
    static int cnt = 0;
    static uint32_t t0, t1;

    if(cnt == 0)
    {
        t0 = TIMER_GetCaptureData(TIMER0);
        cnt++;
    }
    else if(cnt == 1)
    {
        t1 = TIMER_GetCaptureData(TIMER0);
        cnt++;
        if(t0 > t1)
        {
            // over run, do nothing
        }
        else
        {
            printf("Input frequency is %dHz\n", 1000000 / (t1 - t0));
        }
    }
    else
    {
        cnt = 0;
    }


    TIMER_ClearCaptureIntFlag(TIMER0);

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

    /* Enable external 12MHz XTAL (UART), and internal 22.1184MHz */
    CLK->PWRCON = CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Enable UART and Timer 0 clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_TMR0_EN_Msk;

    /* Select UART and Timer 0 clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_UART_S_Msk | CLK_CLKSEL1_TMR0_S_Msk)) |
                   CLK_CLKSEL1_UART_S_XTAL |
                   CLK_CLKSEL1_TMR0_S_XTAL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P3 multi function pin for Timer 0 capture pin */
    SYS->P3_MFP = SYS_MFP_P32_T0EX;

    /* Lock protected registers */
    SYS_LockReg();
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
    UART_Open(UART, 115200);

    printf("\nThis sample code demonstrate timer free counting mode.\n");
    printf("Please connect input source with Timer 0 capture pin T0EX (P3.2), press any key to continue\n");
    getchar();

    // Give a dummy target frequency here. Will over write capture resolution with macro
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000000);

    // Update prescale to set proper resolution.
    // Timer 0 clock source is 12MHz, to set resolution to 1ms, we need to
    // set clock divider to 12. e.g. set prescale to 12 - 1 = 11
    TIMER_SET_PRESCALE_VALUE(TIMER0, 11);

    // Set compare value as large as possible, so don't need to worry about counter overrun too frequently.
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);

    // Configure Timer 0 free counting mode, capture TDR value on rising edge
    TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_RISING_EDGE);

    // Start Timer 0
    TIMER_Start(TIMER0);

    // Enable timer interrupt
    TIMER_EnableCaptureInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


