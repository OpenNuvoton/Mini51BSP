/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/10/06 11:24a $
 * @brief    This sample demonstrates how to configure timer in periodic mode
 *           and watchdog timer. The interrupt status of timer and WDT is
 *           shown on LCD control via SPI interface
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>  // for sprintf()
#include "Mini51Series.h"
#include "lcd_driver.h"


/* Callback function */
volatile uint32_t u32Timer0Cnt = 0;
volatile uint8_t b8WDTINT = FALSE;

void TMR0_IRQHandler()
{
    char TimerValue[12] = "Timer:";

    ++u32Timer0Cnt;
    sprintf(TimerValue + 6, "%d", u32Timer0Cnt);
    LCD_Print(3, TimerValue);

    // clear timer interrupt flag
    TIMER0->TISR = TIMER_TISR_TIF_Msk;
}

void WDT_IRQHandler()
{
    b8WDTINT = TRUE;

    // Must unlock before modify WDT control register
    SYS_UnlockReg();

    // Reset WDT and clear time out flag
    WDT->WTCR |= WDT_WTCR_WTR_Msk;
    SYS_LockReg();

    LCD_Print(3, "WDT interrupt !!");

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

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK->PWRCON |= CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk |CLK_PWRCON_IRC10K_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk | CLK_CLKSTATUS_IRC10K_STB_Msk);

    /* Enable Timer0 and WDT clock */
    CLK->APBCLK = CLK_APBCLK_TMR0_EN_Msk | CLK_APBCLK_WDT_EN_Msk;

    /* Set IRC 10K as WDT clock and external crystal as Timer0 clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_WDT_S_IRC10K | CLK_CLKSEL1_TMR0_S_XTAL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main(void)
{

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    // Initial LCD panel. SPI clock and interface is configured in this function.
    LCD_Init();
    // Clear LCD screen. Do this before turn back light on
    LCD_ClearScreen();
    // Enable LCD back light
    LCD_EnableBackLight();

    LCD_Print(0, "TIMER Test      ");

    //Enable Timer0 interrupt
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Configure Timer 0 in 2Hz periodic mode
       Timer clock source is external crystal, to get a 2Hz, set comparator
       to __XTAL (external crystal frequency) /2, and prescale to 1
    */
    TIMER0->TCMPR = __XTAL / 2;
    TIMER0->TCSR = TIMER_TCSR_CEN_Msk | TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE | (1);

    LCD_Print(2, "Test 10 times   ");

    while (u32Timer0Cnt < 10);

    /* Timer test complete, stop timer, disable timer interrupt */
    TIMER0->TCSR = 0;
    NVIC_DisableIRQ(TMR0_IRQn);
    LCD_Print(2, "Test Timer OK  ");



    LCD_ClearScreen();
    LCD_Print(0, "WDT Test        ");

    // Enable WDT interrupt
    NVIC_EnableIRQ(WDT_IRQn);

    /* WDT Function Test
      The sample code will set WDT interval is 2^16 WDT clock, and WDT clock source is 10K.
      Wait about 6 seconds to exit WDT test function. */
    // Must unlock before modify WDT control register
    SYS_UnlockReg();
    // Set WDT time out period, enable WDT, and enable WDT interrupt
    WDT->WTCR = WDT_TIMEOUT_2POW16 | WDT_WTCR_WTE_Msk | WDT_WTCR_WTIE_Msk;

    // Lock up
    SYS_LockReg();

    LCD_Print(2, "Wait WDT INT 6s ");

    // Wait for WDT interrupt
    while (!b8WDTINT);

    // WDT test complete, disable WDT and its interrupt */
    SYS_UnlockReg();
    WDT->WTCR = 0; // Disable WDT
    SYS_LockReg();
    NVIC_DisableIRQ(WDT_IRQn);

    LCD_Print(2, "Test WDT OK     ");

    while (1);
}
/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
