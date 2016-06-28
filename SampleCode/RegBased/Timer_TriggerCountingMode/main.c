
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/10/06 11:48a $
 * @brief    Use the timer pin P3.2 to demonstrate timer trigger counting mode 
 *           function. And displays the measured input frequency to UART console.
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

    cnt++;
    if(cnt == 10) {
        printf("Input frequency is %dHz\n", 1000000 / TIMER0->TCAP);
        cnt = 0;
    }
    // Clear Timer 0 capture interrupt flag
    TIMER0->TEXISR = TIMER_TEXISR_TEXIF_Msk;

}


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

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable external 12MHz XTAL (UART), and internal 22.1184MHz */
    CLK->PWRCON = CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    while((CLK->CLKSTATUS & (CLK_CLKSTATUS_IRC22M_STB_Msk |CLK_CLKSTATUS_XTL_STB_Msk)) !=
            (CLK_CLKSTATUS_IRC22M_STB_Msk |CLK_CLKSTATUS_XTL_STB_Msk));

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
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P3 multi function pin for Timer 0 capture pin */
    SYS->P3_MFP = SYS_MFP_P32_T0EX;

    /* Lock protected registers */
    SYS->RegLockAddr = 0;
}

void UART_Init(void)
{
    // Set UART to 8 bit character length, 1 stop bit, and no parity
    UART->LCR = UART_LCR_WLS_Msk;
    // 22.1184 MHz reference clock input, for 115200 bps
    // 22118400 / 115200 = 192. Using mode 2 to calculate baudrate, 192 - 2 = 190 = 0xBE
    UART->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk | (0xBE);
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

    printf("\nThis sample code demonstrate timer trigger counting mode.\n");
    printf("Please connect input source with Timer 0 capture pin T0EX (P3.2), press any key to continue\n");
    getchar();

    /*
      Timer 0 clock source is 12MHz, to set resolution to 1ms, we need to
      set clock divider to 12. e.g. set prescale to 12 - 1 = 11.
    */
    TIMER0->TCSR = TIMER_TCSR_CEN_Msk | TIMER_PERIODIC_MODE | 11;

    // Set compare value as large as possible
    TIMER0->TCMPR = 0xFFFFFF;

    // Configure Timer 0 trigger counting mode, capture TDR value on falling edge, enable capture interrupt
    TIMER0->TEXCON = TIMER_CAPTURE_TRIGGER_COUNTING_MODE |
                     TIMER_CAPTURE_FALLING_EDGE |
                     TIMER_TEXCON_TEXIEN_Msk |
                     TIMER_TEXCON_TEXEN_Msk |
                     TIMER_CAPTURE_FALLING_EDGE;


    NVIC_EnableIRQ(TMR0_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


