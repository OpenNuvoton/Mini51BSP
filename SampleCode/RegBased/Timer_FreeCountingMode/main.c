
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/24 7:07p $
 * @brief    Use the timer pin P3.2 to demonstrate timer free counting mode
 *           function. Also display the measured input frequency to UART console.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

void TMR0_IRQHandler(void)
{
    // printf takes long time and affect the freq. calculation, we only print out once a while
    static int cnt = 0;
    static uint32_t t0, t1;

    if(cnt == 0) {
        t0 = TIMER0->TCAP;
        cnt++;
    } else if(cnt == 1) {
        t1 = TIMER0->TCAP;
        cnt++;
        if(t0 > t1) {
            // over run, do nothing
        } else {
            printf("Input frequency is %dHz\n", 22118400 / (t1 - t0));
        }
    } else {
        cnt = 0;
    }


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

    SYS->P5_MFP = SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2;
    /* Enable HIRC */
    CLK->PWRCON = CLK_PWRCON_OSC22M_EN_Msk | CLK_PWRCON_XTL12M;

    /* Waiting for clock ready */
    while((CLK->CLKSTATUS & (CLK_CLKSTATUS_OSC22M_STB_Msk | CLK_CLKSTATUS_XTL_STB_Msk)) !=
            (CLK_CLKSTATUS_OSC22M_STB_Msk | CLK_CLKSTATUS_XTL_STB_Msk));
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

    /* Set P3 multi function pin for Timer 0 capture pin */
    SYS->P3_MFP = SYS_MFP_P32_T0EX;

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

    printf("\nThis sample code demonstrate timer trigger counting mode.\n");
    printf("Please connect input source with Timer 0 capture pin T0EX (P3.2), press any key to continue\n");
    getchar();

    /*
      Timer 0 clock source is 22.118400MHz
    */
    TIMER0->TCSR = TIMER_TCSR_CEN_Msk | TIMER_PERIODIC_MODE;

    // Set compare value as large as possible
    TIMER0->TCMPR = 0xFFFFFF;

    // Configure Timer 0 free counting mode, capture TDR value on falling edge, enable capture interrupt
    TIMER0->TEXCON = TIMER_CAPTURE_FREE_COUNTING_MODE |
                     TIMER_CAPTURE_FALLING_EDGE |
                     TIMER_TEXCON_TEXEN_Msk |
                     TIMER_TEXCON_TEXIEN_Msk;


    NVIC_EnableIRQ(TMR0_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


