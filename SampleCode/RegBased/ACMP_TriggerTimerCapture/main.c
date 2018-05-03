/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/10/06 11:42a $
 * @brief    Show how to use Analog comparator (ACMP) state change to trigger
 *           timer capture function. P1.5 is used as comparator positive input
 *           and Band-gap voltage as negative input.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


void TMR0_IRQHandler(void)
{
    // printf takes long time and affect the freq. calculation, we only print out once a while
    static int cnt = 0;

    cnt++;
    if(cnt == 60)
    {
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
    while(SYS->RegLockAddr != 1)
    {
        SYS->RegLockAddr = 0x59;
        SYS->RegLockAddr = 0x16;
        SYS->RegLockAddr = 0x88;
    }

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable internal 22.1184MHz */
    CLK->PWRCON = CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    while((CLK->CLKSTATUS & (CLK_CLKSTATUS_IRC22M_STB_Msk |CLK_CLKSTATUS_XTL_STB_Msk)) !=
            (CLK_CLKSTATUS_IRC22M_STB_Msk |CLK_CLKSTATUS_XTL_STB_Msk));

    /* Enable Timer, UART and ACMP clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_CMP_EN_Msk | CLK_APBCLK_TMR0_EN_Msk | CLK_APBCLK_PWM01_EN_Msk;

    /* Select system clock source from external crystal*/
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLK_S_Msk) | CLK_CLKSEL0_HCLK_S_XTAL;

    /* Select Timer 0 clock source from HCLK, to use ACMP trigger timer, timer clock source must be HCLK */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0_S_Msk) | CLK_CLKSEL1_TMR0_S_HCLK;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P1 multi-function pins for ACMP CPP0*/
    SYS->P1_MFP = SYS_MFP_P15_CPP0;

    /* Set P3.6 to ACMP CPO0 function and P3.2 to Timer 0 capture pin*/
    SYS->P3_MFP = SYS_MFP_P36_CPO0 | SYS_MFP_P32_T0EX;

    /* Analog pin OFFD to prevent leakage */
    P1->OFFD |= (1 << 5) << GPIO_OFFD_OFFD_Pos;

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

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Init();

    PWM->PHCHG |= PWM_PHCHGNXT_CE0_Msk | PWM_PHCHGNXT_ACCNT0_Msk;//Enable signal path from ACMP to Timer

    printf("\nThis sample code demonstrate ACMP0 function. Using CPP0 (P1.5) as ACMP0\n");
    printf("positive input and internal bandgap voltage as the negative input\n");
    printf("The compare result reflects on CPO0 (P3.6)\n");
    printf("Timer detects ACMP signal to calculate its output frequency\n");

    /* Configure ACMP0 Comparator 0. Enable ACMP0, enable interrupt and select internal reference voltage as negative input */
    ACMP->CMPCR[0] = ACMP_CMPCR_ACMPEN_Msk | ACMP_CMPCR_ACMPIE_Msk | ACMP_CMPCR_NEGSEL_Msk | ACMP_CMPCR_FALLING_Msk;

    TIMER0->TCSR = TIMER_TCSR_CAP_SRC_Msk | TIMER_TCSR_CEN_Msk | TIMER_PERIODIC_MODE | 11;

    // Set compare value as large as possible
    TIMER0->TCMPR = 0xFFFFFF;

    // Configure Timer 0 trigger counting mode, capture TDR value on falling edge, enable capture interrupt
    TIMER0->TEXCON = TIMER_CAPTURE_TRIGGER_COUNTING_MODE |
                     TIMER_CAPTURE_FALLING_THEN_RISING_EDGE |
                     TIMER_TEXCON_TEXIEN_Msk |
                     TIMER_TEXCON_TEXEN_Msk |
                     TIMER_CAPTURE_FALLING_EDGE;

    NVIC_EnableIRQ(TMR0_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


