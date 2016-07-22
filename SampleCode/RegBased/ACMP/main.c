
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/10/06 11:41a $
 * @brief    Demonstrate Analog comparator (ACMP) comparison by comparing
 *           CPP0 (P1.5) with Band-gap voltage and shows the result on UART console.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


void ACMP_IRQHandler(void)
{
    static uint32_t u32Cnt = 0;

    u32Cnt++;
    /* Clear ACMP 0 interrupt flag */
    ACMP->CMPSR = ACMP_CMPSR_ACMPF0_Msk;

    /* Check Comparator 0 Output Status */
    if (ACMP->CMPSR & ACMP_CMPSR_ACMPCO0_Msk)
        printf("CPP0 > CPN0 (%d)\n", u32Cnt);
    else
        printf("CPP0 <= CPN0 (%d)\n", u32Cnt);
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

    /* Enable internal 22.1184MHz */
    CLK->PWRCON = CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC22M_STB_Msk));

    /* Enable UART and ACMP clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_CMP_EN_Msk;

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

    /* Set P3.6 to ACMP CPO0 function */
    SYS->P3_MFP = SYS_MFP_P36_CPO0;

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

    printf("\nThis sample code demonstrate ACMP0 function. Using CPP0 (P1.5) as ACMP0\n");
    printf("positive input and internal bandgap voltage as the negative input\n");
    printf("The compare result reflects on CPO0 (P3.6)\n");

    /* Configure ACMP0 Comparator 0. Enable ACMP0, enable interrupt and select internal reference voltage as negative input */
    ACMP->CMPCR[0] = ACMP_CMPCR_ACMPEN_Msk | ACMP_CMPCR_ACMPIE_Msk | ACMP_CMPCR_NEGSEL_Msk;

    // Enable ADC interrupt
    NVIC_EnableIRQ(ACMP_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


