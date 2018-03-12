
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/24 7:22p $
 * @brief    Demonstrate ADC conversion and comparison function by monitoring
 *           the conversion result of channel 5.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


void ADC_IRQHandler(void)
{
    uint32_t reg = ADC->ADSR;
    // Get ADC Comapre result
    if(reg & ADC_CMP0_INT)
        printf("Channel 5 input < 0x200\n");
    if(reg & ADC_CMP1_INT)
        printf("Channel 5 input >= 0x200\n");

    // Clear ADC interrupt flag
    ADC->ADSR |= reg;
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
    CLK->PWRCON = CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Enable UART and ADC clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_ADC_EN_Msk;

    /* ADC clock source is HIRC, set divider to (3 + 1), ADC clock is HIRC/4 MHz */
    CLK->CLKDIV |= (6 << CLK_CLKDIV_ADC_N_Pos);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;
    /* Set P1 multi-function pin for ADC channel 5 */
    SYS->P1_MFP = SYS_MFP_P15_AIN5;

    /* Analog pin OFFD to prevent leakage */
    P1->OFFD |= (1 << 5) << GPIO_OFFD_OFFD_Pos;

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

    printf("\nThis sample code demonstrate ADC channel 5 (P1.5)conversion function\n");

    // Enable channel 5
    ADC->ADCHER = 1 << 5;
    // Turn on ADC power and enable covert complete interrupt
    ADC->ADCR = ADC_ADCR_ADEN_Msk | ADC_ADCR_ADIE_Msk;

    // Configure and enable Comparator 0 to monitor channel 5 input less than 0x200
    ADC->ADCMPR[0] = ADC_ADCMPR_CMPEN_Msk |
                     ADC_ADCMPR_CMPIE_Msk |
                     ADC_CMP_LESS_THAN |
                     (5 << ADC_ADCMPR_CMPCH_Pos) |
                     (0xF << ADC_ADCMPR_CMPMATCNT_Pos) |
                     (0x200 << ADC_ADCMPR_CMPD_Pos);

    // Configure and enable Comparator 1 to monitor channel 5 input greater or equal to 0x200
    ADC->ADCMPR[1] = ADC_ADCMPR_CMPEN_Msk |
                     ADC_ADCMPR_CMPIE_Msk |
                     ADC_CMP_GREATER_OR_EQUAL_TO |
                     (5 << ADC_ADCMPR_CMPCH_Pos) |
                     (0xF << ADC_ADCMPR_CMPMATCNT_Pos) |
                     (0x200 << ADC_ADCMPR_CMPD_Pos);

    // Enable ADC interrupt
    NVIC_EnableIRQ(ADC_IRQn);

    while(1)
    {
        // Check if ADC is busy
        if(!(ADC->ADSR & ADC_ADSR_BUSY_Msk))
        {
            // Trigger ADC conversion
            ADC->ADCR |= ADC_ADCR_ADST_Msk;
        }
    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


