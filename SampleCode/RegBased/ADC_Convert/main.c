/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/10/06 11:42a $
 * @brief    Demonstrate ADC function by repeatedly convert the input of ADC
 *           channel 0 (P5.3) and shows the result on UART console.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


void ADC_IRQHandler(void)
{
    // Get ADC convert result
    printf("Convert result is %x\n", (uint32_t)(ADC->ADDR & ADC_ADDR_RSLT_Msk));

    // Clear ADC convert complete flag
    ADC->ADSR |= ADC_ADSR_ADF_Msk;
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

    /* Enable internal 22.1184MHz */
    CLK->PWRCON = CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC22M_STB_Msk));

    /* Enable UART and ADC clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_ADC_EN_Msk;

    /* ADC clock source is 22.1184MHz, set divider to (3 + 1), ADC clock is 22.1184/4 MHz */
    CLK->CLKDIV |= (3 << CLK_CLKDIV_ADC_N_Pos);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P5.3 to ADC channel 0 input pin */
    SYS->P5_MFP = SYS_MFP_P53_AIN0;
    /* Analog pin OFFD to prevent leakage */
    P5->OFFD |= (1 << 3) << GPIO_OFFD_OFFD_Pos;

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

    printf("\nThis sample code demonstrate ADC channel 0 (P5.3)conversion function\n");

    // Enable channel 0
    ADC->ADCHER = 1;
    // Turn on ADC power and enable covert complete interrupt
    ADC->ADCR = ADC_ADCR_ADEN_Msk | ADC_ADCR_ADIE_Msk;



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

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


