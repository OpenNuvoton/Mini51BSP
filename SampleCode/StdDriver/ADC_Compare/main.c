
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/10/06 1:19p $
 * @brief    Demonstrate ADC conversion and comparison function by monitoring
 *           the conversion result of channel 0.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


void ADC_IRQHandler(void)
{
    uint32_t u32Flag;

    // Get ADC comparator interrupt flag
    u32Flag = ADC_GET_INT_FLAG(ADC, ADC_CMP0_INT | ADC_CMP1_INT);

    if(u32Flag & ADC_CMP0_INT)
        printf("Channel 0 input < 0x200\n");
    if(u32Flag & ADC_CMP1_INT)
        printf("Channel 0 input >= 0x200\n");

    ADC_CLR_INT_FLAG(ADC, u32Flag);
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

    /* Enable external 12MHz XTAL (UART), internal 22.1184MHz */
    CLK->PWRCON = CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Enable UART and ADC clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_ADC_EN_Msk;

    /* Select UART clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_S_Msk) | CLK_CLKSEL1_UART_S_XTAL;

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
    SYS_LockReg();
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
    UART_Open(UART, 115200);

    printf("\nThis sample code demonstrate ADC conversion and comparison function\n");
    printf("by monitoring the conversion result of channel 0 (P5.3)\n");

    // Enable channel 0
    ADC_Open(ADC, 0, 0, 0x01);

    // Power on ADC
    ADC_POWER_ON(ADC);

    // Configure and enable Comparator 0 to monitor channel 0 input less than 0x200
    ADC_ENABLE_CMP0(ADC, 0, ADC_CMP_LESS_THAN, 0x200, 16);
    // Configure and enable Comparator 1 to monitor channel 0 input greater or equal to 0x200
    ADC_ENABLE_CMP1(ADC, 0, ADC_CMP_GREATER_OR_EQUAL_TO, 0x200, 16);

    // Enable ADC comparator 0 and 1 interrupt
    ADC_EnableInt(ADC, ADC_CMP0_INT);
    ADC_EnableInt(ADC, ADC_CMP1_INT);
    NVIC_EnableIRQ(ADC_IRQn);

    while(1)
    {
        // Trigger ADC conversion if it is idle
        if(!ADC_IS_BUSY(ADC))
        {
            ADC_START_CONV(ADC);
        }
    }

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


