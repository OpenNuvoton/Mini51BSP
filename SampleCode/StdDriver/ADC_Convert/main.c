/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/24 5:50p $
 * @brief    Demonstrate ADC function by repeatedly convert the input of ADC
 *           channel 5 (P1.5) and shows the result on UART console.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

void ADC_IRQHandler(void)
{
    uint32_t u32Flag;

    // Get ADC comparator interrupt flag
    u32Flag = ADC_GET_INT_FLAG(ADC, ADC_ADF_INT);

    // Get ADC convert result
    printf("Convert result is %x\n", (uint32_t)ADC_GET_CONVERSION_DATA(ADC, 0));

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
    SYS->P5_MFP = SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2;

    /* Enable external 12MHz XTAL, HIRC */
    CLK->PWRCON = CLK_PWRCON_OSC22M_EN_Msk | CLK_PWRCON_HXT;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk | CLK_CLKSTATUS_XTL_STB_Msk);

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_XTAL,CLK_CLKDIV_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART_MODULE);
    CLK_EnableModuleClock(ADC_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART_MODULE,CLK_CLKSEL1_UART_S_XTAL,CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(ADC_MODULE,CLK_CLKSEL1_ADC_S_XTAL,CLK_CLKDIV_ADC(6));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P1 multi-function pin ADC channel 5 input*/
    SYS->P1_MFP = SYS_MFP_P15_AIN5;

    /* Analog pin OFFD to prevent leakage */
    P1->OFFD |= (1 << 5) << GPIO_OFFD_OFFD_Pos;

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

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
    UART_Open(UART0, 115200);

    printf("\nThis sample code demonstrate ADC channel 5 conversion and printf the result on UART\n");

    // Enable channel 5
    ADC_Open(ADC, 0, 0, 0x01 << 5);

    // Power on ADC
    ADC_POWER_ON(ADC);


    // Enable ADC convert complete interrupt
    ADC_EnableInt(ADC, ADC_ADF_INT);
    NVIC_EnableIRQ(ADC_IRQn);

    while(1) {
        // Trigger ADC conversion if it is idle
        if(!ADC_IS_BUSY(ADC)) {
            ADC_START_CONV(ADC);
        }
    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


