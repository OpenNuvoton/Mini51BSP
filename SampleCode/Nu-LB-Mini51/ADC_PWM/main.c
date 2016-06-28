/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/10/06 11:22a $
 * @brief    This sample adjusts the PWM output duty according to ADC conversion 
 *           result where the input voltage is control by VR. The PWM output 
 *           connects to a buzzer so user can control the buzzer tone with VR
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>      // For sprintf()
#include "Mini51Series.h"
#include "lcd_driver.h"

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable ADC and PWM23 clock */
    CLK->APBCLK = CLK_APBCLK_PWM23_EN_Msk | CLK_APBCLK_ADC_EN_Msk;

    /* ADC clock source is 22.1184MHz, set divider to (3 + 1), ADC clock is 22.1184/4 MHz */
    CLK->CLKDIV |= (3 << CLK_CLKDIV_ADC_N_Pos);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P2.5 to PWM3 function */
    SYS->P2_MFP = SYS_MFP_P25_PWM3;
    /* Set P5.3 to AIN0 function */
    SYS->P5_MFP = SYS_MFP_P53_AIN0;

    /* Analog pin OFFD to prevent leakage */
    P5->OFFD |= (1 << 3) << GPIO_OFFD_OFFD_Pos;

    /* Lock protected registers */
    SYS_LockReg();
}


void PWM_Init(void)
{
    /* Set PWM 3 clock prescale to 60, and divider to 1*/
    PWM->PPR = 60 << PWM_PPR_CP23_Pos;
    PWM->CSR = PWM_CLK_DIV_1 << PWM_CSR_CSR3_Pos;

    /* Enable PWM3 and set to auto-reload mode.
       Configure PWM mode before setting CNR, CMR. Otherwise CNR will be reset */
    PWM->PCR = PWM_PCR_CH3EN_Msk | PWM_PCR_CH3MOD_Msk;
    /* PWM 3 frequency = HCLK / (prescale * divider * (CNR + 1))
       PWM 3 duty ratio = (CMR + 1) / (CNR + 1) */
    PWM->CNR[3] = 12500 - 1;
    PWM->CMR[3] = 12400 - 1;

    /* Enable PWM3 Output */
    PWM->POE = PWM_POE_PWM3_Msk;

}

void ADC_Init(void)
{
    /* Set to convert ADC channel 0 */
    ADC->ADCHER = 0x1;
    /* Enable the ADC converter */
    ADC->ADCR = ADC_ADCR_ADEN_Msk;

}

/* Main function  */
int main (void)
{
    uint32_t u32Data;
    char adc_value[15]="ADC Value:";

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

    LCD_Print(0, "ADC_PWM");
    LCD_Print(1, "Sample code");

    /* Init PWM channel 3*/
    PWM_Init();

    /* Init ADC to get the value of variable resistor */
    ADC_Init();


    while(1) {
        /* Start convert */
        ADC_START_CONV(ADC);

        /* Waiting for convert complete */
        while(ADC_IS_BUSY(ADC));

        /* Read the result from ADC */
        u32Data = ADC->ADDR & ADC_ADDR_RSLT_Msk;

        /* Update ADC conversion result to LCD */
        sprintf(adc_value + 10, "%d", u32Data);
        LCD_Print(3, "              ");
        LCD_Print(3, adc_value);

        /* Adjust the duty cycle of PWM3 according to ADC value. */
        PWM->CMR[3] = (u32Data * PWM->CNR[3]) / 1024;

    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


