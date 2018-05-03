/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/24 5:28p $
 * @brief    Demonstrate the PWM double buffer feature.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

uint32_t duty30, duty60;
void PWM_IRQHandler(void)
{
    static int toggle = 0;  // First two already fill into PWM, so start from 30%

    // Update PWM channel 0 duty
    if(toggle == 0)
    {
        PWM_SET_CMR(PWM, 0, duty30);
    }
    else
    {
        PWM_SET_CMR(PWM, 0, duty60);
    }
    toggle ^= 1;
    // Clear channel 0 period interrupt flag
    PWM_ClearPeriodIntFlag(PWM, 0);
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

    /* Enable external 12MHz XTAL (UART), HIRC */
    CLK->PWRCON = CLK_PWRCON_OSC22M_EN_Msk | CLK_PWRCON_HXT;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk | CLK_CLKSTATUS_XTL_STB_Msk);

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_XTAL,CLK_CLKDIV_HCLK(1));

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_PWM01_EN_Msk;

    /* Select UART clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_S_Msk) | CLK_CLKSEL1_UART_S_XTAL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;


    /* Set P2 multi-function pin for PWM Channel 0  */
    SYS->P2_MFP = SYS_MFP_P22_PWM0;


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

    printf("This sample changed PWM0 output between 30%% and 60%% duty ratio\n");
    /*
        PWM channel 0 wave form of this sample changed between 30% and 60% duty ratio
    */
    PWM_ConfigOutputChannel(PWM, 0, 1000, 30);

    // Save 30% duty setting
    duty30 = PWM->CMR[0];
    // Calculate 60% duty setting. CMR store the actual value minus 1.
    duty60 = (duty30 + 1) * 2 - 1;

    // Enable output of all PWM channel 0
    PWM_EnableOutput(PWM, 1);

    // Enable PWM channel 0 period interrupt
    PWM_EnablePeriodInt(PWM, 0, PWM_PERIOD_INT_UNDERFLOW);
    NVIC_EnableIRQ(PWM_IRQn);

    // Start
    PWM_Start(PWM, 0x1);

    // Fill second duty setting immediately after PWM start
    PWM_SET_CMR(PWM, 0, duty60);

    while(1);

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


