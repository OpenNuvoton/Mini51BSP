
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/10/06 1:21p $
 * @brief    Demonstrate the dead-zone feature with PWM.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


void PWM_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;

    // Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times.
    if(++cnt == 100)
    {
        if(out)
            PWM_EnableOutput(PWM, 0x3F);
        else
            PWM_DisableOutput(PWM, 0x3F);
        out ^= 1;
        cnt = 0;
    }
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
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK->PWRCON = CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_PWM01_EN_Msk | CLK_APBCLK_PWM23_EN_Msk | CLK_APBCLK_PWM45_EN_Msk;

    /* Select UART clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_S_Msk) | CLK_CLKSEL1_UART_S_XTAL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P0 multi-function pins for PWM Channel 5  */
    SYS->P0_MFP = SYS_MFP_P04_PWM5;
    /* Set P2 multi-function pins for PWM Channel 0~4  */
    SYS->P2_MFP = SYS_MFP_P22_PWM0 | SYS_MFP_P23_PWM1 | SYS_MFP_P24_PWM2 | SYS_MFP_P25_PWM3 | SYS_MFP_P26_PWM4;


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

    printf("\nThis sample code will output PWM channel 0 to with different\n");
    printf("frequency and duty, enable dead zone function of all PWM pairs.\n");
    printf("And also enable/disable PWM output every 1 second.\n");
    // PWM0 frequency is 100Hz, duty 30%,
    PWM_ConfigOutputChannel(PWM, 0, 100, 30);
    PWM_EnableDeadZone(PWM, 0, 400);

    // PWM2 frequency is 300Hz, duty 50%
    PWM_ConfigOutputChannel(PWM, 2, 300, 50);
    PWM_EnableDeadZone(PWM, 2, 200);

    // PWM4 frequency is 500Hz, duty 70%
    PWM_ConfigOutputChannel(PWM, 4, 600, 70);
    PWM_EnableDeadZone(PWM, 4, 100);

    // Enable output of all PWM channels
    PWM_EnableOutput(PWM, 0x3F);

    // Enable PWM channel 0 period interrupt, use channel 0 to measure time.
    PWM_EnablePeriodInt(PWM, 0, 0);
    NVIC_EnableIRQ(PWM_IRQn);

    // Start
    PWM_Start(PWM, 0x3F);

    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


