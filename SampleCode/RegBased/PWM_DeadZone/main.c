/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/24 7:17p $
 * @brief    Demonstrate the dead-zone feature with PWM.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


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

    /* Enable HIRC */
    CLK->PWRCON = CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_PWM01_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Set P2 multi-function pins for PWM Channel 0 and 1 */
    SYS->P2_MFP = SYS_MFP_P22_PWM0 | SYS_MFP_P23_PWM1;


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

    printf("\nThis sample code will output PWM channel 0 and 1 with\n");
    printf("1.2kHz 50%% duty waveform, and 10us deadzone\n");


    // Set channel 0 prescaler to 2. Actual value fill into register needs to minus 1.
    PWM->PPR = 0x1;
    // Set channel 0 clock divider to 1
    PWM->CSR = (PWM_CLK_DIV_1 << PWM_CSR_CSR0_Pos);
    // Enable PWM channel 0 auto-reload mode, enable Dead Zone.
    // No need to configure channel 1 because it's the inverse of channel 0 output here
    PWM->PCR = PWM_PCR_CH0MOD_Msk | PWM_PCR_DZEN01_Msk;
    /*
      Configure PWM channel 0 init period and duty.
      Period is HCLK / (prescaler * clock divider * (CNR + 1))
      Duty ratio = (CMR + 1) / (CNR + 1)
      Period = 22.1184 MHz / (2 * 1 * (9215 + 1)) =  1200 Hz
      Duty ratio = (4607 + 1) / (9215 + 1) = 50%
    */
    PWM->CMR[0] = 4607;
    PWM->CNR[0] = 9215;

    // Configure dead zone duration to 10us.
    // 22118400(clk) / 2(perscaler) * 10 * 10^-6 ~= 111
    PWM->PDZIR = 111;

    // Enable PWM channel 0 and 1 output
    PWM->POE = PWM_POE_PWM0_Msk | PWM_POE_PWM1_Msk;

    // Start
    PWM->PCR |= PWM_PCR_CH0EN_Msk;

    while(1);

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


