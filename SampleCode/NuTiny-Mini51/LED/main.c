/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 15/10/06 11:04a $
 * @brief    This sample toggles P3.6 to turn on board LED on and off
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "mini51series.h"
#include "gpio.h"

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P5.0 and P5.1 -> XTAL  */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK->PWRCON |= CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_XTAL,CLK_CLKDIV_HCLK(1));

    /* STCLK to XTL STCLK to XTL */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLK_S_XTAL);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART_MODULE,CLK_CLKSEL1_UART_S_XTAL,CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_RXD | SYS_MFP_P00_TXD);

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(SYS_IPRSTC2_UART_RST_Msk);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART, 115200);

}

void delay_loop(void)
{
    __IO uint32_t j;

    for(j=0; j<60000; j++);
    for(j=0; j<60000; j++);
    for(j=0; j<60000; j++);
    for(j=0; j<60000; j++);
}



int main(void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART_Init();

    printf("+---------------------------------+\n");
    printf("|    Mini51 NuTiny Sample Code    |\n");
    printf("+---------------------------------+\n");

    /*set P3.6 to output mode */
    GPIO_SetMode(P3, BIT6, GPIO_PMD_OUTPUT);        // For NuTiny-SDK-Mini51L
    /*set P2.4 or P3.6 to output mode */
    //GPIO_SetMode(P2, BIT4, GPIO_PMD_OUTPUT);        // For NuTiny-SDK-Mini51F

    while(1) {
        P36 = 0;            // For NuTiny-SDK-Mini51L
        //P24 = 0;          // For NuTiny-SDK-Mini51F
        delay_loop();
        P36 = 1;            // For NuTiny-SDK-Mini51L
        //P24 = 1;          // For NuTiny-SDK-Mini51F
        delay_loop();
    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
