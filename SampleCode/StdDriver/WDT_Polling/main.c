/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/24 4:23p $
 * @brief    Use polling mode to check WDT time-out state and reset WDT after time out occurs.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


void WDT_IRQHandler(void)
{

    // Clear WDT interrupt flag
    WDT_CLEAR_TIMEOUT_INT_FLAG();

    // Check WDT wake up flag
    if(WDT_GET_TIMEOUT_WAKEUP_FLAG())
    {
        printf("Wake up by WDT\n");
        // Clear WDT wake up flag
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();
    }

}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable external 12MHz XTAL (UART), HIRC, and  RC 10K (fro WDT) */
    CLK->PWRCON = CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk | CLK_PWRCON_IRC10K_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk | CLK_CLKSTATUS_IRC10K_STB_Msk);

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_WDT_EN_Msk;

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

    printf("\nThis sample code demonstrate using WDT in polling mode\n");

    // WDT register is locked, so it is necessary to unlock protect register before configure WDT
    SYS_UnlockReg();

    // WDT timeout every 2^14 WDT clock, disable system reset, disable wake up system
    WDT_Open(WDT_TIMEOUT_2POW14, 0, FALSE, FALSE);
    // Enable WDT interrupt so interrupt flag raise on timeout.
    // NVIC _not_ enabled, so there'll be _no_ interrupt
    WDT_EnableInt();
    while(1)
    {
        // WDT timeout flag set
        if(WDT_GET_TIMEOUT_INT_FLAG())
        {
            // Reset WDT and clear time out flag
            WDT_CLEAR_TIMEOUT_INT_FLAG();
            printf("Reset WDT counter\n");
        }
    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


