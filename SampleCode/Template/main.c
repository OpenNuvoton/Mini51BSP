/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/10/06 11:26a $
 * @brief    A project template for Mini51DE MCU.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    // Enable UART clock
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Lock protected registers */
    SYS_LockReg();

}


int main()
{

    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART, 115200);

    printf("Hello World\n");


    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
