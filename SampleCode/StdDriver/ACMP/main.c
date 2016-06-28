/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 5 $
 * $Date: 15/10/06 1:19p $
 * @brief    Demonstrate Analog comparator (ACMP) comparison by comparing 
 *           CPP0 (P1.5) with Band-gap voltage and shows the result on UART console.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

/* Function prototype declaration */
void SYS_Init(void);

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART, 115200);


    printf("\n\n");
    printf("+---------------------------------------+\n");
    printf("|         Mini51 ACMP Sample Code       |\n");
    printf("+---------------------------------------+\n");

    printf("\nThis sample code demonstrates ACMP0 function. Using ACMP0_P (P1.5) as ACMP0\n");
    printf("positive input and using internal band-gap voltage as the negative input\n");
    printf("The compare result reflects on ACMP0_O (P3.6)\n");

    /* Configure ACMP0. Enable ACMP0 and select internal reference voltage as negative input. */
    ACMP_Open(ACMP, 0, ACMP_VNEG_BANDGAP, ACMP_HYSTERESIS_DISABLE);
    /* Enable ACMP0 interrupt function */
    ACMP_ENABLE_INT(ACMP, 0);

    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP_IRQn);

    while(1);

}

void ACMP_IRQHandler(void)
{
    static uint32_t u32Cnt = 0;

    /* Clear ACMP 0 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP, 0);
    /* Check Comparator 0 Output Status */
    if(ACMP_GET_OUTPUT(ACMP, 0))
        printf("ACMP0_P voltage > Band-gap voltage (%d)\n", u32Cnt);
    else
        printf("ACMP0_P voltage <= Band-gap voltage (%d)\n", u32Cnt);

    u32Cnt++;
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Register write-protection disabled */
    SYS_UnlockReg();

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable external 12MHz XTAL (UART), internal 22.1184MHz */
    CLK->PWRCON = CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Enable UART and ACMP clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_CMP_EN_Msk;

    /* Select UART clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_S_Msk) | CLK_CLKSEL1_UART_S_XTAL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1.5 multi-function pin for ACMP0 positive input pin */
    SYS->P1_MFP = SYS_MFP_P15_CPP0;

    /* Disable digital input path of analog pin ACMP0_P to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(P1, (1 << 5));

    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Register write-protection enabled */
    SYS_LockReg();
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


