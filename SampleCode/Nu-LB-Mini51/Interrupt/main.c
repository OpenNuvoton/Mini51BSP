/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/10/06 11:23a $
 * @brief    This sample code demonstrates how to let system enter
 *           and exit deep sleep mode with GPIO interrupts
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"
#include "lcd_driver.h"


void GPIO01_IRQHandler(void)
{
    /* Re-enable debounce function */
    P1->DBEN |= (1UL << 3);

    /* Clear the interrupt */
    P1->ISRC = 1 << 3;

    printf("P1.3 Interrupt!\n");

    LCD_Print(3, "P1.3 Interrupt!");

    /* Toggle LED */
    P22 = P22 ^ 1;
}

void GPIO234_IRQHandler(void)
{
    /* Re-enable debounce function */
    P3->DBEN |= (1UL << 0);

    /* Clear the interrupt */
    P3->ISRC = 1 << 0;

    printf("P2P3P4 Interrupt!\n");

    LCD_Print(3, "P3.0 Interrupt!");

    /* Toggle LED */
    P22 = P22 ^ 1;
}

void EINT0_IRQHandler(void)
{
    /* Re-enable debounce function */
    P3->DBEN |= (1UL << 2);

    P3->ISRC = 1 << 2;

    /* Toggle LED */
    P22 = P22 ^ 1;

    printf("EINT0 Interrupt!\n");

    LCD_Print(3, "EINT0 Interrupt!");
}

void EINT1_IRQHandler(void)
{
    /* Re-enable debounce function */
    P5->DBEN |= (1UL << 2);

    P5->ISRC = 1 << 2;

    /* Toggle LED */
    P22 = P22 ^ 1;

    printf("EINT1 Interrupt!\n");

    LCD_Print(3, "EINT1 Interrupt!");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set P5.0 and P5.1 -> XTAL  */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable external 12MHz XTAL, 10kHz */
    CLK->PWRCON &= ~(CLK_PWRCON_XTLCLK_EN_Msk | CLK_PWRCON_IRC10K_EN_Msk);
    CLK->PWRCON |= (0x1 << CLK_PWRCON_XTLCLK_EN_Pos) | CLK_PWRCON_IRC10K_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_XTL_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_XTAL;

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_SPI_EN_Msk;
    /* IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_XTAL;// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD  */
    SYS->P0_MFP &= ~(SYS_MFP_P00_Msk | SYS_MFP_P01_Msk);
    SYS->P0_MFP |= (SYS_MFP_P00_TXD | SYS_MFP_P01_RXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART, 115200);
}

void GPIO_Init(void)
{
    /* Set p2.2 as output pin */
    P2->PMD = (GPIO_PMD_OUTPUT << GPIO_PMD_PMD2_Pos);

    /* Set p1.3, p3.0 as Quasi-bidirectional mode */
    P1->PMD = (GPIO_PMD_QUASI << GPIO_PMD_PMD3_Pos);
    P3->PMD = (GPIO_PMD_QUASI << GPIO_PMD_PMD0_Pos);

    /* Set p1.3 as falling edge trigger and enable its interrupt */
    GPIO_EnableInt(P1, 3, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPIO01_IRQn);

    /* Set p3.0 as low level trigger */
    GPIO_EnableInt(P3, 0, GPIO_INT_LOW);
    NVIC_EnableIRQ(GPIO234_IRQn);

    /* Debounce function control */
    GPIO->DBNCECON = GPIO_DBNCECON_ICLK_ON | GPIO_DBNCECON_DBCLKSRC_HCLK | GPIO_DBNCECON_DBCLKSEL_32768;
    P1->DBEN = (1UL << 2);
    P3->DBEN = (1UL << 0);
    P3->DBEN = (1UL << 2);
    P5->DBEN = (1UL << 2);

    /* Configure external interrupt */

    GPIO_EnableInt(P3, 2, GPIO_INT_FALLING);
    NVIC_EnableIRQ(EINT0_IRQn);

    GPIO_EnableInt(P5, 2, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(EINT1_IRQn);

}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("CPU @ %dHz\n", SystemCoreClock);

    /* Init SPI and LCD */
    LCD_Init();
    LCD_EnableBackLight();
    LCD_ClearScreen();

    LCD_Print(0, "Welcome! Nuvoton");
    LCD_Print(1, "This is INT test");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Test                                                                                 */
    /*-----------------------------------------------------------------------------------------------------*/

    printf("P13, P30, P32(INT0) and P52(INT1) are used to test interrupt\n  and control LEDs(P20)\n");

    /* Init P2.0 (output), P1.3, P2.5 (Quasi-bidirectional) and relative interrupts */
    GPIO_Init();

    /* Waiting for interrupts */
    while (1)
    {
        printf("Deep Sleep\n");

        LCD_Print(2, "Deep Sleep");
        while ((UART->FSR & UART_FSR_TE_FLAG_Msk) == 0);

        /* Disable P3.2, P5.2, p1.3, p3.0 debounce to avoid double interrupts when wakeup */
        P3->DBEN = 0;
        P1->DBEN = 0;
        P5->DBEN = 0;

        SYS_UnlockReg();
        /* Settings for power down (deep sleep) */
        /* Lock protected registers */
        SYS_LockReg();

        /* Hold in wakeup state when P3.2 or P1.3 or P3.0 is low. */
        while((P32 == 0) || (P13 == 0) || (P30 == 0));

        LCD_Print(3, "                ");

        /* Enter power down. Only INT0, INT1 can used to wakeup system */
        CLK_PowerDown();

    }

}




