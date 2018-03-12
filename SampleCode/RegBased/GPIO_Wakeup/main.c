/**************************************************************************//**
 * @file     main.c
 * @version  V2.10
 * $Date: 15/10/12 8:04p $
 * @brief    Show how to wake up system from Power-down mode by GPIO interrupt.
 *
 * @note
 * Copyright (C) 2012 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"
#include "gpio.h"

/**
 * @brief       Port0/Port1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port0/Port1 default IRQ, declared in startup_Mini51.s.
 */
void GPIO01_IRQHandler(void)
{
    /* To check if P1.5 interrupt occurred */
    if (P1->ISRC & BIT5)
    {
        P1->ISRC = BIT5;
        printf("P1.5 INT occurred. \n");

    }
    else
    {
        /* Un-expected interrupt. Just clear all PORT0, PORT1 interrupts */
        P0->ISRC = P0->ISRC;
        P1->ISRC = P1->ISRC;
        printf("Un-expected interrupts. \n");
    }
}



void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    while(SYS->RegLockAddr != 1)
    {
        SYS->RegLockAddr = 0x59;
        SYS->RegLockAddr = 0x16;
        SYS->RegLockAddr = 0x88;
    }

    /* Enable internal RC 22.1184MHz, and  RC 10K (fro WDT) */
    CLK->PWRCON = CLK_PWRCON_IRC22M_EN_Msk | CLK_PWRCON_IRC10K_EN_Msk;

    /* Waiting for clock ready */
    while((CLK->CLKSTATUS & (CLK_CLKSTATUS_IRC22M_STB_Msk | CLK_CLKSTATUS_IRC10K_STB_Msk)) !=
            (CLK_CLKSTATUS_IRC22M_STB_Msk | CLK_CLKSTATUS_IRC10K_STB_Msk));


    /* Enable UART and WDT clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk | CLK_APBCLK_WDT_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P0_MFP = SYS_MFP_P00_TXD | SYS_MFP_P01_RXD;

    /* Lock protected registers */
    SYS->RegLockAddr = 0;
}

void UART_Init(void)
{
    // Set UART to 8 bit character length, 1 stop bit, and no parity
    UART->LCR = UART_LCR_WLS_Msk;
    // 22.1184 MHz reference clock input, for 115200 bps
    // 22118400 / 115200 = 192. Using mode 2 to calculate baudrate, 192 - 2 = 190 = 0xBE
    UART->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk | (0xBE);
}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------------------+ \n");
    printf("|    MINI51 GPIO Wake-up Sample Code   | \n");
    printf("+--------------------------------------+ \n");

    /* Configure P1.5 as Input mode and enable interrupt by rising edge trigger */
    P1->PMD = (P1->PMD & ~0xC00) | (GPIO_PMD_INPUT << 10);
    P1->IMD &= ~0x20;
    P1->IEN |= 0x200000;
    NVIC_EnableIRQ(GPIO01_IRQn);

    /* Waiting for interrupts */
    while (1)
    {
        printf("Enter to Power-Down ......\n");
        while(!((UART0->FSR & UART_FSR_TE_FLAG_Msk) >> UART_FSR_TE_FLAG_Pos));
        SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;
        CLK->PWRCON |= (CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WU_STS_Msk);
        __WFI();
        printf("System waken-up done.\n\n");
    }

}


