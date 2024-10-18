/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/10/06 11:23a $
 * @brief    This sample code shows how to wake system up from idle mode with
 *           WDT interrupt
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"
#include "lcd_driver.h"


void WDT_IRQHandler(void)
{

    /* Clear WDT interrupt flag */
    WDT->WTCR |= WDT_WTCR_WTIF_Msk;
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

    /* Enable External XTAL  and IRC_22MHz */
    CLK->PWRCON &= ~(CLK_PWRCON_XTLCLK_EN_Msk);
    CLK->PWRCON |= CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk | CLK_PWRCON_IRC10K_EN_Msk;

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk);
    CLK_WaitClockReady(CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_IRC22M;

    /* Enable clock  of WDT, Timer0 and PWM01 */
    CLK->APBCLK = CLK_APBCLK_WDT_EN_Msk | CLK_APBCLK_TMR0_EN_Msk |
                  CLK_APBCLK_PWM01_EN_Msk | CLK_APBCLK_SPI_EN_Msk;

    /* Enable IP clock */
    CLK->CLKSEL1 = CLK_CLKSEL1_WDT_S_IRC10K | CLK_CLKSEL1_TMR0_S_HCLK |
                   CLK_CLKSEL1_PWM01_S_HCLK;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P2.2 => PWM0 */
    SYS->P2_MFP = SYS_MFP_P22_PWM0;

    /* Lock protected registers */
    SYS_LockReg();
}


void WDT_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* start WDT. timeout = (2^14 + 1024) * (1 / 10kHz)*/
    WDT->WTCR = (WDT_WTCR_WTE_Msk | WDT_WTCR_WTIF_Msk | WDT_WTCR_WTR_Msk |
                 (5 << WDT_WTCR_WTIS_Pos) | WDT_WTCR_WTIE_Msk | WDT_WTCR_WTWKE_Msk);

    /* Lock protected registers */
    SYS_LockReg();
    NVIC_EnableIRQ(WDT_IRQn);
}

void PWMA_Init(void)
{
    /* Enable PWM Timer 0 and set to auto-reload mode */
    PWM->PCR = PWM_PCR_CH0EN_Msk | PWM_PCR_CH0MOD_Msk;

    /* Set pre-scale for PWM01 */
    PWM->PPR = ((2-1) << PWM_PPR_CP01_Pos);

    /* Select the divider of PWM01 clock */
    PWM->CSR = (4 << PWM_CSR_CSR0_Pos);

    /* PWM0 output is HCLK / 2 / 2 */
    PWM->CNR[0] = 2 - 1;
    PWM->CMR[0] = 0;

    /* PWM output enable */
    PWM->POE = PWM_POE_PWM0_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    /* Init system, IP clock and multi-function I/O */
    SYS_Init();

    /* Init SPI and LCD */
    LCD_Init();
    LCD_EnableBackLight();
    LCD_ClearScreen();
    LCD_Print(0, "Idle Sample Code");

    /*----------------------- Enable WDT to wakeup CPU from idle ------------------*/
    WDT_Init();

    /*----------------------- Enable PWM to toggle output -------------------------*/
    PWMA_Init();


    /*
        The Idle sample uses __WFI() instruction to disable CPU clock to enter idle mode.
        In the sample code, CPU will enter idle mode to wait WDT timeout. User may check
        LCD display to know if it is wakeup.
    */

    /* Unlock protected registers */
    SYS_UnlockReg();
    while(1)
    {
        /* Select system clock source as IRC10K */
        CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_IRC10K;

        /* Update current system core clock */
        SystemCoreClockUpdate();

        /* Disable 22.1184MHz IRC and 12MHz XTL */
        CLK->PWRCON &= ~CLK_PWRCON_IRC22M_EN_Msk;

        LCD_Print(1, "Idle  ");
        /* Idle to WDT timeout. HCLK should be 10KHz here. PWM0 = 2.5KHz*/
        __WFI();
        LCD_Print(1, "Wakeup");

        /* Enable 22.1184MHz IRC */
        CLK->PWRCON |= CLK_PWRCON_IRC22M_EN_Msk;

        /* Waiting for clock ready */
        CLK_WaitClockReady(CLK_CLKSTATUS_IRC22M_STB_Msk);

        /* Use IRC22M as system clock source */
        CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_IRC22M;

        /* Update System Core Clock */
        SystemCoreClockUpdate();

        LCD_Print(1, "Idle  ");
        /* Idle to WDT timeout. HCLK should be 22.1184MHz here. PWM0 = 5.5MHz*/
        __WFI();
        LCD_Print(1, "Wakeup");
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Empty functions for reduce code size to fit into LDROM & solve the functions are not be defined.       */
/*---------------------------------------------------------------------------------------------------------*/
void ProcessHardFault()
{}

void SH_Return()
{}


