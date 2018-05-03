/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 10 $
 * $Date: 15/10/06 11:23a $
 * @brief    This sample code demonstrates how to control a LCD
 *           module via SPI interface
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "Mini51Series.h"
#include "LCD_Driver.h"

// gpio interrupt uses to change the level periodically
uint32_t g_level_change = 0;
// there are five bright levels
uint32_t brightlight_level[] = {1, 25, 50, 75, 95};

uint32_t g_au32TMRINTCount = 0;

/**
 * @brief       External INT0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT0(P3.2) default IRQ, declared in startup_Mini51.s.
 */
void EINT0_IRQHandler(void)
{
    //printf("!@$^\n");
    /* For P3.2, clear the INT flag */
    GPIO_CLR_INT_FLAG(P3,1<<2);

    if(g_level_change==4)
        g_level_change = 0;
    else
        g_level_change++;
}

/**
 * @brief       Timer-1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The TIMER1 default IRQ, declared in startup_Mini51.s.
 */
void TMR1_IRQHandler(void)
{
#if 1
    TIMER_ClearIntFlag(TIMER1);
#else
    if (TIMER_GetIntFlag(TIMER1) == 1)
    {
        /* Clear TIMER1 Timeout Interrupt Flag */
        TIMER_ClearIntFlag(TIMER1);
    }
    else if (TIMER_GetCaptureIntFlag(TIMER1) == 1)
    {
        /* Clear TIMER1 Capture Interrupt Flag */
        TIMER_ClearCaptureIntFlag(TIMER1);
    }
#endif
    g_au32TMRINTCount++;
    if( brightlight_level[g_level_change] >=g_au32TMRINTCount )
    {
        /* dark LCD */
        P54 =1;
    }
    else
    {
        /* bright LCD */
        P54 = 0;
        if(g_au32TMRINTCount>=100)
        {
            g_au32TMRINTCount=0;
        }
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable Internal RC clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M | CLK_PWRCON_OSC10K_EN_Msk | CLK_PWRCON_IRC22M_EN_Msk;

    CLK_SysTickDelay(1200);

    /* Waiting for IRC22M clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* IP clock divider */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLK_S_XTAL);

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_XTAL,CLK_CLKDIV_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);

    /* IP clock source */
    CLK->CLKSEL1 = ( CLK->CLKSEL1 & ~(CLK_CLKSEL1_UART_S_Msk|CLK_CLKSEL1_TMR1_S_Msk) ) |  CLK_CLKSEL1_UART_S_XTAL | CLK_CLKSEL1_TMR1_S_HCLK;

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

/**
  * @brief  Main routine.
  * @param  None.
  * @return None.
  */
int32_t main(void)
{
    SYS_UnlockReg();

    SYS_Init();

    /* SPI test */
    LCD_Init();
    LCD_EnableBackLight();
    LCD_ClearScreen();
    LCD_Print(0, "Welcome! Nuvoton");
    LCD_Print(1, "This is LB board");
    LCD_Print(2, "Mini51");
    LCD_Print(3, "TEST");

    // back light control pin P5.4
    GPIO_SetMode(P5,1<<4,GPIO_PMD_OUTPUT);

    /* INT button triggers P3.2 */
    GPIO_SetMode(P3,(1<<2),GPIO_PMD_OPEN_DRAIN);
    GPIO_EnableInt(P3, 2, GPIO_INT_FALLING);
    NVIC_EnableIRQ(EINT0_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBNCECON_DBCLKSRC_HCLK,GPIO_DBNCECON_DBCLKSEL_16);
    GPIO_ENABLE_DEBOUNCE(P3,1<<2);

    /* Reset and stop TIMER0, TIMER1 counting first */
    TIMER1->TCSR = TIMER_TCSR_CRST_Msk;

    /* To Configure TCMPR values based on Timer clock source and pre-scale value */
    TIMER_SET_PRESCALE_VALUE(TIMER1,0);

    /* Open TIMER1 counting and setting*/
    TIMER_Open(TIMER1,TIMER_PERIODIC_MODE,SystemCoreClock/1000);

    /* Enable TIMER1 interrupt, NVIC */
    NVIC_EnableIRQ(TMR1_IRQn);
    TIMER_EnableInt(TIMER1);

    /* Start TIMER1 */
    TIMER_Start(TIMER1);

    while(1) ;    // loop forever

}

/*** (C) COPYRIGHT 2012 Nuvoton Technology Corp. ***/



