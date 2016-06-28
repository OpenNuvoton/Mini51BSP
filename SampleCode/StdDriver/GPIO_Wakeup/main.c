/**************************************************************************//**
 * @file     main.c
 * @version  V2.10
 * $Date: 15/10/12 8:06p $ 
 * @brief    Show how to wake up system from Power-down mode by GPIO interrupt.
 *
 * @note
 * Copyright (C) 2012 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "mini51series.h"
#include "GPIO.h"

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
        
    }else
    {
        /* Un-expected interrupt. Just clear all PORT0, PORT1 interrupts */
        P0->ISRC = P0->ISRC;
        P1->ISRC = P1->ISRC;
        printf("Un-expected interrupts. \n");
    }
}



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
           
    /* Set P3 multi-function pins for Clock Output */
    SYS->P3_MFP = SYS_MFP_P36_CKO;

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
    GPIO_SetMode(P1, BIT5, GPIO_PMD_INPUT);
    GPIO_EnableInt(P1, 5, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPIO01_IRQn);

   /* Waiting for interrupts */
   while (1)
   {
        printf("Enter to Power-Down ......\n");
        while(!UART_IS_TX_EMPTY(UART0));
        CLK_PowerDown();
        printf("System waken-up done.\n\n");
   }

}


