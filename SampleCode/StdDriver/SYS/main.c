/******************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Date: 15/10/06 1:21p $
 * @brief    Demonstrate how to get PDID, get and clear reset source, configure
 *           BOD, and output system clock to CKO pin with the system
 *           clock / 4 frequency
 *
 * @note
 * Copyright (C) 2012 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

#define SIGNATURE       0x125ab234
#define FLAG_ADDR       0x200007FC

extern char GetChar(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector IRQ Handler                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void BOD_IRQHandler(void)
{
    /* Clear Interrupt Flag */
    SYS->BODCTL |= SYS_BODCR_BOD_INTF_Msk;

    printf("Brown Out is Detected\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Power Down Wake Up IRQ Handler                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    /* Clear Power Down Wake Up interrupt flag */
    CLK->PWRCON |= CLK_PWRCON_PD_WU_STS_Msk;

    printf("Wake Up Interrupt is asserted\n");
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK_EnableXtalRC(CLK_PWRCON_HXT|CLK_PWRCON_HIRC_EN_Msk);

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
    SYS_ResetModule(UART_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART, 115200);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    uint32_t u32data;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    /*
        This sample code will show some function about system manager controller and clock controller:
        1. Read PDID
        2. Get and clear reset source
        3. Setting about BOD
        4. Output system clock from CKO pin, and the output frequency = system clock / 4
    */

    printf("+----------------------------------------+\n");
    printf("|    Mini51 System Driver Sample Code    |\n");
    printf("+----------------------------------------+\n");

    if (M32(FLAG_ADDR) == SIGNATURE) {
        printf("  CPU Reset success!\n");
        M32(FLAG_ADDR) = 0;
        printf("  Press any key to continue ...\n");
        GetChar();
    }

    /*---------------------------------------------------------------------------------------------------------*/
    /* Misc system function test                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Read Part Device ID */
    printf("Product ID 0x%x\n", SYS->PDID);

    /* Get reset source from last operation */
    u32data = SYS_GetResetSrc();
    printf("Reset Source 0x%x\n", u32data);

    /* Clear reset source */
    SYS_ClearResetSrc(u32data);

    /* Unlock protected registers for Brown-Out Detector settings */
    SYS_UnlockReg();

    /* Check if the write-protected registers are unlocked before BOD setting and CPU Reset */
    if (SYS->RegLockAddr != 0) {
        printf("Protected Address is Unlocked\n");
    }

    /* Enable Brown-Out Detector and Low Voltage Reset function, and set Brown-Out Detector voltage 2.7V ,
       Enable Brown-Out Interrupt function */
    SYS_EnableBOD(SYS_BODCR_BOD_INTERRUPT_EN,SYS_BODCR_BOD_VL_2_7V);

    /* Enable BOD IRQ */
    NVIC_EnableIRQ(BOD_IRQn);

    /* Write a signature work to SRAM to check if it is reset by software */
    M32(FLAG_ADDR) = SIGNATURE;
    printf("\n\n  >>> Reset CPU <<<\n");

    /* Waiting for message send out */
    UART_WAIT_TX_EMPTY(UART);

    /* Switch HCLK clock source to Internal 22MHz */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_IRC22M,0);

    /* Enable CKO and output frequency = system clock / 4 */
    CLK_EnableCKO(CLK_CLKSEL2_FRQDIV_HCLK,1,0);

    /* Reset CPU */
    SYS_ResetCPU();

}




