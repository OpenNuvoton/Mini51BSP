/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * $Revision: 8 $
 * $Date: 15/10/06 11:23a $
 * @brief    This sample code demonstrates how to use GPIO pins to
 *           simulate an I2C interface.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

#include "i2c_software_gpio.h"
#include "i2c_hardware.h"

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
    CLK_EnableModuleClock(I2C_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART_MODULE,CLK_CLKSEL1_UART_S_XTAL,CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_RXD | SYS_MFP_P00_TXD);

    /* Lock protected registers */
    SYS_LockReg();

    /* Update System Core Clock */
    SystemCoreClockUpdate();
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

int32_t main (void)
{
    uint8_t Tx_Data[6];
    uint8_t Rx_Data[6];

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("+-------------------------------------------------------+\n");
    printf("|    Software I2C sample code                           |\n");
    printf("|    SW Master                                          |\n");
    printf("|    HW Slave                                           |\n");
    printf("+-------------------------------------------------------+\n");

    printf("Initiate I2C for slave\n");
    InitI2C_HW();

    printf("Initiate software I2C for Master\n");
    I2C_SW_Open(100000);

    printf("Please connect:\n");
    printf("SW SDA - P1.4 (PIN47)  <->   HW SDA - P3.4 (PIN9) \n");
    printf("SW SCL - P1.5 (PIN2)   <->   HW SCL - P3.5 (PIN10) \n");

    Tx_Data[0]=0;
    Tx_Data[1]=0;
    Tx_Data[2]=0xA5;
    Tx_Data[3]=0xcc;
    Tx_Data[4]=0xbb;
    Tx_Data[5]=0xdd;

    printf("Access I2C slave:\n");

    I2C_SW_Send(0x15,Tx_Data,6);
    CLK_SysTickDelay(5000);
    I2C_SW_Get(0x15,Rx_Data,4);

    if((Rx_Data[0] != 0xaa) || (Rx_Data[1] != 0x22) || (Rx_Data[2] != 0x33) || (Rx_Data[3] != 0x44))
        printf("Data Error!!\n");
    else
        printf("Pass!!\n");

    return 0;
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
