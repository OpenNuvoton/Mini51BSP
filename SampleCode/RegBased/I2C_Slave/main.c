/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/10/12 8:09p $
 * @brief    Demonstrate how to set I2C in Slave mode to receive the data of a Master.
 *           This sample code needs to work with I2C_MASTER.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

uint32_t slave_buff_addr;
uint8_t g_u8SlvData[256];
uint8_t g_au8RxData[3];
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8DeviceAddr;
uint8_t g_au8TxData[3];
uint8_t g_u8RxData;
uint8_t g_u8DataLen;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static I2C_FUNC s_I2CHandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C->I2CSTATUS;

    if (I2C->I2CTOC & I2C_I2CTOC_TIF_Msk)
    {
        /* Clear I2C Timeout Flag */
        I2C->I2CTOC |= I2C_I2CTOC_TIF_Msk;
    }
    else
    {
        if (s_I2CHandlerFn != NULL)
            s_I2CHandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8DataLen = 0;
        I2C->I2CON |= I2C_I2CON_AA_Msk | I2C_I2CON_SI_Msk;
    }
    else if (u32Status == 0x80)                     /* Previously address with own SLA address Data has been received; ACK has been returned*/
    {
        g_au8RxData[g_u8DataLen] = I2C->I2CDAT;
        g_u8DataLen++;

        if (g_u8DataLen == 2)
        {
            slave_buff_addr = (g_au8RxData[0] << 8) + g_au8RxData[1];
        }

        if (g_u8DataLen == 3)
        {
            g_u8SlvData[slave_buff_addr] = g_au8RxData[2];
            g_u8DataLen = 0;
        }
        I2C->I2CON |= I2C_I2CON_AA_Msk | I2C_I2CON_SI_Msk;
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        I2C->I2CDAT = g_u8SlvData[slave_buff_addr];
        slave_buff_addr++;
        I2C->I2CON |= I2C_I2CON_AA_Msk | I2C_I2CON_SI_Msk;
    }
    else if (u32Status == 0xC0)                    /* Data byte or last data in I2CDAT has been transmitted Not ACK has been received */
    {
        I2C->I2CON |= I2C_I2CON_AA_Msk | I2C_I2CON_SI_Msk;
    }
    else if (u32Status == 0x88)                   /* Previously addressed with own SLA address; NOT ACK has been returned */
    {
        g_u8DataLen = 0;
        I2C->I2CON |= I2C_I2CON_AA_Msk | I2C_I2CON_SI_Msk;
    }
    else if (u32Status == 0xA0)                    /* A STOP or repeated START has been received while still addressed as Slave/Receiver*/
    {
        g_u8DataLen = 0;
        I2C->I2CON |= I2C_I2CON_AA_Msk | I2C_I2CON_SI_Msk;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

void SYS_Init(void)
{
    int32_t i32TimeOutCnt;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    while(SYS->RegLockAddr != SYS_RegLockAddr_RegUnLock_Msk)
    {
        SYS->RegLockAddr = 0x59;
        SYS->RegLockAddr = 0x16;
        SYS->RegLockAddr = 0x88;
    }

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK->PWRCON |= CLK_PWRCON_XTL12M | CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    i32TimeOutCnt = __HSI / 200; /* About 5ms */
    while((CLK->CLKSTATUS & (CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk)) !=
            (CLK_CLKSTATUS_XTL_STB_Msk | CLK_CLKSTATUS_IRC22M_STB_Msk))
    {
        if(i32TimeOutCnt-- <= 0)
            break;
    }

    /* Switch HCLK clock source to XTL, STCLK to XTL */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_XTAL | CLK_CLKSEL0_HCLK_S_XTAL;

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_I2C_EN_Msk | CLK_APBCLK_UART_EN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_XTAL;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP = SYS_MFP_P01_RXD | SYS_MFP_P00_TXD;

    /* Set P3.4 and P3.5 for I2C SDA and SCL */
    SYS->P3_MFP = SYS_MFP_P34_SDA | SYS_MFP_P35_SCL;

    /* Lock protected registers */
    SYS->RegLockAddr = 0;

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART_RST_Msk;

    /* Configure UART and set UART Baudrate */
    UART->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk | (((__XTAL + (115200/2)) / 115200)-2);
    UART->LCR = 0x3 | (0x0 << UART_LCR_PBE_Pos) | (0x0 << UART_LCR_NSB_Pos) ;
}

void I2C_Init(void)
{
    /* Reset I2C */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_I2C_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C_RST_Msk;

    /* Enable I2C Controller */
    I2C->I2CON |= I2C_I2CON_ENSI_Msk;

    /* I2C clock divider, I2C Bus Clock = 100kHz */
    I2C->I2CLK = 0x1D;

    /* Set I2C 4 Slave Addresses */
    I2C->I2CADDR0 = (0x15 << 1);
    I2C->I2CADDR1 = (0x35 << 1);
    I2C->I2CADDR2 = (0x55 << 1);
    I2C->I2CADDR3 = (0x75 << 1);

    I2C->I2CADM0  = (0x1 << 1);
    I2C->I2CADM1  = (0x4 << 1);
    I2C->I2CADM2  = (0x1 << 1);
    I2C->I2CADM3  = (0x4 << 1);

    /* Enable I2C interrupt */
    I2C->I2CON |= I2C_I2CON_EI_Msk;
    NVIC_EnableIRQ(I2C_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|               I2C Driver Sample Code(Slave)           |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C */
    I2C_Init();

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C, I2C_SI | I2C_AA);

    for (i = 0; i < 0x100; i++)
    {
        g_u8SlvData[i] = 0;
    }

    /* I2C function to Slave receive/transmit data */
    s_I2CHandlerFn=I2C_SlaveTRx;

    printf("\n");
    printf("I2C Slave Mode is Running.\n");

    while(1);
}
