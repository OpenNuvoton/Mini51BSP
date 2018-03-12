/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/10/12 8:09p $
 * @brief    Demonstrate how a Master access Slave. This sample code needs to work with I2C_SLAVE
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8DeviceAddr;
uint8_t g_au8TxData[3];
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_I2CHandlerFn = NULL;

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
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted and prepare SLA+W */
    {
        I2C->I2CDAT = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C->I2CON |= I2C_I2CON_SI_Msk;
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C->I2CDAT = g_au8TxData[g_u8DataLen++];
        I2C->I2CON |= I2C_I2CON_SI_Msk;
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C->I2CON |= I2C_I2CON_STO_Msk | I2C_I2CON_SI_Msk;
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen != 2)
        {
            I2C->I2CDAT = g_au8TxData[g_u8DataLen++];
            I2C->I2CON |= I2C_I2CON_SI_Msk;
        }
        else
        {
            I2C->I2CON |= I2C_I2CON_STA_Msk | I2C_I2CON_SI_Msk;
        }
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C->I2CDAT = ((g_u8DeviceAddr << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
        I2C->I2CON |= I2C_I2CON_SI_Msk;
    }
    else if (u32Status == 0x40)                 /* SLA+R has been transmitted and ACK has been received */
    {
        I2C->I2CON |= I2C_I2CON_SI_Msk;
    }
    else if (u32Status == 0x58)                 /* DATA has been received and NACK has been returned */
    {
        g_u8RxData = I2C->I2CDAT;
        I2C->I2CON |= I2C_I2CON_STO_Msk | I2C_I2CON_SI_Msk;
        while(I2C->I2CON & I2C_I2CON_STO_Msk);
        g_u8EndFlag = 1;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        I2C->I2CDAT = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C->I2CON |= I2C_I2CON_SI_Msk;
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C->I2CDAT = g_au8TxData[g_u8DataLen++];
        I2C->I2CON |= I2C_I2CON_SI_Msk;
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C->I2CON |= I2C_I2CON_STO_Msk | I2C_I2CON_SI_Msk;
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen != 3)
        {
            I2C->I2CDAT = g_au8TxData[g_u8DataLen++];
            I2C->I2CON |= I2C_I2CON_SI_Msk;
        }
        else
        {
            I2C->I2CON |= I2C_I2CON_STO_Msk | I2C_I2CON_SI_Msk;
            while(I2C->I2CON & I2C_I2CON_STO_Msk);
            g_u8EndFlag = 1;
        }
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

    /* Enable I2C interrupt */
    I2C->I2CON |= I2C_I2CON_EI_Msk;
    NVIC_EnableIRQ(I2C_IRQn);
}

int32_t Read_Write_SLAVE(uint8_t slvaddr)
{
    uint32_t i;

    g_u8DeviceAddr = slvaddr;

    for (i = 0; i < 0x100; i++)
    {
        g_au8TxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8TxData[1] = (uint8_t)(i & 0x00FF);
        g_au8TxData[2] = (uint8_t)(g_au8TxData[1] + 3);

        g_u8DataLen = 0;
        g_u8EndFlag = 0;

        /* I2C function to write data to slave */
        s_I2CHandlerFn = (I2C_FUNC)I2C_MasterTx;

        /* I2C as master sends START signal */
        I2C->I2CON |= I2C_I2CON_STA_Msk;

        /* Wait I2C Tx Finish */
        while (g_u8EndFlag == 0);
        g_u8EndFlag = 0;

        /* I2C function to read data from slave */
        s_I2CHandlerFn = (I2C_FUNC)I2C_MasterRx;

        g_u8DataLen = 0;
        g_u8DeviceAddr = slvaddr;

        /* I2C as master sends START signal */
        I2C->I2CON |= I2C_I2CON_STA_Msk;

        /* Wait I2C Rx Finish */
        while (g_u8EndFlag == 0);

        /* Compare data */
        if (g_u8RxData != g_au8TxData[2])
        {
            printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8RxData);
            return -1;
        }
    }
    printf("Master Access Slave (0x%X) Test OK\n", slvaddr);
    return 0;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|       I2C Driver Sample Code(Master) for access Slave |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C */
    I2C_Init();

    /* Access Slave with no address mask */
    printf("\n");
    printf(" == No Mask Address ==\n");
    Read_Write_SLAVE(0x15);
    Read_Write_SLAVE(0x35);
    Read_Write_SLAVE(0x55);
    Read_Write_SLAVE(0x75);
    printf("SLAVE Address test OK.\n");

    /* Access Slave with address mask */
    printf("\n");
    printf(" == Mask Address ==\n");
    Read_Write_SLAVE(0x15 & ~0x01);
    Read_Write_SLAVE(0x35 & ~0x04);
    Read_Write_SLAVE(0x55 & ~0x01);
    Read_Write_SLAVE(0x75 & ~0x04);
    printf("SLAVE Address Mask test OK.\n");

    while(1);
}
