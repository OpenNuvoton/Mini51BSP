/**************************************************************************//**
 * @file     i2c_hardware.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 13/10/01 9:57a $
 * @brief    MINI51 series hardware I2C driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Mini51Series.h"
#include "i2c_hardware.h"

uint8_t Tx_Data[4]= {0xaa,0x22,0x33,0x44};
uint8_t Rx_Data[5];
uint8_t DataLen;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C);

    switch(u32Status)
    {
    /* Slave Transmitter Mode */
    case 0xC0:                        /* DATA has been transmitted and NACK has been returned */
    case 0xC8:                        /* DATA has been transmitted and ACK has been returned */
        I2C_SET_CONTROL_REG(I2C, I2C_SI | I2C_AA);
        printf("Slave Transmitter Success\n");
        break;

    case 0xA8:                        /* SLA+R has been received and ACK has been returned */
    case 0xB0:
        DataLen = 0;
    case 0xB8:                        /* DATA has been transmitted and ACK has been returned */
        I2C_SET_DATA(I2C, Tx_Data[DataLen++]);
        if(DataLen<sizeof(Tx_Data))
            I2C_SET_CONTROL_REG(I2C, I2C_SI | I2C_AA);
        else
            I2C_SET_CONTROL_REG(I2C, I2C_SI);
        break;

    /* Slave Receiver Mode*/
    case 0x68:                        /* SLA+W has been received and ACK has been returned */
    case 0x60:
        DataLen = 0;
        Rx_Data[0] = 0;
        I2C_SET_CONTROL_REG(I2C, I2C_SI | I2C_AA);
        break;
    case 0x80:                        /* DATA has been received and ACK has been returned */
        Rx_Data[DataLen++] = I2C_GET_DATA(I2C);
        if(DataLen<(sizeof(Rx_Data)-1))
            I2C_SET_CONTROL_REG(I2C, I2C_SI | I2C_AA);
        else
            I2C_SET_CONTROL_REG(I2C, I2C_SI);
        break;
    case 0x88:                        /* DATA has been received and NACK has been returned */
        Rx_Data[DataLen++] = I2C_GET_DATA(I2C);
        I2C_SET_CONTROL_REG(I2C, I2C_SI | I2C_AA);
        break;

    case 0xA0:                      /* STOP or Repeat START has been received */
        I2C_SET_CONTROL_REG(I2C, I2C_SI | I2C_AA);
        printf("Slave Receiver Success\n");
        break;
    }
}

void InitI2C_HW(void)
{
    /* Set P3.4 and P3.5 for I2C SDA and SCL */
    SYS->P3_MFP |= SYS_MFP_P34_SDA | SYS_MFP_P35_SCL;

    /* Open I2C module and set bus clock */
    I2C_Open(I2C, 120);

    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C, 3, 0x75, 0);   /* Slave Address : 0x75 */

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C);
    NVIC_EnableIRQ(I2C_IRQn);

    /* I2C as slave */
    I2C_SET_CONTROL_REG(I2C, I2C_SI | I2C_AA);
}

