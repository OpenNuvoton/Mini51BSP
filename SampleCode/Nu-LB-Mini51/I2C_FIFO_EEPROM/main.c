/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * $Revision: 3 $
 * $Date: 15/10/06 11:22a $
 * @brief    This  sample  demonstrates  how  to  read/write  EEPROM
 *           via I2C interface using FIFO mode.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Mini51Series.h"
#include "LCD_Driver.h"

#define EEPROM_READ_ADDR      0xA1 /* Address of slave for read  */
#define EEPROM_WRITE_ADDR     0xA0 /* Address of slave for write */
uint8_t WBuf[3], RBuf[3];

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
    CLK_EnableModuleClock(I2C_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART_MODULE,CLK_CLKSEL1_UART_S_XTAL,CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3.4 and P3.5 for I2C SDA and SCL */
    SYS->P3_MFP = SYS_MFP_P34_SDA | SYS_MFP_P35_SCL;

    /* Lock protected registers */
    SYS_LockReg();

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

void ACK_Polling(void)
{
    uint32_t u32Status;

    /* Disable FIFO mode , don't need FIFO here */
    I2C_DISABLE_FIFO(I2C);

    do
    {
        /* Send start */
        I2C_START(I2C);                             // S
        I2C_WAIT_READY(I2C);                        // (INT), S

        /* Send control byte */
        I2C_SET_DATA(I2C, EEPROM_WRITE_ADDR);       // ConByte(W)
        I2C_SET_CONTROL_REG(I2C, I2C_SI);
        I2C_WAIT_READY(I2C);                        // (INT), ConByte(W)
        u32Status = I2C_GET_STATUS(I2C);
        I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI); // STOP
    }
    while( u32Status!= 0x18);

    /* Enable FIFO mode again */
    I2C_ENABLE_FIFO(I2C);
}

void EEPROM_Write(void)
{
    /* Send start */
    I2C_START(I2C);                             // S
    I2C_WAIT_READY(I2C);                        // (INT), S

    /* Send control byte */
    I2C_SET_DATA(I2C, EEPROM_WRITE_ADDR);       // (DATA), ConByte(W)
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_SET_DATA(I2C, (0x00 >> 8) & 0xFFUL);    // (DATA), Add-H
    I2C_WAIT_READY(I2C);                        // (INT), ConByte(W)

    I2C_SET_DATA(I2C, 0x01 & 0xFFUL);           // (DATA), Add-L
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), Add-H

    I2C_SET_DATA(I2C, WBuf[0]);                 // (DATA), data0
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), Add-L

    I2C_SET_DATA(I2C, WBuf[1]);                 // (DATA), data1
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), data0

    I2C_SET_DATA(I2C, WBuf[2]);                 // (DATA), data2
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), data1

    I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI); // STOP
    I2C_WAIT_READY(I2C);                        // (INT), data2

    I2C_SET_CONTROL_REG(I2C, I2C_SI);
}

void EEPROM_Read(void)
{
    /* Send start */
    I2C_START(I2C);                             // S
    I2C_WAIT_READY(I2C);                        // (INT), S

    /* Send control byte */
    I2C_SET_DATA(I2C, EEPROM_WRITE_ADDR);       // (DATA), ControlByte-Write
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_SET_DATA(I2C, (0x00 >> 8) & 0xFFUL);    // (DATA), Add-H
    I2C_WAIT_READY(I2C);                        // (INT), Con-W

    I2C_SET_DATA(I2C, 0x01 & 0xFFUL);           // (DATA), Add-L
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), Add-H

    I2C_SET_CONTROL_REG(I2C, I2C_STA | I2C_SI); // Sr
    I2C_WAIT_READY(I2C);                        // (INT), ADD-L

    I2C_SET_DATA(I2C, EEPROM_READ_ADDR);        // (DATA), ControlByte-Read
    I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), Sr

    I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), ConrtolByte-Read

    I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), data0
    RBuf[0] = I2C_GET_DATA(I2C);                // (DATA), data0

    I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), data1
    RBuf[1] = I2C_GET_DATA(I2C);                // (DATA), data1

    I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI); // STOP
    I2C_WAIT_READY(I2C);                        // (INT), data2
    RBuf[2] = I2C_GET_DATA(I2C);                // (DATA), data2

    I2C_SET_CONTROL_REG(I2C, I2C_SI);
}

int main(void)
{
    uint32_t i;
    uint8_t u8String[17];

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init LCD */
    LCD_Init();
    LCD_EnableBackLight();
    LCD_ClearScreen();

    /* Setup write buffer */
    WBuf[0] = 0x11;
    WBuf[1] = 0x22;
    WBuf[2] = 0x33;

    /* Open I2C and set to 100k */
    I2C_Open(I2C, 100000);

    sprintf((void *)u8String, "I2C Speed:%dkHz", I2C_GetBusClockFreq(I2C)/1000);
    LCD_Print(0, (void *)u8String);

    /* Enable FIFO mode */
    I2C_ENABLE_FIFO(I2C);

    /* Write data to EEPROM */
    EEPROM_Write();

    /* Polling ACK from EEPROM */
    ACK_Polling();

    /* Read data from EEPROM*/
    EEPROM_Read();

    /* Check receive buffer */
    for(i=0; i<3; i++)
    {
        if(WBuf[i] != RBuf[i])
            sprintf((void *)u8String, "Data-%d Error!", i);
        else
            sprintf((void *)u8String, "Data-%d OK!", i);
        LCD_Print(i+1, (void *)u8String);
    }

    /* Disable FIFO mode */
    I2C_DISABLE_FIFO(I2C);
    I2C_Close(I2C);
}
