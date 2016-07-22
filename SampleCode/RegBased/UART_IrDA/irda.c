/****************************************************************************
 * @file     IrDA.c
 * @version  V2.00
 * $Date: 13/11/07 4:40p $
 * @brief    Mini51 Series UART Interface Controller Driver Sample Code
 *
 * @note
 * Copyright (C) 2012 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define RXBUFSIZE 1024

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionTxTest(void);
void IrDA_FunctionRxTest(void);
void IrDA_FunctionTest(void);

extern char GetChar(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  IrDA Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionTest()
{
    uint8_t u8item;

    printf("+-------------------------------------------------------------+\n");
    printf("|     Pin Configure                                           |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  ______                                      _______        |\n");
    printf("| |      |                                    |       |       |\n");
    printf("| |Master|---TXD0(pin46) <====> RXD0(pin45)---|Slave  |       |\n");
    printf("| |      |                                    |       |       |\n");
    printf("| |______|                                    |_______|       |\n");
    printf("|                                                             |\n");
    printf("+-------------------------------------------------------------+\n");

    printf("\n\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     IrDA Function Test                                      |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Please enable semihosted to show messages on debug session |\n");
    printf("|  Keil users must define DEBUG_ENABLE_SEMIHOST in both C/C++ |\n");
    printf("|  and Asm preprocessor symbols.                              |\n");
    printf("|  IAR users must define DEBUG_ENABLE_SEMIHOST in both C/C++  |\n");
    printf("|  Compiler and Assembler preprocessor symbols.               |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Description :                                              |\n");
    printf("|    The sample code needs two boards. One is Master and      |\n");
    printf("|    the other is slave.  Master will send data based on      |\n");
    printf("|    terminal input and Slave will printf received data on    |\n");
    printf("|    terminal screen.                                         |\n");
    printf("|  Please select Master or Slave test                         |\n");
    printf("|  [0] Master    [1] Slave                                    |\n");
    printf("+-------------------------------------------------------------+\n\n");
    u8item = GetChar();

    if(u8item=='0')
        IrDA_FunctionTxTest();
    else
        IrDA_FunctionRxTest();

    printf("\nIrDA Sample Code End.\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/*  IrDA Function Transmit Test                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionTxTest()
{
    uint8_t u8OutChar;

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     IrDA Function Tx Mode Test                            |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Input char by UART terminal.                         |\n");
    printf("| 2). UART will send a char according to step 1.           |\n");
    printf("| 3). Return step 1. (Press '0' to exit)                    |\n");
    printf("+-----------------------------------------------------------+\n\n");

    printf("\nIRDA Sample Code Start. \n");

    /* In IrDA Mode, Baud Rate configuration must be used MODE0*/
    UART->BAUD = (((12000000 + (57600*8)) / 57600 >> 4)-2) ;

    /* IrDA Function Enable */
    UART->FUN_SEL = (0x2 << UART_FUN_SEL_FUN_SEL_Pos);

    /* Set IrDA Tx mode */
    UART->IRCR = UART_IRCR_TX_SELECT_Msk;

    /* Wait Terminal input to send data to UART TX pin */
    do {
        u8OutChar = GetChar();
        printf("   Input: %c , Send %c out\n",u8OutChar,u8OutChar);
        UART->THR = u8OutChar;
    } while(u8OutChar !='0');

}

/*---------------------------------------------------------------------------------------------------------*/
/*  IrDA Function Receive Test                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionRxTest()
{
    uint8_t u8InChar=0xFF;

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     IrDA Function Rx Mode Test                            |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Polling RDA_Flag to check data input though UART     |\n");
    printf("| 2). If received data is '0', the program will exit.       |\n");
    printf("|     Otherwise, print received data on terminal            |\n");
    printf("+-----------------------------------------------------------+\n\n");

    /* In IrDA Mode, Baud Rate configuration must be used MODE0*/
    UART->BAUD = (((12000000 + (57600*8)) / 57600 >> 4)-2) ;

    /* IrDA Function Enable */
    UART->FUN_SEL = (0x2 << UART_FUN_SEL_FUN_SEL_Pos);

    /* Set IrDA Rx mode */
    UART->IRCR = (1 << UART_IRCR_INV_RX_Pos);

    printf("Waiting...\n");

    /* Use polling method to wait master data */
    do {
        if(UART->ISR & UART_ISR_RDA_IF_Msk) {
            u8InChar = UART->RBR;
            printf("   Input: %c \n",u8InChar);
        }
    } while(u8InChar !='0');

}
