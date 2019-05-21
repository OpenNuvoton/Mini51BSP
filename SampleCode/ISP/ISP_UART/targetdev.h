/******************************************************************************
 * @file     targetdev.h
 * @brief    General UART ISP slave header file
 * @version  1.0.0
 * @date     22, Sep, 2014
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "Mini51Series.h"

/* rename for uart_transfer.c */
#define UART_T                          UART
#define UART_T_IRQHandler       UART_IRQHandler
#define UART_T_IRQn                 UART_IRQn
