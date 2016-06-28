/****************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 6 $
 * $Date: 15/10/06 11:49a $
 * @brief    Transmit and receive data from PC terminal through RS232 interface.
 *
 * @note
 * Copyright (C) 2011 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini51Series.h"


#define RXBUFSIZE 1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8SendData[12] = {0};
uint8_t g_u8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;
volatile int32_t g_i32pointer = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART_TEST_HANDLE(void);
void UART_FunctionTest(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS->RegLockAddr = 0x59;
    SYS->RegLockAddr = 0x16;
    SYS->RegLockAddr = 0x88;

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCON &= ~CLK_PWRCON_XTLCLK_EN_Msk;
    CLK->PWRCON |= (0x1 << CLK_PWRCON_XTLCLK_EN_Pos); // XTAL12M (HXT) Enabled

    /* Waiting for 12MHz clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL_STB_Msk));


    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART_EN_Msk; // UART Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART_S_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART1 RXD and TXD  */
    SYS->P1_MFP &= ~(0x00000404 | 0x00000808);
    SYS->P1_MFP |= (0x00000400 | 0x00000800);

    /* Set P0 multi-function pins for UART1 RTS and CTS */
    SYS->P0_MFP &= ~(0x01000101 | 0x02000202);
    SYS->P0_MFP |= (0x00000100 | 0x00000200);

    /* Lock protected registers */
    SYS->RegLockAddr = 0;

}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk |
                 (((__XTAL + (115200/2)) / 115200)-2);

    UART->LCR = 0x3 | (0x0 << UART_LCR_PBE_Pos) | (0x0 << UART_LCR_NSB_Pos) ;
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to Hyper-Terminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART for printf */
    UART_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------+\n");
    printf("| UART function test |\n");
    printf("+--------------------+\n");

    UART_FunctionTest();

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE()
{
    uint8_t u8InChar=0xFF;
    uint32_t u32IntSts= UART->ISR;

    if(u32IntSts & UART_ISR_RDA_INT_Msk) {
        printf("\nInput:");

        /* Get all the input characters */
        while(UART->ISR & UART_ISR_RDA_IF_Msk) {
            /* Get the character from UART Buffer */
            u8InChar = UART->RBR;

            printf("%c ", u8InChar);

            if(u8InChar == '0') {
                g_bWait = FALSE;
            }

            /* Check if buffer full */
            if(g_u32comRbytes < RXBUFSIZE) {
                /* En-queue the character */
                g_u8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE-1)) ? 0 : (g_u32comRtail+1);
                g_u32comRbytes++;
            }
        }
        printf("\nTransmission Test:");
    }

    if(u32IntSts & UART_ISR_THRE_INT_Msk) {
        uint16_t tmp;
        tmp = g_u32comRtail;
        if(g_u32comRhead != tmp) {
            u8InChar = g_u8RecData[g_u32comRhead];
            UART->THR = u8InChar;
            g_u32comRhead = (g_u32comRhead == (RXBUFSIZE-1)) ? 0 : (g_u32comRhead+1);
            g_u32comRbytes--;
        }
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest()
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Function Test                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    Please enter any to start     (Press '0' to exit)      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART and PC.
        UART is set to debug port. UART is enable RDA and RLS interrupt.
        When inputting char to terminal screen, RDA interrupt will happen and
        UART will print the received char on screen.
    */

    /* Enable Interrupt and install the call back function */
    UART->IER |= UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_RTO_IEN_Msk ;
    NVIC_EnableIRQ(UART_IRQn);
    while(g_bWait);

    /* Disable Interrupt */
    UART->IER &= ~(UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_RTO_IEN_Msk);
    NVIC_DisableIRQ(UART_IRQn);
    g_bWait =TRUE;
    printf("\nUART Sample Demo End.\n");

}


