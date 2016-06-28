/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 15/10/06 11:24a $
 * @brief    This is a starter kit sample enables all peripherals on learning 
 *           board. Peripherals enabled are UART, SPI, I2C, Timer, ADC, and PWM
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
 #include <stdio.h>
#include "Mini51Series.h"
#include "lcd_driver.h"
#include "eeprom_24lc64.h"

void Delay(int32_t ms)
{
    int32_t i;

    for(i=0; i<ms; i++)
        CLK_SysTickDelay(1000);
}

void EINT0_IRQHandler(void)
{
    P3->ISRC = 1 << 2;

    /* Toggle PWM3 enable/disable */
    PWM->POE |= PWM_POE_PWM3_Msk;

    printf("EINT0 Interrupt!\n");

}

void TMR0_IRQHandler(void)
{
    /* Clear Interrupt */
    TIMER0->TISR = 1;

}

void ADC_Init(void)
{
    /*reset ADC */
    SYS->IPRSTC2 |= SYS_IPRSTC2_ADC_RST_Msk;
    SYS->IPRSTC2 &= (~SYS_IPRSTC2_ADC_RST_Msk);

    /* ADC clock source */
    CLK->CLKSEL1 &= (~CLK_CLKSEL1_ADC_S_Msk);
    CLK->CLKSEL1 |= CLK_CLKSEL1_ADC_S_XTAL;

    /* Set ADC divisor */
    CLK->CLKDIV &= (~CLK_CLKDIV_ADC_N_Msk);
    CLK->CLKDIV |= CLK_CLKDIV_ADC(3);

    /* ADC engine clock enable */
    CLK->APBCLK |= CLK_APBCLK_ADC_EN_Msk;

    /* ADC enable */
    ADC_POWER_ON(ADC);
}

void PWM_Init(void)
{

    CLK->APBCLK |= (CLK_APBCLK_PWM01_EN_Msk | CLK_APBCLK_PWM23_EN_Msk | CLK_APBCLK_PWM45_EN_Msk) ;
    SYS->IPRSTC2 |= SYS_IPRSTC2_PWM_RST_Msk;
    SYS->IPRSTC2 &= (~SYS_IPRSTC2_PWM_RST_Msk);

    /* PWM Timer0: Clk = HCLK / 120 / 16, Freq = clk / 6250, duty cycle = 3125/6250 % */
    /* PWM Timer1: Clk = HCLK / 120 / 16, Freq = clk / 3125, duty cycle = 1563/3125 % */
    /* PWM Timer2: Clk = HCLK / 60 / 16, Freq = clk / 3125, duty cycle = 1563/3125 % */
    /* PWM Timer3: Clk = HCLK / 60 / 1, Freq = clk / 50, duty cycle = 25/50 % */
    /* PWM0 = 12000000 / 120 / 16 / 6250 =    1Hz */
    /* PWM1 = 12000000 / 120 / 16 / 3125 =    2Hz */
    /* PWM2 = 12000000 /  60 / 16 / 3125 =    4Hz */
    /* PWM3 = 12000000 /  60 /  1 /   50 = 4000Hz */
    PWM_ConfigOutputChannel(PWM,0,1,50);
    PWM_ConfigOutputChannel(PWM,1,1,50);
    PWM_ConfigOutputChannel(PWM,2,1,50);
    PWM_ConfigOutputChannel(PWM,3,1,50);
    PWM_EnableOutput(PWM,0xF);
    PWM_Start(PWM,0xF);

    SYS->P2_MFP &= ~(SYS_MFP_P22_Msk | SYS_MFP_P23_Msk | SYS_MFP_P24_Msk | SYS_MFP_P25_Msk);
    SYS->P2_MFP |=  (SYS_MFP_P22_PWM0| SYS_MFP_P23_PWM1| SYS_MFP_P24_PWM2| SYS_MFP_P25_PWM3) ;
}

void TMR0_Init(void)
{
    TIMER_EnableInt(TIMER0);
    TIMER_SET_PRESCALE_VALUE(TIMER0,11); // 12MHz / (11+1) / 100000 = 10Hz
    TIMER_Start(TIMER0);
    TIMER_SET_CMP_VALUE(TIMER0,100000);
    NVIC_EnableIRQ(TMR0_IRQn);
}

void EINT0_Init(void)
{
    /* Debounce function control */
    GPIO->DBNCECON = GPIO_DBNCECON_ICLK_ON | GPIO_DBNCECON_DBCLKSRC_HCLK | GPIO_DBNCECON_DBCLKSEL_32768;
    GPIO_ENABLE_DEBOUNCE(P3,1<<2);

    /* Configure external interrupt */
    GPIO_EnableInt(P3, 2, GPIO_INT_FALLING);
    NVIC_EnableIRQ(EINT0_IRQn);
}

void I2C_Init(void)
{
    /* Enable I2C */
    CLK->APBCLK |= CLK_APBCLK_I2C_EN_Msk ;
    SYS->IPRSTC2 |= SYS_IPRSTC2_I2C_RST_Msk;
    SYS->IPRSTC2 &= (~SYS_IPRSTC2_I2C_RST_Msk);
    I2C->I2CON = I2C_I2CON_ENSI_Msk ;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set P5.0 and P5.1 -> XTAL  */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XTAL1 | SYS_MFP_P51_XTAL2);

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK->PWRCON = (CLK->PWRCON & ~CLK_PWRCON_XTLCLK_EN_Msk) | CLK_PWRCON_XTL12M;

    CLK_SysTickDelay(12000);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL_STB_Msk);


    /* Switch HCLK clock source to XTL, STCLK to XTL */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLK_S_Msk) | CLK_CLKSEL0_HCLK_S_XTAL;

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART_EN_Msk;

    /* IP clock source */
    CLK->CLKSEL1 = ( CLK->CLKSEL1 & (~CLK_CLKSEL1_UART_S_Msk) ) |  CLK_CLKSEL1_UART_S_XTAL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD  */
    SYS->P0_MFP &= ~(SYS_MFP_P00_Msk | SYS_MFP_P01_Msk);
    SYS->P0_MFP |= (SYS_MFP_P00_TXD | SYS_MFP_P01_RXD);

    /* Set P3.4 and P3.5 for I2C SDA and SCL */
    SYS->P3_MFP = SYS_MFP_P34_SDA | SYS_MFP_P35_SCL;

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    uint32_t u32Counter = 0, u32AdcData = 0, u32I2cData;
    char AdcValue[15] = "ADC Value:";
    char strClearAdcValue[15] = "ADC Value:     ";

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init Other peripherals                                                                                  */
    /*---------------------------------------------------------------------------------------------------------*/
    LCD_Init();

    /* Initialize I2C */
    I2C_Init();
    EEPROM_Init();
    TMR0_Init();

    /* Initialize ADC */
    ADC_Init();

    /* Initialize PWM */
    PWM_Init();

    /*
        StartKit is used to demo Nu-LB_004 learning board. This sample will test EEPROM, SPI Flash read/write by
        I2C, SPI. And test LED by GPIO and PWM. Test KEY by EINT0. Use ADC to convert the voltage of
        variable resistor by AD0. Generate 4kHz key sound by buzzer with PWM. LCD display is enabled to show working
        message. CPU will be idle by __WFI() when a loop has been done. Timer events (every 100ms) or key events are
        used to wakeup CPU to execute next loop.
    */

    LCD_EnableBackLight();
    LCD_ClearScreen();

    printf("CPU @ %dHz\n", SystemCoreClock);

    // SPI test
    LCD_Print(0, "Welcome! Nuvoton");
    LCD_Print(1, "This is LB test ");

    /*Initialize external interrupt*/
    EINT0_Init();

    while (1) {
        if(u32Counter > 8) {
            /* Disable Buzzer */
            PWM->POE &= ~PWM_POE_PWM3_Msk;
        }
        printf("\nTest time: %d\n", u32Counter++);

        /* LED test */
        P2->DOUT=u32Counter;

        /* ADC test */
        ADC->ADSR = ( ADC->ADSR & (~ADC_ADSR_CHANNEL_Msk) ) | ADC_ADSR_ADF_Msk;
        ADC->ADCR |= ADC_ADCR_ADST_Msk;
        while(!(ADC->ADSR & ADC_ADSR_ADF_Msk));
        u32AdcData = ADC->ADDR & 0xFFFUL;
        printf("ADC value: %d\n", u32AdcData);
        sprintf((char *)AdcValue + 10, "%d", u32AdcData);
        LCD_Print(3, strClearAdcValue);
        LCD_Print(3, AdcValue);
        /* Single end, single mode, start convert */
        ADC->ADCR = ADC_ADCR_ADEN_Msk | ADC_ADCR_ADST_Msk;

        /*I2C test*/
        u32AdcData = (u32Counter + u32AdcData) & 0xff;
        EEPROM_Write(u32Counter,u32AdcData);
        u32I2cData = EEPROM_Read(u32Counter);
        if (u32I2cData != u32AdcData) {
            LCD_Print(2, "I2C fail ");
            while (1);
        }
        printf("I2C address:0x%x, Data:0x%x\n", u32Counter, u32I2cData);

    }
}





