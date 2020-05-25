/**************************************************************************//**
 * @file     sys.h
 * @version  V1.00
 * $Revision: 17 $
 * $Date: 15/09/25 9:20a $
 * @brief    Mini51 series SYS driver header file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_SYS_Driver SYS Driver
  @{
*/

/** @addtogroup MINI51_SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_RST  ((0x4<<24) | SYS_IPRSTC2_ADC_RST_Pos  ) /*!< ADC  reset is one of the SYS_ResetModule parameter */
#define ACMP_RST ((0x4<<24) | SYS_IPRSTC2_ACMP_RST_Pos ) /*!< ACMP reset is one of the SYS_ResetModule parameter */
#define PWM_RST  ((0x4<<24) | SYS_IPRSTC2_PWM_RST_Pos  ) /*!< PWM  reset is one of the SYS_ResetModule parameter */
#define UART_RST ((0x4<<24) | SYS_IPRSTC2_UART_RST_Pos ) /*!< UART reset is one of the SYS_ResetModule parameter */
#define SPI_RST  ((0x4<<24) | SYS_IPRSTC2_SPI_RST_Pos  ) /*!< SPI  reset is one of the SYS_ResetModule parameter */
#define I2C_RST  ((0x4<<24) | SYS_IPRSTC2_I2C_RST_Pos  ) /*!< I2C  reset is one of the SYS_ResetModule parameter */
#define TMR1_RST ((0x4<<24) | SYS_IPRSTC2_TMR1_RST_Pos ) /*!< TMR1 reset is one of the SYS_ResetModule parameter */
#define TMR0_RST ((0x4<<24) | SYS_IPRSTC2_TMR0_RST_Pos ) /*!< TMR0 reset is one of the SYS_ResetModule parameter */
#define GPIO_RST ((0x4<<24) | SYS_IPRSTC2_GPIO_RST_Pos ) /*!< GPIO reset is one of the SYS_ResetModule parameter */


/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCR_BOD_RST_EN              (1UL<<SYS_BODCR_BOD_RSTEN_Pos)    /*!< Brown-out Reset Enable */
#define SYS_BODCR_BOD_INTERRUPT_EN        (0UL<<SYS_BODCR_BOD_RSTEN_Pos)    /*!< Brown-out Interrupt Enable */
#define SYS_BODCR_BOD_DISABLE             (3UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Disable Brown Out Detector */
#define SYS_BODCR_BOD_VL_4_4V             (3UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 4.4V */
#define SYS_BODCR_BOD_VL_3_7V             (2UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 3.7V */
#define SYS_BODCR_BOD_VL_2_7V             (1UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.7V */
#define SYS_BODCR_BOD_VL_2_2V             (0UL<<SYS_BODCR_BOD_VL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.2V */


/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_MFP_TYPE_Msk(bit)       (1UL << ((bit) +16)) /*!< TYPE mask for Multiple Function Port */
#define SYS_MFP_ALT_Msk(bit)        (1UL << ((bit) + 8)) /*!< ALT mask for Multiple Function Port */
#define SYS_MFP_MFP_Msk(bit)        (1UL << ((bit)    )) /*!< MFP mask for Multiple Function Port */

#define SYS_MFP_P00_GPIO    0x00000000UL /*!< P0_MFP pin 0 setting for GPIO */
#define SYS_MFP_P00_CTS     0x00000100UL /*!< P0_MFP pin 0 setting for CTS  */
#define SYS_MFP_P00_TXD     0x00000101UL /*!< P0_MFP pin 0 setting for TXD  */
#define SYS_MFP_P00_Msk     0x01000101UL /*!< P0_MFP pin 0 mask             */

#define SYS_MFP_P01_GPIO    0x00000000UL /*!< P0_MFP pin 1 setting for GPIO  */
#define SYS_MFP_P01_SPISS   0x00000002UL /*!< P0_MFP pin 1 setting for SPISS */
#define SYS_MFP_P01_RTS     0x00000200UL /*!< P0_MFP pin 1 setting for RTS   */
#define SYS_MFP_P01_RXD     0x00000202UL /*!< P0_MFP pin 1 setting for RXD   */
#define SYS_MFP_P01_Msk     0x02000202UL /*!< P0_MFP pin 1 mask              */

#define SYS_MFP_P04_GPIO    0x00000000UL /*!< P0_MFP pin 4 setting for GPIO   */
#define SYS_MFP_P04_SPISS   0x00001000UL /*!< P0_MFP pin 4 setting for SPISS1 */
#define SYS_MFP_P04_PWM5    0x00001010UL /*!< P0_MFP pin 4 setting for PWM5   */
#define SYS_MFP_P04_Msk     0x00001010UL /*!< P0_MFP pin 4 mask               */

#define SYS_MFP_P05_GPIO    0x00000000UL /*!< P0_MFP pin 5 setting for GPIO   */
#define SYS_MFP_P05_MOSI    0x00002000UL /*!< P0_MFP pin 5 setting for MOSI   */
#define SYS_MFP_P05_Msk     0x00002020UL /*!< P0_MFP pin 5 mask               */

#define SYS_MFP_P06_GPIO    0x00000000UL /*!< P0_MFP pin 6 setting for GPIO   */
#define SYS_MFP_P06_MISO    0x00004000UL /*!< P0_MFP pin 6 setting for MISO   */
#define SYS_MFP_P06_Msk     0x00004040UL /*!< P0_MFP pin 6 mask               */

#define SYS_MFP_P07_GPIO    0x00000000UL /*!< P0_MFP pin 7 setting for GPIO    */
#define SYS_MFP_P07_SPICLK  0x00008000UL /*!< P0_MFP pin 7 setting for SPICLK  */
#define SYS_MFP_P07_Msk     0x00008080UL /*!< P0_MFP pin 7 mask                */

#define SYS_MFP_P10_GPIO    0x00000000UL /*!< P1_MFP pin 0 setting for GPIO */
#define SYS_MFP_P10_AIN1    0x00000001UL /*!< P1_MFP pin 0 setting for AIN1 */
#define SYS_MFP_P10_CPP0    0x00000101UL /*!< P1_MFP pin 0 setting for CPP0 */
#define SYS_MFP_P10_Msk     0x00000101UL /*!< P1_MFP pin 0 mask             */

#define SYS_MFP_P12_GPIO    0x00000000UL /*!< P1_MFP pin 2 setting for GPIO */
#define SYS_MFP_P12_AIN2    0x00000004UL /*!< P1_MFP pin 2 setting for AIN2 */
#define SYS_MFP_P12_RXD     0x00000400UL /*!< P1_MFP pin 2 setting for RXD  */
#define SYS_MFP_P12_CPP0    0x00000404UL /*!< P1_MFP pin 2 setting for CPP0 */
#define SYS_MFP_P12_Msk     0x00000404UL /*!< P1_MFP pin 2 mask             */

#define SYS_MFP_P13_GPIO    0x00000000UL /*!< P1_MFP pin 3 setting for GPIO */
#define SYS_MFP_P13_AIN3    0x00000008UL /*!< P1_MFP pin 3 setting for AIN3 */
#define SYS_MFP_P13_TXD     0x00000800UL /*!< P1_MFP pin 3 setting for TXD  */
#define SYS_MFP_P13_CPP0    0x00000808UL /*!< P1_MFP pin 3 setting for CPP0 */
#define SYS_MFP_P13_Msk     0x00000808UL /*!< P1_MFP pin 3 mask             */

#define SYS_MFP_P14_GPIO    0x00000000UL /*!< P1_MFP pin 4 setting for GPIO   */
#define SYS_MFP_P14_AIN4    0x00000010UL /*!< P1_MFP pin 4 setting for AIN4   */
#define SYS_MFP_P14_CPN0    0x00001010UL /*!< P1_MFP pin 4 setting for CPN0   */
#define SYS_MFP_P14_Msk     0x00001010UL /*!< P1_MFP pin 4 mask               */

#define SYS_MFP_P15_GPIO    0x00000000UL /*!< P1_MFP pin 5 setting for GPIO   */
#define SYS_MFP_P15_AIN5    0x00000020UL /*!< P1_MFP pin 5 setting for AIN5   */
#define SYS_MFP_P15_CPP0    0x00002020UL /*!< P1_MFP pin 5 setting for CPP0   */
#define SYS_MFP_P15_Msk     0x00002020UL /*!< P1_MFP pin 5 mask               */

#define SYS_MFP_P22_GPIO    0x00000000UL /*!< P2_MFP pin 2 setting for GPIO */
#define SYS_MFP_P22_PWM0    0x00000400UL /*!< P2_MFP pin 2 setting for PWM0 */
#define SYS_MFP_P22_Msk     0x00000404UL /*!< P2_MFP pin 2 mask             */

#define SYS_MFP_P23_GPIO    0x00000000UL /*!< P2_MFP pin 3 setting for GPIO */
#define SYS_MFP_P23_PWM1    0x00000800UL /*!< P2_MFP pin 3 setting for PWM1 */
#define SYS_MFP_P23_Msk     0x00000808UL /*!< P2_MFP pin 3 mask             */

#define SYS_MFP_P24_GPIO    0x00000000UL /*!< P2_MFP pin 4 setting for GPIO */
#define SYS_MFP_P24_PWM2    0x00001000UL /*!< P2_MFP pin 4 setting for PWM2 */
#define SYS_MFP_P24_Msk     0x00001010UL /*!< P2_MFP pin 4 mask             */

#define SYS_MFP_P25_GPIO    0x00000000UL /*!< P2_MFP pin 5 setting for GPIO */
#define SYS_MFP_P25_PWM3    0x00002000UL /*!< P2_MFP pin 5 setting for PWM3 */
#define SYS_MFP_P25_Msk     0x00002020UL /*!< P2_MFP pin 5 mask             */

#define SYS_MFP_P26_GPIO    0x00000000UL /*!< P2_MFP pin 6 setting for GPIO */
#define SYS_MFP_P26_PWM4    0x00004000UL /*!< P2_MFP pin 6 setting for PWM4 */
#define SYS_MFP_P26_CPO1    0x00004040UL /*!< P2_MFP pin 6 setting for CPO1 */
#define SYS_MFP_P26_Msk     0x00004040UL /*!< P2_MFP pin 6 mask             */

#define SYS_MFP_P30_GPIO    0x00000000UL /*!< P3_MFP pin 0 setting for GPIO */
#define SYS_MFP_P30_CPN1    0x00000100UL /*!< P3_MFP pin 0 setting for CPN1 */
#define SYS_MFP_P30_AIN6    0x00000101UL /*!< P3_MFP pin 0 setting for AIN6 */
#define SYS_MFP_P30_Msk     0x00000101UL /*!< P3_MFP pin 0 mask             */

#define SYS_MFP_P31_GPIO    0x00000000UL /*!< P3_MFP pin 1 setting for GPIO */
#define SYS_MFP_P31_CPP1    0x00000200UL /*!< P3_MFP pin 1 setting for CPP1 */
#define SYS_MFP_P31_AIN7    0x00000202UL /*!< P3_MFP pin 1 setting for AIN7 */
#define SYS_MFP_P31_Msk     0x00000202UL /*!< P3_MFP pin 1 mask             */

#define SYS_MFP_P32_GPIO    0x00000000UL /*!< P3_MFP pin 2 setting for GPIO  */
#define SYS_MFP_P32_INT0    0x00000004UL /*!< P3_MFP pin 2 setting for /INT0 */
#define SYS_MFP_P32_T0EX    0x00000400UL /*!< P3_MFP pin 2 setting for T0EX  */
#define SYS_MFP_P32_STADC   0x00000404UL /*!< P3_MFP pin 2 setting for STADC */
#define SYS_MFP_P32_CPP1    0x01000000UL /*!< P3_MFP pin 2 setting for CPP1  */
#define SYS_MFP_P32_Msk     0x01000404UL /*!< P3_MFP pin 2 mask              */

#define SYS_MFP_P34_GPIO    0x00000000UL /*!< P3_MFP pin 4 setting for GPIO */
#define SYS_MFP_P34_T0      0x00000010UL /*!< P3_MFP pin 4 setting for T0   */
#define SYS_MFP_P34_SDA     0x00001000UL /*!< P3_MFP pin 4 setting for SDA  */
#define SYS_MFP_P34_CPP1    0x00001010UL /*!< P3_MFP pin 4 setting for CPP1 */
#define SYS_MFP_P34_Msk     0x00001010UL /*!< P3_MFP pin 4 mask             */

#define SYS_MFP_P35_GPIO    0x00000000UL /*!< P3_MFP pin 5 setting for GPIO */
#define SYS_MFP_P35_T1      0x00000020UL /*!< P3_MFP pin 5 setting for T1   */
#define SYS_MFP_P35_SCL     0x00002000UL /*!< P3_MFP pin 5 setting for SCL  */
#define SYS_MFP_P35_CPP1    0x00002020UL /*!< P3_MFP pin 5 setting for CPP1 */
#define SYS_MFP_P35_Msk     0x00002020UL /*!< P3_MFP pin 5 mask             */

#define SYS_MFP_P36_GPIO    0x00000000UL /*!< P3_MFP pin 6 setting for GPIO */
#define SYS_MFP_P36_T1EX    0x00000040UL /*!< P3_MFP pin 6 setting for T1EX */
#define SYS_MFP_P36_CKO     0x00004000UL /*!< P3_MFP pin 6 setting for CKO  */
#define SYS_MFP_P36_CPO0    0x00004040UL /*!< P3_MFP pin 6 setting for CPO0 */
#define SYS_MFP_P36_Msk     0x00004040UL /*!< P3_MFP pin 6 mask             */

#define SYS_MFP_P46_GPIO    0x00000000UL /*!< P4_MFP pin 6 setting for GPIO    */
#define SYS_MFP_P46_ICE_CLK 0x00000040UL /*!< P4_MFP pin 6 setting for ICE_CLK */
#define SYS_MFP_P46_Msk     0x00004040UL /*!< P4_MFP pin 6 mask                */

#define SYS_MFP_P47_GPIO    0x00000000UL /*!< P4_MFP pin 7 setting for GPIO    */
#define SYS_MFP_P47_ICE_DAT 0x00000080UL /*!< P4_MFP pin 7 setting for ICE_DAT */
#define SYS_MFP_P47_Msk     0x00008080UL /*!< P4_MFP pin 7 mask                */

#define SYS_MFP_P50_GPIO    0x00000000UL /*!< P5_MFP pin 0 setting for GPIO */
#define SYS_MFP_P50_XTAL1   0x00000001UL /*!< P5_MFP pin 0 setting for XTAL1*/
#define SYS_MFP_P50_Msk     0x00000101UL /*!< P5_MFP pin 0 mask             */

#define SYS_MFP_P51_GPIO    0x00000000UL /*!< P5_MFP pin 1 setting for GPIO */
#define SYS_MFP_P51_XTAL2   0x00000002UL /*!< P5_MFP pin 1 setting for XTAL2*/
#define SYS_MFP_P51_Msk     0x00000202UL /*!< P5_MFP pin 1 mask             */

#define SYS_MFP_P52_GPIO    0x00000000UL /*!< P5_MFP pin 2 setting for GPIO */
#define SYS_MFP_P52_INT1    0x00000004UL /*!< P5_MFP pin 2 setting for /INT1*/
#define SYS_MFP_P52_Msk     0x00000404UL /*!< P5_MFP pin 2 mask             */

#define SYS_MFP_P53_GPIO    0x00000000UL /*!< P5_MFP pin 3 setting for GPIO */
#define SYS_MFP_P53_AIN0    0x00000008UL /*!< P5_MFP pin 3 setting for AIN0 */
#define SYS_MFP_P53_Msk     0x00000808UL /*!< P5_MFP pin 3 mask             */

#define SYS_MFP_P54_GPIO    0x00000000UL /*!< P5_MFP pin 4 setting for GPIO */
#define SYS_MFP_P54_Msk     0x00001010UL /*!< P5_MFP pin 4 mask             */

#define SYS_MFP_P55_GPIO    0x00000000UL /*!< P5_MFP pin 5 setting for GPIO */
#define SYS_MFP_P55_Msk     0x00002020UL /*!< P5_MFP pin 5 mask             */



/*@}*/ /* end of group MINI51_SYS_EXPORTED_CONSTANTS */

/** @addtogroup MINI51_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

/**
  * @brief      Clear Brown-out detector interrupt flag
  * @param      None
  * @return     None
  * @details    This macro clear Brown-out detector interrupt flag.
  */
#define SYS_CLEAR_BOD_INT_FLAG()        (SYS->BODCTL |= SYS_BODCR_BOD_INTF_Msk)

/**
  * @brief      Set Brown-out detector function to normal mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to normal mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_CLEAR_BOD_LPM()             (SYS->BODCTL &= ~SYS_BODCR_BOD_LPM_Msk)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD()               (SYS->BODCTL = (SYS->BODCTL &~(SYS_BODCR_BOD_VL_Msk|SYS_BODCR_BOD_VL_EXT_Msk))|SYS_BODCR_BOD_DISABLE)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function and set BOD_LV=3.7V
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD()                (SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCR_BOD_VL_Msk) | SYS_BODCR_BOD_VL_3_7V)

/**
  * @brief      Get Brown-out detector interrupt flag
  * @param      None
  * @retval     0   Brown-out detect interrupt flag is not set.
  * @retval     >=1 Brown-out detect interrupt flag is set.
  * @details    This macro get Brown-out detector interrupt flag.
  */
#define SYS_GET_BOD_INT_FLAG()          (SYS->BODCTL & SYS_BODCR_BOD_INTF_Msk)

/**
  * @brief      Get Brown-out detector status
  * @param      None
  * @retval     0   System voltage is higher than BOD threshold voltage setting or BOD function is disabled.
  * @retval     >=1 System voltage is lower than BOD threshold voltage setting.
  * @details    This macro get Brown-out detector output status.
  *             If the BOD function is disabled, this function always return 0.
  */
#define SYS_GET_BOD_OUTPUT()            (SYS->BODCTL & SYS_BODCR_BOD_OUT_Msk)

/**
  * @brief      Enable Brown-out detector interrupt function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector interrupt function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_BOD_RST()           (SYS->BODCTL &= ~SYS_BODCR_BOD_RSTEN_Msk)

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_BOD_RST()            (SYS->BODCTL |= SYS_BODCR_BOD_RSTEN_Msk)

/**
  * @brief      Set Brown-out detector function low power mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to low power mode.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LPM()               (SYS->BODCTL |= SYS_BODCR_BOD_LPM_Msk)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCR_BOD_VL_4_4V
  *             - \ref SYS_BODCR_BOD_VL_3_7V
  *             - \ref SYS_BODCR_BOD_VL_2_7V
  *             - \ref SYS_BODCR_BOD_VL_2_2V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
  *             The write-protection function should be disabled before using this macro.
  */
#define SYS_SET_BOD_LEVEL(u32Level) (SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCR_BOD_VL_Msk) | u32Level)

/**
  * @brief      Get reset source is from Brown-out detector reset
  * @param      None
  * @retval     0   Previous reset source is not from Brown-out detector reset
  * @retval     >=1 Previous reset source is from Brown-out detector reset
  * @details    This macro get previous reset source is from Brown-out detect reset or not.
  */
#define SYS_IS_BOD_RST()                (SYS->RSTSRC & SYS_RSTSRC_RSTS_BOD_Msk)

/**
  * @brief      Get reset source is from CPU reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU reset
  * @retval     >=1 Previous reset source is from CPU reset
  * @details    This macro get previous reset source is from CPU reset.
  */
#define SYS_IS_CPU_RST()                (SYS->RSTSRC & SYS_RSTSRC_RSTS_CPU_Msk)

/**
  * @brief      Get reset source is from Power-on Reset
  * @param      None
  * @retval     0   Previous reset source is not from Power-on Reset
  * @retval     >=1 Previous reset source is from Power-on Reset
  * @details    This macro get previous reset source is from Power-on Reset.
  */
#define SYS_IS_POR_RST()                (SYS->RSTSRC & SYS_RSTSRC_RSTS_POR_Msk)

/**
  * @brief      Get reset source is from reset pin reset
  * @param      None
  * @retval     0   Previous reset source is not from reset pin reset
  * @retval     >=1 Previous reset source is from reset pin reset
  * @details    This macro get previous reset source is from reset pin reset.
  */
#define SYS_IS_RSTPIN_RST()             (SYS->RSTSRC & SYS_RSTSRC_RSTS_RESET_Msk)

/**
  * @brief      Get reset source is from system reset
  * @param      None
  * @retval     0   Previous reset source is not from system reset
  * @retval     >=1 Previous reset source is from system reset
  * @details    This macro get previous reset source is from system reset.
  */
#define SYS_IS_SYSTEM_RST()             (SYS->RSTSRC & SYS_RSTSRC_RSTS_MCU_Msk)

/**
  * @brief      Get reset source is from window watch dog reset
  * @param      None
  * @retval     0   Previous reset source is not from window watch dog reset
  * @retval     >=1 Previous reset source is from window watch dog reset
  * @details    This macro get previous reset source is from window watch dog reset.
  */
#define SYS_IS_WDT_RST()                (SYS->RSTSRC & SYS_RSTSRC_RSTS_WDT_Msk)

/**
  * @brief      Disable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_POR()               (SYS->PORCR = 0x5AA5)

/**
  * @brief      Enable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_POR()                (SYS->PORCR = 0)

/**
  * @brief      Clear reset source flag
  * @param[in]  u32RstSrc is reset source. Including :
  *             - \ref SYS_RSTSRC_RSTS_POR_Msk
  *             - \ref SYS_RSTSRC_RSTS_RESET_Msk
  *             - \ref SYS_RSTSRC_RSTS_WDT_Msk
  *             - \ref SYS_RSTSRC_RSTS_BOD_Msk
  *             - \ref SYS_RSTSRC_RSTS_MCU_Msk
  *             - \ref SYS_RSTSRC_RSTS_CPU_Msk
  * @return     None
  * @details    This macro clear reset source flag.
  */
#define SYS_CLEAR_RST_SOURCE(u32RstSrc) (SYS->RSTSRC = u32RstSrc )


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
void SYS_LockReg(void);
void SYS_UnlockReg(void);
uint32_t  SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);

/*@}*/ /* end of group MINI51_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_SYS_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SYS_H__
