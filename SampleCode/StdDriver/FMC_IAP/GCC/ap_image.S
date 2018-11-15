;/**************************************************************************//**
; * @file     ap_images.S
; * @version  V1.00
; * $Revision: 1 $
; * $Date: 18/06/21 4:41p $
; * @brief    Embedded LDROM_code.bin image into aprom_main.
; *
; * @note
; * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/

.syntax unified
.arch   armv7-m

.globl  loaderImage1Base
.globl  loaderImage1Limit

.align  4

.text

loaderImage1Base:
.incbin  "../../LDROM_code.bin"
loaderImage1Limit:
.space   4

.end

    
