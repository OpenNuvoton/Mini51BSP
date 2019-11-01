/******************************************************************************
 * @file     targetdev.c
 * @brief    M5451 series ISP sample source file
 * @version  0x27
 * @date     31, December, 2014
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "targetdev.h"
#include "ISP_USER.h"

//the smallest of APROM size is 4K (4K, 8K, 16K)
uint32_t GetApromSize()
{
    uint32_t size = 0x1000, data;
    int result;

    do
    {
        result = FMC_Read_User(size, &data);

        if (result < 0)
        {
            return size;
        }
        else
        {
            size *= 2;
        }
    }
    while (1);
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    *size = 0;
    FMC_Read_User(Config0, &uData);

    if ((uData & 0x01) == 0)   //DFEN enable
    {
        FMC_Read_User(Config1, &uData);

        // filter the reserved bits in CONFIG1
        uData &= 0x000FFFFF;

        if ((uData > g_apromSize) || (uData & FMC_FLASH_PAGE_SIZE - 1))   //avoid config1 value from error
        {
            uData = g_apromSize;
        }

        *addr = uData;
        *size = g_apromSize - uData;
    }
    else
    {
        *addr = g_apromSize;
        *size = 0;
    }
}
