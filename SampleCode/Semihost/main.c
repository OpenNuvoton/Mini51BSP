/****************************************************************************
* @file     main.c
* @version  V2.00
* $Revision: 4 $
* $Date: 15/10/06 11:25a $
* @brief    Show how to print and get character with IDE console window.
*
* @note
* Copyright (C) 2011 Nuvoton Technology Corp. All rights reserved.
*
******************************************************************************/

#include <stdio.h>
#include "Mini51Series.h"



/*---------------------------------------------------------------------------------------------------------*/
/* Main Function                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main()
{
    int8_t item;

    printf("\n Start SEMIHOST test: \n");

    while(1) {
        item = getchar();
        printf("%c\n",item);
    }

}





/*** (C) COPYRIGHT 2012 Nuvoton Technology Corp. ***/



