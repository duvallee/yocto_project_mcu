/*
 * File: stm32mp1xx_hal_msp.c
 *       This file provides code for the MSP Initialization 
 *       and de-Initialization codes.
 *
 * Written by duvallee in 2020
 *
 */
#include "main.h"


/* --------------------------------------------------------------------------
 * Name : HAL_MspInit()
 *
 *
 * -------------------------------------------------------------------------- */
void HAL_MspInit(void)
{
   __HAL_RCC_HSEM_CLK_ENABLE();
}


