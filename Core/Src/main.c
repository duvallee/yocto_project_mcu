/*
 * File: main.c
 *
 * Written by duvallee in 2020
 *
 */
#include "main.h"

// --------------------------------------------------------------------------
void SystemClock_Config(void);
void SetSystick(void);

/* --------------------------------------------------------------------------
 * Name : main()
 *
 *
 * -------------------------------------------------------------------------- */
int main(void)
{
   GPIO_InitTypeDef  gpio_init_structure = {0, };

   // ----------------------------------------------------------------------
   HAL_Init();

   if (IS_ENGINEERING_BOOT_MODE())
   {
      /* Configure the system clock */
      SystemClock_Config();
   }

   // ----------------------------------------------------------------------
   __HAL_RCC_GPIOA_CLK_ENABLE();

   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);

   gpio_init_structure.Pin = GPIO_PIN_13;
   gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
   gpio_init_structure.Pull = GPIO_PULLUP;
   gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   HAL_GPIO_Init(GPIOA, &gpio_init_structure);

   gpio_init_structure.Pin = GPIO_PIN_14;
   gpio_init_structure.Pull = GPIO_NOPULL;
   gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
   gpio_init_structure.Mode = GPIO_MODE_IT_RISING_FALLING;
   HAL_GPIO_Init(GPIOA, &gpio_init_structure);

   HAL_NVIC_SetPriority((IRQn_Type) EXTI14_IRQn, 0x01, 0x00);
   HAL_NVIC_EnableIRQ((IRQn_Type) EXTI14_IRQn);

   // ----------------------------------------------------------------------
   SetSystick();

   // ----------------------------------------------------------------------
   scheduler_init();

   // ----------------------------------------------------------------------
   while (1)
   {
      scheduler_run();
   }
}

/* --------------------------------------------------------------------------
 * Name : SetSystick()
 *
 *
 * -------------------------------------------------------------------------- */
void SetSystick(void)
{
#if 0
   // Configure the Systick interrupt time 
   HAL_SYSTICK_Config(SystemCoreClock / SYSTICK_MS_DIV);

   // Configure the Systick
   HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

   // SysTick_IRQn interrupt configuration
   HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
#endif
}


/* --------------------------------------------------------------------------
 * Name : SystemClock_Config()
 *
 *
 * -------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

   /* Configure LSE Drive Capability */
   HAL_PWR_EnableBkUpAccess();
   __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

   /* Initializes the CPU, AHB and APB busses clocks */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
   RCC_OscInitStruct.LSEState = RCC_LSE_ON;
   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
   RCC_OscInitStruct.HSICalibrationValue = 16;
   RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;

   /* PLL1 Config */
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLL12SOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLM = 3;
   RCC_OscInitStruct.PLL.PLLN = 81;
   RCC_OscInitStruct.PLL.PLLP = 1;
   RCC_OscInitStruct.PLL.PLLQ = 1;
   RCC_OscInitStruct.PLL.PLLR = 1;
   RCC_OscInitStruct.PLL.PLLFRACV = 0x800;
   RCC_OscInitStruct.PLL.PLLMODE = RCC_PLL_FRACTIONAL;
   RCC_OscInitStruct.PLL.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
   RCC_OscInitStruct.PLL.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

   /* PLL2 Config */
   RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
   RCC_OscInitStruct.PLL2.PLLM = 3;
   RCC_OscInitStruct.PLL2.PLLN = 66;
   RCC_OscInitStruct.PLL2.PLLP = 2;
   RCC_OscInitStruct.PLL2.PLLQ = 1;
   RCC_OscInitStruct.PLL2.PLLR = 1;
   RCC_OscInitStruct.PLL2.PLLFRACV = 0x1400;
   RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
   RCC_OscInitStruct.PLL2.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
   RCC_OscInitStruct.PLL2.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

   /* PLL3 Config */
   RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
   RCC_OscInitStruct.PLL3.PLLM = 2;
   RCC_OscInitStruct.PLL3.PLLN = 34;
   RCC_OscInitStruct.PLL3.PLLP = 2;
   RCC_OscInitStruct.PLL3.PLLQ = 17;
   RCC_OscInitStruct.PLL3.PLLR = 37;
   RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
   RCC_OscInitStruct.PLL3.PLLFRACV = 0x1A04;
   RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
   RCC_OscInitStruct.PLL3.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
   RCC_OscInitStruct.PLL3.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

   /* PLL4 Config */
   RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;
   RCC_OscInitStruct.PLL4.PLLM = 4;
   RCC_OscInitStruct.PLL4.PLLN = 99;
   RCC_OscInitStruct.PLL4.PLLP = 6;
   RCC_OscInitStruct.PLL4.PLLQ = 8;
   RCC_OscInitStruct.PLL4.PLLR = 8;
   RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
   RCC_OscInitStruct.PLL4.PLLFRACV = 0;
   RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
   RCC_OscInitStruct.PLL4.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
   RCC_OscInitStruct.PLL4.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
   {
      Error_Handler();
   }
   /* RCC Clock Config */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK            |
                                 RCC_CLOCKTYPE_ACLK            |
                                 RCC_CLOCKTYPE_PCLK1           |
                                 RCC_CLOCKTYPE_PCLK2           |
                                 RCC_CLOCKTYPE_PCLK3           |
                                 RCC_CLOCKTYPE_PCLK4           |
                                 RCC_CLOCKTYPE_PCLK5           |
                                 RCC_CLOCKTYPE_MPU;
   RCC_ClkInitStruct.MPUInit.MPU_Clock = RCC_MPUSOURCE_PLL1;
   RCC_ClkInitStruct.MPUInit.MPU_Div = RCC_MPU_DIV2;
   RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
   RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
   RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
   RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
   RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
   RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
   RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
   RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
   RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
   {
      Error_Handler();
   }

   /* Set the HSE division factor for RTC clock */
   __HAL_RCC_RTC_HSEDIV(24);

}

/* --------------------------------------------------------------------------
 * Name : Error_Handler()
 *
 *
 * -------------------------------------------------------------------------- */
void Error_Handler(void)
{
}

#ifdef  USE_FULL_ASSERT
/* --------------------------------------------------------------------------
 * Name : assert_failed()
 *
 *
 * -------------------------------------------------------------------------- */
void assert_failed(uint8_t *file, uint32_t line)
{ 
}
#endif /* USE_FULL_ASSERT */


