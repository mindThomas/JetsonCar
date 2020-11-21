/* Copyright (C) 2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include "ProcessorInit.h"
#include "main.h"
#include "cmsis_os.h"
#include "usbd_core.h"

/* Configure clocks:
	- CPU (SYSCLK) = 168 MHz
	- HCLK to AHB bus, core, memory and DMA = 168 MHz
	- SysTick timer clock = 168 MHz
	- APB1 (PCLK1) Periphiral clock = 42 MHz
	- APB2 (PCLK2) Periphiral clock = 84 MHz
	- APB1 timer clock = 84 MHz
	- APB2 timer clock = 168 MHz
*/

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 224;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Enter_DFU_Bootloader(void)
{
	//call this at any time to initiate a reboot into bootloader
	//__disable_irq();

	// Important to stop current USB periphiral before entering DFU bootloader (see "EnterBootloader_Callback" in MainTask.cpp)

	//HAL_SuspendTick();
	/* Disable SysTick Interrupt */
    //SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    //taskENTER_CRITICAL();
    //portDISABLE_INTERRUPTS();
    *BOOTLOADER_MAGIC_ADDR = BOOTLOADER_MAGIC_TOKEN;

	//(*(__IO uint32_t *) (BKPSRAM_BASE + 0)) = A_VALUE;
    NVIC_SystemReset();
}
