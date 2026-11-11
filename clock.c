#include "stm32f1xx_hal.h"

// STM32F103CBT6 with an external 8 MHz crystal: 72 MHz SYSCLK, 48 MHz USB.
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef oscillator = {0};
  RCC_ClkInitTypeDef clocks = {0};
  RCC_PeriphCLKInitTypeDef peripherals = {0};

  oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscillator.HSEState = RCC_HSE_ON;
  oscillator.PLL.PLLState = RCC_PLL_ON;
  oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  oscillator.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
    for (;;) {}
  }

  clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clocks.APB1CLKDivider = RCC_HCLK_DIV2;
  clocks.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_2) != HAL_OK) {
    for (;;) {}
  }

  peripherals.PeriphClockSelection = RCC_PERIPHCLK_USB;
  peripherals.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&peripherals) != HAL_OK) {
    for (;;) {}
  }
}
