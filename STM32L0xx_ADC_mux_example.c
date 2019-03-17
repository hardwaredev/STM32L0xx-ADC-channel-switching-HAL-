/********************************************************************************
  * @file    STM32L0xx_ADC_mux_example.c
  * @author  Lev Tarasenko // linkedin.com/in/lev-tarasenko
  * @brief   this file provides a simple example of ADC Chanel switching for
  *          modified ADC HAL library "stm32l0xx_hal_adc.c" for STM32L0xx targets.
  *            -Initialise appropriate ADC channel pins in ANALOG mode as in "GPIO_Init()"
  *            -Replace "stm32l0xx_hal_adc.c" in Src folder.
  *          
 ********************************************************************************/

#include "main.h"
#include "stm32l0xx_hal.h"


uint32_t ADC_val;

ADC_HandleTypeDef hadc;

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
   
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

   
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

    
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
   
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
static void ADC_Init(void)
{

  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  HAL_ADC_Init(&hadc);
   
}
static void GPIO_Init(void)
{
/**
 * This function initialises GPIO pin 0 and GPIO pin 1 from GPIO port A
 * This pins are defined as ADC_CHANNEL_0 and ADC_CHANNEL_1 in ADC multiplexer
**/
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
/**
 you can add more inputs here by replacing "X"-pin "Y"-port
    GPIO_InitStruct.Pin = GPIO_PIN_"X";
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO"Y", &GPIO_InitStruct);
**/

}
uint32_t ADC_MEASURE(uint32_t CH)
{
/**
 * This function defines ADC channel and returns converted value
 * NOTE: function works only with modified library "stm32l0xx_hal_adc.c" from this repository.
**/
	uint16_t val=0;
	ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = CH;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  HAL_ADC_ConfigChannel(&hadc, &sConfig); 
	HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc,100);
  val = (uint32_t) HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
  HAL_Delay(50);
	return val;
}



int main(void)
{

  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  ADC_Init();
  
 while(1) 
 { 
  
	ADC_val=ADC_MEASURE(ADC_CHANNEL_0);
  HAL_Delay(300);
  ADC_val=ADC_MEASURE(ADC_CHANNEL_1);
  HAL_Delay(300);    
      
 }
 
}




