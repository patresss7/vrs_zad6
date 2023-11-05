/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "gpio.h"
#include "string.h"
#include "usart.h"
#include "dma.h"
#include "string.h"
#include "../Drivers/HTS221/HTS221.h"
#include "../Drivers/LP22HB/LP22HB.h"

// I2C slave device useful information
#define 	LSM6DSL_DEVICE_ADDRESS		0xD7U
#define 	LSM6DSL_WHO_AM_I_VALUE		0x6AU
#define 	LSM6DSL_WHO_AM_I_ADDRES		0x0FU


void SystemClock_Config(void);


#define START_SIGN 	'#'
#define END_SIGN	'$'
#define MAX_MSG_LEN	35

int processedSignsCount = 0;
uint8_t transimissionEnabled = 0;
char messageBuffer[MAX_MSG_LEN];
int messageBufferIndex = 0;
letter_count_ thisLetterCount;
char messageToBeSent[128] = {'a','a','\0'};
char statusMessage[128];
float temp,humid,pressure,altitude;


int main(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  USART2_RegisterCallback(proccesDmaData);
  USART2_PutBuffer("start\n", strlen("start\n"));
  uint8_t hts_good = hts221_init();
  uint8_t lp_good = lp22hb_init();

  uint8_t *buffer;
  uint8_t len = 0;

  if(hts_good)
  {
	  USART2_PutBuffer("HTS good\n", strlen("HTS good\n"));
  }
  else
  {
	  USART2_PutBuffer("HTS bad", strlen("HTS bad"));
  }

  LL_mDelay(100);

  if(lp_good)
  {
	  USART2_PutBuffer("LP good\n", strlen("LP good\n"));

  }
  else
  {
	  USART2_PutBuffer("LP bad", strlen("LP bad"));
  }

  while (1)
  {
//	  if(i2c_master_read_byte(LSM6DSL_DEVICE_ADDRESS, LSM6DSL_WHO_AM_I_ADDRES) == LSM6DSL_WHO_AM_I_VALUE)
//	  {
//		  LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_3);
//	  }


	  temp = hts221_get_temperature();
	  humid = hts221_get_humidity();
	  pressure = lp22hb_get_pressure();
	  altitude = lp22hb_calculate_altitude(pressure);


	  buffer = malloc(32*sizeof(uint8_t));
	  len = sprintf(buffer, "%05.1f,%02.0f,%07.2f,%06.2f\n", temp, humid, pressure, altitude);
	  USART2_PutBuffer(buffer,len);
	  free(buffer);

	  LL_mDelay(1000);
  }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(8000000);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

void proccesDmaData(uint8_t sign)
{
	if(transimissionEnabled)
		{
			if(sign >= 'a' && sign <= 'z')
			{
				thisLetterCount.small_letter++;
				processedSignsCount++;
				messageBuffer[messageBufferIndex++] = sign;
			}
			else if(sign >= 'A' && sign <= 'Z')
			{
				thisLetterCount.capital_letter++;
				processedSignsCount++;
				messageBuffer[messageBufferIndex++] = sign;
			}
			else if(sign == END_SIGN)
			{
				transimissionEnabled = 0;
				processedSignsCount = 0;
				sprintf(messageToBeSent,"Valid string: %s, lower-case: %d, upper-case: %d \r\n",messageBuffer,thisLetterCount.small_letter,thisLetterCount.capital_letter);
				thisLetterCount.small_letter = 0;
				thisLetterCount.capital_letter = 0;
				memset(messageBuffer, 0, MAX_MSG_LEN);
				messageBufferIndex = 0;
			}
			else
			{
				processedSignsCount++;
				messageBuffer[messageBufferIndex++] = sign;
			}
		}
		if(processedSignsCount > MAX_MSG_LEN)
		{
			transimissionEnabled = 0;
			processedSignsCount = 0;
			thisLetterCount.small_letter = 0;
			thisLetterCount.capital_letter = 0;
			memset(messageBuffer, 0, MAX_MSG_LEN);
			messageBufferIndex = 0;
		}
		if(sign == START_SIGN)
		{
			transimissionEnabled = 1;
		}
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
