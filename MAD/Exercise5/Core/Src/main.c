/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t adc_val = 0;
GPIO_TypeDef* GPIO_Ports[] = {GPIOF,GPIOE,GPIOE,GPIOF};
uint16_t GPIO_Pins[] = {GPIO_PIN_13, GPIO_PIN_9,GPIO_PIN_11,GPIO_PIN_14};
#define ADC_RESOLUTION 4096;
#define VREF 3.3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void displayHex(uint32_t adc)
{
	char output[10];
	float vin = (float)adc/ 4096 * 3.3;
	sprintf(output, "0x%08x \n\r" ,adc);
	HAL_UART_Transmit(&huart3, (uint8_t*) output, strlen(output), 1000);

}

void displayLED(int value)
{
	int range = 4095/5;

	if(value >=0 && value <= range)
	{
		HAL_GPIO_WritePin(GPIO_Ports[0], GPIO_Pins[0], RESET);
		HAL_GPIO_WritePin(GPIO_Ports[1], GPIO_Pins[1], RESET);
		HAL_GPIO_WritePin(GPIO_Ports[2], GPIO_Pins[2], RESET);
		HAL_GPIO_WritePin(GPIO_Ports[3], GPIO_Pins[3], RESET);

	}
	else if(value >= range && value <= range*2)
		{
			HAL_GPIO_WritePin(GPIO_Ports[0], GPIO_Pins[0], SET);
			HAL_GPIO_WritePin(GPIO_Ports[1], GPIO_Pins[1], RESET);
			HAL_GPIO_WritePin(GPIO_Ports[2], GPIO_Pins[2], RESET);
			HAL_GPIO_WritePin(GPIO_Ports[3], GPIO_Pins[3], RESET);

		}
	else if(value >= range*2 && value <= range*3)
			{
				HAL_GPIO_WritePin(GPIO_Ports[0], GPIO_Pins[0], SET);
				HAL_GPIO_WritePin(GPIO_Ports[1], GPIO_Pins[1], SET);
				HAL_GPIO_WritePin(GPIO_Ports[2], GPIO_Pins[2], RESET);
				HAL_GPIO_WritePin(GPIO_Ports[3], GPIO_Pins[3], RESET);

			}
	else if(value >= range*3 && value <= range*4)
				{
					HAL_GPIO_WritePin(GPIO_Ports[0], GPIO_Pins[0], SET);
					HAL_GPIO_WritePin(GPIO_Ports[1], GPIO_Pins[1], SET);
					HAL_GPIO_WritePin(GPIO_Ports[2], GPIO_Pins[2], SET);
					HAL_GPIO_WritePin(GPIO_Ports[3], GPIO_Pins[3], RESET);

				}
	else if(value >= range*4)
	{
		HAL_GPIO_WritePin(GPIO_Ports[0], GPIO_Pins[0], SET);
		HAL_GPIO_WritePin(GPIO_Ports[1], GPIO_Pins[1], SET);
		HAL_GPIO_WritePin(GPIO_Ports[2], GPIO_Pins[2], SET);
		HAL_GPIO_WritePin(GPIO_Ports[3], GPIO_Pins[3], SET);

	}
}

int average_8(int x)
{
	static int samples[8];
	static int i = 0;
	static int total = 0;

	total += x - samples[i];
	samples[i] =x;

	i = (i==7? 0: i+1);
	return total>>3;
}

int average_16(int x)
{
	static int samples[16];
	static int i = 0;
	static int total = 0;

	total += x - samples[i];
	samples[i] =x;

	i = (i==15? 0: i+1);
	return total>>4;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_ADC_Start(&hadc1);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  while(HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK){}
	  adc_val = HAL_ADC_GetValue(&hadc1);
	  displayHex(average_16(adc_val));
	  displayLED(average_16(adc_val));
	  HAL_Delay(10);
;  }
  /* USER CODE END 3 */
}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
