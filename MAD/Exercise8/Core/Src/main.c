/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ILI9341_Touchscreen.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "math.h"
#include "snow_tiger.h"
#include "img_view.h"
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
/* Private variables ---------------------------------------------------------*/
float t = 30.0, h = 50.0;
float R = 60, G = 40, B = 80;
uint16_t adc_val = 4095;
uint32_t count;
float bg = 100;
uint16_t bgCode = 0xFFFF;
uint16_t last_adc_val = 4095;
uint16_t x_pos, y_pos;
uint8_t step = 0;
HAL_StatusTypeDef status;
uint16_t CRC16_2(uint8_t*, uint8_t);
uint8_t cmdBuffer[3];
uint8_t dataBuffer[8];
int state = 1;
uint32_t startTime = 0;
uint16_t mixColor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache---------------------------------------------------------*/
	SCB_EnableDCache();

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
	MX_USART3_UART_Init();
	MX_SPI5_Init();
	MX_TIM1_Init();
	MX_RNG_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	ILI9341_Init(); //initial driver setup to drive ili9341
	cmdBuffer[0] = 0x03;
	cmdBuffer[1] = 0x00;
	cmdBuffer[2] = 0x04;
	ILI9341_Fill_Screen(bgCode);
	HAL_TIM_Base_Start_IT(&htim2);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		//----------------------------------------------------------PERFORMANCE TEST
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("FPS TEST, 40 loop 2 screens", 10, 10, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		uint32_t Timer_Counter = 0;
//	  		for(uint32_t j = 0; j < 2; j++)
//	  		{
//	  			HAL_TIM_Base_Start(&htim1);
//	  			for(uint16_t i = 0; i < 10; i++)
//	  			{
//	  				ILI9341_Fill_Screen(WHITE);
//	  				ILI9341_Fill_Screen(BLACK);
//	  			}
//
//	  			//20.000 per second!
//	  			HAL_TIM_Base_Stop(&htim1);
//	  			Timer_Counter += __HAL_TIM_GET_COUNTER(&htim1);
//	  			__HAL_TIM_SET_COUNTER(&htim1, 0);
//	  		}
//	  		Timer_Counter /= 2;
//
//	  		char counter_buff[30];
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		sprintf(counter_buff, "Timer counter value: %ld", Timer_Counter*2);
//	  		ILI9341_Draw_Text(counter_buff, 10, 10, BLACK, 1, WHITE);
//
//	  		double seconds_passed = 2*((float)Timer_Counter / 20000);
//	  		sprintf(counter_buff, "Time: %.3f Sec", seconds_passed);
//	  		ILI9341_Draw_Text(counter_buff, 10, 30, BLACK, 2, WHITE);
//
//	  		double timer_float = 20/(((float)Timer_Counter)/20000);	//Frames per sec
//
//	  		sprintf(counter_buff, "FPS:  %.2f", timer_float);
//	  		ILI9341_Draw_Text(counter_buff, 10, 50, BLACK, 2, WHITE);
//	  		double MB_PS = timer_float*240*320*2/1000000;
//	  		sprintf(counter_buff, "MB/S: %.2f", MB_PS);
//	  		ILI9341_Draw_Text(counter_buff, 10, 70, BLACK, 2, WHITE);
//	  		double SPI_utilized_percentage = (MB_PS/(6.25 ))*100;		//50mbits / 8 bits
//	  		sprintf(counter_buff, "SPI Utilized: %.2f", SPI_utilized_percentage);
//	  		ILI9341_Draw_Text(counter_buff, 10, 90, BLACK, 2, WHITE);
//	  		HAL_Delay(10000);
//
//
//	  		static uint16_t x = 0;
//	  		static uint16_t y = 0;
//
//	  		char Temp_Buffer_text[40];
//
//	  //----------------------------------------------------------COUNTING MULTIPLE SEGMENTS
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Counting multiple segments at once", 10, 10, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//
//	  		for(uint16_t i = 0; i <= 10; i++)
//	  		{
//	  		sprintf(Temp_Buffer_text, "Counting: %d", i);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 30, BLUE, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 50, RED, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 70, GREEN, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 90, BLACK, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 110, BLUE, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 130, RED, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 150, GREEN, 2, WHITE);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 170, WHITE, 2, BLACK);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 190, BLUE, 2, BLACK);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 210, RED, 2, BLACK);
//	  		}
//
//	  		HAL_Delay(1000);
//
//	  //----------------------------------------------------------COUNTING SINGLE SEGMENT
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Counting single segment", 10, 10, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint16_t i = 0; i <= 100; i++)
//	  		{
//	  		sprintf(Temp_Buffer_text, "Counting: %d", i);
//	  		ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 3, WHITE);
//	  		}
//
//	  		HAL_Delay(1000);
//
//	  //----------------------------------------------------------ALIGNMENT TEST
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Rectangle alignment check", 10, 10, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		ILI9341_Draw_Hollow_Rectangle_Coord(50, 50, 100, 100, BLACK);
//	  		ILI9341_Draw_Filled_Rectangle_Coord(20, 20, 50, 50, BLACK);
//	  		ILI9341_Draw_Hollow_Rectangle_Coord(10, 10, 19, 19, BLACK);
//	  		HAL_Delay(1000);
//
//	  //----------------------------------------------------------LINES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Horizontal and Vertical lines", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint32_t i = 0; i < 30000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Horizontal_Line(xr, yr, radiusr, colourr);
//	  			ILI9341_Draw_Vertical_Line(xr, yr, radiusr, colourr);
//	  		}
//
//	  		HAL_Delay(1000);
//
//	  //----------------------------------------------------------HOLLOW CIRCLES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Circles", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//
//	  		for(uint32_t i = 0; i < 3000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Hollow_Circle(xr, yr, radiusr*2, colourr);
//	  		}
//	  		HAL_Delay(1000);
//
//	  //----------------------------------------------------------FILLED CIRCLES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Filled Circles", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint32_t i = 0; i < 1000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Filled_Circle(xr, yr, radiusr/2, colourr);
//	  		}
//	  		HAL_Delay(1000);
//
//	  //----------------------------------------------------------HOLLOW RECTANGLES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Rectangles", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint32_t i = 0; i < 20000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Hollow_Rectangle_Coord(xr, yr, xr+radiusr, yr+radiusr, colourr);
//	  		}
//	  		HAL_Delay(1000);
//
//	  //----------------------------------------------------------FILLED RECTANGLES EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Filled Rectangles", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//	  		for(uint32_t i = 0; i < 20000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			uint16_t radiusr = 0;
//	  			uint16_t colourr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			radiusr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			colourr = random_num;
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			radiusr &= 0x001F;
//	  			//ili9341_drawpixel(xr, yr, WHITE);
//	  			ILI9341_Draw_Rectangle(xr, yr, radiusr, radiusr, colourr);
//	  		}
//	  		HAL_Delay(1000);
//
//	  //----------------------------------------------------------INDIVIDUAL PIXEL EXAMPLE
//
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Slow draw by selecting", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("and adressing pixels", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//
//	  		x = 0;
//	  		y = 0;
//	  		while (y < 240)
//	  		{
//	  		while ((x < 320) && (y < 240))
//	  		{
//
//	  			if(x % 2)
//	  			{
//	  				ILI9341_Draw_Pixel(x, y, BLACK);
//	  			}
//
//	  			x++;
//	  		}
//
//	  			y++;
//	  			x = 0;
//	  		}
//
//	  		x = 0;
//	  		y = 0;
//
//
//	  		while (y < 240)
//	  		{
//	  		while ((x < 320) && (y < 240))
//	  		{
//
//	  			if(y % 2)
//	  			{
//	  				ILI9341_Draw_Pixel(x, y, BLACK);
//	  			}
//
//	  			x++;
//	  		}
//
//	  			y++;
//	  			x = 0;
//	  		}
//	  		HAL_Delay(2000);
//
//	  //----------------------------------------------------------INDIVIDUAL PIXEL EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Random position and colour", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("500000 pixels", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Fill_Screen(WHITE);
//
//
//	  		for(uint32_t i = 0; i < 500000; i++)
//	  		{
//	  			uint32_t random_num = 0;
//	  			uint16_t xr = 0;
//	  			uint16_t yr = 0;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			xr = random_num;
//	  			random_num = HAL_RNG_GetRandomNumber(&hrng);
//	  			yr = random_num;
//	  			uint16_t color = HAL_RNG_GetRandomNumber(&hrng);
//
//	  			xr &= 0x01FF;
//	  			yr &= 0x01FF;
//	  			ILI9341_Draw_Pixel(xr, yr, color);
//	  		}
//	  		HAL_Delay(2000);
//
//	  //----------------------------------------------------------565 COLOUR EXAMPLE, Grayscale
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Colour gradient", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("Grayscale", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//
//
//	  		for(uint16_t i = 0; i <= (320); i++)
//	  		{
//	  			uint16_t Red = 0;
//	  			uint16_t Green = 0;
//	  			uint16_t Blue = 0;
//
//	  			Red = i/(10);
//	  			Red <<= 11;
//	  			Green = i/(5);
//	  			Green <<= 5;
//	  			Blue = i/(10);
//
//
//
//	  			uint16_t RGB_color = Red + Green + Blue;
//	  			ILI9341_Draw_Rectangle(i, x, 1, 240, RGB_color);
//
//	  		}
//	  		HAL_Delay(2000);
//
//	  //----------------------------------------------------------IMAGE EXAMPLE, Snow Tiger
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("RGB Picture", 10, 10, BLACK, 1, WHITE);
//	  		ILI9341_Draw_Text("TIGER", 10, 20, BLACK, 1, WHITE);
//	  		HAL_Delay(2000);
//	  		ILI9341_Draw_Image((const char*)snow_tiger, SCREEN_VERTICAL_2);
//	  		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
//	  		HAL_Delay(5000);
//
//
//	  //----------------------------------------------------------TOUCHSCREEN EXAMPLE
//	  		ILI9341_Fill_Screen(WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  		ILI9341_Draw_Text("Touchscreen", 10, 10, BLACK, 2, WHITE);
//	  		ILI9341_Draw_Text("Touch to draw", 10, 30, BLACK, 2, WHITE);
//	  		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
//
//
//	  		while(1)
//	  		{
//	  			HAL_Delay(20);
//
//	  			if(TP_Touchpad_Pressed())
//	          {
//
//	  					uint16_t x_pos = 0;
//	  					uint16_t y_pos = 0;
//
//
//	  					HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_SET);
//
//	            uint16_t position_array[2];
//
//	  					if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK)
//	  					{
//	  					x_pos = position_array[0];
//	  					y_pos = position_array[1];
//	  					ILI9341_Draw_Filled_Circle(x_pos, y_pos, 2, BLACK);
//
//	  					ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	  					char counter_buff[30];
//	  					sprintf(counter_buff, "POS X: %.3d", x_pos);
//	  					ILI9341_Draw_Text(counter_buff, 10, 80, BLACK, 2, WHITE);
//	  					sprintf(counter_buff, "POS Y: %.3d", y_pos);
//	  					ILI9341_Draw_Text(counter_buff, 10, 120, BLACK, 2, WHITE);
//	  					ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
//	  					}
//
//	  					ILI9341_Draw_Pixel(x_pos, y_pos, BLACK);
//
//	          }
//	  			else
//	  			{
//	  				HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);
//	  			}
//
//	  		}
//
//		HAL_ADC_Start(&hadc1);
//		while (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {
//		}
//		adc_val = HAL_ADC_GetValue(&hadc1);
//		adc_val = average_16(adc_val);
//		if (abs(last_adc_val - adc_val) > 150) {
//			last_adc_val = adc_val;
//			bg = (float) adc_val / 4095.0 * 100.0;
//			if (bg < 20) {
//				bg = 20;
//			}
//			bgCode = ((int) (31.0 / 100.0 * bg) << 11)
//					+ ((int) (63.0 / 100.0 * bg) << 5)
//					+ ((int) (31.0 / 100.0 * bg));
//			ILI9341_Fill_Screen(bgCode);
//		}
		if (state == 1) {
			display1();
		} else if (state == 2) {
			display2();
			if (count - startTime > 5000) {
				state = 1;
				ILI9341_Fill_Screen(bgCode);
			}
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 200;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3
			| RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
int average_16(int x) {
	static int samples[16];
	static int i = 0;
	static int totle = 0;

	totle += x - samples[i];
	samples[i] = x;

	i = (i == 15 ? 0 : i + 1);

	return totle >> 4;
}
uint16_t CRC16_2(uint8_t *ptr, uint8_t lenght) {
	uint16_t crc = 0xFFFF;
	uint8_t s = 0x00;

	while (lenght--) {
		crc ^= *ptr++;
		for (s = 0; s < 8; s++) {
			if ((crc & 0x01) != 0) {
				crc >>= 1;
				crc ^= 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc;
}

void display1() {
	HAL_I2C_Master_Transmit(&hi2c1, 0x5c << 1, cmdBuffer, 3, 200);
	HAL_I2C_Master_Transmit(&hi2c1, 0x5c << 1, cmdBuffer, 3, 200);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c1, 0x5c << 1, dataBuffer, 8, 200);
	uint16_t Rcrc = dataBuffer[7] << 8;
	Rcrc += dataBuffer[6];
	if (Rcrc == CRC16_2(dataBuffer, 6)) {
		uint16_t temp = ((dataBuffer[4] & 0x7F) << 8) + dataBuffer[5];
		t = temp / 10.0;
		t = (((dataBuffer[4] & 0x80) >> 7) == 1) ? (t * (-1)) : t;

		uint16_t humi = (dataBuffer[2] << 8) + dataBuffer[3];
		h = humi / 10.0;
	}

	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
	char Temp_Buffer_text[20];
	sprintf(Temp_Buffer_text, "%.1f C", t);
	ILI9341_Draw_Text(Temp_Buffer_text, 15, 20, BLACK, 2, bgCode);
	sprintf(Temp_Buffer_text, "%.1f %%RH", h);
	ILI9341_Draw_Text(Temp_Buffer_text, 170, 20, BLACK, 2, bgCode);

	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	uint16_t position_array[2];
	if (TP_Touchpad_Pressed()) {
		if (TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK) {
			y_pos = position_array[0];
			x_pos = position_array[1];
			//ILI9341_Fill_Screen(bgCode);
			y_pos = 240-y_pos;
//			sprintf(Temp_Buffer_text, "%d %d", x_pos,y_pos);
//			ILI9341_Draw_Text(Temp_Buffer_text, 10, 65, BLACK, 2, bgCode);
			if (sqrt(pow(x_pos - 30, 2) + pow(y_pos - 85, 2)) <= 22.0) {
				R += 10.0;
				if (R > 100) {
					R = 0;
					ILI9341_Fill_Screen(bgCode);
				}
			}
			if (sqrt(pow(x_pos - 30, 2) + pow(y_pos - 145, 2)) <= 22.0) {
				G += 10.0;
				if (G > 100) {
					G = 0;
					ILI9341_Fill_Screen(bgCode);
				}
			}
			if (sqrt(pow(x_pos - 30, 2) + pow(y_pos - 205, 2)) <= 22.0) {
				B += 10.0;
				if (B > 100) {
					B = 0;
					ILI9341_Fill_Screen(bgCode);
				}
			}
			if (sqrt(pow(x_pos - 130, 2) + pow(y_pos - 32, 2)) <= 25.0) {
				state = 2;
				startTime = count;
			}

		}
	}
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
	ILI9341_Draw_Filled_Circle(30, 85, 22, RED); //x y
	ILI9341_Draw_Filled_Circle(30, 145, 22, GREEN);
	ILI9341_Draw_Filled_Circle(30, 205, 22, BLUE);

	mixColor = (((int) (31.0 * R / 100.0)) << 11)
			+ (((int) (63.0 * G / 100.0)) << 5) + (((int) (31.0 * B / 100.0)));
	ILI9341_Draw_Filled_Circle(130, 32, 25, mixColor);

	int w1, w2;
	w1 = (int) (R * 1.35);
	w2 = 135 - w1;
	ILI9341_Draw_Rectangle(60, 70, w1, 30, RED);
	ILI9341_Draw_Rectangle(60 + w1, 70, w2, 30, 0xFC10);
	sprintf(Temp_Buffer_text, "%.0f %%", R);
	ILI9341_Draw_Text(Temp_Buffer_text, 205, 77, BLACK, 2, bgCode);

	w1 = (int) (G * 1.35);
	w2 = 135 - w1;
	ILI9341_Draw_Rectangle(60, 130, w1, 30, GREEN);
	ILI9341_Draw_Rectangle(60 + w1, 130, w2, 30, 0x87F0);
	sprintf(Temp_Buffer_text, "%.0f %%", G);
	ILI9341_Draw_Text(Temp_Buffer_text, 205, 137, BLACK, 2, bgCode);

	w1 = (int) (B * 1.35);
	w2 = 135 - w1;
	ILI9341_Draw_Rectangle(60, 190, w1, 30, BLUE);
	ILI9341_Draw_Rectangle(60 + w1, 190, w2, 30, 0x841F);
	sprintf(Temp_Buffer_text, "%.0f %%", B);
	ILI9341_Draw_Text(Temp_Buffer_text, 205, 197, BLACK, 2, bgCode);
	if (state == 2) {
		ILI9341_Fill_Screen(bgCode);
		ILI9341_Draw_Image((const char*) img_view, SCREEN_VERTICAL_1);
	}
}
void display2() {
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//	ILI9341_Draw_Image((const char*) img_view, SCREEN_VERTICAL_1);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
	//ILI9341_Draw_Rectangle(150, 50, 300, 300, 0xFFFF);
	ILI9341_Draw_Text("Group No.14", 130, 120, mixColor, 2, bgCode);
	ILI9341_Draw_Text("Bodinphat", 130, 140, mixColor, 2, bgCode);
	ILI9341_Draw_Text("Rachai", 130, 160, mixColor, 2, bgCode);
	ILI9341_Draw_Text("64010451", 130, 180, mixColor, 2, bgCode);
	if (TP_Touchpad_Pressed()) {
		state = 1;
		ILI9341_Fill_Screen(bgCode);
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
