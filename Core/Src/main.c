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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sk6812.h"
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
extern uint32_t BUF_DMA [ARRAY_LEN];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TRUE 1

#define FALSE 0

enum bool{false,true};

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
  MX_DMA_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 0);
	

	
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
//	for(int i =0; i<100; i++){
//		BUF_DMA[i] = i;
//	}
//	
//	for(int i =100; i<200; i++){
//		BUF_DMA[i] = i-100;
//	}	
//	
//	for(int i = 200; i<300; i++){
//		BUF_DMA[i] = i-200;
//	}	
	
//	SET_BIT(TIM1->DIER, TIM_IT_UPDATE);
	
//	htim1.Instance->SR = TIM_FLAG_UPDATE;
//		HAL_TIM_PWM_Stop(&htim6, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start_IT(&htim6, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
//	HAL_TIM_Base_Start_IT(&htim6); 
	//__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);
//	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
//	TIM_CCxChannelCmd(htim2.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
//	__HAL_TIM_ENABLE(&htim2);

//	fill_buffer(0x00000F00, 0, 15);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, 1200);
//	
//	HAL_Delay(1000);
//	fill_buffer(0x00000000, 0, 15);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, 10000);
//	HAL_Delay(1000);
//	fill_buffer(0x01000000, 0, 15);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, 1200);
//	
//	HAL_Delay(1000);
//	fill_buffer(0x00010000, 0, 15);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, 1200);
//	
//	HAL_Delay(1000);
//	fill_buffer(0x00000100, 0, 15);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, 1200);	
//	
//	HAL_Delay(1000);
//	fill_buffer(0x00000000, 0, 15);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, 1200);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
	
	fill_buffer(0x67003100, 0, 1);
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, 150);	
	HAL_Delay(8);
	
//	for(int qua = 0; qua < 0xFFFFFFFF; qua++){
//		fill_buffer(qua&0xFFFFFF00, 0, 1);
//		HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, 150);		
//		HAL_Delay(8);
//	}
	

//	HAL_Delay(1000);
////	fill_buffer(0x00000000, 0, 15);
//	fill_buffer(0x00FF0000, 10, 15);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, 10000);

//	HAL_Delay(1000);
////	fill_buffer(0x00000000, 0, 15);
//	fill_buffer(0x0000FF00, 15, 20);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, 10000);

//	HAL_Delay(1000);
////	fill_buffer(0x00000000, 0, 15);
//	fill_buffer(0x000000FF, 20, 25);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, BUF_DMA, 10000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	

	
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

	
//int leds[300] = {0};
//uint8_t number_of_leds = 0;

//void send_bite(uint8_t bite){
//    if(bite){
//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
//        for(int i = 0; i<15; i++){}

//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
//        for(int i = 0; i<7; i++){}
//    }

//    else{
//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
//        for(int i = 0; i<7; i++){}

//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
//        for(int i = 0; i<15; i++){}
//    }
//}

//void send_pixel(int rgbw){
//    uint8_t green =     (rgbw & 0xFF000000)>>24;
//    uint8_t red =       (rgbw & 0x00FF0000)>>16;
//    uint8_t blue =      (rgbw & 0x0000FF00)>>8;
//    uint8_t warm =      rgbw & 0xFF;

//    for (uint8_t co = 0; co<8; co++){
//        send_bite(green>>(7-co));
//    }
//    for (uint8_t co = 0; co<8; co++){
//        send_bite(red>>(7-co));
//    }
//    for (uint8_t co = 0; co<8; co++){
//        send_bite(blue>>(7-co));
//    }
//    for (uint8_t co = 0; co<8; co++){
//        send_bite(warm>>(7-co));
//    }
//}

//void send_word(uint8_t green, uint8_t red, uint8_t blue, uint8_t warm){

//}

//void fill_color(uint8_t green, uint8_t red, uint8_t blue, uint8_t warm, uint8_t low, uint8_t high){
//		
//		int rosto = 0;
//		rosto	= rosto | ((int) green<<24);
//		rosto	= rosto | ((int) red<<16);
//		rosto	= rosto | ((int) blue<<8);
//		rosto	= rosto | ((int) warm);
//		
//    uint8_t number = high - low;
//    for(uint8_t i = 0; i<number; i++){
//			send_pixel(rosto);
//    }
//}

//void int_to_uint(void){
//    
//}

//void fill_strip(void){
//    for(uint8_t i = 0; i < number_of_leds; i++){
//        send_pixel(leds[i]);
//    }

//    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
//    for (int i = 0; i<1500; i++){}
//}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
