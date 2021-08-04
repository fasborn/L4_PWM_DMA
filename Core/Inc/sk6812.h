/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SK6812_H__
#define __SK6812_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE END Private defines */

#include <stdint.h>

//--------------------------------------------------

#define DELAY_LEN 64
#define LED_COUNT 300
#define ARRAY_LEN DELAY_LEN + LED_COUNT*32

#define HIGH 64
#define LOW 36


void send_bite(uint8_t bite);
void send_pixel(int rgbw);
void fill_color(uint8_t green, uint8_t red, uint8_t blue, uint8_t warm, uint8_t low, uint8_t high);
void clear_strip(void);
void fill_buffer(int color, uint8_t lower, uint8_t upper);

/* USER CODE BEGIN Prototypes */

#endif /*__ GPIO_H__ */

/************************END OF FILE****/
