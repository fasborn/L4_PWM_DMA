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
#ifndef __DELAY_H__
#define __DELAY_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "delay.h"

/* USER CODE END Private defines */

//uint32_t m_nStart;               //DEBUG Stopwatch start cycle counter value
//uint32_t m_nStop;                //DEBUG Stopwatch stop cycle counter value

#define DEMCR_TRCENA    0x01000000

/* Core Debug registers */
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define CLK_SPEED         168000000 // EXAMPLE for CortexM4, EDIT as needed

//#define STOPWATCH_START { m_nStart = *((volatile unsigned int *)0xE0001004);}
//#define STOPWATCH_STOP  { m_nStop = *((volatile unsigned int *)0xE0001004);}


//void DWT_Init(void);
//void delay_micros(uint32_t us);

void stopwatch_reset(void);
inline void stopwatch_delay(uint32_t ticks);
uint32_t CalcNanosecondsFromStopwatch(uint32_t nStart, uint32_t nStop);

/* USER CODE BEGIN Prototypes */

#endif /*__ GPIO_H__ */

/************************END OF FILE****/
