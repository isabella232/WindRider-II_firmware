//! Initialization file.
/**
 * @file      initialization.h
 * @brief     This file contains initialization of pheripherals shared between different pcb modules.
 * @author    Stanislav Sotnikov (stanislav.sotnikov145@gmail.com)
 * TODO:      Integrate this file with the HardwareDriver::initialize()
 *
 */
#ifndef __INITIALIZATION_H
#define __INITIALIZATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


void Initialize_Peripherals(void);

void SystemClock_Config(void);

void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_USART1_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Error_Handler(void);

#define TIM2_PRESCALER 25
#define TIM2_PERIOD 27700.0//55400.0

#define TIM3_PRESCALER 1
#define TIM3_PERIOD 36000.0

#define LED1_EN_Pin GPIO_PIN_0
#define LED1_EN_GPIO_Port GPIOA

#define LED2_EN_Pin GPIO_PIN_1
#define LED2_EN_GPIO_Port GPIOA

#define LED3_EN_Pin GPIO_PIN_2
#define LED3_EN_GPIO_Port GPIOA

#define LED4_EN_Pin GPIO_PIN_3
#define LED4_EN_GPIO_Port GPIOA

#define SUCTION_EN_Pin GPIO_PIN_4
#define SUCTION_EN_GPIO_Port GPIOA

#define SUCTION_PWM_Pin GPIO_PIN_6
#define SUCTION_PWM_GPIO_Port GPIOA

#define SERVO1_Pin GPIO_PIN_10
#define SERVO1_GPIO_PORT GPIOB

#define SERVO2_Pin GPIO_PIN_11
#define SERVO2_GPIO_PORT GPIOB

#define SOLENOID_Pin GPIO_PIN_12
#define SOLENOID_GPIO_Port GPIOB

#define SPARE_Pin GPIO_PIN_13
#define SPARE_GPIO_Port GPIOB

// Definition for USARTx's DMA
#define USARTx_TX_DMA_CHANNEL             DMA1_Channel4

// Definition for USARTx's NVIC
#define USARTx_DMA_TX_IRQn                DMA1_Channel4_IRQn

#ifdef __cplusplus
}
#endif

#endif
