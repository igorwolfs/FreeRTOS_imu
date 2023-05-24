/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "stdio.h"
#include <string.h>
#include <stdint.h>
#include <queue.h>


#include "semphr.h"
#include "adxl345.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// States
typedef enum {
	sMainMenu = 0, // Main menu state
	sImuMeas,	   // IMU measurement state
	sImuCalib,	   // IMU calibration state
	sAlarm,		   // Alarm state
	sLedBlink,	   // Led blink state
} state_t;

// Actions
typedef enum {
	aButtonPress=0, // Indicates button press interrupt call
	aImuDetect=1,	// Indicates IMU movement detect in imu-task
	// Possible UART input states
	aNumber1,
	aNumber2,
	aNumber3,
	aInvalidOption,
	// IMU-states
	aCalibDone,
	aImuMeas,
	aInvalidImu,
	aLedBlink
} action_t;

// Struct containing curr task, next task and action
typedef struct {
	state_t curr_task;
	state_t next_task;
	action_t action;
} command_t;


// Functions
void menu_handler(void* params);
void print_handler(void* params);
void imucalib_handler(void* params);
void imumeas_handler(void* params);
void alarm_handler(void* params);
void cmd_handler(void* params);
void led_handler(void* params);

// Task Handles
extern TaskHandle_t handle_menu_task;
extern TaskHandle_t handle_print_task;
extern TaskHandle_t handle_imucalib_task;
extern TaskHandle_t handle_imumeas_task;
extern TaskHandle_t handle_alarm_task;
extern TaskHandle_t handle_cmd_task;
extern TaskHandle_t handle_led_task;

// Queue handles
extern QueueHandle_t q_data;
extern QueueHandle_t q_print;

// Mutexes
extern SemaphoreHandle_t CmdMutex;
extern SemaphoreHandle_t IMUMutex;

// Typedefs
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c;

// State variables
extern command_t curr_cmd;
extern adxl_Config_t adxl_config;


//

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
