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
#include "stdlib.h"

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void menu_handler(void* params);
void print_handler(void* params);
void imucalib_handler(void* params);
void imumeas_handler(void* params);
void alarm_handler(void* params);
void cmd_handler(void* params);

void button_handler(void);
const char *msg_inv = "////Invalid option////\n";

TaskHandle_t handle_menu_task;
TaskHandle_t handle_print_task;
TaskHandle_t handle_imucalib_task;
TaskHandle_t handle_imumeas_task;
TaskHandle_t handle_alarm_task;
TaskHandle_t handle_cmd_task;

SemaphoreHandle_t CmdMutex;
SemaphoreHandle_t IMUMutex;

QueueHandle_t q_data;
QueueHandle_t q_print;

command_t curr_cmd;

volatile uint8_t user_input;
BaseType_t status;

adxl_Config_t adxl_config;

float imu_avg[3];
float imu_std[3];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	adxl_config.hi2c = &hi2c1;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  curr_cmd.curr_task = sMainMenu;
  curr_cmd.next_task = sMainMenu;
  curr_cmd.action = aNoAction;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  status = xTaskCreate(menu_handler, "menu_task", 200, NULL, 2, &handle_menu_task);
  configASSERT(status == pdPASS);
  status = xTaskCreate(print_handler, "print_task", 200, NULL, 2, &handle_print_task);
  configASSERT(status == pdPASS);
  status = xTaskCreate(imucalib_handler, "imucalib_task", 200, NULL, 2, &handle_imucalib_task);
  configASSERT(status == pdPASS);
  status = xTaskCreate(alarm_handler, "alarm_task", 200, NULL, 2, &handle_alarm_task);
  configASSERT(status == pdPASS);
  status = xTaskCreate(imumeas_handler, "imumeas_task", 200, NULL, 2, &handle_imumeas_task);
  configASSERT(status == pdPASS);
  status = xTaskCreate(cmd_handler, "cmd_task", 200, NULL, 2, &handle_cmd_task);
  configASSERT(status == pdPASS);

  q_print = xQueueCreate(10, sizeof(size_t));
  configASSERT(q_print != NULL);
  q_data = xQueueCreate(10, sizeof(uint8_t));
  configASSERT(q_data != NULL);

  CmdMutex = xSemaphoreCreateMutex();
  xSemaphoreGive(CmdMutex);
  IMUMutex = xSemaphoreCreateMutex();
  xSemaphoreGive(IMUMutex);

  HAL_UART_Receive_IT(&huart2, &user_input, 1);

  vTaskStartScheduler();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void button_handler(void) {
	const char* button_msg = "BUTTON HANDLER\n\r";
	// Set button event
    xSemaphoreTake(CmdMutex, 0);
	curr_cmd.action = aButtonPress;
    xSemaphoreGive(CmdMutex);
	xQueueSendFromISR(q_print, (void*)&button_msg, NULL);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	const char* uart_msg = "UART HANDLER\n\r";

	xQueueSendFromISR(q_print, (void*)&uart_msg, NULL);

    char buffer[50];
    memset(buffer, 0, sizeof(buffer));
    // 1: 49, 2: 50, 3: 51, 4: 52

    xSemaphoreTakeFromISR(CmdMutex, 0);
    uint8_t number_rcv = user_input-48;
    xSemaphoreGiveFromISR(CmdMutex, NULL);

    if ((curr_cmd.action == aInvalidImu) || (curr_cmd.action == aCalibDone) ||
    		(curr_cmd.action == aButtonPress) || (curr_cmd.action == aNoAction) || (curr_cmd.action == aInvalidImu)) {
    	xQueueSendFromISR(q_data, &number_rcv, NULL);
    }
	HAL_UART_Receive_IT(&huart2, &user_input, 1);
}



void print_handler(void *parameters) {
	uint32_t *msg;
	while(1) {
		xQueueReceive(q_print, &msg, portMAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen((char*)msg),HAL_MAX_DELAY);
	}
}

// make a menu here that makes you enter 0 to return
float stdev[3]={0,0,0};
float avg[3]={0,0,0};

void imucalib_handler(void* params) {
	// ask for number of samples or return
	// read IMU samples
	const char* calib_msg = "CMD TASK\n\r";

	int n_samples=20;
	float vals[n_samples*3];
	int idx=0;

	// Buffer
	float RxBuffer[3];
	memset(RxBuffer, 0, sizeof(RxBuffer));

	while (1) {
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		xQueueSend(q_print, &calib_msg,0);

	    xSemaphoreTake(CmdMutex, 0);
		curr_cmd.curr_task = sImuCalib;
	    xSemaphoreGive(CmdMutex);

	    // Set vals to zero
		memset(vals, 0, sizeof(vals));
		memset(stdev, 0, 3);
		memset(avg, 0, sizeof(n_samples));

		while (curr_cmd.next_task == sImuCalib) {
			xSemaphoreTake(IMUMutex, 0);
			adxl345_init(&adxl_config);
			if (adxl345_get_xyz(&adxl_config, RxBuffer) != ADXL_OK) {
			    xSemaphoreTake(CmdMutex, 0);
				curr_cmd.action = aInvalidImu;
			    xSemaphoreGive(CmdMutex);
				xTaskNotify(handle_menu_task, 0, eSetValueWithOverwrite);
				break;
			}
			xSemaphoreGive(IMUMutex);


			vals[3*idx] = RxBuffer[0];
			avg[0] += RxBuffer[0];
			vals[3*idx+1] = RxBuffer[1];
			avg[1] += RxBuffer[1];
			vals[3*idx+2] = RxBuffer[2];
			avg[2] += RxBuffer[2];

			if (idx >= n_samples) {
				avg[0] /= n_samples;
				avg[1] /= n_samples;
				avg[2] /= n_samples;

				for (int i=0; i<idx; i++) {
					stdev[0] += (vals[3*i]   - avg[0]);
					stdev[1] += (vals[3*i+1] - avg[1]);
					stdev[2] += (vals[3*i+2] - avg[2]);
				}

				stdev[0] /= sqrt(n_samples);
				stdev[1] /= sqrt(n_samples);
				stdev[2] /= sqrt(n_samples);


				stdev[0] *= 2.576;
				stdev[0] = abs(stdev[0]);
				stdev[1] *= 2.576;
				stdev[1] = abs(stdev[1]);
				stdev[2] *= 2.576;
				stdev[2] = abs(stdev[2]);

				stdev[0] = stdev[0] < 0.1 ? 0.1 : stdev[0];
				stdev[1] = stdev[1] < 0.1 ? 0.1 : stdev[1];
				stdev[2] = stdev[2] < 0.1 ? 0.1 : stdev[2];

			    xSemaphoreTake(CmdMutex, 0);
				curr_cmd.action = aCalibDone;
			    xSemaphoreGive(CmdMutex);
				xTaskNotify(handle_menu_task, 0, eSetValueWithOverwrite);
				idx=0;
				break;
			}
		idx++;
		}
	}
}

const char* inv_msg = "invalid option, try again\n\r";
const char* waiting_msg = "waiting for input \n\r";
void menu_handler(void* params) {
	uint8_t option;

	const char* msg_menu = 		"\n========================\n\r"
								"|         Menu         |\n\r"
								"========================\n\r"
								"RUN IMU		   ----> 1\n\r"
								"RUN Calibration   ----> 2\n\r"
								"EXIT			   ----> 3\n\r";
	while (1) {
		xQueueSend(q_print, &msg_menu,0);
		//wait for menu commands
		xQueueReceive(q_data, &option, portMAX_DELAY);
	    xSemaphoreTake(CmdMutex, 0);
	    curr_cmd.curr_task = sMainMenu;
		switch (option) {
		case (1):
			printf("option 1");
			curr_cmd.action = aNumber1;
			curr_cmd.next_task = sImuMeas;
			xTaskNotify(handle_cmd_task, 0, eSetValueWithOverwrite);
			break;
		case (2):
			printf("option 2");
			curr_cmd.action = aNumber2;
			curr_cmd.next_task = sImuCalib;
			xTaskNotify(handle_cmd_task, 0, eSetValueWithOverwrite);
			break;
		case (3):
			printf("option 3");
			curr_cmd.action = aNumber3;
			xTaskNotify(handle_cmd_task, 0, eSetValueWithOverwrite);
			break;
		case (4):
			printf("option 4");
			xQueueSend(q_print, &waiting_msg, 0);
			curr_cmd.action = aButtonPress;
			xTaskNotify(handle_cmd_task, 0, eSetValueWithOverwrite);
			break;
		default:
			xQueueSend(q_print, &inv_msg, 0);
			curr_cmd.action = aInvalidOption;
			break;
		}
		xTaskNotify(handle_cmd_task, 0, eSetValueWithOverwrite);
	    xSemaphoreGive(CmdMutex);
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		// Wait to run again when another task notifies.
	}
}

void cmd_handler(void *param) {
	const char* cmd_msg = " CMD TASK \n\r";

	while(1) {
		//Implement notify wait

		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		xQueueSend(q_print, &cmd_msg, 0);

			//process the user data(command) stored in input data queue
	    xSemaphoreTake(CmdMutex, 0);
		switch (curr_cmd.curr_task) {
		case sAlarm:
			if (curr_cmd.action == aButtonPress) {
				curr_cmd.next_task = sImuMeas;
				xTaskNotify(handle_imumeas_task, 0, eSetValueWithOverwrite);
				break;
				// Unblock measure task
			}
		case sImuMeas:
			if (curr_cmd.action == aImuDetect) {
				curr_cmd.next_task = sAlarm;
				xTaskNotify(handle_alarm_task, 0, eSetValueWithOverwrite);
				break;
				// Unblock alarm task
			}
		case sMainMenu:
			switch (curr_cmd.action) {
			case aNumber1:
				curr_cmd.next_task = sImuMeas;
				xTaskNotify(handle_imumeas_task, 0, eSetValueWithOverwrite);
				break;
			case aNumber2:
				curr_cmd.next_task = sImuCalib;
				xTaskNotify(handle_imucalib_task, 0, eSetValueWithOverwrite);
				break;
			case aNumber3:
				curr_cmd.next_task = sNoState;
				//xTaskNotify(handle_imucalib_task, 0, eSetValueWithOverwrite);
				break;
			}
		}
	    xSemaphoreGive(CmdMutex);

	}
}

void alarm_handler(void* parameters) {
	const char* alarm_msg =  "ALARM TASK \n\r";

	while(1) {
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		// Print to show entering alarm task
		xQueueSend(q_print, &alarm_msg, 0);
	    xSemaphoreTake(CmdMutex, 0);
		curr_cmd.curr_task = sAlarm;
	    xSemaphoreGive(CmdMutex);

		while (curr_cmd.next_task == sAlarm) {
			if (curr_cmd.action == aButtonPress) {
				xTaskNotify(handle_cmd_task, 0, eSetValueWithOverwrite);
				break;
			}
			// Make noise
		}
	}
}

const char* imu_inv = "INVALID IMU \n\r";

void imumeas_handler(void* parameters) {
	const char* imu_msg = "IMU TASK  \n\r";

	// Buffer
	float RxBuffer[3];
	memset(RxBuffer, 0, sizeof(RxBuffer));

	while(1) {
		//Wait for menu commands
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		// Print to show entering alarm task
		xQueueSend(q_print, &imu_msg, NULL);
	    xSemaphoreTake(CmdMutex, 0);
		curr_cmd.curr_task = sImuMeas;
	    xSemaphoreGive(CmdMutex);

		while (curr_cmd.next_task == sImuMeas) {
			xSemaphoreTake(IMUMutex, portMAX_DELAY);
			adxl345_init(&adxl_config);
			if (adxl345_get_xyz(&adxl_config, RxBuffer) != ADXL_OK) {
			    xSemaphoreTake(CmdMutex, 0);
				curr_cmd.action = aInvalidImu;
			    xSemaphoreGive(CmdMutex);
				xTaskNotify(handle_menu_task, 0, eSetValueWithOverwrite);
				xQueueSend(q_print, &imu_inv, NULL);
				break;
			}
			xSemaphoreGive(IMUMutex);

			if (curr_cmd.action == aButtonPress) {
				// aButtonPresss
				xTaskNotify(handle_menu_task, 0, eSetValueWithOverwrite);
				break;
			}
			if ((stdev[0] == 0) && (stdev[1] == 0) && (stdev[2] == 0)) {
				// sigma-IMU not initialized
			    xSemaphoreTake(CmdMutex, 0);
				curr_cmd.action = aInvalidOption;
			    xSemaphoreGive(CmdMutex);
				xTaskNotify(handle_menu_task, 0, eSetValueWithOverwrite);
				break;
			}
			// stdev = standard deviation * 2.576 (> 99 % confidence interval)
			if ((abs(avg[0] - RxBuffer[0]) > stdev[0]) || (abs(avg[1] - RxBuffer[1]) > stdev[1]) || (abs(avg[2] - RxBuffer[2]) > stdev[2])) {
				// DETECTED OFFSET
			    xSemaphoreTake(CmdMutex, 0);
				curr_cmd.action = aImuDetect;
			    xSemaphoreGive(CmdMutex);
				xTaskNotify(handle_cmd_task, 0, eSetValueWithOverwrite);
				break;
			}
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
