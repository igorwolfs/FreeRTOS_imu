#include "main.h"


void print_handler(void *parameters) {
	uint32_t *msg;
	while(1) {
		xQueueReceive(q_print, &msg, portMAX_DELAY);
		// Print data over UART
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen((char*)msg),HAL_MAX_DELAY);
	}
}

// make a menu here that makes you enter 0 to return
float imu_stdev[3]={0,0,0};
float imu_avg[3]={0,0,0};

void set_imu_default(void) {
	// Set bounded values for IMU in case of erroneous values.
	imu_stdev[0] = imu_stdev[0] < 0.1 ? 0.1 : imu_stdev[0];
	imu_stdev[1] = imu_stdev[1] < 0.1 ? 0.1 : imu_stdev[1];
	imu_stdev[2] = imu_stdev[2] < 0.1 ? 0.1 : imu_stdev[2];

	imu_stdev[0] = imu_stdev[0] > 0.5 ? 0.1 : imu_stdev[0];
	imu_stdev[1] = imu_stdev[1] < 0.5 ? 0.1 : imu_stdev[1];
	imu_stdev[2] = imu_stdev[2] < 0.5 ? 0.1 : imu_stdev[2];
}

/*
 * Handles the calibration of the IMU values by setting "imu_avg" and "imu_stdev" values.
*/
void imucalib_handler(void* params) {
	const char* calib_msg = "CMD TASK\n\r";

	int n_samples=20;
	float vals[n_samples*3];
	int idx=0;

	// Buffer
	float RxBuffer[3];
	memset(RxBuffer, 0, sizeof(RxBuffer));

	while (1) {
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		// Print current state over UART
		xQueueSend(q_print, &calib_msg,0);

	    xSemaphoreTake(CmdMutex, 0);
		curr_cmd.curr_task = sImuCalib;
	    xSemaphoreGive(CmdMutex);

	    // Set vals to zero
		memset(vals, 0, sizeof(vals));
		memset(imu_stdev, 0, 3);
		memset(imu_avg, 0, sizeof(n_samples));

		while (curr_cmd.next_task == sImuCalib) {
			xSemaphoreTake(IMUMutex, 0);
			// Initialize IMU
			adxl345_init(&adxl_config);

			// Get IMU measurement
			if (adxl345_get_xyz(&adxl_config, RxBuffer) != ADXL_OK) {
				xSemaphoreGive(IMUMutex);
				xSemaphoreTake(CmdMutex, 0);
				curr_cmd.action = aInvalidImu;
			    xSemaphoreGive(CmdMutex);
			    set_imu_default();

				xTaskNotify(handle_menu_task, 0, eSetValueWithOverwrite);
				break;
			}
			xSemaphoreGive(IMUMutex);


			vals[3*idx] = RxBuffer[0];
			imu_avg[0] += RxBuffer[0];
			vals[3*idx+1] = RxBuffer[1];
			imu_avg[1] += RxBuffer[1];
			vals[3*idx+2] = RxBuffer[2];
			imu_avg[2] += RxBuffer[2];

			idx++;
			if (idx >= n_samples) {
				imu_avg[0] /= n_samples;
				imu_avg[1] /= n_samples;
				imu_avg[2] /= n_samples;

				for (int i=0; i<idx; i++) {
					imu_stdev[0] += (vals[3*i]   - imu_avg[0]);
					imu_stdev[1] += (vals[3*i+1] - imu_avg[1]);
					imu_stdev[2] += (vals[3*i+2] - imu_avg[2]);
				}

				imu_stdev[0] /= sqrt(n_samples);
				imu_stdev[1] /= sqrt(n_samples);
				imu_stdev[2] /= sqrt(n_samples);

				imu_stdev[0] *= 2.576;
				imu_stdev[0] = abs(imu_stdev[0]);
				imu_stdev[1] *= 2.576;
				imu_stdev[1] = abs(imu_stdev[1]);
				imu_stdev[2] *= 2.576;
				imu_stdev[2] = abs(imu_stdev[2]);

				// In the cases below calibration failed.
				set_imu_default();

			    xSemaphoreTake(CmdMutex, 0);
				curr_cmd.action = aCalibDone;
			    xSemaphoreGive(CmdMutex);
				xTaskNotify(handle_menu_task, 0, eSetValueWithOverwrite);
				idx=0;
				break;
			}
		}
	}
}


const char* inv_msg = "invalid option, try again\n\r";
const char* waiting_msg = "waiting for input \n\r";
/**
 * Handles UART input from UART-interrupt
*/
void menu_handler(void* params) {
	uint8_t option;

	const char* msg_menu = 		"========================\n\r"
								"|         Menu         |\n\r"
								"========================\n\r"
								"1: RUN IMU \n\r"
								"2: RUN Calibration\n\r"
								"3: LED BLINK\n\r";
	while (1) {
	    curr_cmd.curr_task = sMainMenu;

		// Print MENU
		xQueueSend(q_print, &msg_menu,0);

		// Wait for commands from UART through q_data-queue
		xQueueReceive(q_data, &option, portMAX_DELAY);
	    xSemaphoreTake(CmdMutex, 0);
		switch (option) {
			case (1):
				curr_cmd.action = aNumber1;
				curr_cmd.next_task = sImuMeas;
				xTaskNotify(handle_cmd_task, 0, eSetValueWithOverwrite);
				break;
			case (2):
				curr_cmd.action = aNumber2;
				curr_cmd.next_task = sImuCalib;
				xTaskNotify(handle_cmd_task, 0, eSetValueWithOverwrite);
				break;
			case (3):
				curr_cmd.action = aNumber3;
				curr_cmd.next_task = sLedBlink;
				xTaskNotify(handle_cmd_task, 0, eSetValueWithOverwrite);
				break;
			default:
				xQueueSend(q_print, &inv_msg, 0);
				curr_cmd.action = aInvalidOption;
				curr_cmd.next_task = sMainMenu;
				break;
		}
		xTaskNotify(handle_cmd_task, 0, eSetValueWithOverwrite);
	    xSemaphoreGive(CmdMutex);
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		// Wait to run again when another task notifies.
	}
}

/**
 * Handles state transitions
*/
void cmd_handler(void *param) {

	while(1) {
		//Implement notify wait

		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

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
				curr_cmd.next_task = sLedBlink;
				xTaskNotify(handle_led_task, 0, eSetValueWithOverwrite);
				break;
			}
		}
	    xSemaphoreGive(CmdMutex);
	}
}

/**
 * Blink-led task
*/
void led_handler(void* parameters) {
	const char* alarm_msg =  "LED BLINK TASK \n\r";

	while(1) {
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

		// Print current state over UART
		xQueueSend(q_print, &alarm_msg, 0);
	    xSemaphoreTake(CmdMutex, 0);
		curr_cmd.curr_task = sLedBlink;
		curr_cmd.action = aLedBlink;
	    xSemaphoreGive(CmdMutex);

	    // Blink led
		while (curr_cmd.next_task == sLedBlink) {
			// Led ON
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_Delay(500);
			// Led OFF
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_Delay(500);
			// Check for button press (through interrupt)
			if (curr_cmd.action == aButtonPress) {
				xTaskNotify(handle_menu_task, 0, eSetValueWithOverwrite);
				break;
			}
		}
	}
}

/**
 * Handles the active buzzer
*/
void alarm_handler(void* parameters) {
	const char* alarm_msg =  "ALARM TASK \n\r";

	while(1) {
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

		// Print current state over UART
		xQueueSend(q_print, &alarm_msg, 0);
	    xSemaphoreTake(CmdMutex, 0);
		curr_cmd.curr_task = sAlarm;
	    xSemaphoreGive(CmdMutex);

		// Ring active buzzer
		while (curr_cmd.next_task == sAlarm) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_Delay(500);
			if (curr_cmd.action == aButtonPress) {
				xTaskNotify(handle_cmd_task, 0, eSetValueWithOverwrite);
				break;
			}
		}
	}
}




/**
 * Handles IMU measurement: exits when IMU measurement deviation superseed sigma*2.576
*/
const char* imu_inv = "INVALID IMU \n\r";
void imumeas_handler(void* parameters) {
	const char* imu_msg = "IMU TASK  \n\r";

	// Buffer
	float RxBuffer[3];
	memset(RxBuffer, 0, sizeof(RxBuffer));

	while(1) {
		//Wait for menu commands
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

		// Print current state over UART
		xQueueSend(q_print, &imu_msg, NULL);
	    xSemaphoreTake(CmdMutex, 0);

		curr_cmd.curr_task = sImuMeas;
		curr_cmd.action = aImuMeas;

	    xSemaphoreGive(CmdMutex);

		while (curr_cmd.next_task == sImuMeas) {


			//Measure and initialize IMU
			xSemaphoreTake(IMUMutex, 0);
			adxl345_init(&adxl_config);
			if (adxl345_get_xyz(&adxl_config, RxBuffer) != ADXL_OK) {
				xSemaphoreGive(IMUMutex);
			    xSemaphoreTake(CmdMutex, 0);
				curr_cmd.action = aInvalidImu;
			    xSemaphoreGive(CmdMutex);
				xTaskNotify(handle_menu_task, 0, eSetValueWithOverwrite);
				xQueueSend(q_print, &imu_inv, NULL);
				break;
			}
			xSemaphoreGive(IMUMutex);

			// Introduce delay (for button press)
			for (int i=0; i<10000; i++);

			// Button-press: return to menu
			if (curr_cmd.action == aButtonPress) {
				// aButtonPresss
				xTaskNotify(handle_menu_task, 0, eSetValueWithOverwrite);
				break;
			}

			// If IMU not calibrated
			if ((imu_stdev[0] == 0) && (imu_stdev[1] == 0) && (imu_stdev[2] == 0)) {
			    xSemaphoreTake(CmdMutex, 0);
				curr_cmd.action = aInvalidOption;
			    xSemaphoreGive(CmdMutex);
				xTaskNotify(handle_menu_task, 0, eSetValueWithOverwrite);
				break;
			}

			// If IMU deviation greater than standard deviation * 2.576 (> 99 % confidence interval)
			if ((abs(imu_avg[0] - RxBuffer[0]) > imu_stdev[0]) || (abs(imu_avg[1] - RxBuffer[1]) > imu_stdev[1]) || (abs(imu_avg[2] - RxBuffer[2]) > imu_stdev[2])) {
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
