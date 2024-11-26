/*
 * console.cpp
 *
 *  Created on: Feb 6, 2024
 *      Author: akamdhillon
 */

#include "cmsis_os.h"
#include "console.h"
#include "main.h"
#include "motor.h"
#include <string>

#define RASPBERRY		1

#define MAX_STRING_SIZE	32
#define MAX_HEIGHT		140
#define HALF_HEIGHT		MAX_HEIGHT / 2
#define MIN_HEIGHT		10


int UART2_Received = 0;
int UART2_Transmitted = 0;
uint32_t ticks = 0;
uint32_t debugEnable = 0;
uint8_t receivedChar;
int nextMove = 1;

extern StepperMotor motor1;
extern StepperMotor motor2;
extern StepperMotor motor3;

extern "C" {
    extern UART_HandleTypeDef huart2;
    extern volatile int I2C_Received;
    extern int command;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if(huart->Instance == USART2) {
		UART2_Received = 1;
		if(receivedChar == '\x7f') {
			const char backspaceSeq[3] = {'\b', ' ', '\b'};
			HAL_UART_Transmit(huart, (uint8_t*)backspaceSeq, sizeof(backspaceSeq), 100);
		}
		else {
			HAL_UART_Transmit(huart, &receivedChar, 1, 100);
		}
	}
}

void processCommand(std::string str) {

	//Remove any whitespace in the beginning.
	while(str.length() && (str[0] == ' ')) {
		str.erase(0,1);
	}

	if(str.find("debug ") == 0) {
		/* Do debug commands */

		if(str.length() > 5) {
			int num = stoi(str.substr(5));

			if (num > 31 || num < 0) {
				printf("Please enter a value between 0-31!\r\n");
			}
			else {
				if(num == 0) debugEnable = 0;
				else {
					// Toggle the specific debug bit
					debugEnable ^= (1 << num);  // Toggle bit
					printf("Debug %i %s.\r\n", num, (debugEnable & (1 << num)) ? "enabled" : "disabled");
				}
			}
		}
		else {
			printf("Missing parameter\r\n");
		}

	}
	else if(str.find("stop") == 0) {
		if(str.length() > 4) {
			int num = stoi(str.substr(4));
			switch(num) {
			case 1:
				motor1.stop();
				break;
			case 2:
				motor2.stop();
				break;
			case 3:
				motor3.stop();
				break;
			default:
				motor1.stop();
				motor2.stop();
				motor3.stop();
				break;
			}
		}
		else {
			motor1.stop();
			motor2.stop();
			motor3.stop();
		}
		printf("Stopped!\r\n");
	}
	//Add commands like test!
	else if(str.find("test") == 0) {
		printf("Test working!\r\n");
	}
	else if (str.find("reset") == 0) {
		*motor1.stepPtr = 0;
		*motor2.stepPtr = 0;
		*motor3.stepPtr = 0;
		command = 2;
	}
	else if (str.find("move") == 0) {
		if(str.length() > 4) {
			int num = stoi(str.substr(4));
			printf("Move to %i\r\n", num);
			motor1.setTarget(num);
			motor2.setTarget(num);
			motor3.setTarget(num);
			debugEnable |= 1;
		}
		else {
			printf("Missing parameter\r\n");
		}
	}

	else if (str.find("on") == 0) {
		HAL_GPIO_WritePin(motor1.enablePort, motor1.enablePin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor2.enablePort, motor2.enablePin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor3.enablePort, motor3.enablePin, GPIO_PIN_SET);
		printf("Motor ON\r\n");
	}
	else if (str.find("off") == 0) {
		HAL_GPIO_WritePin(motor1.enablePort, motor1.enablePin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor2.enablePort, motor2.enablePin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor3.enablePort, motor3.enablePin, GPIO_PIN_RESET);
		printf("Motor OFF\r\n");
	}
	else if(str.find("n") == 0) {
		switch(nextMove) {
		case 1:
			motor1.setTarget(HALF_HEIGHT);
			motor2.setTarget(HALF_HEIGHT);
			motor3.setTarget(HALF_HEIGHT);
			break;
		case 2:
			motor1.setTarget(MIN_HEIGHT);
			motor2.setTarget(MIN_HEIGHT);
			motor3.setTarget(MIN_HEIGHT);
			break;
		case 3:
			motor1.setTarget(HALF_HEIGHT);
			break;
		case 4:
			motor1.setTarget(MAX_HEIGHT);
			motor2.setTarget(HALF_HEIGHT);
			motor3.setTarget(HALF_HEIGHT);
			break;
		case 5:
			motor1.setTarget(MAX_HEIGHT);
			motor2.setTarget(MAX_HEIGHT);
			motor3.setTarget(MAX_HEIGHT);
			break;
		case 6:
			motor1.setTarget(HALF_HEIGHT);
			break;
		case 7:
			motor1.setTarget(MIN_HEIGHT);
			motor2.setTarget(HALF_HEIGHT);
			motor3.setTarget(HALF_HEIGHT);
			break;
		case 8:
			motor1.setTarget(MIN_HEIGHT);
			motor2.setTarget(MIN_HEIGHT);
			motor3.setTarget(MIN_HEIGHT);
			break;
		case 9:
			motor1.setTarget(MIN_HEIGHT);
			motor2.setTarget(MIN_HEIGHT);
			motor3.setTarget(MIN_HEIGHT);
			break;
		case 10:
			motor1.setTarget(MAX_HEIGHT);
			motor2.setTarget(MIN_HEIGHT);
			motor3.setTarget(MIN_HEIGHT);
			break;
		case 11:
			motor1.setTarget(MIN_HEIGHT);
			motor2.setTarget(MAX_HEIGHT);
			motor3.setTarget(MAX_HEIGHT);
			break;
		default:
			break;
		}
		nextMove++;
		if(nextMove > 11) nextMove = 1;
	}
	else {
		printf("Invalid Input!\r\n");
	}
}

void StartConsoleTask(void const * argument) {
	std::string input = "";
	char ch;

	osDelay(10);

	//Start UART Recieve interrupt
	HAL_UART_Receive_IT(&huart2, &receivedChar, 1);

	int selected_motor = 0;

	while(1) {
		if(UART2_Received) {
			UART2_Received = 0;
			ch = receivedChar;
			switch(ch) {
			case '\x7f':
				if (!input.empty()) input.pop_back();
				break;
			case '\r':
				if(input.length()) {
					processCommand(input);
					input.clear();
				}
				printf("\r\n");
				break;
			default:
				if(input.length() < MAX_STRING_SIZE)	input += ch;
				break;
			}
			HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
		}
//		if(I2C_Received) {
//			switch(selected_motor) {
//			case 1:
//				motor1.setTarget(received_value);
//				motor1.start();
//				break;
//			case 2:
//				motor2.setTarget(received_value);
//				motor2.start();
//				break;
//			case 3:
//				motor3.setTarget(received_value);
//				motor3.start();
//				break;
//			default:
//				selected_motor = received_value;
//				break;
//			}
//			I2C_Received = 0;
//			received_flag = 1;
//		}
		// Debug prints
		uint32_t currTick = HAL_GetTick();
		if (currTick - ticks >= 1000) {
		    ticks = currTick;
		    if(debugEnable & 1) {
				int temp = *motor1.stepPtr;
//				printf("Step1: %i\r\n", temp);
//				temp = *motor2.stepPtr;
//				printf("Step2: %i\r\n", temp);
//				temp = *motor3.stepPtr;
//				printf("Step3: %i\r\n", temp);
		    }
		}
	}

}

