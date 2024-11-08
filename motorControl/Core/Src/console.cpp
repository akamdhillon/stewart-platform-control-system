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

#define MAX_STRING_SIZE	32


int UART2_Received = 0;
int UART2_Transmitted = 0;
uint32_t ticks = 0;
uint32_t debugEnable = 0;
uint8_t receivedChar;

extern volatile uint8_t I2C_Recieved;
extern StepperMotor motor1;
extern uint32_t pulses1;

extern "C" {
    extern UART_HandleTypeDef huart2;
    extern volatile uint32_t timer1;
    extern volatile uint8_t timer3;
    extern volatile uint8_t timer4;
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
	//Add commands like test!
	else if(str.find("test") == 0) {
		printf("Test working!\r\n");
	}
	else if (str.find("motor") == 0) {
		if(str.length() > 5) {
			int num = stoi(str.substr(5));
			printf("Motor speed %i\r\n", num);
			motor1.setSpeed(num);
			timer1 = 0;
			debugEnable |= 1;
		}
		else {
			printf("Missing parameter\r\n");
		}

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
		// Debug prints
		uint32_t currTick = HAL_GetTick();
		if (currTick - ticks >= 1000) {
		    ticks = currTick;
		    if (debugEnable & 1) {
		        printf("Time: %i Pulses: %i\r\n", currTick, timer1);
		    }
		}
	}

}

