/*
 * motor.cpp
 *
 *  Created on: Nov 6, 2024
 *      Author: akamdhillon
 */


#include "motor.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>


#define MAX_FREQ		2000
#define CLOCK_SPEED		1000000
#define PULSE			(CLOCK_SPEED / SPEED) / 2
#define SEND_DATA_SIZE		8
#define RECEIVE_DATA_SIZE	1

int SPEED = 600;
int OFFSET = 50;
int temp = 0;

extern "C" {
    extern TIM_HandleTypeDef htim1;
    extern I2C_HandleTypeDef hi2c1;
    extern volatile int isMoving1;
    extern volatile int dir1;
    extern volatile int step1;
    extern volatile int isMoving2;
	extern volatile int dir2;
	extern volatile int step2;
	extern volatile int isMoving3;
	extern volatile int dir3;
	extern volatile int step3;
}

volatile uint8_t first_byte = 1;
volatile uint32_t received_value = 0;

enum COMMAND {
	COMMAND_NONE = 0,
    COMMAND_SPEED = 1,
    COMMAND_RESET = 2
};

COMMAND command = COMMAND_NONE;

int resetFlag = 0;
int height1 = 0;
int height2 = 0;
int height3 = 0;
int numBytes = 0;

// Global buffer and flag for transmit
volatile uint8_t txBuffer[1] = {1};
volatile uint8_t txDataReady = 0;
// Global buffer for receive
volatile uint8_t rxBuffer[RECEIVE_DATA_SIZE];


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if(first_byte) {
		received_value = rxBuffer[0];
		first_byte = 0;
	}
	else {
		received_value |= (rxBuffer[0] << 8);
		temp = received_value;
		first_byte = 1;
		if(command == COMMAND_SPEED) {
			SPEED = received_value;
			command = COMMAND_NONE;
		}
		else {
			switch(received_value) {
			case 0xFFF0:
				numBytes = 0;
				break;
			case 0xFFF1:
				command = COMMAND_SPEED;
				break;
			case 0xFFF2:
				resetFlag = 1;
				break;
			default:
				if(numBytes == 0) height1 = received_value;
				else if(numBytes == 1) height2 = received_value;
				else if(numBytes == 2) height3 = received_value;
				numBytes++;
				if(numBytes >= 3) numBytes = 0;
				break;
			}
		}
		received_value = 0;
	}
	HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)rxBuffer, RECEIVE_DATA_SIZE);
}


// Constructor to initialize GPIO ports, pins, and PWM timer
StepperMotor::StepperMotor(GPIO_TypeDef* enablePort_, uint16_t enablePin_,
		GPIO_TypeDef* dirPort_, uint16_t dirPin_, uint32_t timChannel_,
		volatile int* isMoving_, volatile int* stepPtr_, volatile int* dirPtr_) {
	// Initialize the member variables
	htim = &htim1;
	enablePort = enablePort_;
	enablePin = enablePin_;
	dirPort = dirPort_;
	dirPin = dirPin_;
	timChannel = timChannel_;
	stepPtr = stepPtr_;
	dirPtr = dirPtr_;
	targetStep = 0;
	isMovingPtr = isMoving_;
	currentSpeed = 0;
}

void StepperMotor::init() {
	HAL_GPIO_WritePin(enablePort, enablePin, GPIO_PIN_SET);
	osDelay(400);
	HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(enablePort, enablePin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(htim, timChannel);
	*dirPtr = 1;
	*isMovingPtr = 0;
	*stepPtr = 0;
	uint32_t period = CLOCK_SPEED / SPEED;
	period--;
	__HAL_TIM_SET_AUTORELOAD(htim, period);           // Set ARR for frequency
	__HAL_TIM_SET_COMPARE(htim, timChannel, 0);
	HAL_TIM_PWM_Start(htim, timChannel);  // Start PWM output

	printf("Period: %i\r\n", period+1);
}


// Sets the target step position and starts the movement
void StepperMotor::setTarget(int steps) {
    targetStep = steps;
    if(targetStep > *stepPtr) {
    	if(*dirPtr == 0) {
    		setDirection(1);
    		stop();
    	}
    }
    else {
    	if(*dirPtr == 1) {
			setDirection(0);
			stop();
		}
    }
    start();
}

void StepperMotor::start() {
	if(*isMovingPtr != 1) {
		setSpeed(SPEED);
		*isMovingPtr = 1;
	}
}

void StepperMotor::setSpeed(int sp) {
	uint32_t period = CLOCK_SPEED / sp;
	uint16_t pulse = (uint16_t)(period / 2);
	period--;
	__HAL_TIM_SET_AUTORELOAD(htim, period);
	__HAL_TIM_SET_COMPARE(htim, timChannel, pulse);
	currentSpeed = sp;
}

void StepperMotor::stop() {
	__HAL_TIM_SET_COMPARE(htim, timChannel, 0);
	*isMovingPtr = 0;
//	printf("Current Step: %i\r\n", *stepPtr);
//	osDelay(2000);
//	if(*dirPtr) {
//		setDirection(0);
//		targetStep = 10;
//	}
//	else {
//		setDirection(1);
//		targetStep = 50;
//	}
//	start();
}

void StepperMotor::setDirection(int dir) {
	if(dir) {
		HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_SET);
	}
	*dirPtr = dir;
}

void StepperMotor::checkStop() {
	if(*isMovingPtr) {
		int temp = *stepPtr;
		if(*dirPtr) {
			if(temp > targetStep) {
				stop();
				return;
			}
		}
		else {
			if(temp < targetStep) {
				stop();
				return;
			}
		}
//		if(currentSpeed < SPEED) {
//			setSpeed(currentSpeed + 20);
//		}
	}
}

/*
 * Pin Mapping for Motors
 *
 * Motor1:
 * - Timer 1, Channel 1, PA8, D7
 * - Enable Pin: 	PA5, D13
 * - Direction Pin: PA6, D12
 *
 *  * Motor2:
 * - Timer 3, Channel 2, PA9, D8
 * - Enable Pin: 	PB10, D6
 * - Direction Pin: PB4, D5
 *
 *  * Motor3:
 * - Timer 4, Channel 1, PA10, D2
 * - Enable Pin: 	PB5, D4
 * - Direction Pin: PB3, D3
 */


StepperMotor motor1(GPIOA, GPIO_PIN_5, GPIOA, GPIO_PIN_6, TIM_CHANNEL_1, &isMoving1, &step1, &dir1);
StepperMotor motor2(GPIOB, GPIO_PIN_10, GPIOB, GPIO_PIN_4, TIM_CHANNEL_2, &isMoving2, &step2, &dir2);
StepperMotor motor3(GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_3, TIM_CHANNEL_3, &isMoving3, &step3, &dir3);



void StartMotorControlTask(void const * argument) {
	motor1.init();
	motor2.init();
	motor3.init();
	motor1.setTarget(OFFSET);
	motor2.setTarget(OFFSET);
	motor3.setTarget(OFFSET);

	HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)rxBuffer, RECEIVE_DATA_SIZE);

	while(1) {
		if(temp) {
//			printf("R: %i\r\n", temp);
			temp = 0;
		}
		motor1.checkStop();
		motor2.checkStop();
		motor3.checkStop();

		if(resetFlag) {
			motor1.init();
			motor2.init();
			motor3.init();
			motor1.setTarget(OFFSET);
			motor2.setTarget(OFFSET);
			motor3.setTarget(OFFSET);
			resetFlag = 0;
		}

		if(height1) {
			printf("H1: %i\r\n", height1);
			motor1.setTarget(height1);
			height1 = 0;
		}
		if(height2) {
			printf("H2: %i\r\n", height2);
			motor2.setTarget(height2);
			height2 = 0;
		}
		if(height3) {
			printf("H3: %i\r\n", height3);
			motor3.setTarget(height3);
			height3 = 0;
		}
	}


//		tempRoll = roll + 20;// * MICROSTEPS;
//		tempPitch = pitch + 20;// * MICROSTEPS;
//
//		h1_target = tempRoll;
//		zn = (OFFSET - tempRoll) / 2;
//
//		z = (tempPitch - OFFSET) / 2;
//		h2_target = zn - z + OFFSET;
//		h3_target = zn + z + OFFSET;

}

