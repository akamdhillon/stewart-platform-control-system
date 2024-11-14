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


#define MAX_FREQ		2000
#define CLOCK_SPEED		1000000
#define SPEED			125
#define PULSE			(CLOCK_SPEED / SPEED) / 2


extern "C" {
    extern TIM_HandleTypeDef htim1;
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
}

void StepperMotor::init() {
	HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_SET);
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
    	setDirection(1);
    }
    else {
    	setDirection(0);
    }
}

void StepperMotor::start() {
	__HAL_TIM_SET_COMPARE(htim, timChannel, PULSE);
	*isMovingPtr = 1;
}

void StepperMotor::stop() {
	__HAL_TIM_SET_COMPARE(htim, timChannel, 0);
	*isMovingPtr = 0;
	printf("Current Step: %i\r\n", *stepPtr);
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
	HAL_GPIO_WritePin(dirPort, dirPin, (GPIO_PinState)dir);
	*dirPtr = dir;
}

void StepperMotor::checkStop() {
	if(*isMovingPtr) {
		int temp = *stepPtr;
		if(*dirPtr) {
			if(temp > targetStep) stop();
		}
		else {
			if(temp < targetStep) stop();
		}
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
	while(1) {
		motor1.checkStop();
		motor2.checkStop();
		motor3.checkStop();
	}
}

