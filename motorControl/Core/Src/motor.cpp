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


extern "C" {
    extern TIM_HandleTypeDef htim1;
    extern volatile uint32_t timer1;
}



// Constructor to initialize GPIO ports, pins, and PWM timer
StepperMotor::StepperMotor(GPIO_TypeDef *enablePort, uint16_t enablePin,
             GPIO_TypeDef *dirPort, uint16_t dirPin,
             TIM_HandleTypeDef *htim, uint32_t timChannel)
    : enablePort(enablePort), enablePin(enablePin),
      dirPort(dirPort), dirPin(dirPin), htim(htim), timChannel(timChannel),
      currentStep(0), targetStep(0), isMoving(false) {}


// Sets the target step position and starts the movement
void StepperMotor::setTarget(int steps) {
    targetStep = steps;
    isMoving = true;
}

void StepperMotor::setSpeed(int speed) {
    // Take input of speed from 0 to 100
    if (speed <= 0) {
    	speed = 0;
//    	HAL_GPIO_WritePin(enablePort, enablePin, GPIO_PIN_RESET);
    	HAL_TIM_PWM_Stop(htim, timChannel);
    	return;
    }
    if (speed > MAX_FREQ) {
    	speed = MAX_FREQ;
    }

    uint32_t period = CLOCK_SPEED / speed;
    uint32_t pulse = (period) / 2;
    period--;
    if(pulse > 0xFFFF) pulse = 0xFFFF;

    __HAL_TIM_SET_AUTORELOAD(htim, period);           // Set ARR for frequency
    __HAL_TIM_SET_COMPARE(htim, timChannel, pulse);

	HAL_TIM_PWM_Start(htim, timChannel);  // Start PWM output

	printf("Period: %i\r\n", period+1);
}

StepperMotor motor1(GPIOB, GPIO_PIN_10, GPIOA, GPIO_PIN_9, &htim1, TIM_CHANNEL_2);
uint32_t pulses1 = 0;

void StartMotorControlTask(void const * argument) {
	while(1) {
//		if(timer1 == 1) {
//			timer1 = 0;
//			pulses1++;
//		}
	}
}

