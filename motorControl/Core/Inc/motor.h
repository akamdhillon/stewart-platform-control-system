/*
 * motor.h
 *
 *  Created on: Nov 6, 2024
 *      Author: akamdhillon
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_



#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
void StartMotorControlTask(void const * argument);

#ifdef __cplusplus
}
#endif


class StepperMotor {

public:

    GPIO_TypeDef *enablePort;
    uint16_t enablePin;
    GPIO_TypeDef *dirPort;
    uint16_t dirPin;
    TIM_HandleTypeDef *htim; // PWM timer handle for step generation
    uint32_t timChannel;
    int currentStep;
    int targetStep;
    bool isMoving;

    // Constructor to initialize GPIO ports, pins, and PWM timer
    StepperMotor(GPIO_TypeDef *enablePort, uint16_t enablePin, GPIO_TypeDef *dirPort, uint16_t dirPin, TIM_HandleTypeDef *htim, uint32_t timChannel);
    void setTarget(int steps);
    void setSpeed(int speed);
};


#endif /* INC_MOTOR_H_ */
