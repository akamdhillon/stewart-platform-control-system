/*
 * commi2c.h
 *
 *  Created on: Feb 5, 2024
 *      Author: akamdhillon
 */

#ifndef INC_COMMI2C_H_
#define INC_COMMI2C_H_

#include <stdio.h>
#include "cmsis_os.h"


void StartI2CSendTask(void const * argument);
void StartI2CReceiveTask(void const * argument);

#endif /* INC_COMMI2C_H_ */
