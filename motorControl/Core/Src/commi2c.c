/*
 * commi2c.c
 *
 *  Created on: Feb 5, 2024
 *      Author: akamdhillon
 */

#include "commi2c.h"
#include "main.h"

#define SEND_DATA_SIZE		8
#define RECEIVE_DATA_SIZE	4

extern I2C_HandleTypeDef hi2c1;

volatile uint8_t I2C_Recieved;
volatile uint8_t startRecieveIT;

// Global buffer and flag for transmit
volatile uint8_t txBuffer[SEND_DATA_SIZE];
volatile uint8_t txDataReady = 0;

// Global buffer for receive
volatile uint8_t rxBuffer[RECEIVE_DATA_SIZE];
uint8_t motorData[RECEIVE_DATA_SIZE];



void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // Transmission complete callback
    txDataReady = 0; // Reset flag after sending data
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	memcpy(motorData,rxBuffer,RECEIVE_DATA_SIZE);
	I2C_Recieved = 1;
	startRecieveIT = 1;
}

void PrepareDataToSend() {
    // Prepare data to be sent in response to a master request
    //txBuffer[0] = ...; // Your data here
    //txBuffer[1] = ...;
    txDataReady = 1;
}

void StartI2CSendTask(void const * argument) {
    osDelay(10); // Initial delay

    while(1) {
        if(txDataReady) {
            // Data is prepared and ready to be sent upon master request
            HAL_I2C_Slave_Transmit_IT(&hi2c1, txBuffer, SEND_DATA_SIZE); // Non-blocking
            txDataReady = 0; // Reset the flag
        }
        osDelay(10); // Short delay or wait for an event
    }
}



void StartI2CReceiveTask(void const * argument) {

    startRecieveIT = 1;

    osDelay(10);

    while(1) {
    	if(startRecieveIT) {
    		startRecieveIT = 0;
    		HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)rxBuffer, RECEIVE_DATA_SIZE);
    	}

        osDelay(10); // Short delay or wait for an event
    }
}

