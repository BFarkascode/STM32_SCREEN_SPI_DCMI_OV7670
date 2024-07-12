/*
 *  Created on: Jul 5, 2024
 *  Author: BalazsFarkas
 *  Project: STM32_SCREEN_SPI_DCMI
 *  Processor: STM32F429ZI
 *  Program version: 1.0
 *  Header file: I2CDriver_STM32F4xx.h
 *  Change history:
 */

#ifndef INC_I2CDRIVER_STM32L4X2_H_
#define INC_I2CDRIVER_STM32L4X2_H_

#include "stdint.h"
#include "ClockDriver_STM32F4xx.h"

//LOCAL CONSTANT

//LOCAL VARIABLE

//EXTERNAL VARIABLE

//FUNCTION PROTOTYPES
void I2C1Config(void);
int I2C1SCANNER (uint8_t slave_addr);
void I2C1TX (uint8_t number_of_bytes, uint8_t *bytes_to_send);
void I2C1_Master_Start(void);
void I2C1_Address_TX(uint8_t slave_addr);
void I2C1_Master_Stop(void);


#endif /* INC_I2CDRIVER_STM32L4X2_H_ */
