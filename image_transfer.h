/*
 *  Created on: Jun 26, 2024
 *  Author: BalazsFarkas
 *  Project: STM32_SCREEN_SPI_DCMI
 *  Processor: STM32F429ZI
 *  Program version: 1.0
 *  Header file: image_transfer.h
 *  Change history:
 */

#ifndef INC_IMAGE_TRANSFER_H_
#define INC_IMAGE_TRANSFER_H_

#include <stdint.h>

//LOCAL CONSTANT

//LOCAL VARIABLE

//EXTERNAL VARIABLE
extern uint8_t image[153600];

//FUNCTION PROTOTYPES
void Transmit320x240Frame(uint8_t* half_pixels);
uint8_t* GenerateImage(void);

#endif /* INC_IMAGE_TRANSFER_H_ */
