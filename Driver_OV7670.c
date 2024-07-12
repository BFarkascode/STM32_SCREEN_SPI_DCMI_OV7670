/*
 *  Created on: Jul 5, 2024
 *  Author: BalazsFarkas
 *  Project: STM32_SCREEN_SPI_DCMI
 *  Processor: STM32F429ZI
 *  Program version: 1.0
 *  File: Driver_OV7670.c
 *  Change history:
 */

/*Below are the basic functions to run the OV7670 camera attached to the F429 DISCO board
  Camera needs an external "master" clock signal of 12 MHz (or more). Clock signal is generated using PWM from the DISCO.
  Camera is driven using standard I2C.
  Camera output frequency is 24 MHz (pixel clock). Output is 8 bit parallel data, refresh with the pixel clock.
  Other outputs are vysnch and hsynch (frame and line end indicator flags). Their activity direction can be picked.
  DCMI runs on DMA.

  Note: an adapter board for the camera is HIGHLY recommended to limit noise */

#include "Driver_OV7670.h"

void OV7670_Clock_Start(void){
	/*
	 * We start the clock signal for the camera
	 *
	 */

	TIM3_CH2_PWM_Config(4,2);													//we start the 12 MHz clock signal on PC7 - D7 on the L452-P

}


void OV7670_Find(void){
	/*
	 * We search for the camera on the bus
	 *
	 */

	uint8_t ret;
	uint8_t device_addr;

	for(uint8_t i = 1; i<128; i++) {
		ret = I2C1SCANNER(i);

		if (ret == 0) {

				//do nothing

		}
		else{

			device_addr = i;													//we define the address of the slave device
																				//Note: this works only if we have just one slave on the bus!
		}
	}

	if(device_addr != OV7670_address) {

		printf("Error...OV7670 not found... \r\n");
		while(1);

	} else {

		  //do nothing

	}

}

void OV7670_init (void){

	/*
	 * We initialise the camera using the init matrix
	 *
	 */

	int array_rows = sizeof(OV7670_QVGA)/sizeof(OV7670_QVGA[0]);				//this is going to be 166

	I2C1->CR1 |= (1<<0);														//we turn on PE
	Delay_us(1);

	for(int i = 0; i < array_rows; i++){

		I2C1_Master_Start();
		I2C1_Address_TX(OV7670_address);
		I2C1TX (2, &OV7670_QVGA[i][0]);
		I2C1_Master_Stop();

	}

	I2C1->CR1 &= ~(1<<0);														//we turn off PE and reset the I2C bus
	Delay_ms(1);																//we need to wait 3 APB cycles after PE is cleared. We wait for 1 us instead.

}


void OV7670_DCMI_DMA_init (void){

	/*
	 * We enable DCMI and the connected DMA.
	 *
	 * 1) Enable peripheries and GPIO clocks
	 * 2) GPIO setting for DCMI - PA4, PA6, PB8, PB9, PC6, PC7, PC8, PC9, PC11, PD3 and PG9
	 * 3) DCMI setup
	 * 4) DMA setup
	 * 5) DCMI enable - no DMA enable, that will be done locally in a function
	 *
	 * */

	//1)
	RCC->AHB2ENR |= (1<<0);														//enable DCMI interface
	RCC->AHB1ENR |= (1<<22);													//enable DMA2 clocking
																				//DCMI is on DMA2

	RCC->AHB1ENR |=	(1<<0);														//PORTA clocking
	RCC->AHB1ENR |=	(1<<1);														//PORTB clocking
	RCC->AHB1ENR |=	(1<<2);														//PORTC clocking
	RCC->AHB1ENR |=	(1<<3);														//PORTD clocking
	RCC->AHB1ENR |=	(1<<6);														//PORTG clocking

	//2)Set GPIO parameters (mode, speed, pullup) - PB6 SCL, PB7 SDA
	GPIOA->MODER &= ~(1<<8);													//alternate function for PA4
	GPIOA->MODER |= (1<<9);														//alternate function for PA4
	GPIOA->MODER &= ~(1<<12);													//alternate function for PA6
	GPIOA->MODER |= (1<<13);													//alternate function for PA6
	GPIOB->MODER &= ~(1<<16);													//alternate function for PB8
	GPIOB->MODER |= (1<<17);													//alternate function for PB8
	GPIOB->MODER &= ~(1<<18);													//alternate function for PB9
	GPIOB->MODER |= (1<<19);													//alternate function for PB9
	GPIOC->MODER &= ~(1<<12);													//alternate function for PC6
	GPIOC->MODER |= (1<<13);													//alternate function for PC6
	GPIOC->MODER &= ~(1<<14);													//alternate function for PC7
	GPIOC->MODER |= (1<<15);													//alternate function for PC7
	GPIOC->MODER &= ~(1<<16);													//alternate function for PC8
	GPIOC->MODER |= (1<<17);													//alternate function for PC8
	GPIOC->MODER &= ~(1<<18);													//alternate function for PC9
	GPIOC->MODER |= (1<<19);													//alternate function for PC9
	GPIOC->MODER &= ~(1<<22);													//alternate function for PC11
	GPIOC->MODER |= (1<<23);													//alternate function for PC11
	GPIOD->MODER &= ~(1<<6);													//alternate function for PD3
	GPIOD->MODER |= (1<<7);														//alternate function for PD3
	GPIOG->MODER &= ~(1<<18);													//alternate function for PG9
	GPIOG->MODER |= (1<<19);													//alternate function for PG9
																				//Note: MODER resets to 0x0 here!

	GPIOA->OSPEEDR |= (3<<8);													//high speed PA4
	GPIOA->OSPEEDR |= (3<<12);													//high speed PA6
	GPIOB->OSPEEDR |= (3<<16);													//high speed PB8
	GPIOB->OSPEEDR |= (3<<18);													//high speed PB9
	GPIOC->OSPEEDR |= (3<<12);													//high speed PC6
	GPIOC->OSPEEDR |= (3<<14);													//high speed PC7
	GPIOC->OSPEEDR |= (3<<16);													//high speed PC8
	GPIOC->OSPEEDR |= (3<<18);													//high speed PC9
	GPIOC->OSPEEDR |= (3<<22);													//high speed PC11
	GPIOD->OSPEEDR |= (3<<6);													//high speed PD3
	GPIOG->OSPEEDR |= (3<<18);													//high speed PG9
																				//no pull-up/pull-down

	//Note: AFR values are in the device datasheet. For DCMI, they all are AF13
	GPIOA->AFR[0] |= (13<<16);													//high speed PA4 - AF13
	GPIOA->AFR[0] |= (13<<24);													//high speed PA6 - AF13
	GPIOB->AFR[1] |= (13<<0);													//high speed PB8 - AF13
	GPIOB->AFR[1] |= (13<<4);													//high speed PB9 - ...
	GPIOC->AFR[0] |= (13<<24);													//high speed PC6
	GPIOC->AFR[0] |= (13<<28);													//high speed PC7
	GPIOC->AFR[1] |= (13<<0);													//high speed PC8
	GPIOC->AFR[1] |= (13<<4);													//high speed PC9
	GPIOC->AFR[1] |= (13<<12);													//high speed PC11
	GPIOD->AFR[0] |= (13<<12);													//high speed PD3
	GPIOG->AFR[1] |= (13<<4);													//high speed PG9 - AF13

	Delay_us(1);																//this delay is necessary, otherwise GPIO setup may freeze

	//3)
	DCMI->CR |= (1<<1);															//snapshot mode
	DCMI->CR |= (1<<7);															//VSYNCH active HIGH
																				//EDM and FCRC are 0x00 which indicates 8-bit width and all frames captured

	//4)
	DMA2_Stream1->CR &= ~(1<<0);												//disable stream (just in case...)

	DMA2_Stream1->CR |= (1<<4);													//transfer complete IRQ activated
																				//Note: we will need the TC IRQ so as to reset the DMA after transfer since we are not using circular mode
	DMA2_Stream1->CR |= (1<<25);												//select channel 1
																				//DCMI is on CH1 Stream1 or Stream 7
	DMA2_Stream1->CR |= (1<<14);												//memory data size is 32 bits
	DMA2_Stream1->CR |= (1<<12);												//peri data size is also 32 bits
	DMA2_Stream1->CR |= (1<<10);												//memory increment active
	DMA2_Stream1->PAR = (uint32_t)(&(DCMI->DR));								//we connect the DMA to the DCMI

	//5)
	DCMI->CR |= (1<<14);														//DCMI enabled
}


void OV7670_Capture(uint32_t* frame_buf_location, uint16_t number_of_transfers){
	/*
	 * Function to capture an image from the camera
	 *
	 * 1) We disable the DMA
	 * 2) We set up the DMA for the transfer
	 * 3) We enable the DMA and start the capture
	 *
	 */

	//1)
	DMA2_Stream1->CR &= ~(1<<0);												//disable stream

	//2)
	DMA2_Stream1->PAR = (uint32_t)(&(DCMI->DR));								//we connect the DMA to the DCMI
	DMA2_Stream1->NDTR = number_of_transfers;
	DMA2_Stream1->M0AR = (uint32_t)frame_buf_location;

	//3)
	DMA2_Stream1->CR |= (1<<0);													//enable stream
	DCMI->CR |= (1<<0);															//DCMI capture
																				//Note: snapshot mode automatically clears this bit so we don't need to reset it

}


void DMA2_Stream1_IRQHandler(void)
{
	/*
	 * We need to reset the DMA when transfer is done
	 *
	 */

   DMA2->LIFCR |= (1<<11);														//clear transfer complete flag for stream 1
   	   	   	   	   	   	   	   	   												//Note: the transfer complete flag goes HIGH and then blocks the DMA from re-engaging until cleared

}

void DMA_DCMI_IRQPriorEnable(void) {
	/*
	 * We call the two special CMSIS functions to set up/enable the IRQ.
	 *
	 */

	NVIC_SetPriority(DMA2_Stream1_IRQn, 1);										//IRQ priority for channel 1
	NVIC_EnableIRQ(DMA2_Stream1_IRQn);											//IRQ enable for channel 1
}
