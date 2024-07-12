/* USER CODE BEGIN Header */
/**
 *  Author: BalazsFarkas
 *  Project: STM32_SCREEN_SPI_DCMI
 *  Processor: STM32F429ZI
 *  Compiler: ARM-GCC (STM32 IDE)
 *  Program version: 1.0
 *  File: main.c
 *  Hardware description/pin distribution: 	I2C pins on PB6 and PB7
 *						camera master clock on PA7
 *      					DCMI pins on PA4, PA6,PB8,PB9,PC6,PC7,PC8,PC9,PC11,PD3 and PG9
 *	     					screen SPI on PF7,PF8 an PF9, CS on PC2, DC pin on PD13
 *  Change history: N/A
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "image_transfer.h"
#include "ClockDriver_STM32F4xx.h"
#include "I2CDriver_STM32F4xx.h"
#include "Driver_OV7670.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t image[153600];										//this image will be stored in RAM!!!
uint8_t* image_read_ptr = &image[0];						//we skip one pixel. Seems to be an artifact?
uint32_t* image_write_ptr = &image[0];
uint8_t OV7670_address = 0x21;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  SysClockConfig();

  TIM6Config();																	//custom delay function
  I2C1Config();																	//I2C for camera communications

  //---------Set up screen interface----------//

  SPI5_w_DMA_Config();
  DMA_SPI5_IRQPriorEnable();

  //---------Set up screen interface----------//

  //---------Set up screen----------//

  ILI9341_Init();

  //---------Set up screen----------//

  //---------Generate an image----------//
#ifdef generated_image_output

  GenerateImage();															//this fills the frame buffer with a constant image

#endif
  //---------Generate an image----------//

  //---------Set up camera----------//

  OV7670_Clock_Start();

  OV7670_Find();

  OV7670_init();


  //---------Set up camera----------//

  //---------Set up camera interface----------//

  OV7670_DCMI_DMA_init();

  DMA_DCMI_IRQPriorEnable();

  //---------Set up camera interface----------//

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //---------Capture camera image----------//

	  OV7670_Capture(image_write_ptr, 38400);
	  Delay_ms(500);

	  //---------Capture camera image----------//

	  //---------Publish image----------//

	  Transmit320x240Frame(image_read_ptr);										//here we read out what we have captured
	  Delay_ms(50);

	  //---------Publish image----------//

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
