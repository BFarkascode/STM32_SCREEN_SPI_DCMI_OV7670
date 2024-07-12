/*
 *  Created on: Jun 26, 2024
 *  Author: BalazsFarkas
 *  Project: STM32_SCREEN_SPI_DCMI
 *  Processor: STM32F429ZI
 *  Program version: 1.0
 *  File: image_transfer.c
 *  Change history:
 */

#include "ili9341.h"
#include "image_transfer.h"

void Transmit320x240Frame(uint8_t* half_pixels)

/*
 * The image transfer takes 4 steps with regular DMA since the DMA is limited to a 65k byte transfer (uint16_t).
 * The entire image is 153k bytes.
 *
 * Note: it is hugely relevant, how we choose the section we want to fill and also, how we load it up
 * Note: we are currently publishing the blocks on the screen along the X axis
 * Note: the section that fails to render is of unknown origin. The image is properly generated and logged, but the transfer fails.
 *
 * */
{
	ILI9341_SetWindow(0, 0, 319, 59);							//we set the window to cover the entire screen
	ILI9341_DrawBitmap(320, 60, half_pixels);					//we publish the entire image
	Delay_ms(50);
	half_pixels += 38400;
	ILI9341_SetWindow(0, 60, 319, 119);							//we set the window to cover the entire screen
	ILI9341_DrawBitmap(320, 60, half_pixels);					//we publish the entire image
	Delay_ms(50);
	half_pixels += 38400;
	ILI9341_SetWindow(0, 120, 319, 179);						//we set the window to cover the entire screen
	ILI9341_DrawBitmap(320, 60, half_pixels);					//we publish the entire image
	Delay_ms(50);
	half_pixels += 38400;
	ILI9341_SetWindow(0, 180, 319, 239);						//we set the window to cover the entire screen
	ILI9341_DrawBitmap(320, 60, half_pixels);					//we publish the entire image

	//Problem: normal DMA is physically limited to, at maximum, 65k transfers (definition register is uin16_t)

}

uint8_t* GenerateImage(void){

	//Note: bit transfer is MSB first or LSB first depending on the ILI setup (though it doesn't seem to be the case)
	//Note: byte transfer is LSB first

//	uint8_t image[153600];										//this image will be stored in RAM!!!
/*
	//mono-colour
	for(int i = 0; i < 153600; i+=2){

			//white
//			image[i+1] = 0xFF;
//			image[i] = 0xFF;

			//red
			image[i+1] = 0xF8;
			image[i] = 0x00;

			//blue
//			image[i+1] = 0x00;
//			image[i] = 0x1F;

			//green
//			image[i+1] = 0x07;
//			image[i] = 0xE0;

		}
*/

/*
	//screen divided into 4 different colour
	for(int i = 0; i < 153600; i+=2){

		if(((i/320) > 119) & ((i/320) <= 239)) {				//this is 119 since we are going byte by byte...which means 120 bytes to define 60 pixels

			image[i+1] = 0xFF;
			image[i] = 0xFF;

		}else if(((i/320) > 239) & ((i/320) <= 359)) {

			image[i+1] = 0xF8;
			image[i] = 0x00;

		} else if((i/320) > 359) {

			image[i+1] = 0x00;
			image[i] = 0x1F;

		} else {

			image[i+1] = 0x07;
			image[i] = 0xE0;

		}

	}
*/
/*
	//one pixel lines
	for(int i = 0; i < 153600; i+=8){

			image[i+1] = 0x07;
			image[i] = 0xE0;

			image[i+2] = 0xFF;
			image[i+3] = 0xFF;

			image[i+5] = 0xF8;
			image[i+4] = 0x00;

			image[i+7] = 0x00;
			image[i+6] = 0x1F;

	}
*/
/*
	//one colour transition
	for(int i = 0; i < 153600; i+=2){

		image[i] = 0x0;
		image[i+1] = ((i/2) % 320) << 3;

	}
*/


	//advanced image
	for(int i = 0; i < 153600; i+=2){

		//mimicked after a similar pattern generated for the FPGA project

		uint8_t W;

		if(((i/2) / 320) == ((i/2) % 320)) {

			W = 0xFF;

		} else {

			W = 0x0;

		}

		uint8_t A;

		if(((((i/2) / 320) & 0x20) == 0x20) && ((((i/2) % 320) & 0x20) == 0x20)) {

			A = 0xFF;

		} else {

			A = 0x0;

		}

		uint8_t pat_43;

		if(  ((((i/2) / 320) >> 3) & 0x3) == ((((i/2) % 320) >> 3) & 0x3)  ) {

			pat_43 = 0xFF;

		} else {

			pat_43 = 0xC0;

		}

		uint8_t red = ((((((i/2) % 320) & 0x1F) & (pat_43)) << 2) | W );						//5-bits of RED
		uint8_t green = ((((i/2) % 320) & A ) | W);												//6-bits of GREEN
		uint8_t blue = (((i/2) / 320) | W) & 0x1F;												//5-bits of BLUE - DONE

		image[i] = (green << 5) | blue;
		image[i+1] = (green >> 5) | (red << 3);

	}

}
