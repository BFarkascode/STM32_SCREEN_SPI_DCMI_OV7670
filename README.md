# STM32_SCREEN_SPI_DCMI_OV7670

Bare metal project on a STM32_F429_DISCO board to extract image from an OV7670 CMOS camera. This is the second project in a sequence of projects aimed.  

## General description
With the base project successfully set up, the next natural step is to disassemble the project to its elements and replace everything with custom solutions. This should allow us a lot of flexibility later when, for instance, we would want to use the LTDC and the DCMI on our DISCO board the same time. CubeMX simply doesn’t allow that to happen due to two pins overlapping. Manually though, we should be able to force the solution with minimal data loss. Having custom solutions also allows us to “trim the fat” from the base project further and have something completely bare-bones for the future.

## What shall we do then?
Firstly, we don’t want TouchGFX (at least, for now), it is perfectly adequate for us to have an image generated by us manually. So our main task will be to find out, where the TouchGFX is interfacing with our code, severe that tie and put something “less complex” on there.

Secondly, we can’t “wing” anymore the ILI driving. We need to get into it, understand how it works (timing, commands and data transfer) and then remove from the provided driver what we don’t need. This is particularly important with the first point above since synchronization and proper layering of the output will not be done for us through TouchGFX, we will have to set up the data flow (timing and pixel positioning) ourselves. 

Thirdly, once we are rock solid in publishing our constant custom image to the screen, we will need to take a look into how to drive our OV7670 CMOS camera, again with timing, commands and data transfer at the heart of the problem. We will be using I2C to command the camera but receive data from it using a parallel serial interface.

Fourth, we can hook up our CMOS camera using the DCMI+DMA interface of the STM32F429 and ensure that, yet again, the data flow and the timing match.

Buckle up, because this will be a complex one and I will skip some steps. I will indicate though which overlapping project allows me to do so. 

## Nose to the grindstone 
### L0xx vs F4xx
Before all these points above, in Zero-th(?) position, we will have to talk about the differences between the MCU we have in our DISCO board (F429) and the NUCLEO board (L053) of other projects. Generally, it can be said that there is a high level of compatibility between STM32 MCUs regarding code, especially if one decides to use HAL.

When doing bare metal, this isn’t the case. Many times, a few registers’ names are changed, meaning that porting of existing code would not work at all without minimal modifications. In other cases, the peripherals might be completely different and would demand rewriting of existing drivers.

In our case, there are four glaring differences between the L0xx and the F4xx:
1)	Clocking is differently organized. This means that register names are different, plus peripherals might be placed on different busses (APB1, APB2, AHB). I simply let HAL do the clocking, the project is already too big.
2)	The I2C peripheral between the two MCUs is completely different. It is necessary to write a brand-new driver for the F429. More on this below.
3)	We have a significantly more advanced DMA on the F429 with a lot more channels/streams than on the L0xx. I won’t go into this; it is not complicated to make heads or tails of it using the refman of the F429. The STM32_DMADriver project can be of help.
4)	GPIO MODER register resets to 0x0, not 0xFF.

### I2C on the F429
Unlike in the L053 where we could rely on a lot of automation within the peripheral (such as using AUTOEND to generate a stop bit, or using different registers for Tx/Rx, or publish the slave address in CR2, or flip a bit in the CR1 register to flush our Tx, or have ACK automatically), we will need to do almost everything manually. We need to write functions that generate start/stop bits, one to send the slave address and one to send data.

The start/stop generating functions are rather straight forwards, we merely need to set a bit in the CR1 register and wait for a flag to go HIGH indicating success. Mind, within the start condition, it is also necessary to enable the master’s acknowledge towards the slave.

Addressing is done by writing to the DR register – the only data transfer register – after a start condition and waiting for the ADDR bit to go HIGH. The ADDR bit will ONLY go HIGH in case there is match in the address, so it can only be used for timing if we are sure about the address. Also, there is a redundancy with the AF/acknowledge bit (I am not sure, why this makes sense, but it can hard lock the code pretty easily). For scanning the bus for addresses, the ADDR bit MUST NOT be checked for timing, we need to solely use the AF bit (see code).

We pick Tx or Rx by setting the LSB in the address. This also means that we need to shift left (“<<1”) any address before we put it into the DR register. LSB 0 is Tx, LSB 1 is Rx. We will only have Tx here. 
Timing is done by defining the driving frequency of the peripheral (APB1 peripheral clock for I2C1) within the CR2 register (must be the same value put as the APB1 frequency in MHz), set the division rate in the CCR register and set the rising time in the TRISE register. The calculations to get the CCR and TRISE are within the refman.

Of note, we clean the flags within the peripheral by reading the SR registers. Doing so does clean ALL flags, so tread lightly and not accidentally remove flags that should not be cleaned.

### Where to TouchGFX?
Investigating the base project, we can see that TouchGFX interfaces with the screen through the ILI driver. More precisely, it calls “ILI9341_SetWindow(x, y, x+w-1, y+h-1)” to set the window we will be writing to on the screen and then the “ILI9341_DrawBitmap(w, h, pixels);” to actually “fill” that window with a bitmap (i.e., with an image). Unfortunately, the width, the height and the pixel pointer (as well as the frame buffer!) for that call are all set up within TouchGFX, so we will have to do it manually to remove TouchGFX. Checking the “ILI9341.c”, setting the window is just a set of commands (what they are, see below), nothing complicated. For drawing the bitmap, we are using the memory stepping within the DMA of the SPI. As such, if we manage to generate a frame buffer ourselves, add a pointer to it and define the proper height/width, everything should be fine.

As mentioned in the base project description, we have a bit of a memory allocation problem where, if we have TouchGFX active, we can’t store an entire image within RAM due to lack of space (we will have at least 70 kB of the 196kB memory used up in RAM without doing anything…while an image is 150 kB). TouchGFX gets around this by using/generating smaller memory frame buffers and then “reconstruct” the image, section by section, which is a pretty efficient way to save memory and sacrifice speed. I do want to have speed eventually…plus, if we don’t use TouchGFX, we can store the entire image in RAM. You see where I am getting to…

So, if no TouchGFX, we can simply define our frame buffer as a variable (an array of 153000 bytes for a 16-bit depth image size of 320x240) and then define a pointer to the first element of the array. The DMA will be able to deal with it from there…

…except that it won’t. DMA’s need to know, how many elements they should transfer ahead of the transfer…and the register to define the number of transfers is capped at 65k (the register is 16-bits wide). As such, we can’t transfer our image in one fell swoop to the screen using the DMA since each transfer has to be 8 bits to comply the SPI. Our custom image transfer function to remove TouchGFX this has to also do the transfer in sections (see the function “Transmit320x240Frame” in the “image_transfer” source file).
I ended up going with a division of 4. Each consecutive step must have the frame buffer pointer manually set to the next step with the window set accordingly. Since we are using DMA for the transfer and we aren’t blocking, we need to also add a delay between the sections, otherwise we will have overrunning of the data.

#### What to publish?
With TouchGFX evicted from our code, we don’t have a way to fill up our frame buffer. I wrote an “GenerateImage” function that will solve that problem and fill up the frame buffer for us with various different patterns. It is possible to choose currently between 5 different patterns (see code, just uncomment the one that you fancy). I picked these five since my experience suggests that they are especially useful for debugging screen timing problems, something that will likely come around once we start to play around with speeding things up. Using a very complex pattern is bad practice for those kind of debugging sections since it prevents us for having a well-educated guess of where the timing goes off.

## ILI
Using simple patterns, it is extremely likely that one will be able to see a desired outcome on the screen without any investigation of the ILI. Unfortunately, when are interested in publishing camera output on a screen, every pixel must go to its exact place, otherwise we will end up with torn image, a rolling image or nothing at all.

This means that we will have to load up our frame buffer exactly the same way how we are publishing it.

In our particular case, we have the image cut into 4 sections. As mentioned above, each section’s size must match the window we set on the ILI (with a bunch of SPI commands to place the starting and the end point within the pixel matrix of the screen). These sections must also be retrieved from the RAM in the exact sequence. Lastly, publishing them on the screen have to match the orientation of the screen. Thus, we need to manipulate the 36h register to define the frame buffer readout in case it does not match the loading of the buffer (in the code provided, we rotate the frame buffer readout by 270 degrees). We can also tell the endian of the readout in case that becomes necessary (F6h).

The ILI runs on its own clock, refreshing the screen from the frame buffer (see B1h register).

We pick the bit depth of the screen in register 3Ah…

…and here is something I want to get into a bit deeper. In the base project I mentioned that I don’t understand what we are using the drive the ILI since we seem to be hard-wired to SPI, yet we still have some LTDC activity on the bus. As it turns out, we have TWO different interfaces, one to control the ILI and one to send it data. The control interface is defined by the IM[3:0] bits and they are hard-wired on our board to be SPI interface II. We can’t change that. What we can change is how we want to send data to the ILI and what it does with it. It can directly publish it as it arrives or it can store it within its own GRAM, which is a localised version of the frame buffer before publishing it. We can also tell the ILI to capture the incoming data from a serial RGB interface or from SPI (see F6h, B0h and 3Ah registers). Without telling it anything, it will capture data from SPI and bypass its GRAM. As such, what happens in the original “base project” is that the LTCD interface is activated in the MCU to send the RGB data to the ILI, that’s why I saw some traffic on those pins. I will explore this further in the next project.

All in all, the ILI driver we had in the base project had a lot more registers manipulated than what we actually needed. I removed them – for now. (Of note, we also use 11h to turn off sleeping, 29h to turn on the display, 1h to reset the display and 2Ch to tell the ILI we are sending data to it.)

Lastly, I need to talk about the command architecture of the ILI. It is commanded using SPI (thanks to the pre-set IM bits) so we have a CS chip select that will have to be controlled using a normal GPIO output in the F429. In the DISCO board, the CS pin is on PC2. We also have the DC pin on the ILI, which is a command input: if it is HIGH, we will be sending it data, if it is LOW, we are sending it a command/register. The DC pin is also a simple GPIO output, connected to PD13 in the DISCO. 
