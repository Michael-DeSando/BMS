/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//*****************************************************************************
//
// Sharp96x240.hh - Prototypes for the Sharp400x240 LCD
//                                     display driver 
// Copyright (c) 2008-2011 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
//
//*****************************************************************************


#ifndef __SHARPLCD_H__
#define __SHARPLCD_H__

#include "grlib/grlib.h"

//*****************************************************************************
//
// User Configuration for the LCD Driver
//
//*****************************************************************************
// LCD Screen Dimensions
#define LCD_VERTICAL_MAX    96
#define LCD_HORIZONTAL_MAX  96

// Define LCD Screen Orientation Here
#define LANDSCAPE

//Maximum Colors in an image color palette
#define MAX_PALETTE_COLORS  2

#define SHARP_SEND_TOGGLE_VCOM_COMMAND		0x01
#define SHARP_SKIP_TOGGLE_VCOM_COMMAND 		0x00

#define SHARP_LCD_TRAILER_BYTE				0x00

#define SHARP_VCOM_TOGGLE_BIT 		   		0x40

#define SHARP_LCD_CMD_CHANGE_VCOM			0x00
#define SHARP_LCD_CMD_CLEAR_SCREEN			0x20
#define SHARP_LCD_CMD_WRITE_LINE			0x80

//*****************************************************************************
//
// Macros for the Display Driver
//
//*****************************************************************************

//
// Translates a 24-bit RGB color to a display driver-specific color.
//
// \param c is the 24-bit RGB color.  The least-significant byte is the blue
// channel, the next byte is the green channel, and the third byte is the red
// channel.
//
// This macro translates a 24-bit RGB color into a value that can be written
// into the display's frame buffer in order to reproduce that color, or the
// closest possible approximation of that color. This particular driver
// requires the 8-8-8 24 bit RGB color to convert into 5-6-5 16 bit RGB Color
//
// \return Returns the display-driver specific color

#define DPYCOLORTRANSLATE(c)    (((c) & 0xff))
//
// Macro used to set the LCD data bus 
//
// \param uByte is the 8 or 16 bit (depending on parallel bus type)
// value to write to bus
//
// Define depending on the bus width, 8 or 16 pins, a char or an int
// is written to the Port Pin registers
//
// \return None


//
// Writes command or data to the LCD Driver
//
// \param ucCmdData is the 8 or 16 bit command to send to the LCD driver
// Uses the SET_LCD_DATA macro
//
// \return None

#define WriteCmdData(ucCmdData)                    	\
        do                                         	\
        {                                           \
            while (!(UCB0IFG & UCTXIFG));      		\
            UCB0TXBUF = ucCmdData;                  \
        }                                           \
        while(0)



//
// Sets the cursor to coordinates X, Y. Increment from Left to Right
//
// \param X and Y are the LCD pixel coordinates to place the cursor
//
// This macro sets the cursor location, and sets auto incrementing
// of the X value from left to right on the LCD screen. This is used
// when drawing rows of pixels in images or lines. Upon exiting this 
// macro, the LCD should be ready to accept a stream of data
//
// Note that left to right is relative to the screen orientation, but
// HORIZ_DIRECTION is defined depending on screen orientation to always
// be left to right. (See Coordinate Space and Mapping above)
//
// \return None

#define SetCursorLtoR(X, Y)                                                \
    __no_operation();

//
// Sets the cursor to coordinates X, Y. Increment from Top to Bottom
//
// \param X and Y are the LCD pixel coordinates to place the cursor
//
// This macro sets the cursor location, and sets auto incrementing
// of the Y value from top to bottom on the LCD screen. This is used
// when drawing rows of pixels in images or lines. Upon exiting this 
// macro, the LCD should be ready to accept a stream of data
//
// Note that top to bottom is relative to the screen orientation, but
// VERT_DIRECTION is defined depending on screen orientation to always
// be top to bottom. (See Coordinate Space and Mapping above)
//
// \return None

#define SetCursorTtoB(X, Y)                                                \
    __no_operation();

extern unsigned char VCOMbit;
extern tContext sContext;
extern unsigned char DisplayBuffer[LCD_VERTICAL_MAX][LCD_HORIZONTAL_MAX / 8];

//*****************************************************************************
//
// Prototypes for the globals exported by this driver.
//
//*****************************************************************************
//extern void WriteDataGPIO(unsigned short usData);
extern void Sharp96x96_LCDInit(void);
extern const tDisplay g_sharp96x96LCD;
extern void Sharp96x96_SendToggleVCOMCommand();
#endif // __SHARPLCD_H__
