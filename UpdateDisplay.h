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
/*******************************************************************************
 *
 * UpdateDisplay.h
 * User Experience Code for the MSP-EXP430FR5969
 * main LCD/graphics header file
 *
 * Created: Version 1.0:
 *
 ******************************************************************************/

#ifndef UPDATE_DISPLAY
#define UPDATE_DISPLAY

extern void LCD_turnOn(void);                                                                     // Turn LCD on
extern void LCD_display(void);                                                                    // Pull DISP high
extern void LCD_turnOff(void);                                                                    // Turn LCD off
extern void LCD_introWrite(void);                                                                 // Write intro
extern void LCD_displayMainMenu(unsigned char choice);                                            // display the main menu
extern void LCD_drawBattery(signed int bars);                                                     // Draw battery remaining
extern void LCD_drawLowBattery(signed int bars);                                                  // Draw low battery warning
extern void LCD_displaySendingScreen(void);                                                       // Sending screen
extern void LCD_showDataSent(unsigned int bytes);                                                 // done sending screen

/*begin BMS display functions
 * author: Michael DeSando a0221117
 * June 2014*/

extern void LCD_displayCommunicationError(void);                                                  // Display error message: Device not detected

extern void LCD_displayCellSelMenu(unsigned char choice);                                         // Display cell voltage menu

extern void LCD_displayCellVoltage(void);                                                         // Display cell 1 voltage
extern void LCD_displayCel2Voltage(void);                                                         // Display cell 2 voltage
extern void LCD_displayCel3Voltage(void);                                                         // Display cell 3 voltage
extern void LCD_displayCel4Voltage(void);                                                         // Display cell 4 voltage
extern void LCD_displayCel5Voltage(void);                                                         // Display cell 5 voltage
extern void LCD_displayCell6Voltage(void);                                                        // Display cell 6 voltage
extern void LCD_displayPackVoltage(void);                                                         // Display pack voltage
extern void LCD_displayTemperature(void);                                                         // Display pack temperatures (both of them)

extern void LCD_displayCellsImbalanced(void);                                                 // Display Cell Imbalance detected
extern void LCD_displayCellsBalanced(void);                                                  // Display Cell Imbalance not detected

/*end BMS display functions*/

extern void LCD_drawPicture(int choice);                    // Draw a picture (from .h file)
extern int UnsignedInt_To_ASCII(unsigned int hex, char *ASCII);
extern int UnsignedLong_To_ASCII(unsigned long hex, char *ASCII);

#endif
