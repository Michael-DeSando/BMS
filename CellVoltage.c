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
 * CellVoltage.c
 * User Experience Code for the MSP-EXP430FR5969
 * Cell Voltage Application
 *
 * Program Execution: User selects which cell to display its cell voltage in mV.
 *
 * Directions for use: User uses slider wheel to navigate the cursor and then selects the cell by pressing
 * the right button. The cell voltage displays for 4 seconds then returns to the cell selection menu. Click
 * the left button to return to main menu.
 *
 * June 2014
 *
 * Author: Michael DeSando a0221117
 *
 ******************************************************************************/

#include "msp430.h"

#include "driverlib.h"
#include "grlib/sharp96x96.h"
#include "grlib/grlib/grlib.h"
#include "CTS/CTS_wolverine.h"

#include "CellVoltage.h"
#include "FR59xx_EXP.h"
#include "myTimer.h"
#include "UpdateDisplay.h"

#include "BQ76PL536/bq_pack.h"
#include "FR59xx_EXP.h"

#include "SPI/spi_if.h"

#define CV_CELL_1    2
#define CV_CELL_2    4
#define CV_CELL_3    6
#define CV_CELL_4    8
#define CV_CELL_5    10
#define CV_CELL_6    12


unsigned char ChooseCVCell(void);
void CellVoltApp(void);

unsigned char CellChoice;



/**********************************************************************//**
 * @brief  Cell Voltage Application - choose a cell (1 to 6) to display its voltage.
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void CellVoltApp(void)
{

    while (mode == APP_CELL_VOLTAGE) {
        select = SEL_CHOOSE_CELL;


        //First menu (choose cell)
        CellChoice = ChooseCVCell();

            if (select == SEL_RUN_CV) {
                switch (CellChoice) {
                case CV_CELL_1:
                	LCD_displayCell1Voltage();                 // Display Cell 1 Voltage
                	TA1_sleep(8192);         // 2s
                	TA1_sleep(8192);         // 2s
                    break;
                case CV_CELL_2:
                	LCD_displayCell2Voltage();                 // Display Cell 2 Voltage
                	TA1_sleep(8192);         // 2s
                	TA1_sleep(8192);         // 2s
                    break;
                case CV_CELL_3:
                	LCD_displayCell3Voltage();                // Display Cell 3 Voltage
                	TA1_sleep(8192);         // 2s
                	TA1_sleep(8192);         // 2s
                    break;
                case CV_CELL_4:
                	LCD_displayCell4Voltage();               // Display Cell 4 Voltage
                	TA1_sleep(8192);         // 2s
                	TA1_sleep(8192);         // 2s
                    break;
                case CV_CELL_5:
                	LCD_displayCell5Voltage();              // Display Cell 5 Voltage
                	TA1_sleep(8192);         // 2s
                	TA1_sleep(8192);         // 2s
                    break;
                case CV_CELL_6:
                	LCD_displayCell6Voltage();             // Display Cell 6 Voltage
                	TA1_sleep(8192);         // 2s
                	TA1_sleep(8192);         // 2s
                    break;
                default: break;
                }
            }
        }
    }

/**********************************************************************//**
 * @brief  Display cell options on LCD, choose one using left CapTouch
 *         and right button. Return the choice that was made.
 *
 * @param  none
 *
 * @return choice - choice selected
 *************************************************************************/
unsigned char ChooseCVCell(void)
{
    unsigned char choice = 0;

        while(select == SEL_CHOOSE_CELL){

        // Get left slider position
        	CTS_getReading();

            if (sliderPos[0] < 10) {
            	choice = CV_CELL_6;
            }
            else if (sliderPos[0] < 16) {
            	choice = CV_CELL_5;
            }
            else if (sliderPos[0] < 22) {
            	choice = CV_CELL_4;
            }
            else if (sliderPos[0] < 28) {
            	choice = CV_CELL_3;
            }
            else if (sliderPos[0] < 34) {
            	choice = CV_CELL_2;
            }
            else {
            	choice = CV_CELL_1;
            }
        LCD_displayCellSelMenu(choice);       // show menu, highlighting selected option
        LCD_display();
    }
    return choice;
}

