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
 * StateOfCharge.c
 * User Experience Code for the MSP-EXP430FR5969
 * State of Charge Application
 *
 * Program Execution: The state of charge of selected cell is displayed on screen.
 *
 * Directions for use: User uses left slider wheel to move cusor on the screen and
 * selects a cell using the right button. The state of charge of the selected cell
 * is displayed for 4 seconds then returns to cell selection menu. Press left button
 * to return to main menu.
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

#include "StateOfCharge.h"
#include "FR59xx_EXP.h"
#include "myTimer.h"
#include "UpdateDisplay.h"

#include "BQ76PL536/bq_pack.h"
#include "FR59xx_EXP.h"

#include "SPI/spi_if.h"

#define SOC_CELL_1    2
#define SOC_CELL_2    4
#define SOC_CELL_3    6
#define SOC_CELL_4    8
#define SOC_CELL_5    10
#define SOC_CELL_6    12


unsigned char ChooseSOCCell(void);
void StateOfChargeApp(void);
signed int findSOC(unsigned char);

unsigned char CellChoice;



/**********************************************************************//**
 * @brief  Cell Voltage Application - choose a cell (1 to 6) to display its voltage.
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void StateOfChargeApp(void)
{
    while (mode == APP_STATE_OF_CHARGE) {
        select = SEL_CHOOSE_CELL;

        //First menu (choose cell)
        CellChoice = ChooseSOCCell();

        if (select == SEL_RUN_SOC) {
            switch (CellChoice) {
            case SOC_CELL_1:
            	LCD_drawBattery(findSOC(1));                 // Display Cell 1 State of charge
            	TA1_sleep(8192);         // 2s
            	TA1_sleep(8192);         // 2s
                break;
            case SOC_CELL_2:
            	LCD_drawBattery(findSOC(2));                // Display Cell 2 State of charge
            	TA1_sleep(8192);         // 2s
            	TA1_sleep(8192);         // 2s
                break;
            case SOC_CELL_3:
            	LCD_drawBattery(findSOC(3));                // Display Cell 3 State of charge
            	TA1_sleep(8192);         // 2s
            	TA1_sleep(8192);         // 2s
                break;
            case SOC_CELL_4:
            	LCD_drawBattery(findSOC(4));                // Display Cell 4 State of charge
            	TA1_sleep(8192);         // 2s
            	TA1_sleep(8192);         // 2s
                break;
            case SOC_CELL_5:
            	LCD_drawBattery(findSOC(5));                // Display Cell 5 State of charge
            	TA1_sleep(8192);         // 2s
            	TA1_sleep(8192);         // 2s
                break;
            case SOC_CELL_6:
            	LCD_drawBattery(findSOC(6));               // Display Cell 6 State of charge
            	TA1_sleep(8192);         // 2s
            	TA1_sleep(8192);         // 2s
                break;
            default: break;
            }
        }
    }
}

/**********************************************************************//**
 * @brief  Display frequency options on LCD, choose one using left CapTouch
 *         and right button. Return the choice that was made.
 *
 * @param  none
 *
 * @return choice - choice selected
 *************************************************************************/
unsigned char ChooseSOCCell(void)
{
    unsigned char choice = 0;


        while(select == SEL_CHOOSE_CELL){

        // Get left slider position
        	CTS_getReading();

            if (sliderPos[0] < 10) {
            	choice = SOC_CELL_6;
            }
            else if (sliderPos[0] < 16) {
            	choice = SOC_CELL_5;
            }
            else if (sliderPos[0] < 22) {
            	choice = SOC_CELL_4;
            }
            else if (sliderPos[0] < 28) {
            	choice = SOC_CELL_3;
            }
            else if (sliderPos[0] < 34) {
            	choice = SOC_CELL_2;
            }
            else {
            	choice = SOC_CELL_1;
            }
        LCD_displayCellSelMenu(choice);      // show menu, highlighting selected option
        LCD_display();
    }
    return choice;
}

signed int findSOC(unsigned char cellnum){

	signed int batteryBars;

	update_bq_pack_data();

        //if 1 bar remains then battery is low and 8 bars remaining means fully charged (from open circuit voltage)
		if(bq_pack.bq_devs[0].cell_voltage[cellnum - 1] < 3200){            //if cell is less than 3300 mV then 1 bar remains
		    batteryBars = 0;
		}
		else if(bq_pack.bq_devs[0].cell_voltage[cellnum - 1] < 3300){       //if cell is less than 3400 mV then 2 bars remain
			batteryBars = 1;
		}
		else if(bq_pack.bq_devs[0].cell_voltage[cellnum - 1] < 3400){       //if cell is less than 3400 mV then 2 bars remain
			batteryBars = 2;
		}
		else if(bq_pack.bq_devs[0].cell_voltage[cellnum - 1] < 3500){       //if cell is less than 3500 mV then 3 bars remain
			batteryBars = 3;
		}
		else if(bq_pack.bq_devs[0].cell_voltage[cellnum - 1] < 3600){       //if cell is less than 3600 mV then 4 bars remain
			batteryBars = 4;
		}
		else if(bq_pack.bq_devs[0].cell_voltage[cellnum - 1] < 3700){       //if cell is less than 3700 mV then 5 bars remain
			batteryBars = 5;
		}
		else if(bq_pack.bq_devs[0].cell_voltage[cellnum - 1] < 3800){       //if cell is less than 3800 mV then 6 bars remain
			batteryBars = 6;
		}
		else if(bq_pack.bq_devs[0].cell_voltage[cellnum - 1] < 3900){       //if cell is less than 3900 mV then 7 bars remain
			batteryBars = 7;
		}
		else if(bq_pack.bq_devs[0].cell_voltage[cellnum - 1] < 4000){       //if cell is less than 4000 mV then 8 bars remain
			batteryBars = 8;
		}
		else{
			batteryBars = 8;
		}

		return batteryBars;

}

