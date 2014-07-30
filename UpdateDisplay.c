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
 * UpdateDisplay.c
 * User Experience Code for the MSP-EXP430FR5969
 * Display things on the LCD
 *
 * February 2012
 *
 *************************************************************************//****
 * Rev 1.1 - FRAM Write > 1GB bug fixed --> totalString array for updating LCD
 *           with write speeds was extended, thus preventing decimalString array
 *           from being overwritten.
 ******************************************************************************/
#include <string.h>

#include "driverlib.h"
#include "grlib/sharp96x96.h"
#include "grlib/grlib/grlib.h"

#include "UpdateDisplay.h"
#include "FR59xx_EXP.h"
#include "Preloaded_Images/TI_logo.h"
#include "Preloaded_Images/Wolverine_slash.h"

#include "BQ76PL536/bq_pack.h"

#define NORMAL_DISPLAY

/**********************************************************************//**
 * @brief  Turn on LCD
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
// TODO: Move to display driver
void LCD_turnOn(void)
{
    P4OUT |= BIT2;                          // Turn on LCD (Vcc = HI)
}

/**********************************************************************//**
 * @brief  Pull LCD DISP pin high
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
// TODO: Move to display driver
void LCD_display(void)
{
    P4OUT |= BIT3;                            // Enable LCD output (DISP = HI)
}

/**********************************************************************//**
 * @brief  Turn off LCD
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
// TODO: Move to display driver
void LCD_turnOff(void)
{
    P4OUT &= ~BIT3;
    __delay_cycles(500);
    P4OUT &= ~BIT2;         // Turn off LCD (DISP = LO & Vcc = LO)
}

/**********************************************************************//**
 * @brief  Write intro before
 *         showing main menu.
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void LCD_introWrite(void)
{
    GrClearDisplay(&sContext);
    GrContextFontSet(&sContext, &g_sFontCmss12);

    GrStringDrawCentered(&sContext, "Battery", -1, 48, 10, 0);
    GrStringDrawCentered(&sContext, "Management", -1, 48, 22, 0);
    GrStringDrawCentered(&sContext, "System", -1, 48, 34, 0);
    GrStringDrawCentered(&sContext, "by", -1, 48, 56, 0);
    GrStringDrawCentered(&sContext, "Michael DeSando", -1, 48, 80, 0);
//    GrStringDrawCentered(&sContext, UE_VERSION_STRING, -1, 30, 62, 0);

    GrFlush(&sContext);
}

/**********************************************************************//**
 * @brief  Create main menu for BMS (highlight correct choice)
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void LCD_displayMainMenu(unsigned char choice)
{
    char outString[32];
    unsigned char text_start = 0, i;

    GrClearDisplay(&sContext);
    GrContextFontSet(&sContext, &g_sFontCmss12);

    // Write "MAIN MENU:"
    GrStringDraw(&sContext, "MAIN MENU:", -1, 16, 1, 0);

    // Write menu options
    GrStringDraw(&sContext, "Cell Voltage", -1, 2, 14, 0);
    GrStringDraw(&sContext, "Pack Voltage", -1, 2, 28, 0);
    GrStringDraw(&sContext, "State of Charge", -1, 2, 42, 0);
    GrStringDraw(&sContext, "Temperature", -1, 2, 56, 0);
    GrStringDraw(&sContext, "Cell Balance", -1, 2, 70, 0);
    GrStringDraw(&sContext, "Enter", -1, 66, 84, 0);

    // Overwrite chosen option with inverse colors
    switch (choice) {
    case 4:
        text_start = 14;
        strcpy(outString, "Cell Voltage");
        break;
    case 6:
        text_start = 28;
        strcpy(outString, "Pack Voltage");
        break;
    case 8:
        text_start = 42;
        strcpy(outString, "State of Charge");
        break;
    case 10:
        text_start = 56;
        strcpy(outString, "Temperature");
        break;
    case 12:
        text_start = 70;
        strcpy(outString, "Cell Balance");
        break;
    default: break;
    }
    GrContextForegroundSet(&sContext, ClrBlack); //ClrBlack       this affects the highlight color
    for (i = text_start; i < (text_start + 14); i++) {
        GrLineDrawH(&sContext, 0, 96, i);
    }
    GrContextForegroundSet(&sContext, ClrWhite);    //ClrWhite      this affects the text color in the highlight
    GrStringDraw(&sContext, outString, -1, 2, text_start, 0);
    GrContextForegroundSet(&sContext, ClrBlack);    //black         this affects the text of everything else.

    GrFlush(&sContext);
}



/**********************************************************************//**
 * @brief  Draw battery based on how many "bars of charge" remain. Also
 *         write the time elapsed since going to LPM3.5 (in the format of
 *         hh:mm:ss) at the bottom of the screen.
 *
 * @param  rm_bars - remaining bars of charge
 *
 * @return none
 *************************************************************************/
void LCD_drawBattery(signed int rm_bars)
{
    char outString[32];
    int i, j, a;
    char dataValue[10] = "";
    unsigned int day, hour, min, sec;

    day = RTCDAY; hour = RTCHOUR; min = RTCMIN; sec = RTCSEC;

    hour += 24 * (day - 1); // Add 24 hours for every day it overflows

    GrContextForegroundSet(&sContext, ClrBlack);
    GrClearDisplay(&sContext);

    // Write "Charge Indicator"
    GrContextFontSet(&sContext, &g_sFontCmss12);
    GrStringDraw(&sContext, "Charge Indicator", -1, 8, 10, 0);

    // Draw Battery Shell
    for (i = 30; i < 72; i++) {
        if (i < 36) {
            GrLineDrawH(&sContext, 4, 81, i);
        }
        else if (i < 41) {
            GrLineDrawH(&sContext, 4, 9, i);
            GrLineDrawH(&sContext, 76, 91, i);
        }
        else if (i < 61) {
            GrLineDrawH(&sContext, 4, 9, i);
            GrLineDrawH(&sContext, 76, 81, i);
            GrLineDrawH(&sContext, 86, 91, i);
        }
        else if (i < 66) {
            GrLineDrawH(&sContext, 4, 9, i);
            GrLineDrawH(&sContext, 76, 91, i);
        }
        else {
            GrLineDrawH(&sContext, 4, 81, i);
        }
    }

    // Draw bars within battery
    if (rm_bars != 0) {
        a = 12;
        for (i = 0; i < rm_bars; i++) {
            for (j = 0; j < 6; j++) {
                GrLineDrawV(&sContext, a, 38, 63);
                a++;
            }
            a += 2;
        }
    }

//    // Write time (hh:mm:ss) below battery
//    if (hour >= 10) {
//        UnsignedInt_To_ASCII(hour, dataValue);
//        strcpy(outString, dataValue);
//    }
//    else {
//        strcpy(outString, "");
//        dataValue[0] = 0x30;
//        dataValue[1] = hour % 10 + 0x30;
//        strncat(outString, dataValue, 2);
//    }
//    strcat(outString, ":");
//    if (min >= 10) {
//        UnsignedInt_To_ASCII(min, dataValue);
//        strncat(outString, dataValue, 2);
//    }
//    else {
//        dataValue[0] = 0x30;
//        dataValue[1] = min % 10 + 0x30;
//        strncat(outString, dataValue, 2);
//    }
//    strcat(outString, ":");
//    if (sec >= 10) {
//        UnsignedInt_To_ASCII(sec, dataValue);
//        strncat(outString, dataValue, 2);
//    }
//    else {
//        dataValue[0] = 0x30;
//        dataValue[1] = sec % 10 + 0x30;
//        strncat(outString, dataValue, 2);
//    }
//    GrStringDraw(&sContext, outString, -1, 26, 80, 0);

    GrFlush(&sContext);

}

/**********************************************************************//**
 * @brief  Draw low battery indicator (battery with "X" through it), along
 *         with a message that tells the user to plug the EXP board into the
 *         PC via USB.
 *
 * @param  rm_bars - remaining bars of charge
 *
 * @return none
 *************************************************************************/
void LCD_drawLowBattery(signed int rm_bars)
{
    int i, j, a;

    GrContextForegroundSet(&sContext, ClrBlack);
    GrClearDisplay(&sContext);

    // Write "Low Battery, Plug in USB"
    GrContextFontSet(&sContext, &g_sFontCmss12);
    GrStringDraw(&sContext, "LOW BATTERY", -1, 8, 2, 0);
    GrStringDraw(&sContext, "Plug into USB", -1, 13, 14, 0);

    // Draw Battery Shell
    for (i = 34; i < 76; i++) {
        if (i < 40) {
            GrLineDrawH(&sContext, 4, 81, i);
        }
        else if (i < 45) {
            GrLineDrawH(&sContext, 4, 9, i);
            GrLineDrawH(&sContext, 76, 91, i);
        }
        else if (i < 65) {
            GrLineDrawH(&sContext, 4, 9, i);
            GrLineDrawH(&sContext, 76, 81, i);
            GrLineDrawH(&sContext, 86, 91, i);
        }
        else if (i < 70) {
            GrLineDrawH(&sContext, 4, 9, i);
            GrLineDrawH(&sContext, 76, 91, i);
        }
        else {
            GrLineDrawH(&sContext, 4, 81, i);
        }
    }

    // Draw bars within battery
    if (rm_bars != 0) {
        a = 12;
        for (i = 0; i < rm_bars; i++) {
            for (j = 0; j < 6; j++) {
                GrLineDrawV(&sContext, a, 42, 67);
                a++;
            }
            a += 2;
        }
    }

    // Draw "X" over battery
    for (i = 17; i < 24; i++) {
        GrLineDraw(&sContext, i, 26, 54 + i, 80);
        GrLineDraw(&sContext, 54 + i, 26, i, 80);
    }

    // Write instructions for reentering LPM3.5 once USB plugged in
    GrStringDraw(&sContext, "Done", -1, 71, 83, 0);

    GrFlush(&sContext);

}

/**********************************************************************//**
 * @brief  This function displays "Sending Data Please Wait" screen while
 *         DMA transfers data to the UART buffer
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void LCD_displaySendingScreen(void)
{
    GrContextForegroundSet(&sContext, ClrBlack);
    GrClearDisplay(&sContext);

    // Write "Sending Data, Please Wait"
    GrContextFontSet(&sContext, &g_sFontCmss12);
    GrStringDraw(&sContext, "Sending Data", -1, 14, 14, 0);
    GrStringDraw(&sContext, "Please Wait", -1, 17, 26, 0);

    GrFlush(&sContext);
}

/**********************************************************************//**
 * @brief  This function displays "Data Send Complete" screen after data has
 *         completed transferring
 *
 * @param  bytes - number of bytes of data that were transmitted
 *
 * @return none
 *************************************************************************/
void LCD_showDataSent(unsigned int bytes)
{
    char dataValue[10] = "";
    int length;

    GrContextForegroundSet(&sContext, ClrBlack);
    GrClearDisplay(&sContext);

    // Write "Data Send Complete"
    GrContextFontSet(&sContext, &g_sFontCmss12);
    GrStringDraw(&sContext, "Data Send", -1, 20, 14, 0);
    GrStringDraw(&sContext, "Complete", -1, 24, 26, 0);

    // Write number of bytes sent via the UART transmission
    length = UnsignedInt_To_ASCII(bytes, dataValue);
    GrStringDraw(&sContext, dataValue, -1, 48 - (4 * length), 50, 0);
    GrStringDraw(&sContext, "Bytes", -1, 32, 62, 0);

    GrFlush(&sContext);
}

/**********************************************************************//**
 * @brief  This function displays communication error which occurs if SPI
 * communication is not detected between BQ76PL536 EVM and MSP430
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void LCD_displayCommunicationError(void)
{
    GrContextForegroundSet(&sContext, ClrBlack);
    GrClearDisplay(&sContext);

    // This function is displayed if a BQ76PL536 device is not found. Check the SPI connection.
    GrContextFontSet(&sContext, &g_sFontCmss12);
    GrStringDrawCentered(&sContext, "ERROR:", -1, 48, 14, 0);
    GrStringDrawCentered(&sContext, "BQ76PL536 device", -1, 48, 26, 0);
    GrStringDrawCentered(&sContext, "not detected", -1, 48, 38, 0);
    GrStringDrawCentered(&sContext, "Check SPI", -1, 48, 62, 0);

    GrFlush(&sContext);
}

/**********************************************************************//**
 * @brief  Display choices for cell selection and highlight current
 *         selection
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void LCD_displayCellSelMenu(unsigned char choice)
{
    char outString[32];
    unsigned char text_start = 0, i;

    GrClearDisplay(&sContext);
    GrContextFontSet(&sContext, &g_sFontCmss12);
    GrContextForegroundSet(&sContext, ClrBlack);

    // Write "Choose Cell:"
    //GrStringDraw(&sContext, "Choose Cell:", -1, 2, 1, 0);

    // Write menu options
    GrStringDraw(&sContext, "Cell 1", -1, 2, 2, 0);
    GrStringDraw(&sContext, "Cell 2", -1, 2, 16, 0);
    GrStringDraw(&sContext, "Cell 3", -1, 2, 30, 0);
    GrStringDraw(&sContext, "Cell 4", -1, 2, 44, 0);
    GrStringDraw(&sContext, "Cell 5", -1, 2, 58, 0);
    GrStringDraw(&sContext, "Cell 6", -1, 2, 72, 0);
    GrStringDraw(&sContext, "Exit         Enter", -1, 2, 84, 0);

    // Overwrite chosen option with inverse colors
    switch (choice) {
    case 2:
        text_start = 2;
        strcpy(outString, "Cell 1");
        break;
    case 4:
        text_start = 16;
        strcpy(outString, "Cell 2");
        break;
    case 6:
        text_start = 30;
        strcpy(outString, "Cell 3");
        break;
    case 8:
        text_start = 44;
        strcpy(outString, "Cell 4");
        break;
    case 10:
        text_start = 58;
        strcpy(outString, "Cell 5");
        break;
    case 12:
        text_start = 72;
        strcpy(outString, "Cell 6");
        break;
    default: break;
    }

    GrContextForegroundSet(&sContext, ClrBlack);
    for (i = text_start; i < (text_start + 14); i++) {
        GrLineDrawH(&sContext, 0, 96, i);
    }
    GrContextForegroundSet(&sContext, ClrWhite);
    GrStringDraw(&sContext, outString, -1, 2, text_start, 0);
    GrContextForegroundSet(&sContext, ClrBlack);

    GrFlush(&sContext);
}

void LCD_displayCell1Voltage(void)
{
	char CellVoltageBuffer[16];

	update_bq_pack_data();
	CELL_VOLTAGE_1 = bq_pack.bq_devs[0].cell_voltage[0];

	UnsignedInt_To_ASCII(CELL_VOLTAGE_1, &CellVoltageBuffer);

    GrClearDisplay(&sContext);
    GrContextFontSet(&sContext, &g_sFontCmss12);

    GrStringDrawCentered(&sContext, "Cell 1 Voltage:", -1, 48, 10, 0);
    GrStringDrawCentered(&sContext, &CellVoltageBuffer, -1, 32, 45, 0);
    GrStringDrawCentered(&sContext, "[mV]", -1, 62, 45, 0);

    GrFlush(&sContext);
}

void LCD_displayCell2Voltage(void)
{
	char CellVoltageBuffer[16];

	update_bq_pack_data();
	CELL_VOLTAGE_2 = bq_pack.bq_devs[0].cell_voltage[1];

	UnsignedInt_To_ASCII(CELL_VOLTAGE_2, &CellVoltageBuffer);

    GrClearDisplay(&sContext);
    GrContextFontSet(&sContext, &g_sFontCmss12);

    GrStringDrawCentered(&sContext, "Cell 2 Voltage:", -1, 48, 10, 0);
    GrStringDrawCentered(&sContext, &CellVoltageBuffer, -1, 32, 45, 0);
    GrStringDrawCentered(&sContext, "[mV]", -1, 62, 45, 0);

    GrFlush(&sContext);
}

void LCD_displayCell3Voltage(void)
{
	char CellVoltageBuffer[16];

	update_bq_pack_data();
	CELL_VOLTAGE_3 = bq_pack.bq_devs[0].cell_voltage[2];

	UnsignedInt_To_ASCII(CELL_VOLTAGE_3, &CellVoltageBuffer);

    GrClearDisplay(&sContext);
    GrContextFontSet(&sContext, &g_sFontCmss12);

    GrStringDrawCentered(&sContext, "Cell 3 Voltage:", -1, 48, 10, 0);
    GrStringDrawCentered(&sContext, &CellVoltageBuffer, -1, 32, 45, 0);
    GrStringDrawCentered(&sContext, "[mV]", -1, 62, 45, 0);

    GrFlush(&sContext);
}

void LCD_displayCell4Voltage(void)
{
	char CellVoltageBuffer[16];

	update_bq_pack_data();
	CELL_VOLTAGE_4 = bq_pack.bq_devs[0].cell_voltage[3];

	UnsignedInt_To_ASCII(CELL_VOLTAGE_4, &CellVoltageBuffer);

    GrClearDisplay(&sContext);
    GrContextFontSet(&sContext, &g_sFontCmss12);

    GrStringDrawCentered(&sContext, "Cell 4 Voltage:", -1, 48, 10, 0);
    GrStringDrawCentered(&sContext, &CellVoltageBuffer, -1, 32, 45, 0);
    GrStringDrawCentered(&sContext, "[mV]", -1, 62, 45, 0);

    GrFlush(&sContext);
}

void LCD_displayCell5Voltage(void)
{
	char CellVoltageBuffer[16];

	update_bq_pack_data();
	CELL_VOLTAGE_5 = bq_pack.bq_devs[0].cell_voltage[4];

	UnsignedInt_To_ASCII(CELL_VOLTAGE_5, &CellVoltageBuffer);

    GrClearDisplay(&sContext);
    GrContextFontSet(&sContext, &g_sFontCmss12);

    GrStringDrawCentered(&sContext, "Cell 5 Voltage:", -1, 48, 10, 0);
    GrStringDrawCentered(&sContext, &CellVoltageBuffer, -1, 32, 45, 0);
    GrStringDrawCentered(&sContext, "[mV]", -1, 62, 45, 0);

    GrFlush(&sContext);
}

void LCD_displayCell6Voltage(void)
{
	char CellVoltageBuffer[16];

	update_bq_pack_data();
	CELL_VOLTAGE_6 = bq_pack.bq_devs[0].cell_voltage[5];

	UnsignedInt_To_ASCII(CELL_VOLTAGE_6, &CellVoltageBuffer);

    GrClearDisplay(&sContext);
    GrContextFontSet(&sContext, &g_sFontCmss12);

    GrStringDrawCentered(&sContext, "Cell 6 Voltage:", -1, 48, 10, 0);
    GrStringDrawCentered(&sContext, &CellVoltageBuffer, -1, 32, 45, 0);
    GrStringDrawCentered(&sContext, "[mV]", -1, 62, 45, 0);


    GrFlush(&sContext);
}

void LCD_displayPackVoltage(void)
{
	char PackVoltageBuffer[17];

	update_bq_pack_data();
	PACK_VOLTAGE = bq_pack.voltage;

	UnsignedInt_To_ASCII(PACK_VOLTAGE, &PackVoltageBuffer);

    GrClearDisplay(&sContext);
    GrContextFontSet(&sContext, &g_sFontCmss12);

    GrStringDrawCentered(&sContext, "Pack Voltage:", -1, 48, 10, 0);
    GrStringDrawCentered(&sContext, &PackVoltageBuffer, -1, 32, 45, 0);
    GrStringDrawCentered(&sContext, "[mV]", -1, 62, 45, 0);
    GrStringDraw(&sContext, "Exit", -1, 2, 84, 0);

    GrFlush(&sContext);
}

void LCD_displayTemperature(void)
{
	char Temperature1Buffer[7];
	char Temperature2Buffer[7];

	update_bq_pack_data();
	TEMPERATURE_1 = (bq_pack.bq_devs[0].temperature1)/100;
	TEMPERATURE_2 = (bq_pack.bq_devs[0].temperature2)/100;

	UnsignedInt_To_ASCII(TEMPERATURE_1, &Temperature1Buffer);
	UnsignedInt_To_ASCII(TEMPERATURE_2, &Temperature2Buffer);

    GrClearDisplay(&sContext);
    GrContextFontSet(&sContext, &g_sFontCmss12);

    GrStringDrawCentered(&sContext, "Pack Temperature:", -1, 48, 10, 0);
    GrStringDrawCentered(&sContext, &Temperature1Buffer, -1, 32, 45, 0);
    GrStringDrawCentered(&sContext, "['C]", -1, 62, 45, 0);
    GrStringDrawCentered(&sContext, &Temperature2Buffer, -1, 32, 65, 0);
    GrStringDrawCentered(&sContext, "['C]", -1, 62, 65, 0);
    GrStringDraw(&sContext, "Exit", -1, 2, 84, 0);

    GrFlush(&sContext);
}

void LCD_displayCellsImbalanced(void)
{
    GrContextForegroundSet(&sContext, ClrBlack);
    GrClearDisplay(&sContext);

    // This function is displayed if cell imbalance is detected.
    GrContextFontSet(&sContext, &g_sFontCmss12);
    GrStringDrawCentered(&sContext, "Cell Imbalance:", -1, 48, 14, 0);
    GrStringDrawCentered(&sContext, "Detected", -1, 48, 26, 0);
    GrStringDrawCentered(&sContext, "Cells differ", -1, 48, 50, 0);
    GrStringDrawCentered(&sContext, "by more than", -1, 48, 62, 0);
    GrStringDrawCentered(&sContext, "50 mV", -1, 48, 74, 0);

    GrFlush(&sContext);
}

void LCD_displayCellsBalanced(void)
{
    GrContextForegroundSet(&sContext, ClrBlack);
    GrClearDisplay(&sContext);

    // This function is displayed if cell imbalance is detected.
    GrContextFontSet(&sContext, &g_sFontCmss12);
    GrStringDrawCentered(&sContext, "Cell Imbalance:", -1, 48, 14, 0);
    GrStringDrawCentered(&sContext, "Not Detected", -1, 48, 26, 0);
    GrStringDrawCentered(&sContext, "Cells differ", -1, 48, 50, 0);
    GrStringDrawCentered(&sContext, "by less than", -1, 48, 62, 0);
    GrStringDrawCentered(&sContext, "50 mV", -1, 48, 74, 0);
    GrStringDraw(&sContext, "Exit", -1, 2, 84, 0);

    GrFlush(&sContext);
}

/**********************************************************************//**
 * @brief  This function takes a pre-loaded image from FRAM and displays
 *         it on the LCD.
 *
 * @param  choice - 1 = TI logo, 2 = Wolverine Logo,
 *
 * @return none
 *************************************************************************/
void LCD_drawPicture(int choice)
{
    int i;

    unsigned char command = SHARP_LCD_CMD_WRITE_LINE; // Image update mode
    command = command ^ VCOMbit;                      //COM inversion bit
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4); // Set P2.4 High for CS

    // Check if SPI is bus is busy first
    while(EUSCI_B_SPI_isBusy(EUSCI_B0_BASE));
    // Send command
    EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, command);

    if (choice == 1) {
        for (i = 0; i < 0x540; i++) {
#ifdef NORMAL_DISPLAY
            while(!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
                    EUSCI_B_SPI_TRANSMIT_INTERRUPT));
            EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, logoTI[i]);
#endif
#ifdef INVERT_DISPLAY
            if (i % 14 != 0) {
                while(!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
                        EUSCI_B_SPI_TRANSMIT_INTERRUPT));
                EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, logoTI[i] ^ 0xFF);
            }
            else {
                while(!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
                        EUSCI_B_SPI_TRANSMIT_INTERRUPT));
                EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, logoTI[i]);
            }
#endif
        }
    }
    else if (choice == 2) {
        for (i = 0; i < 0x540; i++) {
#ifdef NORMAL_DISPLAY
            while(!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
                    EUSCI_B_SPI_TRANSMIT_INTERRUPT));
            EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, logoWolverine[i]);
#endif
#ifdef INVERT_DISPLAY
            if (i % 14 != 0) {
                while(!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
                        EUSCI_B_SPI_TRANSMIT_INTERRUPT));
                EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, logoWolverine[i] ^ 0xFF);
            }
            else {
                while(!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
                        EUSCI_B_SPI_TRANSMIT_INTERRUPT));
                EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, logoWolverine[i]);
            }
#endif
        }
    }

    // Wait for last byte to be sent, then drop SCS
    while(EUSCI_B_SPI_isBusy(EUSCI_B0_BASE));

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);    // Set P2.4 Low for CS
}

/**********************************************************************//**
 * @brief  UnsignedInt_To_ASCII
 *
 * @param  none
 *
 * @return i - number of digits that the Unsigned Int is
 *************************************************************************/
int UnsignedInt_To_ASCII(unsigned int hex, char *ASCII)
{
    int i, j;
    char flipped[5] = { 0, 0, 0, 0, 0 };

    // Convert from hex to ASCII
    for (i = 1; i < 6; i++) {
        flipped[i] = hex % 10 + 0x30;
        hex /= 10;
        if (hex == 0) {
            break;
        }
    }

    // Flip data values to correct position in array
    for (j = 0; j < 5; j++) {
        ASCII[j] = flipped[i - j];
    }

    return i;
}

/**********************************************************************//**
 * @brief  UnsignedLong_To_ASCII
 *
 * @param  none
 *
 * @return i - number of digits that the Unsigned Long is
 *************************************************************************/
int UnsignedLong_To_ASCII(unsigned long hex, char *ASCII)
{
    int i, j;
    char flipped[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    // Convert from hex to ASCII
    for (i = 1; i < 10; i++) {
        flipped[i] = hex % 10 + 0x30;
        hex /= 10;
        if (hex == 0) {
            break;
        }
    }

    // Flip data values to correct position in array
    for (j = 0; j < i; j++) {
        ASCII[j] = flipped[i - j];
    }

    return i;
}
