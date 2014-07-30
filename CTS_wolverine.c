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
 * CTS_wolverine.c
 * User Experience Code for the MSP-EXP430FR5969
 * Main CapTouch Library File
 *
 * This file contains all the important function calls for returning a CapTouch
 * reading.
 *
 * Created: Version 1.0:
 * - rev 1.1 (WJS) Reversed slider order to comply with UE code
 *                              if ILLEGAL_SLIDER_WHEEL_POSITION, set sliderPos[i] to oldSliderPos[i],
 *                              to save menu positions
 *
 ******************************************************************************/
#define BREAKPOINT ( (sliderPtr[i]->points) - ((sliderPtr[i]->points) / (sliderPtr[i]->numElements) / 2) )
#define SLOP            2

#include "CTS_wolverine.h"
#include "FR59xx_EXP.h"

//-----------------------------------------------------------------------------
// CapTouch Library Global Variables
//-----------------------------------------------------------------------------
volatile unsigned int sliderPos[2] = { 0, 0 };
volatile unsigned int oldSliderPos[2] = { 0, 0 };

int16_t sliderArray[2][ARRAY_SIZE] = { { 0, 0 }, { 0, 0 } };
unsigned char arrayFill[2] = { 0, 0 };

const struct Sensor* sliderPtr[2] =
{
    &slider0,
    &slider1
};

/**********************************************************************//**
 * @brief  CTS_getReading - this function will get a CapTouch reading from
 *         both sliders. Final positions are stored in sliderPos[x].
 *
 *         The variables sliderPos and oldSliderPos indicate the detection
 *         status and the current location.  In the event that sliderPos
 *         equals ILLEGAL_SLIDER_WHEEL_POSITION, then the previous location
 *         information is found in oldSliderPos.
 *
 * @param  none
 *
 * @return unsigned char - for menu operation, returns the position of the
 *         left CapTouch slider
 *************************************************************************/
void CTS_getReading(void)
{
    volatile uint8_t i, j;

    for (i = 0; i < 2; i++) {
        // Determine Threshold crossing
        sliderPos[i] = TI_CAPT_Wheel(sliderPtr[i]);

        if (sliderPos[i] != ILLEGAL_SLIDER_WHEEL_POSITION) {
            // Check for slop zone
            if ((sliderPos[i] >= BREAKPOINT - SLOP) && (sliderPos[i] <= BREAKPOINT + SLOP)) {
                if (oldSliderPos[i] < ((sliderPtr[i]->points) / 2)) {
                    sliderPos[i] = 0;
                }
                else {
                    sliderPos[i] = 40;
                }
            }
            // Translate wrap around of wheel to slider
            else if (sliderPos[i] < (sliderPtr[i]->points) - ((sliderPtr[i]->points) / (sliderPtr[i]->numElements) / 2)) {
                sliderPos[i] = ((sliderPtr[i]->points) / (sliderPtr[i]->numElements) / 2) + sliderPos[i];
            }
            else {
                sliderPos[i] -= (sliderPtr[i]->points) - ((sliderPtr[i]->points) / (sliderPtr[i]->numElements) / 2);
            }

            if (arrayFill[i] < ARRAY_SIZE) {
                sliderArray[i][arrayFill[i]] = sliderPos[i];
                sliderPos[i] = oldSliderPos[i];
                arrayFill[i]++;
            }
            else {
                sliderArray[i][ARRAY_SIZE - 1] = sliderPos[i];
                for (j = 0; j < ARRAY_SIZE - 1; j++) {
                    sliderPos[i] += sliderArray[i][j];
                    sliderArray[i][j] = sliderArray[i][j + 1];
                }
                sliderPos[i] = sliderPos[i] / ARRAY_SIZE;
                if (sliderPos[i] > (sliderPtr[i]->points - 1)) {
                    sliderPos[i] = (sliderPtr[i]->points) - 1;
                }
                oldSliderPos[i] = sliderPos[i];
            }
        }
        else {
            sliderPos[i] = oldSliderPos[i];
            arrayFill[i] = 0;
        }
    }     // end for loop
    return;
}


