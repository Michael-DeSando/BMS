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
 * FR59xx_EXP.h
 * User Experience Code for the MSP-EXP430FR5969
 * Experience Header File
 *
 * Created: Version 1.0:
 *
 ******************************************************************************/

#ifndef FR59XX_EXP_UE_CODE
#define FR59XX_EXP_UE_CODE

#include "grlib/grlib/grlib.h"

// The project's version string as it will get output to the LCD during startup.
#define UE_VERSION_STRING           "V2.1.0"

// The system's configured MCLK frequency.
#define UE_MCLK_FREQUENCY           8000000

// Helper definition that can be used for SW delays etc.. Note that the pre-processor constant is
// defined as a floating point constant to make sure not to loose precision in case the calculation
// would yield a non-integer result. This won't affect application runtime as this definition is
// resolved at compile time.
#define MCLK_CYCLES_PER_US          (UE_MCLK_FREQUENCY / 1000000)

// Application Mode Definitions
#define APP_MAIN_MENU       2                   // Display Main Menu
#define APP_CELL_VOLTAGE           4                   // Cell voltage
#define APP_PACK_VOLTAGE      6                   // Pack Voltage
#define APP_STATE_OF_CHARGE         8                   // State of Charge
#define APP_TEMPERATURE     10                  // Temperature
#define APP_CELL_BALANCE      12                  // Cell Balance


extern tContext sContext;
extern volatile unsigned char mode;
extern volatile unsigned char select;

extern unsigned short CELL_VOLTAGE_1;
extern unsigned short CELL_VOLTAGE_2;
extern unsigned short CELL_VOLTAGE_3;
extern unsigned short CELL_VOLTAGE_4;
extern unsigned short CELL_VOLTAGE_5;
extern unsigned short CELL_VOLTAGE_6;
extern unsigned short PACK_VOLTAGE;
extern unsigned short TEMPERATURE_1;
extern unsigned short TEMPERATURE_2;

// Function Declarations
extern unsigned char MainMenu(void);
extern void DisableLeftSwitch(void);
extern void EnableLeftSwitch(void);
extern void DisableRightSwitch(void);
extern void EnableRightSwitch(void);
extern void StartDebounceTimer(void);
extern void onLED(void);
extern void offLED(void);

extern void InitBQ76PL536ports(void);
extern void InitMCU(void);
extern void BMSstart(void);

extern void SystemInit(void);

#endif /* FR59XX_EXP_UE_CODE */
