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
/*
 * MSP430 WOLVERINE USER EXPERIENCE
 * CAPTOUCH STRUCTURE
 *
 * TWO SLIDERS, IMPLEMENTED AS WHEELS (4 physical electrodes, 3 keys)
 * UNRWRAPPED BACK TO SLIDER IN CTS_wolverine layer
 *
 * Tuned for 1ms Scan Time (8192 cycles at 8MHz SMCLK)
 * WITH NO LAMINATE ON PCB
 *
 * October 3, 2012
 */


#include "structure.h"
#include "msp430.h"

//#define CHARACTERIZE
#define APPLICATION

// P3.4 Upper Left (slider0)
const struct Element upperLeft = {  //middle left

    .inputBits        = CAPSIOPOSEL0 + CAPSIOPOSEL1 + CAPSIOPISEL2,
    .maxResponse    = 655 + 230,
#ifdef CHARACTERIZE
    .threshold        = 0
#endif
#ifdef APPLICATION
    .threshold        = 230
#endif
};
// P3.5 Lower Left (slider0)
const struct Element lowerLeft = { //bottom left

    .inputBits        = CAPSIOPOSEL0 + CAPSIOPOSEL1 + CAPSIOPISEL0 + CAPSIOPISEL2,
    .maxResponse    = 655 + 160,
#ifdef CHARACTERIZE
    .threshold        = 0
#endif
#ifdef APPLICATION
    .threshold        = 160
#endif
};
// P3.6 Split Left (slider0)
const struct Element splitLeft = {  //top left

    .inputBits        = CAPSIOPOSEL0 + CAPSIOPOSEL1 + CAPSIOPISEL1 + CAPSIOPISEL2,
    .maxResponse    = 655 + 180,
#ifdef CHARACTERIZE
    .threshold        = 0
#endif
#ifdef APPLICATION
    .threshold        = 180
#endif
};
// P1.3 Split Right (slider1)
const struct Element splitRight = {

    .inputBits        = CAPSIOPOSEL0 + CAPSIOPISEL0 + CAPSIOPISEL1,
    .maxResponse    = 655 + 50,
#ifdef CHARACTERIZE
    .threshold        = 0
#endif
#ifdef APPLICATION
    .threshold        = 50
#endif
};
// P1.4 Lower
const struct Element lowerRight = {
    .inputBits        = CAPSIOPOSEL0 + CAPSIOPISEL2,
    .maxResponse    = 655 + 50,
#ifdef CHARACTERIZE
    .threshold        = 0
#endif
#ifdef APPLICATION
    .threshold        = 50        //
#endif
};
// P1.5 Upper
const struct Element upperRight = {

    .inputBits        = CAPSIOPOSEL0 + CAPSIOPISEL0 + CAPSIOPISEL2,
    .maxResponse    = 655 + 70,
#ifdef CHARACTERIZE
    .threshold        = 0
#endif
#ifdef APPLICATION
    .threshold        = 70
#endif
};

const struct Sensor slider0 = {
    .halDefinition            = RO_CSIO_TA2_WDTA,
    .inputCapsioctlRegister = (uint16_t *) &CAPSIO0CTL,
    .numElements            = 3,
    .baseOffset                = 0,
    .points                    = 40,
    .sensorThreshold        = 8,
    // Pointer to elements
    .arrayPtr[0] = &lowerLeft,
    .arrayPtr[1] = &upperLeft,
    .arrayPtr[2] = &splitLeft,
    .measGateSource            = GATE_WDT_SMCLK,
    .accumulationCycles        = WDTA_GATE_8192
};

const struct Sensor slider1 = {
    .halDefinition            = RO_CSIO_TA2_WDTA,
    .inputCapsioctlRegister = (uint16_t *) &CAPSIO0CTL,
    .numElements            = 3,
    .baseOffset                = 3,
    .points                    = 40,
    .sensorThreshold        = 8,
    // Pointer to elements
    .arrayPtr[0] = &lowerRight,
    .arrayPtr[1] = &upperRight,
    .arrayPtr[2] = &splitRight,
    .measGateSource            = GATE_WDT_SMCLK,
    .accumulationCycles        = WDTA_GATE_8192
};



