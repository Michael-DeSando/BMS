/* --COPYRIGHT--,BSD
 * Copyright (c) 2013, Texas Instruments Incorporated
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
// cs.h - Driver for the CS Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_CS_H__
#define __MSP430WARE_CS_H__

#include "inc/hw_memmap.h"

#if defined(__MSP430_HAS_CS__) || defined(__MSP430_HAS_SFR__)

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// The following are deprecated values. Please refer to documentation for the
// correct values to use.
//
//*****************************************************************************
#define CS_LFXTDRIVE_0                                              LFXTDRIVE_0
#define CS_LFXTDRIVE_1                                              LFXTDRIVE_1
#define CS_LFXTDRIVE_2                                              LFXTDRIVE_2
#define CS_LFXTDRIVE_3                                              LFXTDRIVE_3

//*****************************************************************************
//
// The following are values that can be passed to the clockSourceDivider
// parameter for functions: CS_clockSignalInit().
//
//*****************************************************************************
#define CS_CLOCK_DIVIDER_1                                              DIVM__1
#define CS_CLOCK_DIVIDER_2                                              DIVM__2
#define CS_CLOCK_DIVIDER_4                                              DIVM__4
#define CS_CLOCK_DIVIDER_8                                              DIVM__8
#define CS_CLOCK_DIVIDER_16                                            DIVM__16
#define CS_CLOCK_DIVIDER_32                                            DIVM__32

//*****************************************************************************
//
// The following are values that can be passed to the selectClock parameter for
// functions: CS_enableClockRequest(), and CS_disableClockRequest(); the
// selectedClockSignal parameter for functions: CS_clockSignalInit().
//
//*****************************************************************************
#define CS_ACLK                                                            0x01
#define CS_MCLK                                                            0x02
#define CS_SMCLK                                                           0x04
#define CS_MODOSC                                                   MODCLKREQEN

//*****************************************************************************
//
// The following are values that can be passed to the clockSource parameter for
// functions: CS_clockSignalInit().
//
//*****************************************************************************
#define CS_VLOCLK_SELECT                                           SELM__VLOCLK
#define CS_DCOCLK_SELECT                                           SELM__DCOCLK
#define CS_LFXTCLK_SELECT                                         SELM__LFXTCLK
#define CS_HFXTCLK_SELECT                                         SELM__HFXTCLK
#define CS_LFMODOSC_SELECT                                       SELM__LFMODOSC
#define CS_MODOSC_SELECT                                           SELM__MODOSC

//*****************************************************************************
//
// The following are values that can be passed to the lfxtdrive parameter for
// functions: CS_LFXTStart(), and CS_LFXTStartWithTimeout().
//
//*****************************************************************************
#define CS_LFXT_DRIVE0                                              LFXTDRIVE_0
#define CS_LFXT_DRIVE1                                              LFXTDRIVE_1
#define CS_LFXT_DRIVE2                                              LFXTDRIVE_2
#define CS_LFXT_DRIVE3                                              LFXTDRIVE_3

//*****************************************************************************
//
// The following are values that can be passed to the hfxtdrive parameter for
// functions: CS_HFXTStart(), and CS_HFXTStartWithTimeout().
//
//*****************************************************************************
#define CS_HFXTDRIVE_4MHZ_8MHZ                                      HFXTDRIVE_0
#define CS_HFXTDRIVE_8MHZ_16MHZ                                     HFXTDRIVE_1
#define CS_HFXTDRIVE_16MHZ_24MHZ                                    HFXTDRIVE_2
#define CS_HFXTDRIVE_24MHZ_32MHZ                                    HFXTDRIVE_3

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: CS_faultFlagStatus(), and CS_clearFaultFlag() as well as returned
// by the CS_faultFlagStatus() function.
//
//*****************************************************************************
#define CS_LFXTOFFG                                                    LFXTOFFG
#define CS_HFXTOFFG                                                    HFXTOFFG

//*****************************************************************************
//
// The following are values that can be passed to the dcorsel parameter for
// functions: CS_setDCOFreq().
//
//*****************************************************************************
#define CS_DCORSEL_0                                                       0x00
#define CS_DCORSEL_1                                                    DCORSEL

//*****************************************************************************
//
// The following are values that can be passed to the dcofsel parameter for
// functions: CS_setDCOFreq().
//
//*****************************************************************************
#define CS_DCOFSEL_0                                                  DCOFSEL_0
#define CS_DCOFSEL_1                                                  DCOFSEL_1
#define CS_DCOFSEL_2                                                  DCOFSEL_2
#define CS_DCOFSEL_3                                                  DCOFSEL_3
#define CS_DCOFSEL_4                                                  DCOFSEL_4
#define CS_DCOFSEL_5                                                  DCOFSEL_5
#define CS_DCOFSEL_6                                                  DCOFSEL_6

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void CS_setExternalClockSource(uint32_t baseAddress,
                                      uint32_t LFXTCLK_frequency,
                                      uint32_t HFXTCLK_frequency);

extern void CS_clockSignalInit(uint32_t baseAddress,
                               uint8_t selectedClockSignal,
                               uint16_t clockSource,
                               uint16_t clockSourceDivider);

extern void CS_LFXTStart(uint32_t baseAddress,
                         uint16_t lfxtdrive);

extern void CS_bypassLFXT(uint32_t baseAddress);

extern bool CS_LFXTStartWithTimeout(uint32_t baseAddress,
                                    uint16_t lfxtdrive,
                                    uint32_t timeout);

extern bool CS_bypassLFXTWithTimeout(uint32_t baseAddress,
                                     uint32_t timeout);

extern void CS_LFXTOff(uint32_t baseAddress);

extern void CS_HFXTStart(uint32_t baseAddress,
                         uint16_t hfxtdrive);

extern void CS_bypassHFXT(uint32_t baseAddress);

extern bool CS_HFXTStartWithTimeout(uint32_t baseAddress,
                                    uint16_t hfxtdrive,
                                    uint32_t timeout);

extern bool CS_bypassHFXTWithTimeout(uint32_t baseAddress,
                                     uint32_t timeout);

extern void CS_HFXTOff(uint32_t baseAddress);

extern void CS_enableClockRequest(uint32_t baseAddress,
                                  uint8_t selectClock);

extern void CS_disableClockRequest(uint32_t baseAddress,
                                   uint8_t selectClock);

extern uint8_t CS_faultFlagStatus(uint32_t baseAddress,
                                  uint8_t mask);

extern void CS_clearFaultFlag(uint32_t baseAddress,
                              uint8_t mask);

extern uint32_t CS_getACLK(uint32_t baseAddress);

extern uint32_t CS_getSMCLK(uint32_t baseAddress);

extern uint32_t CS_getMCLK(uint32_t baseAddress);

extern void CS_VLOoff(uint32_t baseAddress);

extern uint16_t CS_clearAllOscFlagsWithTimeout(uint32_t baseAddress,
                                               uint32_t timeout);

extern void CS_setDCOFreq(uint32_t baseAddress,
                          uint16_t dcorsel,
                          uint16_t dcofsel);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_CS_H__
