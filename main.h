//******************************************************************************
//THIS PROGRAM IS PROVIDED "AS IS". TI MAKES NO WARRANTIES OR
//REPRESENTATIONS, EITHER EXPRESS, IMPLIED OR STATUTORY,
//INCLUDING ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
//FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR
//COMPLETENESS OF RESPONSES, RESULTS AND LACK OF NEGLIGENCE.
//TI DISCLAIMS ANY WARRANTY OF TITLE, QUIET ENJOYMENT, QUIET
//POSSESSION, AND NON-INFRINGEMENT OF ANY THIRD PARTY
//INTELLECTUAL PROPERTY RIGHTS WITH REGARD TO THE PROGRAM OR
//YOUR USE OF THE PROGRAM.
//
//IN NO EVENT SHALL TI BE LIABLE FOR ANY SPECIAL, INCIDENTAL,
//CONSEQUENTIAL OR INDIRECT DAMAGES, HOWEVER CAUSED, ON ANY
//THEORY OF LIABILITY AND WHETHER OR NOT TI HAS BEEN ADVISED
//OF THE POSSIBILITY OF SUCH DAMAGES, ARISING IN ANY WAY OUT
//OF THIS AGREEMENT, THE PROGRAM, OR YOUR USE OF THE PROGRAM.
//EXCLUDED DAMAGES INCLUDE, BUT ARE NOT LIMITED TO, COST OF
//REMOVAL OR REINSTALLATION, COMPUTER TIME, LABOR COSTS, LOSS
//OF GOODWILL, LOSS OF PROFITS, LOSS OF SAVINGS, OR LOSS OF
//USE OR INTERRUPTION OF BUSINESS. IN NO EVENT WILL TI'S
//AGGREGATE LIABILITY UNDER THIS AGREEMENT OR ARISING OUT OF
//YOUR USE OF THE PROGRAM EXCEED FIVE HUNDRED DOLLARS
//(U.S.$500).
//
//Unless otherwise stated, the Program written and copyrighted
//by Texas Instruments is distributed as "freeware".  You may,
//only under TI's copyright in the Program, use and modify the
//Program without any charge or restriction.  You may
//distribute to third parties, provided that you transfer a
//copy of this license to the third party and the third party
//agrees to these terms by its first use of the Program. You
//must reproduce the copyright notice and any other legend of
//ownership on each copy or partial copy, of the Program.
//
//You acknowledge and agree that the Program contains
//copyrighted material, trade secrets and other TI proprietary
//information and is protected by copyright laws,
//international copyright treaties, and trade secret laws, as
//well as other intellectual property laws.  To protect TI's
//rights in the Program, you agree not to decompile, reverse
//engineer, disassemble or otherwise translate any object code
//versions of the Program to a human-readable form.  You agree
//that in no event will you alter, remove or destroy any
//copyright notice included in the Program.  TI reserves all
//rights not specifically granted under this license. Except
//as specifically provided herein, nothing in this agreement
//shall be construed as conferring by implication, estoppel,
//or otherwise, upon you, any license or other right under any
//TI patents, copyrights or trade secrets.
// 
//You may not use the Program in non-TI devices.
//
//This software has been submitted to export control regulations
//The ECCN is EAR99 
//*****************************************************************************
/**
*  @file main.h
*
*  @brief this file contains all the definitions of the functions declared in
*  main.c. It also contains the global variables used in other modules
*
*
*  @author Daniel Torres - Texas Instruments, Inc
*  @date November 2010
*  @version 1.0 Initial version
*  @note Built with CCS for MSP430 Version: 4.2.1
*/

#ifndef _MAIN_H_
#define _MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif  
  
#include "Common/types.h"

//*****************************************************************************
/**
* @brief  System definitions                              
*/
//*****************************************************************************

///System defines
#define LOW_POWER_MODE            LPM0_bits //MCLK, SMCLK = OFF, CPUOFF
//definitions for the 1sec timer
#define Timer1Sec_TAxCTL		      TA0CTL
#define Timer1Sec_TAxR 			      TA0R
#define Timer1Sec_TAxCCTLx	      TA0CCTL0
#define Timer1Sec_TAxCCRx         TA0CCR0 
#define Timer1Sec_TAxIV   	      TA0IV
#define Timer1Sec_TIMER_VECTOR    TIMER0_A0_VECTOR

//communication protocol define
//#define UART_COMM
//#define USB_COMM
#define MAX_STR_LENGTH 128
#define MCLK_FREQ 8000000 // MCLK frequency of MCU, in Hz
//definitions for the UART  
#ifdef UART_COMM
  #include "UART.h"
#define MCLK_FREQ 8000000 // MCLK frequency of MCU, in Hz
#endif
//definitions for the USB files
#ifdef USB_COMM
#include "..\SourceCode\USB_Common\descriptors.h"
#include "USB_Common\usb.h"        // USB-specific functions
#include "USBCDC_constructs.h"
#define MCLK_FREQ USB_MCLK_FREQ   // MCLK frequency of MCU, in Hz
  #ifdef _CDC_
    #include "USB_CDC_API\UsbCdc.h"
  #endif
#endif


//*****************************************************************************
/**
* @brief  Global variables                              
*/
//*****************************************************************************

//buffer used to send outgoing messages
#ifdef USB_COMM  
extern  char OutputString[MAX_STR_LENGTH];   // Holds the outgoing string  
#endif
  
#ifdef UART_COMM  
extern char OutputString[MAX_STR_LENGTH];   // Holds the outgoing string  
#endif 

//*****************************************************************************
/**
* @brief  Global Functions                              
*/
//*****************************************************************************
extern void Init100mSecTimer(void);
extern void Disable100mSecTimer(void);

//*****************************************************************************
/**
* @brief  Local Functions                              
*/
//*****************************************************************************
void InitMCU(void);
void Init_Ports(void);
BYTE retInString(char*);
void InitMCU(void);
void Init_Clock(void);
void InitBQ76PL536ports(void);
void Init1SecTimer(void);
void Disable1SecTimer(void);
void GoToLowPowerMode(void);
void BQFaultSignalTask(void);
void BQAlertSignalTask(void);
void Timer1SecExpiredTask(void);
void DRDYSignalTask(void);
void USBCommunicationTask(void);
void UARTCommunicationTask(void);
void HostCommunicationTask(void);
void SendFaultMessageToHost(void);
void SendAlertMessageToHost(void);
  
#ifdef __cplusplus
}
#endif
#endif /* _MAIN_H_ */
/*------------------------ Nothing Below This Line --------------------------*/

