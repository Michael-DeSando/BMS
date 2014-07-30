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
// 
//You may not use the Program in non-TI devices.
//
//
//This software has been submitted to export control regulations
//The ECCN is EAR99 
//****************************************************************************//
/** 
* @file TI_IO_Regs.h
* @brief Generic I/O ports register definition.
* @author Daniel Torres - Texas Instruments*
* @date 11/2010
* @version 1.0  Initial Version
*/

#ifndef TI_IO_Regs
#define TI_IO_Regs
#define BQ76PL536_PORT 4
#define __MSP430_HAS_PORT4_R__

// BQ76PL536 I/O PORT DEFINITIONS*/ 
#if (BQ76PL536_PORT == 1)
  #ifdef   __MSP430_HAS_PORT1_R__ 

    #define BQ76PL536_PxIN   P1IN
    #define BQ76PL536_PxOUT  P1OUT
    #define BQ76PL536_PxDIR  P1DIR
    #define BQ76PL536_PxIFG  P1IFG
    #define BQ76PL536_PxIES  P1IES
    #define BQ76PL536_PxIE   P1IE
    #define BQ76PL536_PxSEL  P1SEL0
    #define BQ76PL536_PxREN  P1REN

  #else
    #error "PORT 1 not supported in this device"
  #endif
#elif (BQ76PL536_PORT == 4)
  #ifdef   __MSP430_HAS_PORT4_R__
  
    #define BQ76PL536_PxIN   P4IN
    #define BQ76PL536_PxOUT  P4OUT
    #define BQ76PL536_PxDIR  P4DIR
    #define BQ76PL536_PxIFG  P4IFG
    #define BQ76PL536_PxIES  P4IES
    #define BQ76PL536_PxIE   P4IE
    #define BQ76PL536_PxSEL  P4SEL1
    #define BQ76PL536_PxREN  P4REN
  
  #else
    #error "PORT 2 not supported in this device"
  #endif
#else 
  #error "I/O PORT not supported by the Push Button"
#endif  




#endif

