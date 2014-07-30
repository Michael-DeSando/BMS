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
 * CellBalance.c
 * User Experience Code for the MSP-EXP430FR5969
 * Cell Balance Application
 *
 * Program Execution: System detects if cell imbalance condition exists and balances cells
 * for a programmed balance time before rechecking. The system returns to main menu if cells
 * are balanced.
 *
 * Directions for use: User decides if they want to check for cell imbalance. The cell balancing is
 * taken care of by the system.
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

#include "FR59xx_EXP.h"
#include "myTimer.h"
#include "UpdateDisplay.h"

#include "BQ76PL536/bq_pack.h"
#include "FR59xx_EXP.h"


//note: this cell balancing funtion is not currently used because of code size restriction.

/**********************************************************************//**
 * @brief  Cell Balance Application - check to see if cells are imbalanced by checking
 * if any batteries are differing in voltage by more than the programmed cell imbalance
 * threshold. If they are imbalanced, balance the cells for the programmed balance time
 * then check again. If the cells are balanced, display the cells are already balanced then
 * return to main menu.
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void CellBalanceApp(void)
{
	if(cell_imbalance_threshold_reached() == 1){
		LCD_displayCellsImbalanced();
	}
	else{
		LCD_displayCellsBalanced();
	}
}
