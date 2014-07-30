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
*  @file data_flash.c
*
*  @brief this file contains all the functions needed to access to the battery 
*  pack information stored in the MSP430 flash memory .
*
*  @author Daniel Torres - Texas Instruments, Inc
*  @date November 2010
*  @version 1.0 Initial version
*  @note Built with IAR for MSP430 Version: 5.10
*/


#include "msp430.h"
#include "bq_pack.h"
#include "..\Common\types.h"
#include "data_flash.h"



/**
* @brief  Local variables        .                     
*/
//Constant Array that stores the battery pack info on flash memory
const DWORD DATA_FLASH[] = 
{
  /*Cell Voltage*/
  dCOV_THRESHOLD,                  //COV_THRESHOLD           [mV]
  dCOV_RECOVERY_THRESHOLD,         //COV_RECOVERY_THRESHOLD  [mV]
  dCOV_TIME,                       //COV_TIME                [100ms]
  dCUV_THRESHOLD,                  //CUV_THRESHOLD           [mV]
  dCUV_RECOVERY_THRESHOLD,         //CUV_RECOVERY_THRESHOLD  [mV]
  dCUV_TIME,                       //CUV_TIME                [100ms]
  
  /*Temperature*/
  dPACK_OVER_TEMP1, //3 PACK_OVER_TEMP1   [st C]
  dPACK_OT_TIME1,   //PACK_OT_TIME1       [ms]
  dPACK_OVER_TEMP2,  //3 PACK_OVER_TEMP2   [st C]
  dPACK_OT_TIME2,   //PACK_OT_TIME2       [ms]
  
  /*Charge and Discharge*/
  dPACK_END_OF_CHARGE_VOLTAGE,     //PACK_END_OF_CHARGE_VOLTAGE    [mV] 
  dCC_CV_QUAL_TIME,                //CC_CV_QUAL_TIME               [s]
  dPACK_END_OF_DISCHARGE_VOLTAGE,  //PACK_END_OF_DISCHARGE_VOLTAGE [mV] 
  dEND_OF_DISCHARGE_QUAL_TIME,     //END_OF_DISCHARGE_QUAL_TIME    [s]
  dCHARGE_CURRENT,                 //CHARGE_CURRENT                [mA]
  dCHARGE_TAPER_CURRENT,           //CHARGE_TAPER_CURRENT          [mA]
  dCHARGE_TAPER_TIME,              //CHARGE_TAPER_TIME             [s]
  dMAX_CHARGE_TIME,                //MAX_CHARGE_TIME               [s]
  dFULL_DISCHARGE_CLEAR_VOLTS,     //FULL_DISCHARGE_CLEAR_VOLTS    [mV] 
  dFULL_CHARGE_CLEAR_VOLTS,        //FULL_CHARGE_CLEAR_VOLTS       [mV] 
  dDELTA_CHARGE_V,                 //DELTA_CHARGE_V                [mv]
  dCHARGE_DISCHARGE_TIME,          //CHARGE_DISCHARGE_TIME         [s]
  dDELTA_DISCHARGE_V,              //DELTA_DISCHARGE_V             [mV]
    
  /*Safety*/
  dSOV_THRESHOLD,                 //SOV_THRESHOLD                 [mV]
  dSOV_RECOVERY_THRESHOLD,        //SOV_RECOVERY_THRESHOLD        [mV]
  dSOV_TIME,                      //SOV_TIME                      [ms]
  
  /*Balancing*/
  dCELL_IMBALANCE_FAIL_THRESHOLD,    //CELL_IMBALANCE_FAIL_THRESHOLD [mV]
  dCELL_IMBALANCE_FAIL_TIME,         //CELL_IMBALANCE_FAIL_TIME      [s]
  dBALANCE_TIME,                     //BALANCE_TIME     [s] max value is 63
  dBALANCE_VOLTS_THRESHOLD,          //BALANCE_VOLTS_THRESHOLD       [mV]
  dMIN_BALANCE_VOLTS,                //MIN_BALANCE_VOLTS             [mV]
  dMAX_BALANCE_TIME,                 //MAX_BALANCE_TIME              [s]
  
};


/**
* @brief Function Name: get_u16_value.                                                 
* @brief Description  : Reads the flash memory and returns the value of the 
* parameter
* @param parameters   : battery pack parameter to be read                                                     
* @return Value       : value of the specific battery parameter                                                    
*/     
unsigned short get_u16_value(param_id_t param_id)
{

  return (unsigned short)DATA_FLASH[param_id];

}

/**
* @brief Function Name: get_u32_value.                                                 
* @brief Description  : Reads the flash memory and returns the value of the 
* parameter
* @param parameters   : battery pack parameter to be read                                                     
* @return Value       : value of the specific battery parameter
*/   
DWORD get_u32_value(param_id_t param_id)
{
  return (DWORD)DATA_FLASH[param_id];  
}

/*EOF*/
