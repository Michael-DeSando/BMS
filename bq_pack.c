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
//****************************************************************************

/**
*  @file bq_pack.c 
*
*  @brief this file contains all the definitions of the BQ76PL536 devices.
*
*  @author Daniel T. & Marcin N. Texas Instruments, Inc
*  @date November 2010
*  @version 1.0 Initial version
*  @note Built with IAR for MSP430 Version: 5.10
*/

#include <intrinsics.h>
#include <string.h>
#include <stdio.h>

#include "MSP430.h"
#include "main.h"
#include "bq_pack.h"
#include "..\SPI\spi_if.h"
#include "data_flash.h"
#include "USBCDC_constructs.h"
#include "Common\types.h"
#include "Common\hal_macros.h"

  
/******************************************************************************/
/*                            Global variables                                */
/******************************************************************************/
//Global variable that contains the battery pack information
bq_pack_t bq_pack;  
//SW flag that controls the Charge/Discharge mode
unsigned char HOST_CONTROL_IN = 0; 

/******************************************************************************/
/*                            Local variables                                 */
/******************************************************************************/
//Array that stores a copy of the last known cell voltages
static unsigned short cell_values[NUMBER_OF_CELLS];

/**
* @brief Function Name: bq_pack_address_discovery                            .                     
* @brief Description  : BQ Pack address discovery and assignement algorithm .
* @param parameters   : void                                                    
* @return Value       : number of stacked BQ76PL536 devices                                                    
*/     
short bq_pack_address_discovery(void)
{
  unsigned short i, n;
  unsigned char reg_val[2];

  i=NUMBER_OF_BQ_DEVICES; //controls iteration loop
  while (i>0)
  {
    //*Send BROADCAST_RESET to address 0x00*/
    bq_dev_write_reg(BROADCAST_ADDR, RESET_REG, BQ76PL536_RESET);
    
    n=0;  //controls number of discovered devices
    while (n<NUMBER_OF_BQ_DEVICES)
    {
    //*Read DEVICE_STATUS reg at address 0x00*/
    bq_dev_read_reg(DISCOVERY_ADDR, DEVICE_STATUS_REG, 1, DISCARD_CRC, reg_val);
  
      //*Verify if MSB is equal to 0*/
      if (reg_val[0] & (1<<7))
      {
        n = NUMBER_OF_BQ_DEVICES; //break internal loop
      }
      else
      {
        //*Assign a new address*/        

        //Save assigned address
        n++;
        bq_pack.bq_devs[n-1].device_address = n;
        
        //ADDR_CTRL = n;   
        bq_dev_write_reg(DISCOVERY_ADDR, ADDRESS_CONTROL_REG, n);
        
        //read ADDR_CTRL
        bq_dev_read_reg(n, ADDRESS_CONTROL_REG, 1, DISCARD_CRC, reg_val);
        if ((reg_val[0]&0x3F) == n)
        {
          //address next device or finish device detection
          if (n==NUMBER_OF_BQ_DEVICES)
            return n;
        }
        else
        {
          //break internal loop
          n = NUMBER_OF_BQ_DEVICES;
        }
      }
    }

    i--;
  }
  
  return 0;
}

/**
* @brief Function Name: bq_pack_init.                                              
* @brief Description  : Configure each BQ device in the stack and initialize 
* data structures for whole BQ stack .
* @param parameters   :  none                                                  
* @return Value       :  return  if there isn't problems while init the pack                                                   
*/
short bq_pack_init(void)
{
  unsigned char i;
  
  for (i=0; i<NUMBER_OF_BQ_DEVICES; i++)
  {
    //*Init cell count for each BQ device*/ 
    bq_pack.bq_devs[i].cell_count = MAX_CELLS_NUMBER_IN_BQ;
       
    //*Configure each BQ device*/
    conf_bq_dev(&bq_pack.bq_devs[i]);

    //*Initilize data structures*/    
    init_bq_dev_data_structure(&bq_pack.bq_devs[i]);
    
    //*Read cell voltage*/
    bq_dev_read_cell_voltage(&bq_pack.bq_devs[i]);
  }
  //battery manager initial mode
  bq_pack.op_mode = FAULT_MODE;
  bq_pack.error_status = 0;
  bq_pack.voltage = 0;
  bq_pack.timer_status = 0;
  bq_pack.eoc_eod_timer = 0;
  bq_pack.charge_taper_timer = 0;
  bq_pack.chg_dschg_op_timer = 0;
  bq_pack.balancing_timer = 0;
  bq_pack.max_balance_timer = 0;
  bq_pack.last_imbalanced_cell_idx = 0;
  bq_pack.cell_imbalance_fail_timer = 0;
  // set the battery manager to initial mode
  update_op_mode(INITIAL_MODE);
  return 0;
}


/**
* @brief Function Name: update_bq_pack_data                                                  
* @brief Description  : Reads data from BQ DEV registers and updates BQ PACK 
* structure 
* @param parameters   : none                                                    
* @return Value       : none                                                    
*/     
void update_bq_pack_data(void)
{
  unsigned char i, cell_cnt;
  unsigned short stack_voltage = 0;
  //read the pack data
  for (i=0; i<NUMBER_OF_BQ_DEVICES; i++)
  {
    //*Read each device status*/
    bq_dev_read_status(&bq_pack.bq_devs[i]);
    
    //*Read each cell voltage and calculate BQ Pack voltage*/
    bq_dev_read_cell_voltage(&bq_pack.bq_devs[i]);
    
    //Identify the lowest and highest voltage in the pack
    if (i == 0)
    {
      bq_pack.lowest_cell_volts = bq_pack.bq_devs[i].cell_voltage[0];
      bq_pack.highest_cell_volts = bq_pack.bq_devs[i].cell_voltage[0];
    }
    //calculate the pack voltage
    for (cell_cnt=0; cell_cnt<bq_pack.bq_devs[i].cell_count; cell_cnt++)
    {
      stack_voltage += bq_pack.bq_devs[i].cell_voltage[cell_cnt];
      
      if (bq_pack.bq_devs[i].cell_voltage[cell_cnt] 
          < bq_pack.lowest_cell_volts)
      {
        bq_pack.lowest_cell_volts = bq_pack.bq_devs[i].cell_voltage[cell_cnt];
      }
      else if (bq_pack.bq_devs[i].cell_voltage[cell_cnt] 
               > bq_pack.highest_cell_volts)
      {
        bq_pack.highest_cell_volts = bq_pack.bq_devs[i].cell_voltage[cell_cnt];
      }
    }
    
    //*Read each device errors*/
    bq_dev_read_errors(&bq_pack.bq_devs[i]);
     
    //*Read each device temperature*/
    bq_dev_read_temps(&bq_pack.bq_devs[i]);
  }
  
  //*Save BQ Pack voltage*/
  bq_pack.voltage = stack_voltage;
   
  return;
}

/**
* @brief Function Name: bq_pack_start_conv   .                                               
* @brief Description  : triggers the ADC on the BQ devices.
* @param parameters   : none                                                    
* @return Value       : none                                                    
*/     
void bq_pack_start_conv(void)
{
  //Should address all devices in the stack
  bq_dev_write_reg(BROADCAST_ADDR, ADC_CONVERT_REG, ADC_CONVERT_VAL); 

}

/**
* @brief Function Name: CheckFaultConditions.                                                 
* @brief Description  : Check fault conditions on the pack.
* @param parameters   : none                                                    
* @return Value       : none                                                    
*/     
void CheckFaultConditions(void)
{
  //check for POT or COV conditions
  if (check_for_pot() || check_for_cov())
  {
    bq_pack.error_status |= STATUS_ERROR_POT_COV;
    
    //can only return to previous mode when error condition disapears
    update_op_mode(FAULT_MODE);
    
  }
  //Check for CUV condition
  if (check_for_cuv())
  {
    bq_pack.error_status |= STATUS_ERROR_CUV;
    
    //can only return to previous mode when error condition disapears
    update_op_mode(FAULT_MODE);
  }
}


/**
* @brief Function Name: CheckChargeDischargeModes.                                                  
* @brief Description  : Check if the pack is in charge or discharge mode.
* @param parameters   : none                                                    
* @return Value       : none                                                    
*/     
void CheckChargeDischargeModes(void)
{
  //Charge/Discharge watchdog timer ON??
  if (bq_pack.timer_status & START_CHG_DSCHG_OP_TIMER)
  { //Charge/Discharge watchdog timer expired??
    if (bq_pack.chg_dschg_op_timer >= (get_u32_value(CHARGE_DISCHARGE_TIME)))
    { //Check if the cells increased their voltage
      if (check_for_charge_op())
      {
        //Charge operation detected!
        update_op_mode(CHARGE_OP);
      }//Check if the cells decreased their voltage
      else if (check_for_discharge_op())
      {
        //Discharge operation detected!
        update_op_mode(DISCHARGE_OP);
      }
      
      //Clear timer count
      bq_pack.chg_dschg_op_timer = 0;
      
      //Save current cell voltages for next comparison
      copy_cell_voltage_values();
    }
  }
}

/**
* @brief Function Name: CellBalancing.                                   
* @brief Description  : Identify the unbalanced cells and enables the bypass 
* resistors to balance the cells while in charging mode .
* @param parameters   : none                                                     
* @return Value       : none                                                    
*/     
#if CELL_BALANCING_EN
void CellBalancing(void)
{

  unsigned short dev_id;


  if (bq_pack.timer_status & START_CELL_BALANCE_TIMER)
  {
    //*Perform cell balancing only if CHARGE OPERATION is in effect*/
    //if battery pack is charging
    if (HOST_CONTROL_IN & IN_HOST_CHG_OP)
    { 
      //balancing time has expired? Yes, then...
      if (bq_pack.balancing_timer >= (get_u32_value(BALANCE_TIME)-2))
      { 
        //cell balancing achieved?
        if ((bq_pack.highest_cell_volts - bq_pack.lowest_cell_volts) >= 
          get_u32_value(BALANCE_VOLTS_THRESHOLD))
        { //cell balancing achieved? ->No, 
          //then reset balancing timer & continue balancing
          for (dev_id=0; dev_id<NUMBER_OF_BQ_DEVICES; dev_id++)
          {
            //*Enable bypass resistor for all balanced cells*/ 
            enable_bypass_resistor(dev_id, (~find_imbalanced_cell(dev_id)));
            
          }
          //reset balancing virtual timer
          bq_pack.balancing_timer = 0;
          
        }
        else
        {
          //cell balancing achieved? ->Yes, 
          //then Stop balancing timer
          bq_pack.timer_status &= ~START_CELL_BALANCE_TIMER;
          
          //Disable bypass resistors
          disable_all_bypass_resistors();
        }
      }
      //If max balancing time has expired, then...
      if (bq_pack.max_balance_timer > get_u32_value(MAX_BALANCE_TIME))
      {
        //flag the error and set the pack in SOV mode.. 
        bq_pack.error_status |= STATUS_ERROR_MAX_BALANCE_TIME;      
        update_op_mode(SOV_MODE);
      }
      
       //If cell Imbalance timer time has expired, then...
      if(bq_pack.cell_imbalance_fail_timer 
         > get_u32_value(CELL_IMBALANCE_FAIL_TIME))
      {
        //flag the error and set the pack in SOV mode.. 
        bq_pack.error_status |= STATUS_ERROR_IMBALANCE_FAIL;
        update_op_mode(SOV_MODE);
      }
    }
    else //If pack is not in charging mode then...
    {
      //Stop cell balancing timers
      bq_pack.timer_status &= ~START_CELL_BALANCE_TIMER;
      //reset all cell balancing timers
      bq_pack.balancing_timer=0;
      bq_pack.max_balance_timer=0;
      bq_pack.cell_imbalance_fail_timer=0;
      
      //*Disable all bypass resistors*/
      disable_all_bypass_resistors();
    }
  }
  else //if cell balancing is not active then start balancing
  {
    /*Perform cell balancing only if CHARGE OPERATION is in effect*/
    if (HOST_CONTROL_IN & IN_HOST_CHG_OP)
    {
      //check if cell balancing is required...
      if (((bq_pack.highest_cell_volts - bq_pack.lowest_cell_volts) >= 
          get_u32_value(BALANCE_VOLTS_THRESHOLD)))
      {
        //and allowed...
        if(bq_pack.lowest_cell_volts > get_u32_value(MIN_BALANCE_VOLTS))
        {
          //Start cell balancing timer
          bq_pack.timer_status |= START_CELL_BALANCE_TIMER;
          //reset cell balancing timers  
          bq_pack.balancing_timer = 0;
          bq_pack.max_balance_timer = 0;
          bq_pack.cell_imbalance_fail_timer = 0;
          //enable the bypass resistors
          
          for (dev_id=0; dev_id<NUMBER_OF_BQ_DEVICES; dev_id++)
          {
            //*Enable bypass resistor for all balanced cells*/
            enable_bypass_resistor(dev_id, (~find_imbalanced_cell(dev_id)));
            
          }
        }
      }
    }
  }
}
#endif

/**
* @brief Function Name: CheckEndOfChargeOrDischargeModes .                                                
* @brief Description  : Identifies if the pack is in end of charge or end of 
* discharge modes .
* @param parameters   : none                                                    
* @return Value       : none                                                    
*/     
void CheckEndOfChargeOrDischargeModes(void)
{
   //*For Operation mode different than FAULT MODE*/
  if (bq_pack.op_mode < FAULT_MODE)
  {  //check if pack voltage is lower than the end of discharge voltage
    if (bq_pack.voltage < get_u32_value(PACK_END_OF_DISCHARGE_VOLTAGE))
    {
      update_op_mode(END_OF_DISCHARGE);

    }//check if the battery pack voltage is higher than the end of charge volt
    else if (bq_pack.voltage > get_u32_value(PACK_END_OF_CHARGE_VOLTAGE))
    {
      update_op_mode(END_OF_CHARGE);

    } 
  }
}

/**
* @brief Function Name: BatteryPackManager    .                                             
* @brief Description  : Battery Manger Function.
* @param parameters   : none                                                    
* @return Value       : none                                                    
*/     
void BatteryPackManager(void)
{
  
  CheckFaultConditions();
  CheckChargeDischargeModes();
  CheckEndOfChargeOrDischargeModes();
#if CELL_BALANCING_EN  
  CellBalancing();  
#endif  
  
}

/**
* @brief Function Name: get_bq_pack_voltage.                                                 
* @brief Description  : Returns Pack voltage parameter.
* @param parameters   : none                                                    
* @return Value       : returns last known pack voltage                                                    
*/     
unsigned short get_bq_pack_voltage(void)
{
  return bq_pack.voltage;
}

/**
* @brief Function Name: get_bq_pack_timer.                                                 
* @brief Description  : Returns BQ Pack time counter (for End of Charge or 
* End of Discharge).
* @param parameters   : none                                                    
* @return Value       : returns the last known value of the pack timer                                                      
*/     
unsigned short get_bq_pack_timer(void)
{
  return bq_pack.eoc_eod_timer;
}

/**
* @brief Function Name: update_bq_pack_timer     .                                            
* @brief Description  : update the multiple pack virtual timers.
* @param parameters   : none                                                    
* @return Value       : none                                                    
*/     
void update_bq_pack_timer(void)
{
  if (bq_pack.timer_status & START_END_OF_CHG_DSCHG_TIMER)
  {
    bq_pack.eoc_eod_timer++;
  }
  
  if (bq_pack.timer_status & START_CHARGE_TAPER_TIMER)
  {
    bq_pack.charge_taper_timer++;
  }
  

  if (bq_pack.timer_status & START_CHG_DSCHG_OP_TIMER)
  {
    bq_pack.chg_dschg_op_timer++;
  }

  
#if CELL_BALANCING_EN
  if (bq_pack.timer_status & START_CELL_BALANCE_TIMER)
  {
    bq_pack.balancing_timer++;
    bq_pack.max_balance_timer++;
    
    //Update imbalance timer if voltage between any 2 cells 
    //> CELL_IMBALANCE_THRESHOLD
    if (cell_imbalance_threshold_reached())
    {
      bq_pack.cell_imbalance_fail_timer++;
    }
    else
    {
      bq_pack.cell_imbalance_fail_timer = 0;
    }
  }
#endif
  
  return;
}


/**
* @brief Function Name: get_bq_pack_mode    .                                             
* @brief Description  : returns current pack mode.
* @param parameters   : none                                                    
* @return Value       : returns the current value of the mode                                                    
*/     
op_modes_t get_bq_pack_mode(void)
{
  return bq_pack.op_mode;
}

void set_bq_pack_mode(op_modes_t mode)
{
  bq_pack.op_mode = mode;
  
  return;
}


/******************************************************************************/
/*                            Local functions                                 */
/******************************************************************************/
/**
* @brief Function Name:  conf_bq_dev                  .                              
* @brief Description  :  configures each BQ76PL536 included in the stack.
* @param parameters   :  number of BQ device in the stack                                                   
* @return Value       :  none                                                   
*/     
void conf_bq_dev(bq_dev_t* this)
{
  unsigned short temp;
  
  bq_dev_write_reg(this->device_address, ADC_CONTROL_REG, ADC_CONTROL_VAL_6);
  
  bq_dev_write_reg(this->device_address, IO_CONTROL_REG, IO_CONTROL_VAL);

  bq_dev_write_reg(this->device_address, CB_CTRL_REG, CB_CTRL_VAL);
  bq_dev_write_reg(this->device_address, CB_TIME_REG, CB_TIME_VAL);

  bq_dev_write_reg(this->device_address, SHDW_CTRL_REG, SHDW_CTRL_ACCESS_EN_VAL);
  
  bq_dev_write_reg(this->device_address, FUNCTION_CONFIG_REG, FUNC_CONFIG_VAL_6);
  
  bq_dev_write_reg(this->device_address, SHDW_CTRL_REG, SHDW_CTRL_ACCESS_EN_VAL);
  bq_dev_write_reg(this->device_address, IO_CONFIG_REG, IO_CONFIG_VAL);

  temp = get_u32_value(COV_THRESHOLD);
  if (temp > 2000)
    temp = (temp - 2000)/50;
  else
    temp = 0;
  
  bq_dev_write_reg(this->device_address, SHDW_CTRL_REG, SHDW_CTRL_ACCESS_EN_VAL);
  bq_dev_write_reg(this->device_address, CONFIG_COV_REG, CONFIG_COV_VAL | temp);
  bq_dev_write_reg(this->device_address, SHDW_CTRL_REG, SHDW_CTRL_ACCESS_EN_VAL);
  bq_dev_write_reg(this->device_address, CONFIG_COVT_REG, 
                   CONFIG_COVT_VAL | COV_TIME);

  temp = get_u32_value(CUV_THRESHOLD);
  if (temp > 700)
    temp = (temp - 700)/100;
  else
    temp = 13;  /*Def CUV Threshold value = 2000mV*/
  bq_dev_write_reg(this->device_address, SHDW_CTRL_REG, SHDW_CTRL_ACCESS_EN_VAL);
  /*Def CUV Threshold = 2V*/
  bq_dev_write_reg(this->device_address, CONFIG_CUV_REG, CONFIG_CUV_VAL | temp);
  bq_dev_write_reg(this->device_address, SHDW_CTRL_REG, SHDW_CTRL_ACCESS_EN_VAL);
  bq_dev_write_reg(this->device_address, CONFIG_CUVT_REG, CONFIG_CUVT_VAL 
                   | CUV_TIME);

  bq_dev_write_reg(this->device_address, SHDW_CTRL_REG, SHDW_CTRL_ACCESS_EN_VAL);
  /*TS2=TS1=50stC*/
  bq_dev_write_reg(this->device_address, CONFIG_OT_REG, CONFIG_OT_VAL);
  bq_dev_write_reg(this->device_address, SHDW_CTRL_REG, SHDW_CTRL_ACCESS_EN_VAL);
  /*Over Temp Time delay = 2000ms*/
  bq_dev_write_reg(this->device_address, CONFIG_OTT_REG, CONFIG_OTT_VAL);

  return;
}

/**
* @brief Function Name: init_bq_dev_data_structure .                                                
* @brief Description  : initializes the structure that stores the information 
* of each BQ device .
* @param parameters   : number of BQ device in the stack                                                      
* @return Value       : none                                                    
*/     
void init_bq_dev_data_structure(bq_dev_t* this)
{
  bq_dev_read_reg(this->device_address, DEVICE_STATUS_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &this->device_status);
    
  bq_dev_read_reg(this->device_address, TEMPERATURE1_L_REG, 2, DISCARD_CRC, 
                 (unsigned char*) &this->temperature1);
  bq_dev_read_reg(this->device_address, TEMPERATURE2_L_REG, 2, DISCARD_CRC, 
                 (unsigned char*) &this->temperature2);

  /*Errors handling*/
  bq_dev_read_reg(this->device_address, ALERT_STATUS_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &this->alert_status);
  bq_dev_read_reg(this->device_address, FAULT_STATUS_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &this->fault_status);
  bq_dev_read_reg(this->device_address, COV_FAULT_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &this->cov_fault);
  bq_dev_read_reg(this->device_address, CUV_FAULT_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &this->cuv_fault);

  /*Clear alerts and faults (May not be so trivial...)*/
  bq_dev_clear_alerts(this);
  bq_dev_clear_faults(this);  
  
  return;
}


/**
* @brief Function Name: bq_dev_read_cell_voltage.                                                  
* @brief Description  : reads the cell voltage on a specific BQ device.
* @param parameters   : BQ device number in the stack                                                   
* @return Value       : returns the cell voltage read from the BQ device                                                    
*/     
short bq_dev_read_cell_voltage(bq_dev_t* this)
{
  unsigned char* pPtr;
  unsigned char temp;
  short i, ret_val;
  unsigned long voltage_comput;
  
  ret_val = bq_dev_read_reg(this->device_address, VCELL1_L_REG, 
            (MAX_CELLS_NUMBER_IN_BQ<<1) , DISCARD_CRC, 
            (unsigned char *) &this->cell_voltage[0]);
  
  for (i=0; i<MAX_CELLS_NUMBER_IN_BQ; i++)
  {
    //Swap the data in the array as BQ dev returns data in Big Endian notation
    pPtr = (unsigned char *)(&this->cell_voltage[i]);
    temp = *pPtr;
    *(pPtr) = *(pPtr+1);
    *(pPtr+1) = temp;
    
    //compute cell voltage -> cell_voltage=VCELL x 6250/16383
    voltage_comput = this->cell_voltage[i] * (unsigned long)(adc_step_mul);
    this->cell_voltage[i] = voltage_comput/((unsigned long)(adc_step_div));
  }
  
  return ret_val;
}



/**
* @brief Function Name: bq_dev_clear_alerts .                                                 
* @brief Description  : Clear the alert flags on the BQ device.
* @param parameters   : specific device ID                                                    
* @return Value       : none                                                    
*/     
void bq_dev_clear_alerts(bq_dev_t* this)
{
  unsigned char Value;

  //clear alert bit in device status register
  bq_dev_read_reg(this->device_address, DEVICE_STATUS_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &Value);
  Value |= BIT5; 
  //set alert bit as 1
  bq_dev_write_reg(this->device_address, DEVICE_STATUS_REG, Value);
  Value &= ~BIT5; 
  //clear alert bit
  bq_dev_write_reg(this->device_address, DEVICE_STATUS_REG, Value);
  
  //Read ALERT_STATUS_REG register
  bq_dev_read_reg(this->device_address, ALERT_STATUS_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &Value); 
  //Write 1's to ALERT_STATUS_REG register
  bq_dev_write_reg(this->device_address, ALERT_STATUS_REG, Value);
  
  Value = 0x00;
  // Write 0's ALERT_STATUS_REG register
  bq_dev_write_reg(this->device_address, ALERT_STATUS_REG, Value);
 
}


/**
* @brief Function Name: bq_dev_clear_faults.                                                 
* @brief Description  : Clears the fault flags on the BQ device.
* @param parameters   : Device ID                                                     
* @return Value       : none                                                    
*/     
void bq_dev_clear_faults(bq_dev_t* this)
{
  unsigned char Value;
  
    //clear fault bit in device status register
  bq_dev_read_reg(this->device_address, DEVICE_STATUS_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &Value);
  Value |= BIT6; 
  //set fault bit as 1
  bq_dev_write_reg(this->device_address, DEVICE_STATUS_REG, Value);
  Value &= ~BIT6; 
  //clear fault bit
  bq_dev_write_reg(this->device_address, DEVICE_STATUS_REG, Value);

  //Read FAULT_STATUS register
  bq_dev_read_reg(this->device_address, FAULT_STATUS_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &Value); 
  //Write 1's to FAULT_STATUS register
  bq_dev_write_reg(this->device_address, FAULT_STATUS_REG, Value);
  
  Value = 0x00;
  // Write 0's FAULT_STATUS register
  bq_dev_write_reg(this->device_address, FAULT_STATUS_REG, Value);
  
  
}


/**
* @brief Function Name: bq_dev_read_errors.                                                 
* @brief Description  : Reads the device status, alert status, fault status,
* COV status, CUV status registers.
* @param parameters   : Device ID                                                    
* @return Value       : zero when the BQ device was read sucessfully                                                    
*/     
short bq_dev_read_errors(bq_dev_t* this)
{
  bq_dev_read_reg(this->device_address, DEVICE_STATUS_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &this->device_status);
  bq_dev_read_reg(this->device_address, ALERT_STATUS_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &this->alert_status);
  bq_dev_read_reg(this->device_address, FAULT_STATUS_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &this->fault_status);
  bq_dev_read_reg(this->device_address, COV_FAULT_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &this->cov_fault);
  bq_dev_read_reg(this->device_address, CUV_FAULT_REG, 1, DISCARD_CRC, 
                 (unsigned char*) &this->cuv_fault);
  
  return 0;
}


/**
* @brief Function Name: bq_dev_read_temps.                                                 
* @brief Description  : reads the temperature registers.
* @param parameters   : Device ID                                                    
* @return Value       : zero when the BQ device was read sucessfully                                                    
*/     
short bq_dev_read_temps(bq_dev_t* this)
{
  unsigned char data[2];
  
  //Bytes need to be swaped as BQ device supports Big Endian
  bq_dev_read_reg(this->device_address, TEMPERATURE1_L_REG, 2, DISCARD_CRC, 
                 (unsigned char*) &data[0]);
  this->temperature1 = ((data[0] << 8) | data[1]);
  bq_dev_read_reg(this->device_address, TEMPERATURE2_L_REG, 2, DISCARD_CRC, 
                 (unsigned char*) &data[0]);
  this->temperature2 = ((data[0] << 8) | data[1]);
 
  return 0;
}


/**
* @brief Function Name: bq_dev_read_status.                                                 
* @brief Description  : reads the devoce status register.
* @param parameters   : Device ID                                                    
* @return Value       : zero when the BQ device was read sucessfully  
*/  
short bq_dev_read_status(bq_dev_t* this)
{
  return bq_dev_read_reg(this->device_address, DEVICE_STATUS_REG, 1, 
         DISCARD_CRC, (unsigned char*) &this->device_status);
}


/**
* @brief Function Name: update_op_mode .                                                 
* @brief Description  : updates the battery pack status.
* @param parameters   : new operating mode                                                    
* @return Value       : none                                                    
*/     
void update_op_mode(op_modes_t new_mode)
{
	char OutputString[MAX_STR_LENGTH] = "";   // Holds the outgoing string

  if (bq_pack.op_mode == new_mode)
    return;
  
  bq_pack.op_mode = new_mode;
  
  switch (new_mode)
  {
    case CHARGE_OP:
// Prepare the outgoing string
      strcpy(OutputString,"\r\nThe battery pack is in CHARGE_OP Mode\r\n"); 
#ifdef USB_COMM
// Send the response over USB                                    
      sendData_waitTilDone((BYTE*)OutputString,strlen(OutputString),1,10);                  
#endif
#ifdef UART_COMM
// Send the message to the host over UART
      putsUART((BYTE*)OutputString,strlen(OutputString)); 
#endif 
      
      break;
      
    case END_OF_CHARGE:
// Prepare the outgoing string
      strcpy(OutputString,"\r\nThe battery pack is in END_OF_CHARGE Mode\r\n");
#ifdef USB_COMM
// Send the response over USB                                    
      sendData_waitTilDone((BYTE*)OutputString,strlen(OutputString),1,10);                  
#endif
#ifdef UART_COMM
// Send the message to the host over UART
      putsUART((BYTE*)OutputString,strlen(OutputString)); 
#endif 
      
      break;
      
    case DISCHARGE_OP:
// Prepare the outgoing string
  strcpy(OutputString,"\r\nThe battery pack is in DISCHARGE_OP Mode\r\n"); 
#ifdef USB_COMM
// Send the response over USB                                    
      sendData_waitTilDone((BYTE*)OutputString,strlen(OutputString),1,10);                  
#endif
#ifdef UART_COMM
// Send the message to the host over UART
      putsUART((BYTE*)OutputString,strlen(OutputString)); 
#endif 
      
      break;
      
    case END_OF_DISCHARGE:
// Prepare the outgoing string
  strcpy(OutputString,"\r\nThe battery pack is in END_OF_DISCHARGE Mode\r\n");
#ifdef USB_COMM
// Send the response over USB                                    
      sendData_waitTilDone((BYTE*)OutputString,strlen(OutputString),1,10);                  
#endif
#ifdef UART_COMM
// Send the message to the host over UART
      putsUART((BYTE*)OutputString,strlen(OutputString)); 
#endif  
      
      break;
      
    case FAULT_MODE:
// Prepare the outgoing string
      strcpy(OutputString,"\r\nThe battery pack is in FAULT_MODE Mode\r\n"); 
#ifdef USB_COMM
// Send the response over USB                                    
      sendData_waitTilDone((BYTE*)OutputString,strlen(OutputString),1,10);                  
#endif
#ifdef UART_COMM
// Send the message to the host over UART
      putsUART((BYTE*)OutputString,strlen(OutputString)); 
#endif 
     
      //Disable Balancing Virtual Timer
      bq_pack.timer_status &= ~START_CELL_BALANCE_TIMER;
      break;
      
    case SOV_MODE:
// Prepare the outgoing string
      strcpy(OutputString,"\r\nThe battery pack is in SOV_MODE Mode\r\n");
#ifdef USB_COMM
// Send the response over USB                                    
      sendData_waitTilDone((BYTE*)OutputString,strlen(OutputString),1,10);                  
#endif
#ifdef UART_COMM
// Send the message to the host over UART
      putsUART((BYTE*)OutputString,strlen(OutputString)); 
#endif     
      //Disable Cell Balancing Timer
      bq_pack.timer_status &= ~START_CELL_BALANCE_TIMER;
      break;
      
    case INITIAL_MODE:

      //Clear error_status
      bq_pack.error_status = 0;
      
      //Clear charge/discharge timer
      bq_pack.chg_dschg_op_timer = 0;

      //Start Charge/Discharge operation timer
      bq_pack.timer_status |= START_CHG_DSCHG_OP_TIMER;
      
//      //Disable DRDY pin interrupt
//      BQ76PL536_DRDY_PxIE  &= ~IN_BQ_DRDY;
  
      //Trigger 'PL536 ADC conversion and then..
      bq_pack_start_conv();  
  
//      //Wait for data to be ready
//      while(!(BQ76PL536_DRDY_PxIN  & IN_BQ_DRDY))
//        ;
  
      //Read data from BQ pack for overall system verification
      update_bq_pack_data();     
      //Copy current cell values
      copy_cell_voltage_values();  
  
//      //Enable DRDY pin interrupt
//      BQ76PL536_DRDY_PxIFG  &= ~IN_BQ_DRDY;
//      BQ76PL536_DRDY_PxIE  |= IN_BQ_DRDY;

// Prepare the outgoing string
      strcpy(OutputString,"\r\nThe battery pack is in INITIAL_MODE Mode\r\n"); 
#ifdef USB_COMM
// Send the response over USB                                    
      sendData_waitTilDone((BYTE*)OutputString,strlen(OutputString),1,10);                  
#endif
#ifdef UART_COMM
// Send the message to the host over UART
      putsUART((BYTE*)OutputString,strlen(OutputString)); 
#endif 
      
      break;
    

    default:
      break;  
  }

  return;
}


/**
* @brief Function Name: check_for_cov    .                                             
* @brief Description  : Checks for COV conditions.
* @param parameters   : none                                                    
* @return Value       : 1 when the COV condition exist, 0 when it doesn't                                                    
*/     
short check_for_cov(void)
{

  unsigned short i, cell;

  for (i=0; i<NUMBER_OF_BQ_DEVICES; i++)
  {
    if (bq_pack.bq_devs[i].fault_status & (1<<FAULT_COV_POS))
    {
      //Check if error condition is still in effect
      for (cell=0; cell < bq_pack.bq_devs[i].cell_count; cell++)
      {
        if (bq_pack.bq_devs[i].cov_fault & (1<<cell))
        {
          if (bq_pack.bq_devs[i].cell_voltage[cell] 
              > get_u32_value(COV_RECOVERY_THRESHOLD))
          {
            //COV condition still present
            return 1; //error
          }
          else
          {
            //COV condition cleared
            //clear FAULT flag
            bq_dev_write_reg(bq_pack.bq_devs[i].device_address, 
                             FAULT_STATUS_REG, (1<<FAULT_COV_POS));
            bq_dev_write_reg(bq_pack.bq_devs[i].device_address, 
                             FAULT_STATUS_REG, (0<<FAULT_COV_POS));
            
            //Assumption COV and CUV are mutually exclusive
            update_op_mode(INITIAL_MODE);
          }
        }
      }
    }
  }
  
  return 0;
}


/**
* @brief Function Name: check_for_cuv .                                                
* @brief Description  : Checks for CUV conditions.
* @param parameters   : none                                                    
* @return Value       : 1 when the condition exist, 0 when it doesn't                                                    
*/     
short check_for_cuv(void)
{
  unsigned short i, cell;

  for (i=0; i<NUMBER_OF_BQ_DEVICES; i++)
  {
    if (bq_pack.bq_devs[i].fault_status & (1<<FAULT_CUV_POS))
    {
      //Check if error condition is still in effect
      for (cell=0; cell < bq_pack.bq_devs[i].cell_count; cell++)
      {
        if (bq_pack.bq_devs[i].cuv_fault & (1<<cell))
        {
          if (bq_pack.bq_devs[i].cell_voltage[cell] 
              < get_u32_value(CUV_RECOVERY_THRESHOLD))
          {
            //CUV condition still present
            return 1; //error
          }
          else
          {
            //CUV condition cleared
            //clear FAULT flag
            bq_dev_write_reg(bq_pack.bq_devs[i].device_address, 
                            FAULT_STATUS_REG, (1<<FAULT_CUV_POS));
            bq_dev_write_reg(bq_pack.bq_devs[i].device_address, 
                             FAULT_STATUS_REG, (0<<FAULT_CUV_POS));
            
            //Assumption COV and CUV are mutually exclusive
            update_op_mode(INITIAL_MODE);
          }
        }
      }
    }
  }
  
  return 0;
}


/**
* @brief Function Name: check_for_pot   .                                            
* @brief Description  : Checks for POT conditions.
* @param parameters   : none                                                    
* @return Value       : 1 when the condition exist, 0 when it doesn't                                                    
*/     
short check_for_pot(void)
{
  /*Temp sensor connected to first BQ device only*/
  if (bq_pack.bq_devs[0].alert_status & (0x3<<ALERT_OT1_POS))
  {
      return 1; //error
  }
  
  return 0;
}


/**
* @brief Function Name: check_for_charge_op.                                               
* @brief Description  : Checks if the battery pack is in charge mode by
* checking if the cell voltages have increased.
* @param parameters   : none                                                    
* @return Value       : 1 when the condition exist, 0 when it doesn't                                                    
*/     
unsigned short check_for_charge_op(void)
{
  unsigned short i, j, cell_cnt;
  
  j=0;
  for (i=0; i<NUMBER_OF_BQ_DEVICES; i++)
  {
    /*Read each cell voltage and compare it vs initial cell voltage*/
    for (cell_cnt=0; cell_cnt<bq_pack.bq_devs[i].cell_count; cell_cnt++)
    {
      if (bq_pack.bq_devs[i].cell_voltage[cell_cnt] > cell_values[j])
      {
        if ((bq_pack.bq_devs[i].cell_voltage[cell_cnt] - cell_values[j]) 
             > get_u32_value(DELTA_CHARGE_V))
        {
          return 1;
        }
      }
      j++;
    }
  }
  
  return 0;
}


/**
* @brief Function Name: check_for_discharge_op  .                                             
* @brief Description  : Checks if the battery pack is in discharge mode by
* checking if the cell voltages have decreased.
* @param parameters   : none                                                    
* @return Value       : 1 when the condition exist, 0 when it doesn't                                                    
*/   
unsigned short check_for_discharge_op(void)
{
  unsigned short i, j, cell_cnt;
  
  j=0;
  for (i=0; i<NUMBER_OF_BQ_DEVICES; i++)
  {
    /*Read each cell voltage and compare it vs initial cell voltage*/
    for (cell_cnt=0; cell_cnt<bq_pack.bq_devs[i].cell_count; cell_cnt++)
    {
      if (cell_values[j] > bq_pack.bq_devs[i].cell_voltage[cell_cnt])
      {
        if ((cell_values[j] - bq_pack.bq_devs[i].cell_voltage[cell_cnt]) 
              > get_u32_value(DELTA_DISCHARGE_V))
        {
          return 1;
        }
      }
      j++;
    }
  }
  
  return 0;
}


/**
* @brief Function Name: copy_cell_voltage_values.                                                  
* @brief Description  : creates a backup of the last known cells voltages.
* @param parameters   : none                                                    
* @return Value       : none                                                    
*/     
void copy_cell_voltage_values(void)
{
  unsigned short i, j, cell_cnt;
  
  j = 0;
  for (i=0; i<NUMBER_OF_BQ_DEVICES; i++)
  {
    /*Read each cell voltage and compare it vs initial cell voltage*/
    for (cell_cnt=0; cell_cnt<bq_pack.bq_devs[i].cell_count; cell_cnt++)
    {
      cell_values[j++] = bq_pack.bq_devs[i].cell_voltage[cell_cnt];
    }
  }
  
  return;
}
    

/**
* @brief Function Name: cell_imbalance_threshold_reached  .                                             
* @brief Description  : Checks if the cell imbalance threshold is reached
* @param parameters   : none                                                    
* @return Value       : 1 when the condition exist, 0 when it doesn't                                                    
*/   
unsigned short cell_imbalance_threshold_reached(void)
{
  if ((bq_pack.highest_cell_volts - bq_pack.lowest_cell_volts) 
       > get_u32_value(CELL_IMBALANCE_FAIL_THRESHOLD))
    return 1;
  else
    return 0;
}

/**
* @brief Function Name: find_imbalanced_cell .                                                 
* @brief Description  : finds the imbalanced cells.
* @param parameters   : device ID                                                    
* @return Value       : Returns 0 if cells are balanced and other if there are 
* any imbalanced cells                                                     .
*/     
unsigned short find_imbalanced_cell(unsigned short in_dev_id)
{
  unsigned short cell_id, imb_cells_mask, cnt;
  
  cnt = 0;
  imb_cells_mask = 0xFFFF;
  
  /*Read each cell voltage and compare it vs lowest cell voltage*/
  for (cell_id=0; cell_id<bq_pack.bq_devs[in_dev_id].cell_count; cell_id++)
  {
    imb_cells_mask &= ~(1<<cnt);
    
    if ((bq_pack.highest_cell_volts 
          - bq_pack.bq_devs[in_dev_id].cell_voltage[cell_id]) >= 
          get_u32_value(BALANCE_VOLTS_THRESHOLD))
    {
      imb_cells_mask |= (1<<cnt);
    }
    cnt++;
  }
  
  return imb_cells_mask;
}

/**
* @brief Function Name: enable_bypass_resistor.                                                 
* @brief Description  : Enable bypass resistors by controlling the CB outputs. 
* @param parameters   : Device ID, CB outputs to be controlled                                                    
* @return Value       : none                                                     
*/     
void enable_bypass_resistor(unsigned short in_dev_id, unsigned short in_value)
{
  bq_dev_write_reg(bq_pack.bq_devs[in_dev_id].device_address, 
                   CB_TIME_REG, get_u32_value(BALANCE_TIME));
  bq_dev_write_reg(bq_pack.bq_devs[in_dev_id].device_address, 
                   CB_CTRL_REG, (0x003F & in_value));
  
  return;
}


/**
* @brief Function Name: disable_bypass_resistor.                                                 
* @brief Description  : disable bypass resistors by controlling the CB outputs .
* @param parameters   : Device ID, CB outputs to be controlled                                                    
* @return Value       : none                                                     
*/
void disable_bypass_resistor(unsigned short in_dev_id, unsigned short in_value)
{
  unsigned char reg_val;
  //Read CB_CTRL_REG register
  bq_dev_read_reg(bq_pack.bq_devs[in_dev_id].device_address, CB_CTRL_REG, 1, 
                  DISCARD_CRC, (unsigned char*) &reg_val);
  
  //Write 0's to the bits in the CB_CTRL_REG register
  reg_val &= ~(in_value);
  bq_dev_write_reg(bq_pack.bq_devs[in_dev_id].device_address, 
                   CB_CTRL_REG, reg_val);
  
  return;
}


/**
* @brief Function Name: disable_all_bypass_resistors.                                                 
* @brief Description  : disable all battery pack bypass resistors by controlling 
* the CB outputs .
* @param parameters   : none                                                    
* @return Value       : none                                                     
*/
void disable_all_bypass_resistors(void)
{
  unsigned short bq_dev_id;
  
  for (bq_dev_id=0; bq_dev_id<NUMBER_OF_BQ_DEVICES; bq_dev_id++)
  {
    bq_dev_write_reg(bq_pack.bq_devs[bq_dev_id].device_address, CB_CTRL_REG, 0);
  }
  
  return;
}


/*EOF*/

