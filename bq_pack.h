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
*  @file bq_pack.h
*
*  @brief this file contains all the definitions of the BQ76PL536 devices
*
*  @author Texas Instruments, Inc
*  @date November 2010
*  @version 1.0 Initial version
*  @note Built with IAR for MSP430 Version: 5.10
*/

#ifndef BQ_PACK_H
#define BQ_PACK_H

#include "data_flash.h"

/**
* @brief  BQ pack signals defines         .                     
*/

///SW flag that controls the charge/discharge mode
#define IN_HOST_CHG_OP     BIT0

///*BQ76PL536 to MSP430 signals defines*/

//Port interrupt vector definition
#define BQ76PL536_OUTPUTS_VECTOR PORT1_VECTOR
#define BQ76PL536_OUTPUTS_PxIV P1IV
//port used to control the CONV signal
//#define BQ76PL536_CONV_PxIN   P1IN
//#define BQ76PL536_CONV_PxOUT  P1OUT
//#define BQ76PL536_CONV_PxDIR  P1DIR
//#define BQ76PL536_CONV_PxIFG  P1IFG
//#define BQ76PL536_CONV_PxIES  P1IES
//#define BQ76PL536_CONV_PxIE   P1IE
//#define BQ76PL536_CONV_PxSEL  P1SEL1
//#define BQ76PL536_CONV_PxREN  P1REN
//#define OUT_BQ_CONV           BIT3 //Port X.4
//Port used to control the LED on the MSP430 board
#define BQ76PL536_LED_PxIN   P1IN
#define BQ76PL536_LED_PxOUT  P1OUT
#define BQ76PL536_LED_PxDIR  P1DIR
#define BQ76PL536_LED_PxIFG  P1IFG
#define BQ76PL536_LED_PxIES  P1IES
#define BQ76PL536_LED_PxIE   P1IE
#define BQ76PL536_LED_PxSEL  P1SEL0
#define BQ76PL536_LED_PxREN  P1REN
#define OUT_BQ_LED            BIT0
//Port used to sensing the DRDY signal
//#define BQ76PL536_DRDY_PxIV   P1IV
//#define BQ76PL536_DRDY_PxIN   P1IN
//#define BQ76PL536_DRDY_PxOUT  P1OUT
//#define BQ76PL536_DRDY_PxDIR  P1DIR
//#define BQ76PL536_DRDY_PxIFG  P1IFG
//#define BQ76PL536_DRDY_PxIES  P1IES
//#define BQ76PL536_DRDY_PxIE   P1IE
//#define BQ76PL536_DRDY_PxSEL  P1SEL0
//#define BQ76PL536_DRDY_PxREN  P1REN
//Port used to sensing the ALERT signal
//#define BQ76PL536_ALERT_PxIN   P1IN
//#define BQ76PL536_ALERT_PxOUT  P1OUT
//#define BQ76PL536_ALERT_PxDIR  P1DIR
//#define BQ76PL536_ALERT_PxIFG  P1IFG
//#define BQ76PL536_ALERT_PxIES  P1IES
//#define BQ76PL536_ALERT_PxIE   P1IE
//#define BQ76PL536_ALERT_PxSEL  P1SEL0
//#define BQ76PL536_ALERT_PxREN  P1REN
//Port used to sensing the FAULT signal
//#define BQ76PL536_FAULT_PxIN   P1IN
//#define BQ76PL536_FAULT_PxOUT  P1OUT
//#define BQ76PL536_FAULT_PxDIR  P1DIR
//#define BQ76PL536_FAULT_PxIFG  P1IFG
//#define BQ76PL536_FAULT_PxIES  P1IES
//#define BQ76PL536_FAULT_PxIE   P1IE
//#define BQ76PL536_FAULT_PxSEL  P1SEL0
//#define BQ76PL536_FAULT_PxREN  P1REN
//#define IN_BQ_DRDY             BIT4 //PORTx.3
//#define IN_BQ_ALERT            BIT2 //PORTx.2
//#define IN_BQ_FAULT            BIT1 //PORTx.1

//ADC_ON, TS2&TS1, GPAI, Cells 1-2-3.....
#define ADC_CONTROL_VAL_6 ((1<<6)|(3<<4)|(0<<3)|(5<<0))
#define ADC_CONTROL_VAL_5 ((1<<6)|(3<<4)|(0<<3)|(4<<0))
#define ADC_CONTROL_VAL_4 ((1<<6)|(3<<4)|(0<<3)|(3<<0))

//AUX,GPIO_OUT,GPIO_IN,SLEEP,TS2,TS1
#define IO_CONTROL_VAL    ((0<<7)|(0<<6)|(0<<5)|(0<<2)|(0<<1)|(3<<0)) //originally (3<<0) was (0<<0) because TS2 and TS1 were disabled to reduce current draw

//cell ballancing defines
#define CB_CTRL_VAL       0
#define CB_TIME_VAL       0

//CONV START
#define ADC_CONVERT_VAL   (1<<0)

//Enables access to Group 3 protected registers
#define SHDW_CTRL_ACCESS_EN_VAL     0x35  
//Preloads Croup 3 registers from data in EPROM
#define SHDW_CTRL_REFRESH_EN_VAL    0x27  
//Enables access to USER block of EPROM
#define E_EN_OTP_EN_VAL             0x91 
 
//Conv time ~6us,GPAI_REF,GPAI_SRC,Series Cells, WTD
#define FUNC_CONFIG_VAL_6 ((0x1<<6)|(0<<5)|(0<<4)|(0x0<<2)|(0<<0))
#define FUNC_CONFIG_VAL_5 ((0x1<<6)|(0<<5)|(0<<4)|(0x1<<2)|(0<<0))
#define FUNC_CONFIG_VAL_4 ((0x1<<6)|(0<<5)|(0<<4)|(0x2<<2)|(0<<0))

//*CRC_DIS => CRC expected*/
#define IO_CONFIG_VAL     (0<<0)

#define DISCARD_CRC               (1)
#define RETURN_CRC                (0)

//Over voltage definitions
//Example: (30<<0)/*COV Threshold = 2.0V + 30*50mV = 3.5V*/)
#define CONFIG_COV_VAL    (0<<7)/*DISABLE*/  
//Example: (20<<0)/*COV Time delay = 20*100ms = 2000ms*/) 
#define CONFIG_COVT_VAL   (1<<7)/*ms*/        

//Under voltage definitions
//Example: (0<<0)/*CUV Threshold = 2.0V + 0*50mV = 2V*/)
#define CONFIG_CUV_VAL    (0<<7)
//Example: (20<<0)/*CUV Time delay = 20*100ms = 2000ms*/)
#define CONFIG_CUVT_VAL   (1<<7)/*ms*/ 
       
//Over temperature definitions
//Example: (3<<0)/*TS1 = 50 stC*/)
#define CONFIG_OT_VAL     (3<<4)/*TS2*/ 
#define CONFIG_OTT_VAL    (200<<0)/*Over Temp Time delay = 200*10ms = 2000ms*/

//Cell Voltage [mV] = (REGMSB * 256 + REGLSB) * 6250 / 16383
#define adc_step_mul 6250
#define adc_step_div 16383

//*BQ Pack special addresses*/
#define BROADCAST_ADDR            0x3F
#define DISCOVERY_ADDR            0x00
#define BQ76PL536_RESET           0xa5

//Definition of the BQ76PL536 registers
typedef enum BQ_DEV_REGS
{
  DEVICE_STATUS_REG=0x00,
  GPAI_L_REG=0x01, 
  GPAI_H_REG=0x02,
  VCELL1_L_REG=0x03, 
  VCELL1_H_REG=0x04,
  VCELL2_L_REG=0x05, 
  VCELL2_H_REG=0x06,
  VCELL3_L_REG=0x07, 
  VCELL3_H_REG=0x08,
  VCELL4_L_REG=0x09, 
  VCELL4_H_REG=0x0a,
  VCELL5_L_REG=0x0b, 
  VCELL5_H_REG=0x0c,
  VCELL6_L_REG=0x0d, 
  VCELL6_H_REG=0x0e,
  TEMPERATURE1_L_REG=0x0f, 
  TEMPERATURE1_H_REG=0x10,
  TEMPERATURE2_L_REG=0x11, 
  TEMPERATURE2_H_REG=0x12,
  ALERT_STATUS_REG=0x20,
  FAULT_STATUS_REG=0x21,
  COV_FAULT_REG=0x22,
  CUV_FAULT_REG=0x23,
  PRESULT_A_REG=0x24,
  PRESULT_B_REG=0x25,
  ADC_CONTROL_REG=0x30,
  IO_CONTROL_REG=0x31,
  CB_CTRL_REG=0x32,
  CB_TIME_REG=0x33,
  ADC_CONVERT_REG=0x34,
  SHDW_CTRL_REG=0x3a,
  ADDRESS_CONTROL_REG=0x3b,
  RESET_REG=0x3c,
  TEST_SELECT_REG=0x3d,
  E_EN_REG=0x3F,
  FUNCTION_CONFIG_REG=0x40,
  IO_CONFIG_REG=0x41,
  CONFIG_COV_REG=0x42,
  CONFIG_COVT_REG=0x43,
  CONFIG_CUV_REG=0x44,
  CONFIG_CUVT_REG=0x45,
  CONFIG_OT_REG=0x46,
  CONFIG_OTT_REG=0x47,
  USER1_REG=0x48,
  USER2_REG=0x49,
  USER3_REG=0x4A,
  USER4_REG=0x4B,
  BQ_SPI_REG_MAX=0x4F
} bq_dev_regs_t;

//Definition of the structure that stores the cell information
typedef struct CELL_DATA
{
  unsigned short voltage;
} cell_data_t;

//Definition of the structure that stores the BQ76PL536 information
typedef struct BQ_DEV
{
  unsigned short cell_count;
  unsigned char device_address;
  
  unsigned char device_status;

  unsigned short cell_voltage[MAX_CELLS_NUMBER_IN_BQ];  //[mV]
  
  unsigned short temperature1;
  unsigned short temperature2;

  unsigned char alert_status;
  unsigned char fault_status;
  unsigned char cov_fault;
  unsigned char cuv_fault;
} bq_dev_t;

//bit position of the COV,CUV,OT1 and OT2 flags
#define FAULT_COV_POS         0
#define FAULT_CUV_POS         1
#define ALERT_OT1_POS         0
#define ALERT_OT2_POS         1

//Define the battery pack operating modes
typedef enum OP_MODES
{
  INITIAL_MODE = 0,
  CHARGE_OP,
  END_OF_CHARGE,
  DISCHARGE_OP,
  END_OF_DISCHARGE,
  FAULT_MODE,
  SOV_MODE
} op_modes_t;


//defines the bit position for the error status
#define STATUS_ERROR_POT_COV          BIT1 
#define STATUS_ERROR_CUV              BIT2 
#define STATUS_ERROR_IMBALANCE_FAIL   BIT3 
#define STATUS_ERROR_MAX_BALANCE_TIME BIT4 
// definition of the structure that stores the battery pack information 
typedef struct BQ_PACK
{
  bq_dev_t bq_devs[NUMBER_OF_BQ_DEVICES];
  op_modes_t op_mode;
  unsigned short error_status;
  unsigned short voltage;
  unsigned short lowest_cell_volts;
  unsigned short highest_cell_volts;
  unsigned short last_imbalanced_cell_idx;
  unsigned short timer_status;
  unsigned short eoc_eod_timer;
  unsigned short charge_taper_timer;
  unsigned short chg_dschg_op_timer;
  unsigned short balancing_timer;
  unsigned short max_balance_timer;
  unsigned short cell_imbalance_fail_timer;
} bq_pack_t;

//SW flag defitions for controlling the virtual timers
#define START_END_OF_CHG_DSCHG_TIMER  BIT0
#define START_CHARGE_TAPER_TIMER      BIT1
#define START_CELL_BALANCE_TIMER      BIT2
#define START_CHG_DSCHG_OP_TIMER      BIT3
//redefines the SPI read/write functions
#define bq_dev_write_reg(a,b,c) spi_write_reg(a,(unsigned char)b,c)
#define bq_dev_read_reg(a,b,c,d,e) spi_read_reg(a,(unsigned char)b,c,d,e)


/******************************************************************************/
/*                            Global Variables                                */
/******************************************************************************/
extern bq_pack_t bq_pack; 
extern unsigned char HOST_CONTROL_IN; 
  

/******************************************************************************/
/*                       Global functions declaration                          */
/******************************************************************************/
extern short bq_pack_address_discovery(void);
extern short bq_pack_init(void);
extern void update_bq_pack_data(void);
extern void BatteryPackManager(void);
extern void bq_pack_start_conv(void);
extern unsigned short get_bq_pack_voltage(void);
extern unsigned short get_bq_pack_timer(void);
extern void update_bq_pack_timer(void);
extern op_modes_t get_bq_pack_mode(void);
extern void set_bq_pack_mode(op_modes_t mode);
extern void CheckFaultConditions(void);
extern unsigned short find_imbalanced_cell(unsigned short bq_pack_id);

 
/******************************************************************************/
/*                            Local functions                                 */
/******************************************************************************/
void conf_bq_dev(bq_dev_t* this);
void init_bq_dev_data_structure(bq_dev_t* this);
short bq_dev_read_cell_voltage(bq_dev_t* this);
short bq_dev_read_temps(bq_dev_t* this);
short bq_dev_read_errors(bq_dev_t* this);
short bq_dev_read_status(bq_dev_t* this);
void bq_dev_clear_alerts(bq_dev_t* this);
void bq_dev_clear_faults(bq_dev_t* this);
void update_state_machine(op_modes_t new_mode);
short check_for_cov(void);
short check_for_cuv(void);
short check_for_pot(void);
void update_op_mode(op_modes_t new_mode);
void copy_cell_voltage_values(void);
unsigned short check_for_discharge_op(void);
unsigned short check_for_charge_op(void);
void CellBalancing(void);
void CheckChargeDischargeModes(void);
void CheckEndOfChargeOrDischargeModes(void);
void enable_bypass_resistor(unsigned short in_dev_id, unsigned short in_value);
void disable_bypass_resistor(unsigned short in_dev_id, unsigned short in_value);
void disable_all_bypass_resistors(void);
unsigned short cell_imbalance_threshold_reached(void);



#endif

/*EOF*/
