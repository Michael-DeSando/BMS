//****************************************************************************
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
*  @file spi_if.c
*  @brief this file contains the SPI functions and the CRC calculation
*  @author Texas Instruments, Inc
*  @date November 2010
*  @version 1.0 Initial version
*  @note Built with IAR for MSP430 Version: 5.10
*/ 

#include "msp430.h"
#include "..\main.h"
#include "spi_if.h"
#include "driverlib.h"
#include "TI_USCI_SPI_Regs.h"

/**
* @brief  Local functions          .                     
*/
unsigned char calculate_crc(unsigned char* buffer, 
                            unsigned char buffer_lenght);

/**
* @brief  constants             .                     
*/
//read write masks for the 'PL536 devices
const unsigned char spi_regs_rw_mask[] = 
{
  0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,
  0x1,0x1,0x1,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
  0x3,0x3,0x1,0x1,0x1,0x1,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
  0x3,0x3,0x3,0x3,0x3,0x0,0x0,0x0,0x0,0x0,0x3,0x3,0x2,0x3,0x0,0x3,
  0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x1,0x1,0x1,0x1,0x0,0x0,0x0,0x0
};

//CRC lookup table
const unsigned char CrcTable[] = {
  0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
  0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
  0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
  0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
  0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
  0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
  0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
  0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
  0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
  0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
  0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
  0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
  0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
  0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
  0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
  0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
  0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
  0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
  0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
  0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
  0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
  0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
  0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
  0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
  0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
  0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
  0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
  0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
  0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
  0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
  0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
  0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

/**
* @brief  Global functions         .                     
*/

/**
* @brief Function Name: init_spi   .                                              
* @brief Description  : Initializes the SPI module and the SPI pins .
* @param parameters   : none                                                    
* @return Value       : none                                                    
*/     
void init_spi(void)
{

//	GPIO_setAsInputPinWithPullUpresistor(GPIO_PORT_P1, GPIO_PIN7);

    // Configure P1.7 for SPI_SOMI mode using B0 for BMS chips
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN7,
            GPIO_SECONDARY_MODULE_FUNCTION);


  // Configure P3.0 output pin for SPI_CS
  GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0);
  // Set P3.0 high for CS
  GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);


}


/**
* @brief Function Name:  spi_write_reg .                                                
* @brief Description  :  Writes just 1 byte into the specified bq pack register.
* @param parameters   :  device address, register address, data                                                  
* @return Value       :  return the number of bytes sent                                                    
*/     
short spi_write_reg(unsigned char dev_addr, 
                    unsigned char reg_addr, 
                    unsigned char data)
{
  unsigned char package[4];
  unsigned char TimeOutCounter = 0;
  short i=0;

  EUSCI_B_SPI_disable(EUSCI_B0_BASE);

  EUSCI_B_SPI_changeClockPhasePolarity(EUSCI_B0_BASE,
  		  EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
  		  EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW);

  EUSCI_B_SPI_enable(EUSCI_B0_BASE);


 
  if (spi_regs_rw_mask[reg_addr] & WRITE_ACCESS)
  {
    package[0] = (dev_addr<<1) | 0x01/*Write*/;
    package[1] = reg_addr;
    package[2] = data;
    package[3] = calculate_crc(package, 3);
    
    //Write 1 byte into the selected register
//    STE_PxOUT &= ~IO_SPI_CS;                   // Enable BQ Pack, /CS asserted

    // Set P3.0 low for CS
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0);
    
   
    for (i=0; i<4; i++)
    {
      // USCI_A1 TX buffer ready?
      while (!(UCBxIFG & UCBxTXIFG)){
      	__delay_cycles(1000);
        TimeOutCounter++;
        if(TimeOutCounter >= 200)
          return 0;

      }
        
       
      UCBxTXBUF = package[i];       // Write data to TX to start SPI transfer
       // Data in USCI_A0 RX buffer
      while (!(UCBxIFG & UCBxRXIFG)){
      	__delay_cycles(1000);
        TimeOutCounter++;
        if(TimeOutCounter >= 200)
          return 0;

      }

      package[0] = UCBxRXBUF;     // Dummy read to clean RX buffer
    }
    
    // Set P3.0 high for CS
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
//    STE_PxOUT |= IO_SPI_CS;       // Disable BQ Pack, /CS deasserted
  }

  EUSCI_B_SPI_disable(EUSCI_B0_BASE);

  EUSCI_B_SPI_changeClockPhasePolarity(EUSCI_B0_BASE,
		  EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
  		  EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW);

    EUSCI_B_SPI_enable(EUSCI_B0_BASE);

  return i;  


}

/**
* @brief Function Name: spi_read_reg.                                                  
* @brief Description  :   Read elem_num bytes from SPI reg of the attached 
* device, read data is returned in pData, returns number of received bytes 
* (including CRC as last pData byte if required) .
* @param parameters   :  device address, register address, num of bytes, 
* CRC enable/disable, receiver buffer .                                                   
* @return Value       :  number of bytes received                                                   
*/     
short spi_read_reg(unsigned char dev_addr,
                   unsigned char reg_addr,
                   short elem_num,
                   unsigned char discard_crc,//*True, False*/
                   unsigned char* pData)
{
  unsigned char package[3];
  short i=0;
  unsigned char TimeOutCounter = 0;

  EUSCI_B_SPI_disable(EUSCI_B0_BASE);

  EUSCI_B_SPI_changeClockPhasePolarity(EUSCI_B0_BASE,
  		  EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
  		  EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW);

  EUSCI_B_SPI_enable(EUSCI_B0_BASE);

 
  if (spi_regs_rw_mask[reg_addr] & READ_ACCESS)
  {
    package[0] = (dev_addr<<1)/*Read*/;
    package[1] = reg_addr;
    package[2] = elem_num;
    
    //Write 1 byte into the selected register
//    STE_PxOUT &= ~IO_SPI_CS;                    // Enable BQ Pack, /CS asserted

    // Set P3.0 low for CS
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0);


    if (UCBxIFG & UCBxRXIFG)
    {
      *pData = UCBxRXBUF;                   // Dummy read to clear RX buffer
    }
    

    for (i=0; i<3; i++)
    {
      // USCI_A0 TX buffer ready?
      while (!(UCBxIFG & UCBxTXIFG)){
      	__delay_cycles(1000);
        TimeOutCounter++;
        if(TimeOutCounter >= 200)
          return 0;

      }

      UCBxTXBUF = package[i];       // Write data to TX to start SPI transfer
      
       // Data in USCI_A0 RX buffer
      while (!(UCBxIFG & UCBxRXIFG)){
      	__delay_cycles(1000);
        TimeOutCounter++;
        if(TimeOutCounter >= 200)
          return 0;

      }
      
      *pData = UCBxRXBUF;            // Dummy read to clear RX buffer
    }

    
    for (i=0; i<elem_num+1/*+1 to count CRC*/; i++)
    { // USCI_B0 TX buffer ready?

      while (!(UCBxIFG & UCBxTXIFG)){
      	__delay_cycles(1000);
        TimeOutCounter++;
        if(TimeOutCounter >= 200)
          return 0;

      }

      UCBxTXBUF = 0x00;               // Dummy write to keep clocking going
       // Data in USCI_A0 RX buffer
      while (!(UCBxIFG & UCBxRXIFG)){
      	__delay_cycles(1000);
        TimeOutCounter++;
        if(TimeOutCounter >= 200)
          return 0;

      }

      if (i==elem_num)
      {
        /*Read CRC*/
        if (discard_crc)
          package[0] = UCBxRXBUF;           // Read and discard CRC
        else
          *pData++ = UCBxRXBUF;             // R15 = 00|MSB
      }
      else
      {
        /*Read data*/
        *pData++ = UCBxRXBUF;               // R15 = 00|MSB
      }
    }

    // Set P3.0 high for CS
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);

//    STE_PxOUT |= IO_SPI_CS;                 // Disable BQ Pack, /CS deasserted


  }

  EUSCI_B_SPI_disable(EUSCI_B0_BASE);

  EUSCI_B_SPI_changeClockPhasePolarity(EUSCI_B0_BASE,
		  EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
  		  EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW);

    EUSCI_B_SPI_enable(EUSCI_B0_BASE);

  return i;
}

/**
* @brief  Local functions        .                     
*/

/**
* @brief Function Name: calculate_crc        .                                          
* @brief Description  : calculates CRC coming/going to the 'PL536 device.
* @param parameters   : buffer, buffer size                                                    
* @return Value       : crc                                                    
*/     
unsigned char calculate_crc(unsigned char* buffer, 
                            unsigned char buffer_lenght)
{
  unsigned char crc = 0;
  unsigned short temp = 0;
  unsigned short i;
  for ( i = 0; i < buffer_lenght; i++)
  {
    temp = crc ^ buffer[i];
    crc = CrcTable[temp];
  }
  return crc;
}

/*EOF*/

