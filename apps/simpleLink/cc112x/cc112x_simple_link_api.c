/******************************************************************************
  Filename:       cc1101_simple_link_api.c
  
  Description: 
  
  Notes: 
  
******************************************************************************/


/*****************************************************************************
* INCLUDES
*/
#include "cc112x_spi.h"
#include "trx_rf_int.h"
#include "trx_rf_spi.h"
#include "simpleLink.h"
#include "cc112x_simple_link_api.h"

/******************************************************************************
 * TYPEDEFS
 */
// defines used for the manual calibration
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2

/******************************************************************************
 * LOCAL FUNCTIONS
 */
static void manualCalibration(void);
/*****************************************************************************
* CONSTANTS
*/
static const registerSetting_t simpleLinkCC112xRfSettings[] =
//static const registerSetting_t preferredSettings[]=
{
  {CC112X_IOCFG3,            0xB0},
  {CC112X_IOCFG2,            0x06},
  {CC112X_IOCFG1,            0xB0},
  {CC112X_IOCFG0,            0x06},
  {CC112X_SYNC3,             0xD3},
  {CC112X_SYNC2,             0x91},
  {CC112X_SYNC1,             0xD3},
  {CC112X_SYNC0,             0x91},
  {CC112X_SYNC_CFG1,         0x08},
  {CC112X_MODCFG_DEV_E,      0x0B},
  {CC112X_DCFILT_CFG,        0x1C},
  {CC112X_PREAMBLE_CFG1,     0x18},
  {CC112X_IQIC,              0xC6},
  {CC112X_CHAN_BW,           0x08},
  {CC112X_MDMCFG0,           0x05},
  {CC112X_AGC_REF,           0x20},
  {CC112X_AGC_CS_THR,        0x19},
  {CC112X_AGC_CFG1,          0xA9},
  {CC112X_AGC_CFG0,          0xCF},
  {CC112X_FIFO_CFG,          0x00},
  {CC112X_SETTLING_CFG,      0x03},
  {CC112X_FS_CFG,            0x12},
  {CC112X_PKT_CFG0,          0x20},
  {CC112X_PA_CFG2,           0x5D},
  {CC112X_PKT_LEN,           0xFF},
  {CC112X_IF_MIX_CFG,        0x00},
  {CC112X_FREQOFF_CFG,       0x22},
  {CC112X_FREQ2,             0x6C},
  {CC112X_FREQ1,             0x89},
  {CC112X_FREQ0,             0x99},
  {CC112X_FS_DIG1,           0x00},
  {CC112X_FS_DIG0,           0x5F},
  {CC112X_FS_CAL1,           0x40},
  {CC112X_FS_CAL0,           0x0E},
  {CC112X_FS_DIVTWO,         0x03},
  {CC112X_FS_DSM0,           0x33},
  {CC112X_FS_DVC0,           0x17},
  {CC112X_FS_PFD,            0x50},
  {CC112X_FS_PRE,            0x6E},
  {CC112X_FS_REG_DIV_CML,    0x14},
  {CC112X_FS_SPARE,          0xAC},
  {CC112X_FS_VCO0,           0xB4},
  {CC112X_XOSC5,             0x0E},
  {CC112X_XOSC1,             0x03},
  {CC112X_PARTNUMBER,        0x48},
  {CC112X_PARTVERSION,       0x21},
//  {CC112X_MODEM_STATUS1,     0x10},
};

// Modem settings for 1.2kbps 2-FSK at 868 MHz addjusted to 25kHz RXBW and 4 Byte preamble(")
// Register settings imported directly from SmartRF Studio
static const registerSetting_t simpleLinkCC112xRfSettings1[] =
{
  {CC112X_IOCFG0            ,0x06}, 
  {CC112X_FS_DIG1           ,0x00},
  {CC112X_FS_DIG0           ,0x5F},
  {CC112X_FS_CAL0           ,0x0E},
  {CC112X_FS_DIVTWO         ,0x03},
  {CC112X_FS_DSM0           ,0x33},
  {CC112X_FS_DVC0           ,0x17},  
  {CC112X_FS_PFD            ,0x50},  
  {CC112X_FS_PRE            ,0x6E},
  {CC112X_FS_REG_DIV_CML    ,0x14},
  {CC112X_FS_SPARE          ,0xAC},
  {CC112X_XOSC5             ,0x0E},
  {CC112X_XOSC4             ,0xA0},
  {CC112X_XOSC3             ,0xC7},  
  {CC112X_XOSC1             ,0x03},
  {CC112X_ANALOG_SPARE      ,0x00},
  {CC112X_FIFO_CFG          ,0x80}, 
  {CC112X_DEV_ADDR          ,0x00},  
  {CC112X_SETTLING_CFG      ,0x03},
  {CC112X_FS_CFG            ,0x12},
  {CC112X_PKT_CFG2          ,0x00},
  {CC112X_PKT_CFG1          ,0x05}, 
  {CC112X_PKT_CFG0          ,0x20}, 
  {CC112X_PKT_LEN           ,0xFF},
  {CC112X_RFEND_CFG1        ,0x0F}, 
  {CC112X_RFEND_CFG0        ,0x00}, 
  {CC112X_FREQ2             ,0x6C},
  {CC112X_FREQ1             ,0x80}, 
  {CC112X_FREQ0             ,0x00},
  {CC112X_SYNC3             ,0x93},
  {CC112X_SYNC2             ,0x0B}, 
  {CC112X_SYNC1             ,0x51}, 
  {CC112X_SYNC0             ,0xDE}, 
  {CC112X_SYNC_CFG1         ,0x0B}, 
  {CC112X_SYNC_CFG0         ,0x17}, 
  {CC112X_DEVIATION_M       ,0x06},        
  {CC112X_MODCFG_DEV_E      ,0x03},       
  {CC112X_DCFILT_CFG        ,0x1C},                
  {CC112X_PREAMBLE_CFG1     ,0x18}, 
  {CC112X_PREAMBLE_CFG0     ,0x2A},                  
  {CC112X_FREQ_IF_CFG       ,0x40},      
  {CC112X_IQIC              ,0xC6},                 
  {CC112X_CHAN_BW           ,0x08},      
  {CC112X_MDMCFG1           ,0x46},                
  {CC112X_MDMCFG0           ,0x05},                 
  {CC112X_DRATE2            ,0x43},      
  {CC112X_DRATE1            ,0xA9},      
  {CC112X_DRATE0            ,0x2A},      
  {CC112X_AGC_REF           ,0x20}, 
  {CC112X_AGC_CS_THR        ,0x19}, 
  {CC112X_AGC_GAIN_ADJUST   ,0x00}, 
  {CC112X_AGC_CFG3          ,0x91}, 
  {CC112X_AGC_CFG2          ,0x20}, 
  {CC112X_AGC_CFG1          ,0xA9}, 
  {CC112X_AGC_CFG0          ,0xCF}, 
  {CC112X_PA_CFG2           ,0x7F}, 
  {CC112X_PA_CFG1           ,0x56}, 
  {CC112X_PA_CFG0           ,0x7C}, 
  {CC112X_IF_MIX_CFG        ,0x00}, 
  {CC112X_FREQOFF_CFG       ,0x22}, 
  {CC112X_TOC_CFG           ,0x0B},
  {CC112X_SOFT_TX_DATA_CFG  ,0x00}
};

static const registerSetting_t simpleLinkCC1125RfSettings[]= 
{
  {CC112X_IOCFG0,            0x06},
  {CC112X_SYNC_CFG1,         0x0B},
  {CC112X_DEVIATION_M,       0xA3},
  {CC112X_MODCFG_DEV_E,      0x02},
  {CC112X_DCFILT_CFG,        0x1C},
  {CC112X_FREQ_IF_CFG,       0x33},
  {CC112X_IQIC,              0xC6},
  {CC112X_CHAN_BW,           0x0A},
  {CC112X_MDMCFG0,           0x05},
  {CC112X_DRATE2,            0x3F},
  {CC112X_DRATE1,            0x75},
  {CC112X_DRATE0,            0x10},
  {CC112X_AGC_REF,           0x20},
  {CC112X_AGC_CS_THR,        0x19},
  {CC112X_AGC_CFG1,          0xA9},
  {CC112X_AGC_CFG0,          0xCF},
  {CC112X_FIFO_CFG,          0x00},
  {CC112X_SETTLING_CFG,      0x03},
  {CC112X_FS_CFG,            0x12},
  {CC112X_PKT_CFG0,          0x20},
  {CC112X_PA_CFG0,           0x7E},
  {CC112X_PKT_LEN,           0xFF},
  {CC112X_IF_MIX_CFG,        0x00},
  {CC112X_FREQOFF_CFG,       0x22},
  {CC112X_FREQ2,             0x56},
  {CC112X_FREQ1,             0xCC},
  {CC112X_FREQ0,             0xCC},
  {CC112X_IF_ADC0,           0x05},
  {CC112X_FS_DIG1,           0x00},
  {CC112X_FS_DIG0,           0x5F},
  {CC112X_FS_CAL0,           0x0E},
  {CC112X_FS_DIVTWO,         0x03},
  {CC112X_FS_DSM0,           0x33},
  {CC112X_FS_DVC0,           0x17},
  {CC112X_FS_PFD,            0x50},
  {CC112X_FS_PRE,            0x6E},
  {CC112X_FS_REG_DIV_CML,    0x14},
  {CC112X_FS_SPARE,          0xAC},
  {CC112X_XOSC5,             0x0E},
  {CC112X_XOSC3,             0xC7},
  {CC112X_XOSC1,             0x07},
};
/******************************************************************************
* LOCAL VARIABLES
*/
// RSSI offset dependent on data rate and frequency, see datasheet
static const uint8   cc112xRssiOffset = 96;

/******************************************************************************
* STATIC FUNCTIONS
*/



/******************************************************************************
 * @fn          simpleLinkCC112xRegConfig
 *
 * @brief       Writes RF register settings to radio
 *                
 * @param       none
 *
 * @return      none
 */
void simpleLinkCC112xRegConfig()
{
  uint8   writeByte;
  uint16 i;
  
  //Write register settings to radio
  for(i = 0; i < (sizeof  simpleLinkCC112xRfSettings/sizeof(registerSetting_t));i++)
  {
     writeByte =  simpleLinkCC112xRfSettings[i].data;
     cc112xSpiWriteReg( simpleLinkCC112xRfSettings[i].addr,&writeByte,1);
  }
  // manual calibration according to errata
  manualCalibration();

}
/******************************************************************************
 * @fn          simpleLinkCC1125RegConfig
 *
 * @brief       Writes RF register settings to radio
 *                
 * @param       none
 *
 * @return      none
 */
void simpleLinkCC1125RegConfig()
{
  uint8   writeByte;
  uint16 i;
  
  //Write register settings to radio
  for(i = 0; i < (sizeof  simpleLinkCC1125RfSettings/sizeof(registerSetting_t));i++)
  {
     writeByte =  simpleLinkCC1125RfSettings[i].data;
     cc112xSpiWriteReg( simpleLinkCC1125RfSettings[i].addr,&writeByte,1);
  } 
  // manual calibration according to errata
  manualCalibration();
}
/******************************************************************************
 * @fn          simpleLinkCC112xSendPacket
 *
 * @brief       function write array to tx fifo, strobes TX to send
 *              packet and waits to packet is sent.
 *                
 * input parameters
 *              
 * @param       *pData - pointer to data array that starts with length byte
 *                       and followed by payload.
 * output parameters
 *
 * @return      void
 */
void simpleLinkCC112xSendPacket(uint8 *pData)
{
  uint8   chipState = 0; 
  uint8 len = *pData;
   
  // be sure radio is in IDLE state
  trxSpiCmdStrobe(CC112X_SIDLE);
  while((trxSpiCmdStrobe(CC112X_SNOP)& 0xF0) != 0x00);
  // write array to fifio
  cc112xSpiWriteTxFifo(pData,(len+1));
  // strobe TX
  trxSpiCmdStrobe(CC112X_STX);
  // wait for packet to be sent    
  while((chipState & 0xF0) != 0x20)
  {
   	chipState = trxSpiCmdStrobe(CC112X_SNOP);
    // wait a bit before polling again (2.5 ms @ 16 MHz system clock)
   	__delay_cycles(40000);  
  }
}
/******************************************************************************
 * @fn          simpleLinkCC112xIdleRx
 *
 * @brief       put radio in RX and enables interrupt 
 *                
 *              
 * @param       none
 *
 *
 * @return      none
 */
void simpleLinkCC112xIdleRx()
{
   trxClearIntFlag();
   trxSpiCmdStrobe(CC112X_SIDLE);
   while((trxSpiCmdStrobe(CC112X_SNOP)& 0xF0) != 0x00);
   trxSpiCmdStrobe(CC112X_SRX);
   trxEnableInt();
}
/******************************************************************************
 * @fn          simpleLinkCC112xPwd
 *
 * @brief       Sends power down strobe to radio to make it sleep
 *                
 *              
 * @param       none
 *
 *
 * @return      noen
 */
void simpleLinkCC112xPwd()
{
   trxSpiCmdStrobe(CC112X_SIDLE);
   while((trxSpiCmdStrobe(CC112X_SNOP)& 0xF0) != 0x00);
   trxSpiCmdStrobe(CC112X_SPWD);

}

/***********************************************************************************
* @fn          radioRXISR
*
* @brief       ISR for packet handling in RX
*
* @param       none
*
* @return      none
*/
void simpleLinkCC112xRxISR(void)
{
  uint8 rxBytes, rssi2compl;
  uint8 rxArray[PKTLEN+3];
 
 
  slPacketSemaphore = PACKET_ISR_ACTION_REQUIRED;

  // Read NUM_RXBYTES for bytes in FIFO  
  cc112xSpiReadReg(CC112X_NUM_RXBYTES, &rxBytes, 1);
  
  if(rxBytes == PKTLEN+3) // PAYLOAD + LENGHT BYTE + 2 STATUS BYTES
  { 
    // Read RX FIFO
    cc112xSpiReadRxFifo(rxArray,rxBytes);
    // Check CRC ok and read packet if CRC ok. (CRC_OK: bit7 in second status byte)
    if(rxArray[rxBytes-1] | 0x80)
    {
      simpleLinkInfo.pktId = (((uint32)rxArray[2])<<24)|(((uint32)rxArray[3])<<16)|(((uint32)rxArray[4])<<8)|(rxArray[5]);
      //update counter
      simpleLinkInfo.pktOk++;
    }
  }
  else
  {
    //FIFO does not match what we expect, flush FIFO
    //put radio in IDLE
    trxSpiCmdStrobe(CC112X_SIDLE);
    trxSpiCmdStrobe(CC112X_SFRX);
    return;
  }
  
  // If this is the first packet we receive, the pktId may be different
  // than what we expect. Update pktIdExpected so we don't count false missed packet.
  if((simpleLinkInfo.pktId >1) && (simpleLinkInfo.pktOk == 1))
  {
    simpleLinkInfo.pktIdExpected = simpleLinkInfo.pktId;
  }
  
  // Check if we missed any packets
  if(simpleLinkInfo.pktId > simpleLinkInfo.pktIdExpected)
  {
    simpleLinkInfo.pktMiss++;
  }

  simpleLinkInfo.pktIdExpected = simpleLinkInfo.pktId+1;
  

  cc112xSpiReadReg(CC112X_RSSI1,&rssi2compl,1);


  simpleLinkInfo.rssi = (int16)((int8)(rssi2compl) - cc112xRssiOffset);
 

  //put radio in IDLE
  trxSpiCmdStrobe(CC112X_SIDLE);

}
/*******************************************************************************
* @fn          manualCalibration
*
* @brief       Perform manual calibration according to the errata note
* @param       none
*
* @return      none
*/
static void manualCalibration(void) {
  uint8 original_fs_cal2;
  uint8 calResults_for_vcdac_start_high[3];
  uint8 calResults_for_vcdac_start_mid[3];
  uint8 marcstate;
  uint8 writeByte;

  // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  writeByte = 0x00;
  cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

  // 2) Start with high VCDAC (original VCDAC_START + 2):
  cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
  writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
  cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

  // 3) Calibrate and wait for calibration to be done (radio back in IDLE state)
  trxSpiCmdStrobe(CC112X_SCAL);
  do {
    cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
  } while (marcstate != 0x41);

  // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with high VCDAC_START value
  cc112xSpiReadReg(CC112X_FS_VCO2, &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
  cc112xSpiReadReg(CC112X_FS_VCO4, &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
  cc112xSpiReadReg(CC112X_FS_CHP, &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

  // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  writeByte = 0x00;
  cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

  // 6) Continue with mid VCDAC (original VCDAC_START):
  writeByte = original_fs_cal2;
  cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

  // 7) Calibrate and wait for calibration to be done (radio back in IDLE state)
  trxSpiCmdStrobe(CC112X_SCAL);
  do {
    cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
  } while (marcstate != 0x41);

  // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with mid VCDAC_START value
  cc112xSpiReadReg(CC112X_FS_VCO2, &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
  cc112xSpiReadReg(CC112X_FS_VCO4, &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
  cc112xSpiReadReg(CC112X_FS_CHP, &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

  // 9) Write back highest FS_VCO2 and corresponding FS_VCO and FS_CHP result
  if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] > calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
    writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
    writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
    writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
    cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
  }
  else {
    writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
    writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
    writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
    cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
  }
}
/***********************************************************************************
  Copyright 2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
***********************************************************************************/
