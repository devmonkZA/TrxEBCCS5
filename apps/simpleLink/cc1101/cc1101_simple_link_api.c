/******************************************************************************
  Filename:       cc1101_simple_link_api.c
  
  Description: 
  
  Notes: 
  
******************************************************************************/


/*****************************************************************************
* INCLUDES
*/
#include "cc1101_spi.h"
#include "trx_rf_int.h"
#include "trx_rf_spi.h"
#include "simpleLink.h"
#include "cc1101_simple_link_api.h"


/*****************************************************************************
* CONSTANTS
*/


/* Radio configurations exported from SmartRF Studio*/ 
/*
 * Deviation = 5.157471 
 * Base frequency = 867.999939 
 * Carrier frequency = 867.999939 
 * Channel number = 0 
 * Modulated = true 
 * Modulation format = GFSK 
 * Manchester enable = false 
 * Sync word qualifier mode = 30/32 sync word bits detected 
 * Preamble count = 4 
 * Channel spacing = 199.951172 
 * Carrier frequency = 433.999969 
 * Data rate = 1.19948 
 * RX filter BW = 58.035714 
 * Data format = Normal mode 
 * Length config = Variable packet length mode. Packet length configured by the first byte after sync word 
 * CRC enable = true 
 * Packet length = 255 
 * Device address = 0 
 * Address config = No address check 
 * CRC autoflush = false 
 * PA ramping = false 
 * TX power = -10 
 */
static const registerSetting_t simpleLinkCC1101RfSettings[] = {
    {CC1101_IOCFG2    ,0x29},  /* IOCFG2        GDO2 Output Pin Configuration                  */
    {CC1101_IOCFG1    ,0x2E},  /* IOCFG1        GDO1 Output Pin Configuration                  */
    {CC1101_IOCFG0    ,0x06},  /* IOCFG0        GDO0 Output Pin Configuration                  */
    {CC1101_FIFOTHR   ,0x07},  /* FIFOTHR       RX FIFO and TX FIFO Thresholds                 */
    {CC1101_SYNC1     ,0xD3},  /* SYNC1         Sync Word, High Byte                           */
    {CC1101_SYNC0     ,0x91},  /* SYNC0         Sync Word, Low Byte                            */
    {CC1101_PKTLEN    ,0xFF},  /* PKTLEN        Packet Length                                  */
    {CC1101_PKTCTRL1  ,0x04},  /* PKTCTRL1      Packet Automation Control                      */
    {CC1101_PKTCTRL0  ,0x05},  /* PKTCTRL0      Packet Automation Control                      */
    {CC1101_ADDR      ,0x00},  /* ADDR          Device Address                                 */
    {CC1101_CHANNR    ,0x00},  /* CHANNR        Channel Number                                 */
    {CC1101_FSCTRL1   ,0x06},  /* FSCTRL1       Frequency Synthesizer Control                  */
    {CC1101_FSCTRL0   ,0x00},  /* FSCTRL0       Frequency Synthesizer Control                  */
    {CC1101_FREQ2     ,0x21},  /* FREQ2         Frequency Control Word, High Byte              */
    {CC1101_FREQ1     ,0x62},  /* FREQ1         Frequency Control Word, Middle Byte            */
    {CC1101_FREQ0     ,0x76},  /* FREQ0         Frequency Control Word, Low Byte               */
    {CC1101_MDMCFG4   ,0xF5},  /* MDMCFG4       Modem Configuration                            */
    {CC1101_MDMCFG3   ,0x83},  /* MDMCFG3       Modem Configuration                            */
    {CC1101_MDMCFG2   ,0x13},  /* MDMCFG2       Modem Configuration                            */
    {CC1101_MDMCFG1   ,0x22},  /* MDMCFG1       Modem Configuration                            */
    {CC1101_MDMCFG0   ,0xF8},  /* MDMCFG0       Modem Configuration                            */
    {CC1101_DEVIATN   ,0x15},  /* DEVIATN       Modem Deviation Setting                        */
    {CC1101_MCSM2     ,0x07},  /* MCSM2         Main Radio Control State Machine Configuration */
    {CC1101_MCSM1     ,0x30},  /* MCSM1         Main Radio Control State Machine Configuration */
    {CC1101_MCSM0     ,0x18},  /* MCSM0         Main Radio Control State Machine Configuration */
    {CC1101_FOCCFG    ,0x16},  /* FOCCFG        Frequency Offset Compensation Configuration    */
    {CC1101_BSCFG     ,0x6C},  /* BSCFG         Bit Synchronization Configuration              */
    {CC1101_AGCCTRL2  ,0x03},  /* AGCCTRL2      AGC Control                                    */
    {CC1101_AGCCTRL1  ,0x40},  /* AGCCTRL1      AGC Control                                    */
    {CC1101_AGCCTRL0  ,0x91},  /* AGCCTRL0      AGC Control                                    */
    {CC1101_WOREVT1   ,0x87},  /* WOREVT1       High Byte Event0 Timeout                       */
    {CC1101_WOREVT0   ,0x6B},  /* WOREVT0       Low Byte Event0 Timeout                        */
    {CC1101_WORCTRL   ,0xFB},  /* WORCTRL       Wake On Radio Control                          */
    {CC1101_FREND1    ,0x56},  /* FREND1        Front End RX Configuration                     */
    {CC1101_FREND0    ,0x10},  /* FREND0        Front End TX Configuration                     */
    {CC1101_FSCAL3    ,0xE9},  /* FSCAL3        Frequency Synthesizer Calibration              */
    {CC1101_FSCAL2    ,0x2A},  /* FSCAL2        Frequency Synthesizer Calibration              */
    {CC1101_FSCAL1    ,0x00},  /* FSCAL1        Frequency Synthesizer Calibration              */
    {CC1101_FSCAL0    ,0x1F},  /* FSCAL0        Frequency Synthesizer Calibration              */
    {CC1101_RCCTRL1   ,0x41},  /* RCCTRL1       RC Oscillator Configuration                    */
    {CC1101_RCCTRL0   ,0x00},  /* RCCTRL0       RC Oscillator Configuration                    */
    {CC1101_FSTEST    ,0x59},  /* FSTEST        Frequency Synthesizer Calibration Control      */
    {CC1101_PTEST     ,0x7F},  /* PTEST         Production Test                                */
    {CC1101_AGCTEST   ,0x3F},  /* AGCTEST       AGC Test                                       */
    {CC1101_TEST2     ,0x81},  /* TEST2         Various Test Settings                          */
    {CC1101_TEST1     ,0x35},  /* TEST1         Various Test Settings                          */
    {CC1101_TEST0     ,0x09}  /* TEST0         Various Test Settings                           */
}; 
/******************************************************************************
* LOCAL VARIABLES
*/
// RSSI offset dependent on data rate and frequency, see datasheet
static const uint8   cc1101RssiOffset = 74;

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
void simpleLinkCC1101RegConfig()
{
  uint8   writeByte;
  uint16 i;
  
  //Write register settings to radio
  for(i= 0; i < (sizeof  simpleLinkCC1101RfSettings/sizeof(registerSetting_t));i++)
  {
    writeByte =  simpleLinkCC1101RfSettings[i].data;
    cc1101SpiWriteReg( simpleLinkCC1101RfSettings[i].addr,&writeByte,1);
  }    
}
/******************************************************************************
 * @fn          simpleLinkCC1101SendPacket
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
void simpleLinkCC1101SendPacket(uint8 *pData)
{
   uint8   chipState = 0 ;
   uint8 len = *pData;
   
   trxSpiCmdStrobe(CC1101_SIDLE);
   while((trxSpiCmdStrobe(CC1101_SNOP)& 0xF0) != 0x00);
   cc1101SpiWriteTxFifo(pData,(len+1));
   trxSpiCmdStrobe(CC1101_STX);
       
   while((chipState & 0xF0) != 0x20)
   {
    	chipState = trxSpiCmdStrobe(CC1101_SNOP);
      // wait a bit before polling again (2.5 ms @ 16 MHz system clock)
     	__delay_cycles(40000);
    }
}
/******************************************************************************
 * @fn          simpleLinkCC1101IdleRx
 *
 * @brief       put radio in RX and enables interrupt 
 *                
 *              
 * @param       none
 *
 *
 * @return      none
 */
void simpleLinkCC1101IdleRx()
{
   trxClearIntFlag();
   trxSpiCmdStrobe(CC1101_SIDLE);
   while((trxSpiCmdStrobe(CC1101_SNOP)& 0xF0) != 0x00);
   trxSpiCmdStrobe(CC1101_SRX);
   trxEnableInt();
}
/******************************************************************************
 * @fn          simpleLinkCC1101Pwd
 *
 * @brief       Sends power down strobe to radio to make it sleep
 *                
 *              
 * @param       none
 *
 *
 * @return      noen
 */
void simpleLinkCC1101Pwd()
{
   trxSpiCmdStrobe(CC1101_SIDLE);
   while((trxSpiCmdStrobe(CC1101_SNOP)& 0xF0) != 0x00);
   trxSpiCmdStrobe(CC1101_SPWD);

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
void simpleLinkCC1101RxISR(void)
{
 uint8 rxBytes, rxBytesVerify, rssi2compl_1,rssi2compl;
 uint8 rxArray[PKTLEN+3];
 
 
  slPacketSemaphore = PACKET_ISR_ACTION_REQUIRED;

  
  cc1101SpiReadReg(CC1101_RXBYTES,&rxBytesVerify,1);
  do
  {
    rxBytes = rxBytesVerify;
    cc1101SpiReadReg(CC1101_RXBYTES,&rxBytesVerify,1);
  }
  while(rxBytes != rxBytesVerify);
  
  if(rxBytes==(PKTLEN+3))
  { 
    cc1101SpiReadRxFifo(rxArray,(rxBytes));
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
    trxSpiCmdStrobe(CC1101_SIDLE);
    trxSpiCmdStrobe(CC1101_SFRX);
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
  do
  {
    rssi2compl_1 = rssi2compl;
    cc1101SpiReadReg(CC1101_RSSI,&rssi2compl,1);
  }
  while(rssi2compl_1 != rssi2compl);

  if(rssi2compl >= 128)
  {
    simpleLinkInfo.rssi = (int16)(((int16)(rssi2compl-256)/2) - cc1101RssiOffset);
  }
  else
  {
    simpleLinkInfo.rssi = (int16)((rssi2compl/2) - cc1101RssiOffset);
  }
  // Restricting to 8 bit signed number range
  if(simpleLinkInfo.rssi < -128)
  {
    simpleLinkInfo.rssi = -128;
  }

  
  //put radio in IDLE
  trxSpiCmdStrobe(CC1101_SIDLE);

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
