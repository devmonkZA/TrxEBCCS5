/******************************************************************************
  Filename:       simpleLink.c
  
  Description: 
  
  Notes: 
  
******************************************************************************/

/*****************************************************************************
* INCLUDES
*/

#include  <msp430.h>
#include "chip_detect.h"
#include "hal_button_trxeb.h"
#include "cc1101_simple_link_api.h"
#include "cc112x_simple_link_api.h"
#include "trx_rf_int.h"
#include "simpleLink.h"
#include "hal_timer_32k.h"

/*****************************************************************************
 * TYPEDEFS
 */  
typedef void  (*VFPTR_U8PTR)(uint8 *a);
typedef void  (*VFPTR_ISR_FUNC_PTR)(ISR_FUNC_PTR a);
  
/******************************************************************************
* LOCAL VARIABLES
*/
static radioChipType_t pRadioChipType;
simpleLinkInfo_t simpleLinkInfo;
uint8          slPacketSemaphore;
static uint8   timerState;
// Array that holds data to be transmitted
// - txArray[0] = length byte
// - txArray[n] = payload[n],
//   where addr is the first part of the payload if applicable.
uint8 txArray[PKTLEN+1];
uint8 rxArray[PKTLEN+3]; //length byte + payload + 2 status bytes
/******************************************************************************
* STATIC FUNCTIONS
*/

VFPTR               simpleLinkRegConfig; 
VFPTR_U8PTR         simpleLinkSendPacket;
VFPTR               simpleLinkRxISR;
VFPTR               simpleLinkIdleRx;
VFPTR               simpleLinkRadioSleep;

static void   ft32kTimerISR(void);

/******************************************************************************
 * @fn          initSimpleLink
 *
 * @brief      Function assigns functions for the simpleLink test depending on
 *             what type of radio connected to the board. Currently supporting
 *             CC1101, CC1120, CC1121 and CC1125
 *                
 * @param        pDummy - pointer to pointer to void. no value used
 *
 * @return      0 -SUCCESS 1- FAILURE
 */
uint8 initSimpleLink(void)
{
  // detect chip type connected to EB
  trxDetectChipType(&pRadioChipType);
  
  // assign functions depending on radio detected
  if(pRadioChipType.deviceName == CHIP_TYPE_CC1101)
  {
    simpleLinkRegConfig  = simpleLinkCC1101RegConfig;
    simpleLinkSendPacket = simpleLinkCC1101SendPacket;
    simpleLinkRxISR      = simpleLinkCC1101RxISR;
    simpleLinkIdleRx     = simpleLinkCC1101IdleRx;
    simpleLinkRadioSleep = simpleLinkCC1101Pwd;
  }
  else if((pRadioChipType.deviceName == CHIP_TYPE_CC1120))
  {
    simpleLinkRegConfig  = simpleLinkCC112xRegConfig;
    simpleLinkSendPacket = simpleLinkCC112xSendPacket;
    simpleLinkRxISR      = simpleLinkCC112xRxISR;
    simpleLinkIdleRx     = simpleLinkCC112xIdleRx;
    simpleLinkRadioSleep = simpleLinkCC112xPwd;
  }
  else
  {
    // Make the message visible for max 2 seconds
    halTimer32kMcuSleepTicks(TIMER_32K_CLK_FREQ);
    halTimer32kMcuSleepTicks(TIMER_32K_CLK_FREQ);
    // clear potential button pushes
    halButtonsPushed();
    
    return 1;
  }
  return 0;
}

/******************************************************************************
 * @fn          simpleLinkMaster
 *
 * @brief       function called from the simpleLinkMainMenu.
 *              Configures radio and sends a packet every 0.2 sec 
 *              until aborted by user.
 *              
 *                
 * @param       pDummy - pointer to pointer to void. no value used
 *
 * @return     0 - SUCCESS
 */
uint8 simpleLinkMaster(void)
{
  uint32  pktCounter;
  
  //write radio registers
  simpleLinkRegConfig();
  
  // Connect ISR function to timer interrupt
  halTimer32kIntConnect(&ft32kTimerISR);
  
  // Set timer interrupt to every 0.2 sec.
  halTimer32kInit(TIMER_32K_CLK_FREQ/5);
  
  // Enable timer interrupt
  halTimer32kIntEnable();
   
  //initialize counter
  pktCounter = 0;
  
  while(1)
  {
    if(timerState == TIMER_ISR_ACTION_REQUIRED)
    {
      //increase counter
      pktCounter++;
      //fill txArray
      txArray[0] = PKTLEN;
      txArray[1] = 0x00; // broadcat address
      txArray[2] = pktCounter >> 24;
      txArray[3] = pktCounter >> 16; 
      txArray[4] = pktCounter >> 8;      
      txArray[5] = pktCounter;
           
      // send packet
      simpleLinkSendPacket(txArray);
           
      timerState = TIMER_ISR_IDLE;
    }
    // If left button pushed by user, abort test and exit to menu
    if(HAL_BUTTON_LEFT == halButtonsPushed())
    {
      // disable interrupt
      halTimer32kIntDisable();
      // put radio to sleep
      simpleLinkRadioSleep();
      break;
    }    
  }

  return 0; 
}
/******************************************************************************
 * @fn          simpleLinkSlave
 *
 * @brief       function called from the simpleLinkMainMenu.
 *              Configures radio and enter RX to receive packets 
 *              until aborted by user.
 *              
 *                
 * @param       pDummy - pointer to pointer to void. no value used
 *
 * @return      0 - SUCCESS
 */
uint8  simpleLinkSlave(void)
{
  // reset simplelink statistics
  simpleLinkInfo.pktId=0;
  simpleLinkInfo.pktOk=0;
  simpleLinkInfo.pktIdExpected=1; // first packet to be received
  simpleLinkInfo.pktMiss=0;
  simpleLinkInfo.rssi=0;
  
 
  //write radio registers
  simpleLinkRegConfig();
  
  // connect ISR function to port interrupt on PORT1 PIN7.(GDO0 on radio)
  trxIsrConnect(simpleLinkRxISR);
  
  // set radio in RX
  simpleLinkIdleRx();
     
    
 
  // enter while loop waiting for ISR for packet received
  while(1)
  {
      if(slPacketSemaphore == PACKET_ISR_ACTION_REQUIRED)
      {
        // packet received, reset semaphore flag and update display
        // reset semaphore
        slPacketSemaphore = PACKET_ISR_IDLE;
        // Update display 
       
        // set radio back in RX
        simpleLinkIdleRx();   
      }
      // If left button pushed by user, abort test and exit to menu
      if(HAL_BUTTON_LEFT == halButtonsPushed())
      {
        // disable interrupt
        trxDisableInt();
        // put radio to sleep
        simpleLinkRadioSleep();
        break;
      }
      // put MCU to sleep
      __low_power_mode_3();
    }
    return 0;
}
/***********************************************************************************
* @fn          ft32kTimerISR
*
* @brief       32KHz timer interrupt service routine.
*
* @param       none
*
* @return      none
*/
static void ft32kTimerISR(void)
{
    timerState = TIMER_ISR_ACTION_REQUIRED;
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
