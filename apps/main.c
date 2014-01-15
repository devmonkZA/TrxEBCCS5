/******************************************************************************
  Filename:   main.c

  Description: This file implements the startup of the board and and a menu
               menu driver.

******************************************************************************/

/******************************************************************************
 * INCLUDES
 */

#include  <msp430.h>
#include "hal_board.h"
#include "hal_mcu.h"
#include "hal_led_trxeb.h"
#include "hal_button_trxeb.h"
#include "hal_timer_32k.h"
#include "trx_rf_spi.h"
#include "simpleLink.h"

/******************************************************************************
* GLOBAL VARIABLES
*/
extern uint8  mclkFrequency; // Defined in hal_board.c

char version[] = "v3.1";    // Version number
/******************************************************************************
* LOCAL FUNCTIONS
*/


/******************************************************************************
 * @fn          main
 *
 * @brief       Main handles all applications attached to the menu system
 *
 * input parameters
 *
 * output parameters
 *
 *@return
 */
void main( void )
{
  /* Stop watchdog timer to prevent time out reset */
  WDTCTL = WDTPW + WDTHOLD;

  /* Settingcapacitor values for XT1, 32768 Hz */
  halMcuStartXT1();

  /* Clocks:
   * mclk  = mclkFrequency
   * smclk = mclkFrequency
   * aclk  = 32768 Hz
   */
  mclkFrequency = HAL_MCU_SYSCLK_16MHZ;
  halMcuSetSystemClock(mclkFrequency);

  /* Care must be taken when handling power modes
   * - Peripheral units can request clocks and have them granted even if
   *   the system is in a power mode. Peripheral clock request is enabled
   *   as default.
   * - Per test only needs ACLK to be enabled to timers
   *   during power mode operation
   */
  halMcuDisablePeripheralClockRequest((MCLKREQEN+SMCLKREQEN));


  /* SPI flash uses same SPI interface as LCD -- we'll disable the SPI flash */
 P8SEL &= BIT6; /*ioflash_csn = gp.      */
 P8DIR |= BIT6; /*tpflash_csn = ouut.    */
 P8OUT |= BIT6; /*flash_csn = 1.         */

  /* Init leds and turn them on */
  halLedInit();

  /* Init Buttons */
  halButtonsInit();
  halButtonsInterruptEnable();

  /* Instantiate tranceiver RF spi interface to SCLK = 1 MHz */
  trxRfSpiInterfaceInit(0x10);

  halLedSet(LED_1);

  initSimpleLink();

  halLedSet(LED_2);

  simpleLinkMaster();

  while(1)
  {
	  halLedSet(LED_3);
	  halTimer32kMcuSleepTicks(3276);
	  halLedClear(LED_3);
	  halTimer32kMcuSleepTicks(3276);
  }
}

/******************************************************************************
  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

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
*******************************************************************************/
