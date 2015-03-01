/**************************************************************************************************
  Filename:       sbl_exec.h
  Revised:        $Date: 2012-08-08 18:03:58 -0700 (Wed, 08 Aug 2012) $
  Revision:       $Revision: 31165 $

  Description:

  Serial Bootloader Executive.


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
**************************************************************************************************/
#ifndef SBL_EXEC_H
#define SBL_EXEC_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_types.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#define SBL_APP_START_ADDR      0x0800
#define SBL_APP_VER_ADDR        0x0900
#define MAGIC_TYPE_ADDR         0x0A00
#define MAGIC_FEATURE_ADDR      0x1000

#define LOAD_MAGIC_TYPE         0x55aa
#define LOAD_MAGIC_FEATURE      0xbe01


#define UART2_SYNC_CMD          (0X55)
#define UART2_CMD_GSENSOR       (0X50)
#define UART2_CMD_TEMPERATURE 	(0X51)
#define UART2_CMD_HUMIDITY 		  (0X52)
#define UART2_CMD_PULSE 		    (0X53)
#define UART2_CMD_LOAD_START 	  (0X5C)
#define UART2_CMD_LOAD_DATA 	  (0X5D)
#define UART2_CMD_LOAD_END 	    (0X5E)
#define UART2_CMD_APP_VER 	    (0X5F)



#define SBL_ACK_OK              0x00
#define SBL_ACK_FAIL            0x01
#define SBL_ACK_ID_ERR          0x02
#define SBL_ACK_CRC_ERR         0x03

   
#define JUMP_TO_APP()     if(checkAppOK()) {  asm("LJMP 0x0800"); }
   
/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

   
/**************************************************************************************************
 * @fn          sblInit
 *
 * @brief       Boot Loader initialization.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if there is a valid RC image; FALSE otherwise.
 */
void sblInit(void);

/**************************************************************************************************
 * @fn          sblPoll
 *
 * @brief       Serial Boot poll & parse according to the RPC protocol.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the downloaded code has been enabled; FALSE otherwise.
 */
uint8 sblPoll(void);

uint8 checkAppOK(void);
uint16 FlashReadUint16(uint16 addr);
void sblTimoutProc(void);
void sblReponseMsg(uint8 cmd, uint8 ack);
void sblReponseAppVer(void);
unsigned char calc_crc8(unsigned char* pDat, unsigned char len);
void sblDataProc(void);

#endif
/**************************************************************************************************
*/
