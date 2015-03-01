/**************************************************************************************************
  Filename:       simpleBLECentral.h
  Revised:        $Date: 2011-03-03 15:46:41 -0800 (Thu, 03 Mar 2011) $
  Revision:       $Revision: 12 $

  Description:    This file contains the Simple BLE Central sample application
                  definitions and prototypes.

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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef SIMPLEBLECENTRAL_H
#define SIMPLEBLECENTRAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


// Simple BLE Central Task Events
#define START_DEVICE_EVT                              0x0001
#define START_DISCOVERY_EVT                           0x0002
#define BLE_TIMER_RESTART_EVT                         0x0004
#define BLE_PERIOD_3S_EVT                             0x0008
#define BLE_PERIOD_PULSE_EVT                          0x0010
#define BLE_PERIOD_GSENSOR_EVT                        0x0020
#define BLE_PERIOD_CHAR_HANDLE_EVT                    0x0040
#define BLE_PERIOD_AUTO_SCAN_EVT                      0x0080
   
   
  
#define BLE_PLUSE_TIMER_5MS_CNT               10
  

#define LICENSE_PGAE            (124)   //FlashStartAddr:250k  
#define LICENSE_START_ADDR      0x03E000  

#define MAIN_TASK_ID                                  simpleBLETaskId


#define LICENSE_IS_GOOD()         (TRUE == g_licenseGoodFlag)   
/*********************************************************************
 * MACROS
 */

// LCD macros
#if HAL_LCD == TRUE
#define LCD_WRITE_STRING(str, option)                       HalLcdWriteString( (str), (option))
#define LCD_WRITE_SCREEN(line1, line2)                      HalLcdWriteScreen( (line1), (line2) )
#define LCD_WRITE_STRING_VALUE(title, value, format, line)  HalLcdWriteStringValue( (title), (value), (format), (line) )
#else
#define LCD_WRITE_STRING(str, option)                     
#define LCD_WRITE_SCREEN(line1, line2)                    
#define LCD_WRITE_STRING_VALUE(title, value, format, line)
#endif

  
// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

  
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void SimpleBLECentral_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events );

void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys );

extern void simpleBLE_UartSimlatorKey(uint8 ch);

extern void simpleBLE_SystemCheckLicense(uint8 ieeeAddr[B_ADDR_LEN]);

char *bdAddr2Str( uint8 *pAddr );

/*********************************************************************
 * Global varible
 */

extern uint8 simpleBLETaskId;
extern uint8 g_licenseGoodFlag;
extern uint8 simpleBleCenterAddr[8];
extern uint16 simpleBLECharHdl;
extern uint16 simpleBLEConnHandle;
extern uint8 simpleBLEState;
extern bool simpleBLEProcedureInProgress;
extern uint8 simpleBLEScanning;

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLECENTRAL_H */
