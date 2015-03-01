/**************************************************************************************************
  Filename:       simpleBLECentral.c
  Revised:        $Date: 2011-06-20 11:57:59 -0700 (Mon, 20 Jun 2011) $
  Revision:       $Revision: 28 $

  Description:    This file contains the Simple BLE Central sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_flash.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "simpleBLECentral.h"

#include "npi.h"
#include "uart_app.h"
#include "sht20_drv.h"
#include "pulse_drv.h"
#include "gsensor_drv.h"
#include "simpleBLEApp.h"
#include "password.h"
#include "des_algo.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 1000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled, 7.5ms~4s
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      20

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      400

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           300

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

 
// 20ms timer count
#define BLE_PERIOD_20MS_CNT                   20


// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */




// Task ID for internal task/event processing
uint8 simpleBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
uint8 simpleBLEScanning = FALSE;

// RSSI polling state
static uint8 simpleBLERssi = FALSE;

// Connection handle of current connection 
uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;

// Application state
uint8 simpleBLEState = BLE_STATE_IDLE;

// Discovery state
uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16 simpleBLESvcStartHdl = 0;
static uint16 simpleBLESvcEndHdl = 0;

// Discovered characteristic handle
uint16 simpleBLECharHdl = 0;

// Value to write
//static uint8 simpleBLECharVal = 0;

// Value read/write toggle
//static bool simpleBLEDoWrite = FALSE;

// GATT read/write procedure state
bool simpleBLEProcedureInProgress = FALSE;


uint8 simpleBleCenterAddr[8] = {0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void simpleBLECentralStartDiscovery( void );
//static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
char *bdAddr2Str ( uint8 *pAddr );
static void simpleBLE_SystemReset(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  simpleBLECentralPasscodeCB,
  simpleBLECentralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLECentral_Init( uint8 task_id )
{
  //halIntState_t intState;
  
  simpleBLETaskId = task_id;

  NPI_InitTransport(uartRecvCB);
#if (defined UART_DEBUG) && (UART_DEBUG == TRUE)
  NPI_WriteString("Hello!\r\n");
  NPI_WriteString("u->up, d->down, r->right, l->left, c->center\r\n");
#endif
  
  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  //HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.
  
  gsensor_init();

  //HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
  
  //halIntState_t intState;
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );

    osal_start_timerEx( simpleBLETaskId, BLE_PERIOD_3S_EVT, 3000 );
    osal_start_timerEx( simpleBLETaskId, BLE_PERIOD_GSENSOR_EVT, BLE_PERIOD_20MS_CNT );

    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & START_DISCOVERY_EVT )
  {
    simpleBLECentralStartDiscovery( );
    
    return ( events ^ START_DISCOVERY_EVT );
  }

  if ( events & BLE_TIMER_RESTART_EVT )
  {
    simpleBLE_SystemReset(); //����
    return (events ^ BLE_TIMER_RESTART_EVT);
  }
  
  if ( events & BLE_PERIOD_3S_EVT )
  {
    // Restart timer
    osal_start_timerEx( simpleBLETaskId, BLE_PERIOD_3S_EVT, 3000 );

    //����������������ָʾ�Ƿ�ע��OK
    if(!g_pulseStartFlag) 
    {
      if(!LICENSE_IS_GOOD())
      {
        HalSetFlashPeriod(1000);
        HalLedSet( HAL_LED_1, HAL_LED_MODE_FLASH); 
      }
      else
      {
        HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF); 
      }
    }
    
    
    return (events ^ BLE_PERIOD_3S_EVT);
  }
  
  if ( events & BLE_PERIOD_PULSE_EVT )
  {
    // Restart timer
    if (( BLE_PLUSE_TIMER_5MS_CNT ) && (g_pulseStartFlag))
    {
      osal_start_timerEx( simpleBLETaskId, BLE_PERIOD_PULSE_EVT, BLE_PLUSE_TIMER_5MS_CNT );
    }

    //HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.
    
    pulse_update(); //������ʱ����

    //HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.

    return (events ^ BLE_PERIOD_PULSE_EVT);
  }
  
  if ( events & BLE_PERIOD_GSENSOR_EVT )
  {
    // Restart timer
    if ( BLE_PERIOD_20MS_CNT )
    {
      osal_start_timerEx( simpleBLETaskId, BLE_PERIOD_GSENSOR_EVT, BLE_PERIOD_20MS_CNT );
    }

    //HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.
    
    //��Ϊ�Ʋ�����ռʱ�䣬������������������ֹͣ�Ʋ���
    if(!g_pulseStartFlag) 
    {
      gsensor_update();  //TBD ���ܲ���
    }  

    //HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.

    return (events ^ BLE_PERIOD_GSENSOR_EVT);
  }
  
  if ( events & BLE_PERIOD_CHAR_HANDLE_EVT )
  {
    bleApp_UserDefUUIDHandle();
    return (events ^ BLE_PERIOD_CHAR_HANDLE_EVT);
  }
  
  if ( events & BLE_PERIOD_AUTO_SCAN_EVT )
  {
    //12s��û���ҵ��豸�����·���ɨ��
    osal_start_timerEx( MAIN_TASK_ID, BLE_PERIOD_AUTO_SCAN_EVT, 12000); 

    if(!LICENSE_IS_GOOD())
    {
      LCD_WRITE_STRING( "Start Scan", HAL_LCD_LINE_1 );
      bleApp_SetConnectType(BLE_DEVICE_TYPE_LICENSE);
    }
    else
    {
#if (defined DEBUG_DUMP) && (DEBUG_DUMP == TRUE)
      LCD_WRITE_STRING( "Scan DumpDev", HAL_LCD_LINE_1 );
      bleApp_SetConnectType(BLE_DEVICE_TYPE_DUMP);
#endif      
    }
    
    return (events ^ BLE_PERIOD_AUTO_SCAN_EVT);
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      simpleBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLECentral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
uint8 gStatus;
void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_UP )
  {
    // Start or stop discovery
    if ( simpleBLEState != BLE_STATE_CONNECTED )
    {
      if ( !simpleBLEScanning )
      {
        simpleBLEScanning = TRUE;
        simpleBLEScanRes = 0;
        
        LCD_WRITE_STRING( "Discovering...", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
        
        GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST );      
      }
      else
      {
        GAPCentralRole_CancelDiscovery();
      }
    }
#if 0  //����
    else if ( simpleBLEState == BLE_STATE_CONNECTED &&
              simpleBLECharHdl != 0 &&
              simpleBLEProcedureInProgress == FALSE )
    {
      uint8 status;
      
      // Do a read or write as long as no other read or write is in progress
      if ( simpleBLEDoWrite )
      {
        // Do a write
        attWriteReq_t req;
        
        req.handle = simpleBLECharHdl;
        req.len = 8;//1;
        req.value[0] = simpleBLECharVal;
        req.value[1] = 0x35; //debug
        req.sig = 0;
        req.cmd = 0;
        status = GATT_WriteCharValue( simpleBLEConnHandle, &req, simpleBLETaskId );         
      }
      else
      {
        // Do a read
        attReadReq_t req;
        
        req.handle = simpleBLECharHdl;
        status = GATT_ReadCharValue( simpleBLEConnHandle, &req, simpleBLETaskId );
      }
      
      if ( status == SUCCESS )
      {
        simpleBLEProcedureInProgress = TRUE;
        simpleBLEDoWrite = !simpleBLEDoWrite;
      }
    } 
#endif    
  }

  if ( keys & HAL_KEY_LEFT )
  {
    // Display discovery results
    if ( !simpleBLEScanning && simpleBLEScanRes > 0 )
    {
        // Increment index of current result (with wraparound)
        simpleBLEScanIdx++;
        if ( simpleBLEScanIdx >= simpleBLEScanRes )
        {
          simpleBLEScanIdx = 0;
        }
        
        LCD_WRITE_STRING_VALUE( "Device", simpleBLEScanIdx + 1,
                                10, HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( simpleBLEDevList[simpleBLEScanIdx].addr ),
                          HAL_LCD_LINE_2 );
    }
  }

  if ( keys & HAL_KEY_RIGHT )
  {
    // Connection update
    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
      GAPCentralRole_UpdateLink( simpleBLEConnHandle,
                                 DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                                 DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                                 DEFAULT_UPDATE_SLAVE_LATENCY,
                                 DEFAULT_UPDATE_CONN_TIMEOUT );
    }
  }
  
  if ( keys & HAL_KEY_CENTER )
  {
    uint8 addrType;
    uint8 *peerAddr;
    
    // Connect or disconnect
    if ( simpleBLEState == BLE_STATE_IDLE )
    {
      // if there is a scan result
      if ( simpleBLEScanRes > 0 )
      {
        // connect to current device in scan result
        peerAddr = simpleBLEDevList[simpleBLEScanIdx].addr;
        addrType = simpleBLEDevList[simpleBLEScanIdx].addrType;
      
        simpleBLEState = BLE_STATE_CONNECTING;
        
        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                      DEFAULT_LINK_WHITE_LIST,
                                      addrType, peerAddr );
  
        LCD_WRITE_STRING( "Connecting", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( peerAddr ), HAL_LCD_LINE_2 ); 
      }
    }
    else if ( simpleBLEState == BLE_STATE_CONNECTING ||
              simpleBLEState == BLE_STATE_CONNECTED )
    {
      // disconnect
      simpleBLEState = BLE_STATE_DISCONNECTING;

      gStatus = GAPCentralRole_TerminateLink( simpleBLEConnHandle );
      
      LCD_WRITE_STRING( "Disconnecting", HAL_LCD_LINE_1 ); 
    }
  }
  
  if ( keys & HAL_KEY_DOWN )
  {
    // Start or cancel RSSI polling
    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
      if ( !simpleBLERssi )
      {
        simpleBLERssi = TRUE;
        GAPCentralRole_StartRssi( simpleBLEConnHandle, DEFAULT_RSSI_PERIOD );
      }
      else
      {
        simpleBLERssi = FALSE;
        GAPCentralRole_CancelRssi( simpleBLEConnHandle );
        
        LCD_WRITE_STRING( "RSSI Cancelled", HAL_LCD_LINE_1 );
      }
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  if ( simpleBLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }
  
  //LCD_WRITE_STRING_VALUE( "GATT method: 0x", pMsg->method, 16, HAL_LCD_LINE_1 );
  
  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      
      LCD_WRITE_STRING_VALUE( "Read Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
      // After a successful read, display the read value
      uint8 valueRead = pMsg->msg.readRsp.value[0];

      LCD_WRITE_STRING_VALUE( "Read rsp:", valueRead, 10, HAL_LCD_LINE_1 );
    }
    
    simpleBLEProcedureInProgress = FALSE;
    
    bleApp_readRespHandle(pMsg);
    
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP)
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      
      LCD_WRITE_STRING_VALUE( "Write Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
      // After a succesful write, display the value that was written and increment value
      //LCD_WRITE_STRING_VALUE( "Write sent:", simpleBLECharVal++, 10, HAL_LCD_LINE_1 );      
    }
    
    simpleBLEProcedureInProgress = FALSE;    
    
    bleApp_writeRespHandle(pMsg);
    
    simpleBLEDiscState = BLE_DISC_STATE_CHAR;

  }
  else if( (pMsg->method == ATT_HANDLE_VALUE_IND) || (pMsg->method == ATT_HANDLE_VALUE_NOTI) )
  {  
    LCD_WRITE_STRING_VALUE( "Indication len:", pMsg->msg.handleValueInd.len, 10, HAL_LCD_LINE_1 );
    bleApp_notifyHandle( pMsg );
  }  
  else if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
  {
    simpleBLEGATTDiscoveryEvent( pMsg );
  }
  
}

/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
    LCD_WRITE_STRING_VALUE( "RSSI -dB:", (uint8) (-rssi), 10, HAL_LCD_LINE_1 );
}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  //uint16 servUuid;
  
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        LCD_WRITE_STRING( "BLE Central", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( pEvent->initDone.devAddr ),  HAL_LCD_LINE_2 );
        
        osal_memcpy(simpleBleCenterAddr, pEvent->initDone.devAddr, B_ADDR_LEN);
        simpleBLE_SystemCheckLicense(pEvent->initDone.devAddr);

#if (defined AUTO_CONNECT) && (AUTO_CONNECT == TRUE)
        osal_start_timerEx( MAIN_TASK_ID, BLE_PERIOD_AUTO_SCAN_EVT, 20); 

        //HalLedSet( HAL_LED_1, HAL_LED_MODE_FLASH); 
#endif
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        
        LCD_WRITE_STRING( "GAP_DEVICE_INFO_EVENT", HAL_LCD_LINE_2 );
        
        // if filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
#if 0          
          //TBD���˴���Ҫ�����豸����ָ��UUID
          servUuid = bleApp_GetServUuid(); //SIMPLEPROFILE_SERV_UUID

          if ( simpleBLEFindSvcUuid(  servUuid,
                                     pEvent->deviceInfo.pEvtData,
                                     pEvent->deviceInfo.dataLen ) )
#endif 
          {
            simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
          }
        }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        simpleBLEScanning = FALSE;

        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          simpleBLEScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }
        
        LCD_WRITE_STRING_VALUE( "Devices Found", simpleBLEScanRes,
                                10, HAL_LCD_LINE_1 );
        if ( simpleBLEScanRes > 0 )
        {
          LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
        }

        // initialize scan index to last device
        simpleBLEScanIdx = simpleBLEScanRes;

#if (defined AUTO_CONNECT) && (AUTO_CONNECT == TRUE)
        bleApp_ScanEndHandle();
        //HalLedSet( HAL_LED_1, HAL_LED_MODE_FLASH);         
#endif
        
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if ( pEvent->gap.hdr.status == SUCCESS )
        {          
          simpleBLEState = BLE_STATE_CONNECTED;
          simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;
          simpleBLEProcedureInProgress = TRUE;    

          // If service discovery not performed initiate service discovery
          if ( simpleBLECharHdl == 0 )
          {
            osal_start_timerEx( simpleBLETaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
          }
                    
          LCD_WRITE_STRING( "Connected", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING( bdAddr2Str( pEvent->linkCmpl.devAddr ), HAL_LCD_LINE_2 );   
                    
        }
        else
        {
          simpleBLEState = BLE_STATE_IDLE;
          simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
          simpleBLERssi = FALSE;
          simpleBLEDiscState = BLE_DISC_STATE_IDLE;
          
          LCD_WRITE_STRING( "Connect Failed", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING_VALUE( "Reason:", pEvent->gap.hdr.status, 10, HAL_LCD_LINE_2 );
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        simpleBLEState = BLE_STATE_IDLE;
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
        simpleBLERssi = FALSE;
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        simpleBLECharHdl = 0;
        simpleBLEProcedureInProgress = FALSE;
          
        LCD_WRITE_STRING( "Disconnected", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING_VALUE( "Reason:", pEvent->linkTerminate.reason,
                                10, HAL_LCD_LINE_2 );
        
#if (defined AUTO_CONNECT) && (AUTO_CONNECT == TRUE)
        bleApp_ScanEndHandle();
        //HalLedSet( HAL_LED_1, HAL_LED_MODE_FLASH);         
#endif
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        LCD_WRITE_STRING( "Param Update", HAL_LCD_LINE_1 );
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
    }
    else
    {
      LCD_WRITE_STRING_VALUE( "Pairing fail", status, 10, HAL_LCD_LINE_1 );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8   str[7];

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/*********************************************************************
 * @fn      simpleBLECentralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void simpleBLECentralStartDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                   HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
  
  // Initialize cached handles
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;

  simpleBLEDiscState = BLE_DISC_STATE_SVC;

  // Discovery simple BLE service
  GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle,
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 simpleBLETaskId );
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
  attReadByTypeReq_t req;
  uint16 uuid;
  bStatus_t status;
  
  if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )
  {
    
    LCD_WRITE_STRING( "BLE_DISC_STATE_SVC:",  HAL_LCD_LINE_1 );
   
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      simpleBLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
    }
    
    // If procedure complete
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
         ( pMsg->hdr.status == bleProcedureComplete || pMsg->hdr.status == 0)) ||         ( pMsg->method == ATT_ERROR_RSP ) )
    {
      if ( simpleBLESvcStartHdl != 0 )
      {
        // Discover characteristic
        simpleBLEDiscState = BLE_DISC_STATE_CHAR;
        
        req.startHandle = simpleBLESvcStartHdl;
        req.endHandle = simpleBLESvcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        
        uuid = bleApp_GetUserDefUUID();  //SIMPLEPROFILE_CHAR1_UUID
        if(uuid != 0)
        {
          req.type.uuid[0] = LO_UINT16(uuid);
          req.type.uuid[1] = HI_UINT16(uuid);
          GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );
        }
      }
    }
  }
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
         pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
      simpleBLECharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],
                                       pMsg->msg.readByTypeRsp.dataList[1] );
      
      LCD_WRITE_STRING_VALUE( "Simple Svc Found: 0x",simpleBLECharHdl, 16, HAL_LCD_LINE_1 );
      simpleBLEProcedureInProgress = FALSE;
      
      osal_start_timerEx( MAIN_TASK_ID, BLE_PERIOD_CHAR_HANDLE_EVT, 300); 
     
      simpleBLEDiscState = BLE_DISC_STATE_IDLE;
    }
    else
    {
      req.startHandle = 1;
      req.endHandle = 65535;
      req.type.len = ATT_BT_UUID_SIZE;
      
      uuid = bleApp_GetUserDefUUID();  //SIMPLEPROFILE_CHAR1_UUID
      req.type.uuid[0] = LO_UINT16(uuid);
      req.type.uuid[1] = HI_UINT16(uuid);
      status = GATT_DiscCharsByUUID( simpleBLEConnHandle, &req, simpleBLETaskId );
      LCD_WRITE_STRING_VALUE( "DiscCharsByUUID: 0x",status, 16, HAL_LCD_LINE_1 );
    }
  }    
}

#if 0
/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}
#endif

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

//license ���
void simpleBLE_SystemCheckLicense(uint8 ieeeAddr[B_ADDR_LEN])
{
  uint8 plainBlock[8];
	uint8 keyBlock[8] = { 0x12, 0x67, 0xab, 0x90, 0x27, 0x44, 0xf3, 0x5d };
	uint8 cipherBlock[8];
  uint8 flashLicense[8];  //������flash�е�license��Ϣ
  void* pKeys;

  if(LICENSE_IS_GOOD())
  {
    return;
  }
  
  //��ֹ�û������ƽ�
  keyBlock[0] += 1;
  keyBlock[1] += 1;
  
	osal_memset(plainBlock, 0, sizeof(plainBlock));
  osal_memcpy(plainBlock, ieeeAddr, B_ADDR_LEN);
 
	des(plainBlock, keyBlock, 0, cipherBlock);

  //compare with flash license info
  HalFlashRead(LICENSE_PGAE,  
               0, 
               flashLicense, 8);
  if(osal_memcmp( cipherBlock, flashLicense, 8))
  {
    g_licenseGoodFlag = TRUE;
    LCD_WRITE_STRING( "License OK!",  HAL_LCD_LINE_3);
  }
  else
  {
    //������ʷ�����㷨
    pKeys = osal_mem_alloc(16*48*sizeof(ElemType));  //ElemType subKeys[16][48];   
    DES_Encrypt(plainBlock, keyBlock, cipherBlock, pKeys);   
    osal_mem_free(pKeys);

    if(osal_memcmp( cipherBlock, flashLicense, 8))
    {
      g_licenseGoodFlag = TRUE;
      LCD_WRITE_STRING( "License ext OK!",  HAL_LCD_LINE_3);
    }
    else
    {
      g_licenseGoodFlag = FALSE;
    }
  }
  
}

#if (defined UART_DEBUG) && (UART_DEBUG == TRUE)
//����ģ�ⰴ��
void simpleBLE_UartSimlatorKey(uint8 ch)
{
  attReadByTypeReq_t req;
  uint16 uuid;
  bStatus_t status;
  attWriteReq_t writeReq;

  NPI_WriteString("\r\n");
  switch(ch)
  {
    case 'u':
      simpleBLECentral_HandleKeys(0, HAL_KEY_UP);
      break;
    case 'd':
      simpleBLECentral_HandleKeys(0, HAL_KEY_DOWN);
      break;
    case 'l':
      simpleBLECentral_HandleKeys(0, HAL_KEY_LEFT);
      break;
    case 'r':
      simpleBLECentral_HandleKeys(0, HAL_KEY_RIGHT);
      break;
    case 'c':
      simpleBLECentral_HandleKeys(0, HAL_KEY_CENTER);
      break;
    case 'b':
      bleApp_SetConnectType(BLE_DEVICE_TYPE_BLOOD_PRESSURE);
      break;
    case 'q':
      req.startHandle = 1;
      req.endHandle = 65535;
      req.type.len = ATT_BT_UUID_SIZE;
      
      uuid = SIMPLEPROFILE_CHAR2_UUID; //bleApp_GetUserDefUUID();  //SIMPLEPROFILE_CHAR1_UUID
      req.type.uuid[0] = LO_UINT16(uuid);
      req.type.uuid[1] = HI_UINT16(uuid);
      status = GATT_DiscCharsByUUID( simpleBLEConnHandle, &req, simpleBLETaskId );
      LCD_WRITE_STRING_VALUE( "DiscCharsByUUID: 0x",status, 16, HAL_LCD_LINE_1 );
 
      break;
  case 'n':
      writeReq.handle = 0x002f;
      writeReq.len = 2;
      writeReq.value[0] = LO_UINT16(GATT_CLIENT_CFG_NOTIFY);        //������ 0x01
      writeReq.value[1] = HI_UINT16(GATT_CLIENT_CFG_NOTIFY);        //������ 0x00
      writeReq.sig = 0;
      writeReq.cmd = 0;
      GATT_WriteCharValue( simpleBLEConnHandle, &writeReq, simpleBLETaskId );
    default:
      break;
  }   
}
#endif

//ϵͳ��λ����
static void simpleBLE_SystemReset(void)
{
   HAL_SYSTEM_RESET(); //����
}

/*********************************************************************
*********************************************************************/