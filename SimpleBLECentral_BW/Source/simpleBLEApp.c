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
#include "simpleBLEApp.h"
#include "uart_app.h"

extern uint16 g_PulseCnt;



//
uint8 g_bleDeviceType = BLE_DEVICE_TYPE_NULL;
uint8 g_bloodNotifyFlag = FALSE;

/////////////////////////////////////////////////////////////////////

//设置ble连接设备类型，如果类型为空，表示停止连接
void bleApp_SetConnectType(uint8 devType)
{
  if(devType != g_bleDeviceType)
  {
    //停止当前连接
    bleApp_DisConnectDevice();
  }

  g_bleDeviceType = devType;
  
  //初始化变量
  g_bloodNotifyFlag = FALSE;
  
  if(devType != BLE_DEVICE_TYPE_NULL)
  {
    bleApp_StartScan();
  }
}

//启动扫描，在接收到init_event执行
void bleApp_StartScan(void)
{
  if(g_bleDeviceType != BLE_DEVICE_TYPE_NULL)
  {
      simpleBLECentral_HandleKeys(0, HAL_KEY_UP); //scan
  }
}

//扫描结束后的处理
void bleApp_ScanEndHandle(void)
{
  if(g_bleDeviceType != BLE_DEVICE_TYPE_NULL)
  {
    simpleBLECentral_HandleKeys(0, HAL_KEY_LEFT); //选择第一个设备
    simpleBLECentral_HandleKeys(0, HAL_KEY_CENTER); //建立连接
  }
}


//获取指定类型对应的Service的UUID
uint16 bleApp_GetServUuid(void)
{
  uint16 uuid = SIMPLEPROFILE_SERV_UUID;
  
  switch(g_bleDeviceType)
  {
    case BLE_DEVICE_TYPE_LICENSE:
        uuid = SIMPLEPROFILE_SERV_UUID;
        break;
      
    case BLE_DEVICE_TYPE_BLOOD_PRESSURE:
        uuid = SIMPLEPROFILE_SERV_UUID;
        break;
        
    case BLE_DEVICE_TYPE_DUMP:
        uuid = SIMPLEPROFILE_SERV_UUID;
        break;
      
    default:
        break;
  }
  
  return uuid;
}

//建立连接后获取指定的CHAR的UUID
uint16 bleApp_GetUserDefUUID(void)
{
  uint16 uuid = 0;
  
  switch(g_bleDeviceType)
  {
    case BLE_DEVICE_TYPE_LICENSE:
        uuid = SIMPLEPROFILE_CHAR6_UUID;
        break;
      
    case BLE_DEVICE_TYPE_BLOOD_PRESSURE:
        if(g_bloodNotifyFlag == FALSE)
        {
          uuid = SIMPLEPROFILE_CHAR1_UUID;
        }
        else
        {
          uuid = SIMPLEPROFILE_CHAR2_UUID;
        }
        break;
        
    case BLE_DEVICE_TYPE_DUMP:
        uuid = SIMPLEPROFILE_CHAR3_UUID;
        break;
      
    default:
        break;
  }
  
  return uuid;
}

//获取UUID后的处理
void bleApp_UserDefUUIDHandle(void)
{
  uint8 status = FAILURE;
  attWriteReq_t wrReq;

  if( simpleBLEState == BLE_STATE_CONNECTED &&
      simpleBLECharHdl != 0 &&
      simpleBLEProcedureInProgress == FALSE )     
  {  
    switch(g_bleDeviceType)
    {
      case BLE_DEVICE_TYPE_LICENSE:
        wrReq.handle = simpleBLECharHdl+1;
        wrReq.len = 8;
        osal_memcpy(&wrReq.value[0], simpleBleCenterAddr, 8);
        wrReq.sig = 0;
        wrReq.cmd = 0;
        status = GATT_WriteCharValue( simpleBLEConnHandle, &wrReq, simpleBLETaskId );         
        break;
      
      case BLE_DEVICE_TYPE_BLOOD_PRESSURE:
        if(g_bloodNotifyFlag == FALSE)
        {
            wrReq.handle = simpleBLECharHdl+2;
            wrReq.len = 2;
            wrReq.value[0] = LO_UINT16(GATT_CLIENT_CFG_NOTIFY);        //这里是 0x01
            wrReq.value[1] = HI_UINT16(GATT_CLIENT_CFG_NOTIFY);        //这里是 0x00
            wrReq.sig = 0;
            wrReq.cmd = 0;
            status = GATT_WriteCharValue( simpleBLEConnHandle, &wrReq, simpleBLETaskId );
            LCD_WRITE_STRING( "Write bloodNotify", HAL_LCD_LINE_1 );
        }
        else
        {
          wrReq.handle = simpleBLECharHdl+1;  //
          wrReq.len = 6;
          wrReq.value[0] = 0xFD;
          wrReq.value[1] = 0xFD;
          wrReq.value[2] = 0xFA;
          wrReq.value[3] = 0x05;
          wrReq.value[4] = 0x0D;
          wrReq.value[5] = 0x0A;
          wrReq.sig = 0;
          wrReq.cmd = 0;
          status = GATT_WriteCharValue( simpleBLEConnHandle, &wrReq, simpleBLETaskId );         
          LCD_WRITE_STRING( "Write bloodStart", HAL_LCD_LINE_1 );
          
        }
        
        break;
      
      default:
        return;
    }
  }
  
  LCD_WRITE_STRING_VALUE( "WriteCharValue: 0x",status, 16, HAL_LCD_LINE_1 );
  if ( status == SUCCESS )
  {
    simpleBLEProcedureInProgress = TRUE;
  }
}

//写回应处理
void bleApp_writeRespHandle( gattMsgEvent_t *pMsg )
{
  uint8 status = FAILURE;
  attReadReq_t rdReq;
  uint16 uuid;
  attReadByTypeReq_t req;
  //attWriteReq_t wrReq;

  switch(g_bleDeviceType)
  {
    case BLE_DEVICE_TYPE_LICENSE:
        if ( pMsg->method != ATT_ERROR_RSP )
        {          
          rdReq.handle = simpleBLECharHdl+1;
          status = GATT_ReadCharValue( simpleBLEConnHandle, &rdReq, simpleBLETaskId );
        }
        break;
      
    case BLE_DEVICE_TYPE_BLOOD_PRESSURE:
        LCD_WRITE_STRING("BloodPress writeResp\r\n", HAL_LCD_LINE_2);
        if(g_bloodNotifyFlag == FALSE)
        {
          g_bloodNotifyFlag = TRUE;

          req.startHandle = 1;
          req.endHandle = 65535;
          req.type.len = ATT_BT_UUID_SIZE;
          
          uuid = SIMPLEPROFILE_CHAR2_UUID; //bleApp_GetUserDefUUID();  //SIMPLEPROFILE_CHAR1_UUID
          req.type.uuid[0] = LO_UINT16(uuid);
          req.type.uuid[1] = HI_UINT16(uuid);
          status = GATT_DiscCharsByUUID( simpleBLEConnHandle, &req, simpleBLETaskId );

          LCD_WRITE_STRING_VALUE( "DiscCharsByUUID: 0x",status, 16, HAL_LCD_LINE_1 );
          
        }
        
        break;
      
    default:
        return;
  }

  LCD_WRITE_STRING_VALUE( "bleApp_writeRespHandle: 0x",status, 16, HAL_LCD_LINE_1 );

  if (status == SUCCESS )
  {
    simpleBLEProcedureInProgress = TRUE;
  }  
}

//读回应处理
void bleApp_readRespHandle( gattMsgEvent_t *pMsg )
{
  uint8 nullData[8] = {0};
  uint8* pValue;
  
  switch(g_bleDeviceType)
  {
    case BLE_DEVICE_TYPE_LICENSE:
        pValue = &pMsg->msg.readRsp.value[0];
        
        LCD_WRITE_STRING_VALUE("readLen:", pMsg->msg.readRsp.len, 10, HAL_LCD_LINE_2);

        if( (pMsg->method != ATT_ERROR_RSP)
          && (osal_memcmp(pValue, nullData, 8) != TRUE) )
        {
          HalFlashErase(LICENSE_PGAE);
          //写flash，请注意地址和长度单位为uint32，即4字节
          HalFlashWrite(LICENSE_START_ADDR / HAL_FLASH_WORD_SIZE, 
                        pValue, 8/HAL_FLASH_WORD_SIZE);
          
          simpleBLE_SystemCheckLicense(simpleBleCenterAddr);
          
          bleApp_DisConnectDevice();
        }

        break;
      
    case BLE_DEVICE_TYPE_BLOOD_PRESSURE:
      
        LCD_WRITE_STRING_VALUE("readLen:", pMsg->msg.handleValueNoti.len, 10, HAL_LCD_LINE_2);
        LCD_WRITE_STRING_VALUE("method:", pMsg->method, 10, HAL_LCD_LINE_3);

      break;
      
    default:
        break;
  }

}
  
//通知消息处理
void bleApp_notifyHandle( gattMsgEvent_t *pMsg )
{
    if((pMsg->msg.handleValueInd.value[0]==0xFD) && (pMsg->msg.handleValueInd.value[1]==0xFD))
    {
      if((pMsg->msg.handleValueInd.len == 8) && (pMsg->msg.handleValueInd.value[2]==0xFC))
      {
          LCD_WRITE_STRING("Blood Good\r\n", HAL_LCD_LINE_2);
          LCD_WRITE_STRING_VALUE("SYS:", pMsg->msg.handleValueInd.value[3], 10, HAL_LCD_LINE_3);
          LCD_WRITE_STRING_VALUE("DIA:", pMsg->msg.handleValueInd.value[4], 10, HAL_LCD_LINE_3);
          LCD_WRITE_STRING_VALUE("PUL:", pMsg->msg.handleValueInd.value[5], 10, HAL_LCD_LINE_3);
          
          uart_send_blood_press(pMsg->msg.handleValueInd.value[3], 
                                pMsg->msg.handleValueInd.value[4], 
                                pMsg->msg.handleValueInd.value[5]);
            
          bleApp_DisConnectDevice();
      }
      else if((pMsg->msg.handleValueInd.len == 6) && (pMsg->msg.handleValueInd.value[2]==0xFD))
      {
          uart_send_blood_press(0, 0, pMsg->msg.handleValueInd.value[3]); //错误码
          bleApp_DisConnectDevice();
      }
    }
}


//断开设备连接
void bleApp_DisConnectDevice(void)
{
    if ( simpleBLEScanning )
    {
        GAPCentralRole_CancelDiscovery();
    }
    
    if ( simpleBLEState != BLE_STATE_IDLE )
    {
        simpleBLECentral_HandleKeys(0, HAL_KEY_CENTER); //disconnect
    }
    
    g_bleDeviceType = BLE_DEVICE_TYPE_NULL;
}

//发送Dump 数据请求命令，专为Dump类型设计的
void bleApp_DumpDataReq(uint8 *pDat, uint8 len)
{
  uint8 status = FAILURE;
  attWriteReq_t wrReq;
  static uint16 sendCnt = 0;
  
  sendCnt++;

  if( simpleBLEState == BLE_STATE_CONNECTED &&
      simpleBLECharHdl != 0 &&
      simpleBLEProcedureInProgress == FALSE )     
  {  
        wrReq.handle = simpleBLECharHdl+1;
        wrReq.len = 6;
        osal_memcpy(&wrReq.value[0], pDat, len);
        //wrReq.value[0] = LO_UINT16(sendCnt);
        //wrReq.value[2] = LO_UINT16(g_PulseCnt);
        //wrReq.value[3] = HI_UINT16(g_PulseCnt);
        
        wrReq.sig = 0;
        wrReq.cmd = 0;
        status = GATT_WriteCharValue( simpleBLEConnHandle, &wrReq, simpleBLETaskId ); 
        
        LCD_WRITE_STRING_VALUE( "Dump: 0x", sendCnt, 16, HAL_LCD_LINE_1 );
        if ( status == SUCCESS )
        {
          simpleBLEProcedureInProgress = TRUE;
        }
  }
    
}
extern uint16 g_pluseCnt, g_PulseTime,g_pulseValue;
extern uint16 g_prec;
void bleApp_DumpPulseDataReq(uint8 *pDat, uint8 len)
{
  uint8 status = FAILURE;
  attWriteReq_t wrReq;
  static uint16 sendCnt = 0;
  
  sendCnt++;

  if( simpleBLEState == BLE_STATE_CONNECTED &&
      (sendCnt&0x3) == 0 &&
      simpleBLEProcedureInProgress == FALSE )     
  {  
        wrReq.handle = simpleBLECharHdl+1;
        wrReq.len = 6;
        osal_memcpy(&wrReq.value[0], pDat, len);
        //wrReq.value[0] = LO_UINT16(sendCnt);
        //wrReq.value[2] = LO_UINT16(sendCnt);
        //wrReq.value[3] = HI_UINT16(sendCnt);
        wrReq.value[2] = LO_UINT16(g_PulseTime);
        wrReq.value[3] = HI_UINT16(g_PulseTime);
        wrReq.value[4] = LO_UINT16(g_pulseValue);
        wrReq.value[5] = HI_UINT16(g_pulseValue);  

        wrReq.sig = 0;
        wrReq.cmd = 0;
        status = GATT_WriteCharValue( simpleBLEConnHandle, &wrReq, simpleBLETaskId ); 
        
        LCD_WRITE_STRING_VALUE( "Dump: 0x", sendCnt, 16, HAL_LCD_LINE_1 );
        if ( status == SUCCESS )
        {
          simpleBLEProcedureInProgress = TRUE;
        }
  }
    
}


