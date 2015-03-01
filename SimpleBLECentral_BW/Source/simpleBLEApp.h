#ifndef _SIMPLE_BLE_APP_H_
#define _SIMPLE_BLE_APP_H_

//bleÖ÷Ó¦ÓÃ
#include "gatt.h"

#define BLE_DEVICE_TYPE_NULL            0x00
#define BLE_DEVICE_TYPE_LICENSE         0x01
#define BLE_DEVICE_TYPE_BLOOD_PRESSURE  0x02
#define BLE_DEVICE_TYPE_DUMP            0x03


extern void bleApp_SetConnectType(uint8 devType);
extern void bleApp_StartScan(void);
extern void bleApp_ScanEndHandle(void);
extern uint16 bleApp_GetUserDefUUID(void);
extern uint16 bleApp_GetServUuid(void);
extern void bleApp_UserDefUUIDHandle(void);
extern void bleApp_writeRespHandle( gattMsgEvent_t *pMsg );
extern void bleApp_readRespHandle( gattMsgEvent_t *pMsg );
extern void bleApp_notifyHandle( gattMsgEvent_t *pMsg );
extern void bleApp_DisConnectDevice(void);
extern void bleApp_DumpDataReq(uint8 *pDat, uint8 len);


#endif