
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "hal_flash.h"
#include "npi.h"
#include "uart_app.h"
#include "i2c.h"
#include "simpleBLECentral.h"
#include "sht20_drv.h"
#include "pulse_drv.h"
#include "gsensor_drv.h"
#include "simpleBLEApp.h"




#define SBL_APP_VER_ADDR        0x0900



#define UART_RESP_HEAD(buf, cmd) \
              buf[0] = UART2_SYNC_CMD; \
              buf[1] = UART2_SYNC_CMD; \
              buf[2] = (cmd)
     

/////////////// static function ///////////////////////
static uint16 FlashReadUint16(uint16 addr);

static uint8 g_uartCmdState = UART2_NULL_CMD;


//串口接收回调函数
void uartRecvCB(uint8 port, uint8 event)
{
    uint8 dat;
    
#if (defined UART_DEBUG) && (UART_DEBUG == TRUE)
    while(NPI_ReadByte(&dat))
    {
        NPI_WriteByte(dat);
        simpleBLE_UartSimlatorKey(dat);
    }
#else
    uint8 buf[5];
    uint16 ver;
    static  uint8 rxLen = 0;
    static  uint8 rxBuf[50];
    uint16 usDat;
    
    while(NPI_ReadByte(&dat))
    {
      
      switch(g_uartCmdState)
      {
        case UART2_NULL_CMD:
            switch(dat)
            {
              case UART2_CMD_APP_VER:
                ver = FlashReadUint16(SBL_APP_VER_ADDR);
                UART_RESP_HEAD(buf, UART2_CMD_APP_VER);
                buf[3] = HI_UINT16(ver);
                buf[4] = LO_UINT16(ver);
                (void)HalUARTWrite(0, buf, 5);
                break;

              case  UART2_CMD_LOAD_START:
                UART_RESP_HEAD(buf, UART2_CMD_LOAD_START);
                buf[3] = SBL_ACK_OK;
                (void)HalUARTWrite(0, buf, 4);
                osal_start_timerEx( MAIN_TASK_ID, BLE_TIMER_RESTART_EVT, 50 ); //delay reset system
                break;

            case  UART2_CMD_LICENSE_INFO:
                rxLen = 0;
                g_uartCmdState = UART2_CMD_LICENSE_INFO;
                break;
                
            case UART2_CMD_TEMPERATURE:
                usDat = sht20_read_temp();
                UART_RESP_HEAD(buf, UART2_CMD_TEMPERATURE);
                buf[3] = HI_UINT16(usDat);
                buf[4] = LO_UINT16(usDat);
                (void)HalUARTWrite(0, buf, 5);
                break;
                
            case UART2_CMD_HUMIDITY:
                usDat = sht20_read_humidity();
                UART_RESP_HEAD(buf, UART2_CMD_HUMIDITY);
                buf[3] = HI_UINT16(usDat);
                buf[4] = LO_UINT16(usDat);
                (void)HalUARTWrite(0, buf, 5);
                break;
                
             case UART2_CMD_PULSE:
                pulse_startRead();
                break;
                
             case UART2_CMD_GSENSOR:
                gsensor_read();
                break;
                
             case UART2_CMD_END_PLUSE:
                pulse_endRead();
                break;
                
             case UART2_CMD_BLOOD_PRESS:
                bleApp_SetConnectType(BLE_DEVICE_TYPE_BLOOD_PRESSURE);
                break;

            case UART2_CMD_CLEAR_GSENSOR:
                step_init();
                break;
                
            case UART2_CMD_END_BLOOD_PRESS:
                bleApp_SetConnectType(BLE_DEVICE_TYPE_NULL);
                break;
                
                
              default:
                break;
            }
            break;
         
        case  UART2_CMD_LICENSE_INFO:
          rxBuf[rxLen] = dat;
          rxLen++;
          if(rxLen>=10)
          {
            if((rxBuf[0] == 0xA5) && (rxBuf[1] == 0xA6))
            {
              HalFlashErase(LICENSE_PGAE);
              //写flash，请注意地址和长度单位为uint32，即4字节
              HalFlashWrite(LICENSE_START_ADDR / HAL_FLASH_WORD_SIZE, &rxBuf[2], 8/HAL_FLASH_WORD_SIZE);
              
              UART_RESP_HEAD(buf, UART2_CMD_LICENSE_INFO);
              buf[3] = SBL_ACK_OK;
              (void)HalUARTWrite(0, buf, 4);
              osal_start_timerEx( MAIN_TASK_ID, BLE_TIMER_RESTART_EVT, 50 ); //delay reset system
            }
            
            g_uartCmdState = UART2_NULL_CMD;
            rxLen = 0;
          }
          break;
        
        default:
          break;
      }
      
    }
#endif     
}


//从flash读取uint16数据
static uint16 FlashReadUint16(uint16 addr)
{
  uint8 buf[20];
  
  HalFlashRead(addr / HAL_FLASH_PAGE_SIZE,
              addr % HAL_FLASH_PAGE_SIZE, 
              buf, 2);
  
  return BUILD_UINT16(buf[0], buf[1]);
}

//发送脉搏
void uart_send_pulse(uint8 pulse)
{
    uint8 buf[20];

    UART_RESP_HEAD(buf, UART2_CMD_PULSE);
    buf[3] = pulse;
    (void)HalUARTWrite(0, buf, 4);
}

//发送血压
void uart_send_blood_press(uint8 sys, uint8 dia, uint8 pul)
{
    uint8 buf[20];

    UART_RESP_HEAD(buf, UART2_CMD_BLOOD_PRESS);
    buf[3] = sys;
    buf[4] = dia;
    buf[5] = pul;
    (void)HalUARTWrite(0, buf, 6);
}




