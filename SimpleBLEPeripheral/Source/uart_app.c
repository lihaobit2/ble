
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "hal_flash.h"
#include "npi.h"
#include "uart_app.h"
//#include "i2c.h"
#include "simpleBLEPeripheral.h"




#define UART2_NULL_CMD            (0X00)
#define UART2_SYNC_CMD            (0X55)
#define UART2_CMD_GSENSOR         (0X50)
#define UART2_CMD_TEMPERATURE 	  (0X51)
#define UART2_CMD_HUMIDITY 		    (0X52)
#define UART2_CMD_PULSE 		        (0X53)
#define UART2_CMD_LOAD_START 	    (0X5C)
#define UART2_CMD_LOAD_DATA 	      (0X5D)
#define UART2_CMD_LOAD_END 		    (0X5E)
#define UART2_CMD_APP_VER 	        (0X5F)
#define UART2_CMD_LICENSE_INFO    (0X60)



#define SBL_ACK_OK              0x00
#define SBL_ACK_FAIL            0x01
#define SBL_ACK_ID_ERR          0x02
#define SBL_ACK_CRC_ERR         0x03

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
    }
#else
    static uint8 rxLen = 0;
    static uint8 rxBuf[50];
    
    uint8 buf[5];
    uint16 ver;
    
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
                osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_TIMER_50MS_EVT, 50 ); //delay reset system
                break;

            case  UART2_CMD_LICENSE_INFO:
                rxLen = 0;
                g_uartCmdState = UART2_CMD_LICENSE_INFO;
                break;
              default:
                break;
            }
            break;
         
        case  UART2_CMD_LICENSE_INFO:
          rxBuf[rxLen] = dat;
          rxLen++;
          if(rxLen>=8)
          {
            HalFlashErase(LICENSE_PGAE);
            //写flash，请注意地址和长度单位为uint32，即4字节
            HalFlashWrite(LICENSE_START_ADDR / HAL_FLASH_WORD_SIZE, &rxBuf[0], 8/HAL_FLASH_WORD_SIZE);
            
            UART_RESP_HEAD(buf, UART2_CMD_LICENSE_INFO);
            buf[3] = SBL_ACK_OK;
            (void)HalUARTWrite(0, buf, 4);

            g_uartCmdState = UART2_NULL_CMD;
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
  uint8 buf[2];
  
  HalFlashRead(addr / HAL_FLASH_PAGE_SIZE,
              addr % HAL_FLASH_PAGE_SIZE, 
              buf, 2);
  
  return BUILD_UINT16(buf[0], buf[1]);
}
