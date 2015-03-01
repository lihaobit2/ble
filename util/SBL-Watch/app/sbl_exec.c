/**************************************************************************************************
  Filename:       sbl_exec.c
  Revised:        $Date: 2012-09-07 14:26:14 -0700 (Fri, 07 Sep 2012) $
  Revision:       $Revision: 31498 $

  Description:  Serial Bootloader Executive.


  Copyright 2011-2012 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_board.h"
#include "hal_mcu.h"
#include "hal_rpc.h"
#include "hal_types.h"
#include "hal_uart.h"

#include "hal_adc.h"
#include "hal_board_cfg.h"
#include "hal_flash.h"
#include "hal_rpc.h"
#include "hal_types.h"
#include "hal_uart.h"
#include "sbl_app.h"
#include "sbl_exec.h"

/* ------------------------------------------------------------------------------------------------
 *                                        Constants
 * ------------------------------------------------------------------------------------------------
 */

#define SBL_DATA_LEN               32

// Buffer size - it has to be big enough for the largest RPC packet and overhead.
#define SBL_BUF_SIZE               (SBL_DATA_LEN+20)
   
#define RPC_STATE_SOF               0
#define RPC_STATE_DATA              1


// The SB page boundary since all SB addresses are "actual address / flash word size".
// Note for MSP - flash word size is 1, but 4 must be used for inter-compatibility w/ SoC.
#define SBL_PAGE_SIZE               (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE)


#define SBL_APP_START_ADDR          0x0800
#define SBL_APP_VER_ADDR            0x0900
   
/* ------------------------------------------------------------------------------------------------
 *                                     Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static uint8 rpcBuf[SBL_BUF_SIZE], sbIdx;
static uint8 *const sbBuf = rpcBuf+1;
static uint8 rpcSte;
static uint16 g_curFrameId;
static uint16 g_curLoadAddr;  //以32bit字为单位
uint16 g_timerOutCnts = 0;

/* ------------------------------------------------------------------------------------------------
 *                                     Local Functions
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
 * @return    
 */
void sblInit(void)
{
    g_curFrameId = 0;
    g_curLoadAddr = SBL_APP_START_ADDR/HAL_FLASH_WORD_SIZE;
    rpcSte = RPC_STATE_SOF;
}


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
uint8 sblPoll(void)
{
  uint8 ch;
  uint8 pg;

  
  while (HalUARTRead(0, &ch, 1))
  {
    
    switch (rpcSte)
    {
      case RPC_STATE_SOF:
        if (UART2_CMD_LOAD_START == ch)
        {
          sblReponseMsg(UART2_CMD_LOAD_START, SBL_ACK_OK);
          sblInit();
        }
        else if (UART2_CMD_LOAD_DATA == ch)
        {
          rpcSte = RPC_STATE_DATA;
          sbIdx = 0;
        }
        else if (UART2_CMD_LOAD_END == ch)
        {
          sblReponseMsg(UART2_CMD_LOAD_END, SBL_ACK_OK);
          sblInit();
          
          
          #if 0 
           for(pg = 1;  pg < 120; pg ++)
          {
            HalFlashErase(pg);
          }
          #endif
          
          JUMP_TO_APP(); //跳转
        }
        else if (UART2_CMD_APP_VER == ch)
        {
          sblReponseAppVer();
        }
        break;

      case RPC_STATE_DATA:
        //发送 UART2_CMD_LOAD_DATA + 帧编号高八位 + 帧编号低八位 + 32字节数据 +crc8	通知BT4主机加载数据
        sbBuf[sbIdx] = ch;
        sbIdx++;
        if (sbIdx >= 35)
        {
          rpcSte = RPC_STATE_SOF;
          
          sblDataProc();
          return FALSE;
        }
        break;

      default:
        break;
    }


    g_timerOutCnts = 0;

  }

  return FALSE;
}

/**************************************************************************************************
 * @fn          sblProc
 *
 * @brief       Process the SB command and received buffer.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void sblDataProc(void)
{
  uint16 frameId = BUILD_UINT16(sbBuf[1], sbBuf[0]);
  uint8 crc8;

  //检查帧编号
  if(frameId != g_curFrameId)
  {
    sblReponseMsg(UART2_CMD_LOAD_DATA, SBL_ACK_ID_ERR);
    return;
  }
 
  //检查crc
  crc8 = calc_crc8(&sbBuf[2], SBL_DATA_LEN);
  if(crc8 != sbBuf[SBL_DATA_LEN+2])
  {
    sblReponseMsg(UART2_CMD_LOAD_DATA, SBL_ACK_CRC_ERR);
    return;
  }
  
  //如果位于flash page的边界，则擦除当前page
  if ((g_curLoadAddr % SBL_PAGE_SIZE) == 0)
  {
    HalFlashErase(g_curLoadAddr / SBL_PAGE_SIZE);
  }
  
  //写flash，请注意长度单位为uint32，即4字节
  HalFlashWrite(g_curLoadAddr, &sbBuf[2], SBL_DATA_LEN/HAL_FLASH_WORD_SIZE);
  
  g_curLoadAddr += SBL_DATA_LEN/HAL_FLASH_WORD_SIZE;
  g_curFrameId += 1;
  
  sblReponseMsg(UART2_CMD_LOAD_DATA, SBL_ACK_OK);
  return;
}



//从flash读取uint16数据
uint16 FlashReadUint16(uint16 addr)
{
  uint8 buf[2];
  
  HalFlashRead(addr / HAL_FLASH_PAGE_SIZE,
              addr % HAL_FLASH_PAGE_SIZE, 
              buf, 2);
  
  return BUILD_UINT16(buf[0], buf[1]);
}

//检查flash中的app是否ok
uint8 checkAppOK(void)
{

  if( (FlashReadUint16(MAGIC_TYPE_ADDR) == LOAD_MAGIC_TYPE)
     && (FlashReadUint16(MAGIC_FEATURE_ADDR) == LOAD_MAGIC_FEATURE) )
  {
    return TRUE;
  }
  
  return FALSE;
}

//超时处理
void sblTimoutProc(void)
{
  static uint16 flashCnt = 0;
  
  if(T3CH0IF)
  {
    T3CH0IF = 0;
    g_timerOutCnts++;
    flashCnt++;
  }

  //闪灯
  LED1_DDR |= LED1_BV;  //output
  if(flashCnt > 300)
  {
    flashCnt = 0;
    LED1_SBIT= !LED1_SBIT;
  }

  //超时处理
  if(g_timerOutCnts > 20000) //20s
  {
    g_timerOutCnts = 0;
    
    JUMP_TO_APP();
  }
}

static uint8 txbuf[4];


//应答处理
void sblReponseMsg(uint8 cmd, uint8 ack)
{
 
  //Load:	UART2_SYNC_CMD + UART2_SYNC_CMD + cmd +八位响应ACK

  txbuf[0] = UART2_SYNC_CMD;
  txbuf[1] = UART2_SYNC_CMD;
  txbuf[2] = cmd;
  txbuf[3] = ack;
  
  (void)HalUARTWrite(0, txbuf, 4);
}

//查询app版本
void sblReponseAppVer(void)
{
  uint8 txbuf[5];
  uint16 ver = FlashReadUint16(SBL_APP_VER_ADDR);
 
  //回应:	UART2_SYNC_CMD + UART2_SYNC_CMD + UART2_CMD_APP_VER +VER高八位+VER低八位

  txbuf[0] = UART2_SYNC_CMD;
  txbuf[1] = UART2_SYNC_CMD;
  txbuf[2] = UART2_CMD_APP_VER;
  txbuf[3] = HI_UINT16(ver);
  txbuf[4] = LO_UINT16(ver);
 
  (void)HalUARTWrite(0, txbuf, 5);
}


//计算crc8
unsigned char calc_crc8(unsigned char* pDat, unsigned char len)
{
  unsigned char i, j;
  unsigned char crc = 0;

  for (j = 0; j < len; j++)
  {
    crc ^= *pDat++;
    for (i = 0; i < 8; i++)
    {
      if ((crc & 0x01) != 0)
      {
          crc >>= 1;
          crc ^= 0x8c;
      }
      else
      {
          crc >>= 1;
      }
    }
  }
  return crc;
}


/**************************************************************************************************
*/
