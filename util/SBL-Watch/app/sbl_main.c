/**************************************************************************************************
  Filename:       sbl_main.c
  Revised:        $Date: 2012-09-07 14:46:45 -0700 (Fri, 07 Sep 2012) $
  Revision:       $Revision: 31500 $

  Description:

  This module contains the definitions for the main functionality of a Serial Boot Loader.


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

#include "hal_adc.h"
#include "hal_assert.h"
#include "hal_dma.h"
#include "hal_sleep.h"
#include "hal_types.h"
#include "hal_led.h"
#include "hal_flash.h"
#include "sbl_exec.h"

/* ------------------------------------------------------------------------------------------------
 *                                       Global Variables
 * ------------------------------------------------------------------------------------------------
 */

halDMADesc_t dmaCh0;  // Locally setup for use by HalFlashWrite() instead of calling HalDMAInit().

/* ------------------------------------------------------------------------------------------------
 *                                       Local Functions
 * ------------------------------------------------------------------------------------------------
 */

#if HAL_UART_SPI
#include "_sbl_spi.c"
#elif HAL_UART_SBL
#include "_sbl_uart.c"
#else
#error No valid transport configured.
#endif


void HalTimer3Init(void);
void TrialCheck(void);


#define LOADER_VER       0x0002
#pragma location=0x0400
const CODE uint16 loaderVer = LOADER_VER;
#pragma required=loaderVer

#define TRIAL_NUM_ADDR   0x07FC


/**************************************************************************************************
 * @fn          main
 *
 * @brief       ISR for the reset vector.
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
 **************************************************************************************************
 */
void main(void)
{
  HAL_DISABLE_INTERRUPTS();
  
  HAL_BOARD_INIT();

  /* This is in place of calling HalDmaInit() which would require init of the other 4 DMA
   * descriptors in addition to just Channel 0.
   */
  HAL_DMA_SET_ADDR_DESC0(&dmaCh0);

  //TrialCheck();
  
  sblInit();
  
  HalTimer3Init();
  
  while (!HalAdcCheckVdd(VDD_MIN_NV));

  sblRun();
}

//定时器
void HalTimer3Init(void)
{

  T3CTL = 0xF6;	//128分频，启动定时器，中断禁止，重复模式
  T3CC0 = 0xFA;	// PRD = 250/(32M/128) = 1ms中断
  T3CCTL0 = 0x44;//比较方式，通道1中断，当定时器值等于在T4CC1中的比较值时指定输出
}

//试用次数检查
void TrialCheck(void)
{
  uint8 buf[4];
  uint8 i;
  uint16 addr = TRIAL_NUM_ADDR;
  uint8 dat;
  
  HalFlashRead(addr / HAL_FLASH_PAGE_SIZE,
            addr % HAL_FLASH_PAGE_SIZE, 
            buf, 4);

  for(i =0; i < 5; i++)
  {
    dat = (0x1<<i);
    
    if((buf[0]&dat) != 0)
    {
      buf[0] = ~dat;
      HalFlashWrite(addr/HAL_FLASH_WORD_SIZE, &buf[0], 1);
      return;
    }
  }
  
  for(;;);  //死循环
  
}


/**************************************************************************************************
*/
