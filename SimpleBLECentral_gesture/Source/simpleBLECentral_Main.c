/**************************************************************************************************
  Filename:       simpleBLECentral_Main.c
  Revised:        $Date: 2011-02-24 15:48:00 -0800 (Thu, 24 Feb 2011) $
  Revision:       $Revision: 11 $

  Description:    This file contains the main and callback functions for
                  the Simple BLE Central sample application.


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

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
/* Hal Drivers */
#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"

/*1147*/
#include "Si114x_functions.h"
#include <string.h>



HANDLE si114x_handle;
U8 xdata     CMD_CNT = 0;
bit     PM_INT  = 0;                    // Port Match flag for Main routine
//xdata   SI114X_IRQ_SAMPLE Si114xsamples;
SI114X_IRQ_SAMPLE Si114xsamples;
bit     TXStartedFlag = 0;
bit     sw_reset = 0;
U16     Timestamp;
extern void Process_INT();
extern void delay_10ms(void);
extern s16 _sendCmd(HANDLE si114x_handle, u8 command);
extern s16 Si114xReadFromRegister(HANDLE si114x_handle, u8 address);

s16 si114x_init(HANDLE si114x_handle);


/**************************************************************************************************
 * FUNCTIONS
 **************************************************************************************************/

/**************************************************************************************************
 * @fn          main
 *
 * @brief       Start of application.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
char astr[48];
#define st(x)      do { x } while (__LINE__ == -1)

int main(void)
{

#if 1
  /* Initialize hardware */
  HAL_BOARD_INIT();

  // Initialize board I/O
  InitBoard( OB_COLD );

  /* Initialze the HAL driver */
  HalDriverInit();

  /* Initialize NV system */
  osal_snv_init();
  
  /* Initialize LL */

  /* Initialize the operating system */
  osal_init_system();

  /* Enable interrupts */
  HAL_ENABLE_INTERRUPTS();

  // Final board initialization
  InitBoard( OB_READY );

  #if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_BATTERY );
  #endif
#else
	/* Initialize hardware */
	HAL_BOARD_INIT();

	/* Initialize NV system */
	//osal_snv_init();

	/* Initialze the HAL driver */
	HalDriverInit();

	osal_init_system();

#endif

  /*Init 1147*/

  s16 retval,i, j;
  u8 command;
#if 0

  P1DIR |= BV(5); //设置pin端口方向为输出
  
  P1SEL &= ~BV(5); //设置pin端口为GPIO功能  

  st( P1_5 = 0;);
  #endif
  NPI_WriteString("\r\n\r\nBeginxx \r\n");
  delay_10ms();
  delay_10ms();
  delay_10ms();
  si114x_handle = &g_114xId;
  g_114xId =  DEVICE_ID_1147;
  //while(1)
  {
   retval =Si114xWriteToRegister(si114x_handle, REG_MEAS_RATE,  0x01);
  NPI_WriteStrValue("114x write ret:%d\r\n", retval);
  }
  #if 1
  retval =Si114xWriteToRegister(si114x_handle, REG_MEAS_RATE,  0x00);
  NPI_WriteStrValue("114x write ret:%d\r\n", retval);
  retval = si114x_init( si114x_handle );

  NPI_WriteStrValue("si114x_init ret:%d\r\n", retval);
  #endif
  retval += si114x_disPlayCfg(si114x_handle);
  NPI_WriteStrValue("display ret:%d,\r\n",retval);
#if 0

  retval = Si114xParamRead(si114x_handle, PARAM_CH_LIST);  
  NPI_WriteStrValue("1ram read:%d,\r\n",retval);  

  Si114xParamSet(si114x_handle, PARAM_CH_LIST, 0x37);
  retval = Si114xParamRead(si114x_handle, PARAM_CH_LIST);  
  NPI_WriteStrValue("ram read:%d,\r\n",retval);  


 retval = Si114xReadFromRegister(si114x_handle, 12); 
  NPI_WriteStrValue("12 read:%d,\r\n",retval); 


 retval = Si114xReadFromRegister(si114x_handle, 13); 
  NPI_WriteStrValue("13 read:%d,\r\n",retval);   

 retval = Si114xReadFromRegister(si114x_handle, 14); 
  NPI_WriteStrValue("14 read:%d,\r\n",retval); 

 retval = Si114xReadFromRegister(si114x_handle, 15); 
  NPI_WriteStrValue("15 read:%d,\r\n",retval);   

 retval = Si114xReadFromRegister(si114x_handle, 16); 
  NPI_WriteStrValue("16 read:%d,\r\n",retval);    
#endif  
  /* Start OSAL */
 #if 1
	 osal_start_system(); // No Return from here
 #else
 	 while(1);
 #endif
  return 0;
}

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
