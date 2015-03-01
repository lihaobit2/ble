#ifndef _UART_APP_H_
#define _UART_APP_H_

//命令字
#define UART2_NULL_CMD            (0X00)
#define UART2_SYNC_CMD            (0X55)
#define UART2_CMD_GSENSOR         (0X50)
#define UART2_CMD_TEMPERATURE 	  (0X51)
#define UART2_CMD_HUMIDITY 		    (0X52)
#define UART2_CMD_PULSE 		      (0X53)
#define UART2_CMD_BLOOD_PRESS     (0x54)  
#define UART2_CMD_END_PLUSE       (0x56)
#define UART2_CMD_CLEAR_GSENSOR     (0x57)   
#define UART2_CMD_END_BLOOD_PRESS   (0x58)   

#define UART2_CMD_LOAD_START 	    (0X5C)
#define UART2_CMD_LOAD_DATA 	    (0X5D)
#define UART2_CMD_LOAD_END 		    (0X5E)
#define UART2_CMD_APP_VER 	      (0X5F)
#define UART2_CMD_LICENSE_INFO    (0X60)

//回应结果
#define SBL_ACK_OK              0x00
#define SBL_ACK_FAIL            0x01
#define SBL_ACK_ID_ERR          0x02
#define SBL_ACK_CRC_ERR         0x03



void uartRecvCB(uint8 port, uint8 event);

void uart_send_pulse(uint8 pulse);

void uart_send_blood_press(uint8 sys, uint8 dia, uint8 pul);

#endif