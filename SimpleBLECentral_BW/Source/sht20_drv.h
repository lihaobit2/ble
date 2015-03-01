#ifndef _SHT20_DRV_H_
#define _SHT20_DRV_H_

//温度湿度传感器查询

#define INVALID_UINT8                 0xff
#define INVALID_UINT16                0xffff  
#define INVALID_UINT32                0xffffffff

uint16 sht20_read_temp(void);
uint16 sht20_read_humidity(void);

extern int16 g_Sht20Temp;
extern uint16 g_Sht20Humidity;


#endif