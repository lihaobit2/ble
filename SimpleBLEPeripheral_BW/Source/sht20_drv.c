
#include "bcomdef.h"
#include "OSAL.h"
#include "sht20_drv.h"
#include "i2c.h"

#define SHT20_I2C_ADDR  (0x40)

#define SHT20_REG_RH      (0xF5)    //no hold, ����ѯ��ȡʪ��
#define SHT20_REG_TEMP    (0xF3)    //no hold, ����ѯ��ȡ�¶�
#define SHT20_REG_USER_W  (0xE6)    //�û�д�Ĵ���


//����sht20��������crc
uint8 sht20_calc_crc(uint8 data[], uint8 nbrOfBytes)
{
  uint8 crc = 0;	
  uint8 byteCtr;
  const uint16 POLYNOMIAL = 0x131;  //P(x)=x^8+x^5+x^4+1 = 100110001
  
  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
  { 
    crc ^= (data[byteCtr]);
    for (uint8 bit = 8; bit > 0; --bit)
    { 
      if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else crc = (crc << 1);
    }
  }
  
  return crc;
}

uint16 g_Temp = 0;

//��ȡsht20���¶�ֵ
void sht20_read_temp(void)
{
    uint8 ret;
    uint8 buff[3];
    uint16 temp;
   
    ret = i2c_read(SHT20_I2C_ADDR, SHT20_REG_TEMP, buff, 3); 
    if( ret != I2C_SUCCESS) {
      return;
    }	

    if(buff[2] != sht20_calc_crc(buff, 2))  {
      return;
    }
    
    temp = BUILD_UINT16(buff[1], buff[0]);
    
    g_Temp = ((uint32)temp*17572)/65536-4685; //�Ŵ���100��
    
    return;
}


uint16 g_Humidity = 0;

//��ȡsht20��ʪ��ֵ
void sht20_read_humidity(void)
{
    uint8 ret;
    uint8 buff[3];
    uint16 temp;
   
    ret = i2c_read(SHT20_I2C_ADDR, SHT20_REG_RH, buff, 3); 
    if( ret != I2C_SUCCESS) {
      return;
    }	

    if(buff[2] != sht20_calc_crc(buff, 2))  {
      return;
    }
    
    temp = BUILD_UINT16(buff[1], buff[0]);
    
    g_Humidity = ((uint32)temp*12500)/65536-600;; //�Ŵ���100��
    
    return;
}



