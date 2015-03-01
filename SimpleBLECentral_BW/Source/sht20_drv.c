
#include "bcomdef.h"
#include "OSAL.h"
#include "hal_mcu.h"
#include "sht20_drv.h"
#include "i2c.h"
#include "hal_i2c.h"

#define SHT20_I2C_ADDR  (0x40)

#define SHT20_REG_RH      (0xF5)    //no hold, 可轮询读取湿度
#define SHT20_REG_TEMP    (0xF3)    //no hold, 可轮询读取温度
#define SHT20_REG_USER_W  (0xE6)    //用户写寄存器


//计算sht20传感器的crc
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

int16 g_Sht20Temp = 0;

//读取sht20的温度值
uint16 sht20_read_temp(void)
{
    halIntState_t intState;
    uint8 ret = I2C_SUCCESS;
    uint8 buff[3];
    uint16 temp = INVALID_UINT16;
    //uint8 cmd = SHT20_REG_TEMP;
    
    HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.

    ret = i2c_read(SHT20_I2C_ADDR, SHT20_REG_TEMP, buff, 3); 
    
    HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.
  
    if( ret != I2C_SUCCESS) {
      return temp;
    }	

#if 0  // 单片机硬件I2C
    HalI2CInit(SHT20_I2C_ADDR,i2cClock_267KHZ);
    if(HalI2CWrite(1, &cmd) != 1)
    {
      return I2C_FAIL;
    }
    if(HalI2CRead(3, buff) != 3)
    {
      return I2C_FAIL;
    }
#endif

    if(buff[2] != sht20_calc_crc(buff, 2))  {
      return temp;
    }
    
    temp = BUILD_UINT16(buff[1], buff[0]);
    
    g_Sht20Temp = ((int32)temp*17572)/65536-4685; //放大了100倍
    
    return temp;
}


uint16 g_Sht20Humidity = 0;

//读取sht20的湿度值
uint16 sht20_read_humidity(void)
{
    halIntState_t intState;
    uint8 ret = I2C_SUCCESS;
    uint8 buff[3];
    uint16 temp = INVALID_UINT16;
   
    HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.
    ret = i2c_read(SHT20_I2C_ADDR, SHT20_REG_RH, buff, 3); 
    HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.
   if( ret != I2C_SUCCESS) {
      return temp;
    }	

    if(buff[2] != sht20_calc_crc(buff, 2))  {
      return temp;
    }
    
    temp = BUILD_UINT16(buff[1], buff[0]);
    
    g_Sht20Humidity = ((uint32)temp*12500)/65536-600;; //放大了100倍
    
    return temp;
}



