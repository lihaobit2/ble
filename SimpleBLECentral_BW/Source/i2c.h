#ifndef _BT4_I2C_H_
#define _BT4_I2C_H_


#define I2C_GPIO        0 //1：使用普通的GPIO模拟I2C

#define I2C_SUCCESS     0
#define I2C_FAIL        1


uint8 i2c_write(uint8 device_id, uint8 reg_address, uint8* data, uint8 len);  
uint8 i2c_read(uint8 device_id, uint8 reg_address, uint8 *buffer, uint8 len); 

uint8 SET_Resolution(void);          
uint8 SHT2x_ReadUserRegister(uint8 *pRegisterValue);
uint8 SHT2x_WriteUserRegister(void);

#endif