#include "bcomdef.h"
#include "OnBoard.h"
#include "i2c.h"

#define SCL (6)
#define SDA (7)

#define INPUT     (0)
#define OUTPUT    (1)

#define OVR       BV(7)
#define SCLOE     BV(1)
#define SDAOE     BV(0)

#define SCLD      BV(1)
#define SDAD      BV(0)

#define ACK       0
#define NACK      1


uint16 g_i2c_delay = 1;//5;  //10us

//i2c延时，单位us
void i2c_delay(uint16 times)
{
  uint16 i,j;
  
	for(i=0; i<times; i++ ){
    for(j=0; j<10; j++) {
      asm("nop"); 
    }
	}
}

//设置i2c管脚方向
void i2c_gpio_direction(uint8 pin, uint8 dir)  
{
#if I2C_GPIO
  if( dir==INPUT ) {
    P1DIR &= ~BV(pin); //设置pin端口方向为输入
  } else {
    P1DIR |= BV(pin); //设置pin端口方向为输出
  }
  
  P1SEL &= ~BV(pin); //设置pin端口为GPIO功能  
#else
  if( SCL==pin ) {
    if( dir==INPUT ) {
      I2CWC &= ~SCLOE; //设置pin端口方向为输入
    } else {
      I2CWC |= SCLOE; //设置pin端口方向为输出
    }
  } else if( SDA==pin ) {
    if( dir==INPUT ) {
      I2CWC &= ~SDAOE; //设置pin端口方向为输入
    } else {
      I2CWC |= SDAOE; //设置pin端口方向为输出
    }
  }
#endif  
}

//设置i2c管脚输出值
void i2c_gpio_set(uint8 pin, uint8 val) 
{
#if I2C_GPIO
  if( SCL==pin ) {
    P1_6 = val;
  } else if( SDA==pin ) {
    P1_7 = val;
  }
#else
  if( SCL==pin ) {
    if( val==0 ) {
      I2CIO &= ~SCLD; 
    } else {
      I2CIO |= SCLD; 
    }
  } else if( SDA==pin ) {
    if( val==0 ) {
      I2CIO &= ~SDAD; 
    } else {
      I2CIO |= SDAD; 
    }
  }
#endif
  
}

//获取i2c管脚值
uint8 i2c_gpio_get(uint8 pin) 
{
#if I2C_GPIO
  if( SCL==pin ) {
    return P1_6 ? 1:0;
  } else if( SDA==pin ) {
    return P1_7 ? 1:0;
  }
#else
  
  if( SCL==pin ) {
    return (I2CIO & SCLD) ? 1:0;
  } else if( SDA==pin ) {
    return (I2CIO & SDAD) ? 1:0;
  }
#endif
  return 0;
}

//起始信号
void i2c_start(void)  
{   
#if !I2C_GPIO  
	//NPI_WriteString("11-1-1-1\n"); 

    I2CWC |= OVR;
#endif
    //NPI_WriteString("11-1-1-2\n"); 
    i2c_gpio_direction(SDA, OUTPUT);    
    i2c_gpio_direction(SCL, OUTPUT);  
    //NPI_WriteString("11-1-1-3\n");     
    i2c_gpio_set(SDA, 1);
    //NPI_WriteString("11-1-1-4\n"); 
    i2c_gpio_set(SCL, 1); 	
    i2c_delay(g_i2c_delay);  
      
    i2c_gpio_set(SDA, 0);  
    i2c_delay(g_i2c_delay);      
            
    i2c_gpio_set(SCL, 0);  
    i2c_delay(g_i2c_delay);  
}  

//停止信号
void i2c_stop(void)  
{   
    i2c_gpio_direction(SCL, OUTPUT);  
    i2c_gpio_direction(SDA, OUTPUT);  
    
    i2c_gpio_set(SDA, 0);
    i2c_gpio_set(SCL, 0); 		
    i2c_delay(g_i2c_delay);   
      
    i2c_gpio_set(SCL, 1);  
    i2c_delay(g_i2c_delay);  
    
    i2c_gpio_set(SDA, 1);  
    i2c_delay(g_i2c_delay); 
    
    i2c_gpio_direction(SCL, INPUT);  //节能
    i2c_gpio_direction(SDA, INPUT);  //节能

}   

//发送ACK信号
void i2c_send_ack(uint8 ack)  
{  
    i2c_gpio_direction(SDA, OUTPUT);  
    if(ack)  
        i2c_gpio_set(SDA, 1);   
    else   
        i2c_gpio_set(SDA, 0);  
    i2c_delay(g_i2c_delay);  
      
    i2c_gpio_set(SCL, 1);  
    i2c_delay(g_i2c_delay);  
      
    i2c_gpio_set(SCL, 0);  
    i2c_delay(g_i2c_delay);    
}  

//接收ACK信号
uint8 i2c_receive_ack(void)  
{  
    uint8 rc = I2C_SUCCESS;  
      
    i2c_gpio_direction(SDA, INPUT);  
    
    i2c_gpio_set(SCL, 1);  
    i2c_delay(g_i2c_delay);  
      
    if(i2c_gpio_get(SDA)) {  
        rc = 1;  
    }
	
    i2c_gpio_set(SCL, 0);  
    i2c_gpio_direction(SDA, OUTPUT);  
    
    return rc;  
}  

//发送一个字节
uint8 i2c_send_byte(uint8 send_byte)  
{  
    uint8 rc = I2C_SUCCESS;  
    uint8 out_mask = 0x80;  
    uint8 value;  
    uint8 count = 8;  
    
    while(count > 0) {                  
        value = ((send_byte & out_mask) ? 1 : 0);     
        if (value == 1) {                                     
            i2c_gpio_set(SDA, 1);       
        }      
        else {                                    
            i2c_gpio_set(SDA, 0);  
        }      
        i2c_delay(g_i2c_delay);  
                                    
        i2c_gpio_set(SCL, 1);                       
        i2c_delay(g_i2c_delay);  
                       
        i2c_gpio_set(SCL, 0);       
        i2c_delay(g_i2c_delay);  
                    
        out_mask >>= 1;        
        count--;         
    }  
      
    rc = i2c_receive_ack();  
    return rc;  
}  

//读取一个字节
void i2c_read_byte(uint8 *buffer, uint8 ack)  
{  
    uint8 count = 0x08;  
    uint8 data = 0x00;  
    uint8 temp = 0;  
      
    i2c_gpio_direction(SDA, INPUT);  
    i2c_delay(g_i2c_delay);  
    
    while(count > 0) {  
        i2c_gpio_set(SCL, 1);  
        i2c_delay(g_i2c_delay);  
        
        temp = i2c_gpio_get(SDA);       
        data <<= 1;  
        if (temp)  
            data |= 0x01;
        
        i2c_gpio_set(SCL, 0);  
        i2c_delay(g_i2c_delay);  
        
        count--;  
    }  
    
    i2c_send_ack(ack);//0 = ACK    1 = NACK   
    *buffer = data;           
}  

//标准写操作
uint8 i2c_write(uint8 device_id, uint8 reg_address, uint8* data, uint8 len)  
{  
    uint8 rc = I2C_SUCCESS;  
    uint8 i;  

    i2c_start();    
    //NPI_WriteString("11-1-1\n");  
    rc |= i2c_send_byte( (device_id << 1) | 0x00 );  
    //NPI_WriteString("11-1-2\n"); 
    rc |= i2c_send_byte(reg_address);  
          
    for(i=0; i<len; i++) {  
        rc |= i2c_send_byte(*data);  
        data++;  
    }  
    //NPI_WriteString("11-1-3\n");   
    i2c_stop();       
    return rc;    
}  
    
//标准读取操作
uint8 i2c_read(uint8 device_id, uint8 reg_address, uint8 *buffer, uint8 len)  
{  
    uint8 rc = I2C_SUCCESS;  
    uint8 i = 0;  
      
    i2c_start();  
    
    rc |= i2c_send_byte( (device_id << 1) | 0x00 ); 
    if(rc) {  
        return rc;  
    }  

    rc |= i2c_send_byte(reg_address);
    if(rc) {  
        return rc;  
    }	
	
    //poll模式
    do
    { 
      i2c_start();
      i2c_delay(g_i2c_delay);  //delay 10ms
      
      if(i++ >= 200) {
        break;
      }
    } while(0 != i2c_send_byte( (device_id << 1) | 0x01 ));

    if (i>=200) {  
        rc = I2C_FAIL;
        return rc;  
    }
        
    
    for(i=0;i<len;i++) {  
        i2c_read_byte(buffer++, !(len-i-1));//  !(len-i-1): ACK,ACK,... NACK   
    }  
    
    i2c_stop();    
    return rc;    
}  

