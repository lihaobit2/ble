#include <stdio.h>
#include "Si114x_types.h"
#include "Si114x_PGM_Toolkit.h"
#include "Si114x_functions.h"

extern HANDLE mcu_handle;

void PortSet(u8 port, u8 portdata)
{
       static u8 current_data[3] = {0xff, 0xff, 0xff};

    if ( portdata != current_data[port] )
    {
        PT_i2c_smbus_write_byte_data(mcu_handle, port, portdata);
        current_data[port] = portdata;
    }
}

s16 PortRead(u8 port)
{
    return PT_i2c_smbus_read_byte_data(mcu_handle, port) < 0 ;
}


s16 Si114xWriteToRegister(HANDLE si114x_handle, u8 address, u8 value)
{
    return PT_i2c_smbus_write_byte_data(si114x_handle, address, value);
}

s16 Si114xReadFromRegister(HANDLE si114x_handle, u8 address)
{
    return PT_i2c_smbus_read_byte_data(si114x_handle, address);
}

s16 Si114xBlockWrite(HANDLE si114x_handle, 
                        u8 address, u8 length, u8 *values)
{
    return PT_i2c_smbus_write_i2c_block_data(si114x_handle,
                           address,    length,           values);
}

s16 Si114xBlockRead(HANDLE si114x_handle, 
                        u8 address, u8 length, u8 *values)
{
    return PT_i2c_smbus_read_i2c_block_data(si114x_handle,
                           address,    length,     values);
}

void delay_10ms(void)
{
    // This is needed immediately after a reset command to the Si114x
    // In the PGM_Toolkit, there is sufficient latency, so none is added
    // here. This is a reminder that when porting code, that this must
    // be implemented.
   u16 counter;
   for(counter = 24500; counter > 0; counter--);     
  
}


