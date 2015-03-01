#include "Si114x_PGM_Toolkit.h"
#include "i2c.h"

SI114X_PGM_TOOLKIT_API PT_RESULT PT_getRev()
{
	return 0;
	
}
// Returns version number of DLL
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_getNumSerialPorts()
{
	return 0;
}
// Queries the system and list a number of serial ports that could be connected
// to an Si114x EVB
//
// Input: 
//          None
//
// Output: 
//          None
//
// Return:
//                                 0 to 255         Number of serial ports found
//                                      -10         Undefined error
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_getSerialPortName
(
    int     serial_port_index, 
    STRING  serial_port_name 
)
{
	return 0;	
}

// A system may have many serial ports that could have an Si114x EVB on it.
// This is determined through the getNumSerialPorts() call. This function 
// returns a human-readable string
//
// Input: 
//          serial_port_index      0 to (N-1)       Where N = getNumSerialPorts() 
//
// Output: 
//          serial_port_name                        if successful, writes name
//                                                  to string pointer.
//                                                  If not, writes 0 to first
//                                                  location
// 
// Return:
//                                         0        command was unsuccessful
//                                       -10        Undefined error
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API HANDLE PT_openSerialPort
(
    STRING serial_port_name
)
{
	return 0;	
}

// This opens the serial port, initializes memory. Note that this does not
// perform any kind of RESET or command to the EVB MCU. 
//
// Input: 
//          serial_port_name            NULL        looks for 'TSxxx' and opens
//                                                  it automatically. If 'TSxxx'
//                                                  not available, looks for 
//                                                  highest 'COMx' port
//
//                                      string      Points to string created
//                                                  by getSerialPortName()
// Output: 
//          None
// 
// Return:
//                                      NULL        command was unsuccessful
//                                  non-NULL        board handle
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API void PT_closeSerialPort
(
    HANDLE board_handle
)
{
	return;	
}

// This closes serial port. Note that this does alter the behavior of the MCU on 
// the EVB. If the EVB is still streaming samples out to the serial port, this 
// will continue even after this command has been sent. 
// Input: 
//          None
// Output: 
//          None
// Result: 
//          None
//
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API HANDLE PT_getDeviceHandle
(
    HANDLE board_handle, 
    int    device_index
)
{
	return 0;	
}

// This is used to create the device handles needed for I2C / SMBus
// communication. The reason for this is to support possible multiple I2C 
// devices that may be present in an EVB
//
// Input: 
//          board_handle                NULL        invalid
//                                  non-NULL        value from openSerialPort()
//
//          device_index                   0        The EVB MCUis treated as an 
//                                                  I2C mapped device. This 
//                                                  allows communication between
//                                                  the PC application and the 
//                                                  MCU through this handle.
//                                                  The features exposed by the
//                                                  virtual MCU is dependent on
//                                                  the firmware programmed in
//                                                  the EVB MCU.
//
//                                         1        First Si114x Device
//
//                                    2 to 8        Reserved for 2nd to 8th 
//                                                  Si114x
//
//                                        >8        Reserved, usable for future
//                                                  devices
//                                                  
// Output: 
//          None
//
// Result: 
//                                      NULL        Command Unsuccessful
//                                  non-NULL        device handle associated with
//                                                  the device implied by 
//                                                  device_index
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_i2c_smbus_read_byte
(
    HANDLE device_handle
)
{
	return 0;

}
// Initiates single-byte I2C Read from the I2C Bus
// 
// Input:
//          device_handle                           from getDeviceHandle()
//
// Output:   
//          None
//
// Return:                         
//                                 0 to 255         Read Data
//                                       -1         NoAck Received
//                                       -2         i2c error
//                                       -3         invalid device handle
//                                       -10        undefined error
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_i2c_smbus_write_byte
(
    HANDLE device_handle, 
    u8 value
)
{
	return 0;	
}

// Initiates single-byte I2C Write out to the I2C Bus
//
// Input:
//          device_handle                           from getDeviceHandle()
//
//          value                 0 to 255          value of byte to be written
//
// Output:  
//          None   
//
// Return:                                
//                                        0         Successful Write
//                                       -1         NoAck Received
//                                       -2         i2c error
//                                       -3         invalid device handle
//                                       -10        undefined error
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_i2c_smbus_read_byte_data
(
    HANDLE device_handle, 
    u8     command
)
{
    uint8 data;
    uint8 ret;
	
	ret = i2c_read(*((uint8*)device_handle), (uint8)command, &data, 1);

	if(ret != I2C_SUCCESS)
	{
		return -1;
	}
	
	return data;
}

// Used for reading a byte from an device with many i2c registers. The command 
// byte is a proxy for a device's address.
//
// Input:  
//          device_handle                           from getDeviceHandle()
//
//          command                0 to 255         device's i2c address
//
// Output:   
//          None   
//
// Return:                         
//                                 0 to 255         Read Data
//                                       -1         NoAck Received
//                                       -2         i2c error
//                                       -3         invalid device handle
//                                       -10        undefined error
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_i2c_smbus_write_byte_data
(
    HANDLE device_handle, 
    u8     command,  
    u8     value
)
{
	uint8 ret;
	ret = i2c_write(*((uint8*)device_handle), (uint8)command, &value, 1); 

 	if(ret != I2C_SUCCESS)
	{
		return -1;
	}
	
	return 0;
}
// Used for writing a byte to an device with many i2c registers. The command 
// byte is a proxy for a device's address.
//
// Input:
//          device_handle                           from getDeviceHandle()
//
//          command                0 to 255         device's i2c address
//
//          value                  0 to 255         value of byte to be written
//
// Output:  
//          None   
//
// Return:                                
//                                        0         Successful Write
//                                       -1         NoAck Received
//                                       -2         i2c error
//                                       -3         invalid device handle
//                                       -10        undefined error
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_i2c_smbus_write_i2c_block_data
(
    HANDLE   device_handle, 
    u8       command, 
    u8       length, 
    const u8 *values 
)
{
	uint8 ret;
	
	ret = i2c_write(*((uint8*)device_handle), (uint8)command, (uint8*)values, length);  

	if(ret != I2C_SUCCESS)
	{
		return -1;
	}

	return 0;
}
// Used for writing multiple bytes to an device with many i2c registers. The 
// command byte is a proxy for a device's address.
//
// Input:    
//          
//          device_handle                          from getDeviceHandle()
//
//          command               0 to 255         first i2c address
//
//          length                0 to 16          valid length
//                                     >16         invalid length
//
//          values                                 pointer to byte array 
//                                                 having values to be written
//
// Output:  
//          None   
//
// Return:                                
//                                        0         Successful Write
//                                       -1         NoAck Received
//                                       -2         i2c error
//                                       -3         invalid device handle
//                                       -4         invalid length
//                                       -10        undefined error
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_i2c_smbus_read_i2c_block_data
(
    HANDLE device_handle, 
    u8     command,  
    u8     length,
    u8     *values
)
{
	return 0;	
}
// The i2c_smbus_read_i2c_block_data () function is used for performing block 
// reads from contiguous i2c registers.
//
// Input:    
//           device_handle                          from getDeviceHandle()
//
//           command             0 to 255           i2c starting address of
//                                                  data to be read 
//                                 
//           length                0 to 16          valid length
//                                     >16          invalid length
// Output:   
//           values                                 pointer passed by reference
//                                                  to character array allocated
//                                                  by caller
// Return:                                
//                                        0         Successful Write
//                                       -1         NoAck Received
//                                       -2         i2c error
//                                       -3         invalid device handle  
//                                       -4         invalid length
//                                       -10        undefined error
//
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_configDebugChannel
(
    HANDLE            device_handle, 
    u8                dbg_channel, 
    DEBUG_DESCRIPTOR *descriptor
)
{
	return 0;	
}
//
// Configures and enables one of 8 debug channels
//
// Input:    
//          device_handle                           from getDeviceHandle()
//                                                  
//          dbg_channel            0 to 7           valid length
//                                     >7           invalid length
//
//          descriptor                              describes properties of 
//                                                  the dbg_channel
//
// Output:  None
//
// Return:
//                                      0           Successful configuration
//                                     -4           invalid device handle
//                                     -5           invalid debug descriptor
//                                                  or invalid dbg_channel
//                                    -10           undefined error
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_tagDebugChannel
(
    HANDLE     device_handle, 
    u8         dbg_channel,
    STRING     string
)
{
	return 0;	
}
//
// Inserts a string, into the UDP Port or file, associated with a given debug 
// channel. If sending data to the UDP Port, and the UDP Port is a waveform
// viewer, make sure that the string starts with '# ' for proper operation
//
// Input:    
//          device_handle                           from getDeviceHandle()
//                                                  
//          dbg_channel            0 to 7           valid length
//                                     >7           invalid length
//
//          string                                  points to string to be
//                                                  sent to the debug channel
// Output:  None
//
// Return:
//                                      0           Successful configuration
//                                     -4           invalid device handle
//                                     -5           invalid debug descriptor
//                                                  or invalid dbg_channel
//                                     -8           debug not enabled
//                                    -10           undefined error
// -----------------------------------------------------------------------------


SI114X_PGM_TOOLKIT_API PT_RESULT PT_replaceSample
(
    HANDLE     device_handle, 
    u8         dbg_channel,
    u8         whatToReplaceIndex,
    s32        valueToReplace
)
{
	return 0;	
}
// This is intended to be a method of reusing possibly unused fields
//
//  Input:
//          device_handle                           from getDeviceHandle() 
//
//          dbg_channel            0 to 7           valid length
//                                     >7           invalid length
//
//          whatToReplaceIndex 
//                                     0            replaces vis
//                                     1            replaces ir
//                                     2            replaces ps1
//                                     3            replaces ps2
//                                     4            replaces ps3
//                                     5            replaces aux
//                                     6            generic 0
//                                     7            generic 1
//
//          valueToReplace      -99999 to 99999     value sent to UDP port
//                                                  or file
//
//                                  0x8000000       vis, ir, ps1, ps2, ps3, 
//                                                  aux raw values are 
//                                                  sent to UDP port or file. 
//
//                                    >  99999      value clipped to  99999
//
//
//                                    < -99999      value clipped to -99999
//
// Return:
//                                      0           Successful configuration
//                                     -4           invalid device handle
//                                     -5           invalid dbg_channel
//                                     -8           debug not enabled
//                                    -10           undefined error
//
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_si114x_irq_queue_numentries
(
    HANDLE device_handle
)
{
	return 0;	
}
// Returns the number of entries in the sample queue
//
// Input: 
//          device_handle                           from getDeviceHandle() 
//
// Output:
//          None
//
// Return:
//                                    >0            number of elements in the 
//                                                  queue
//                                    -3            Invalid Device Handle
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_si114x_irq_queue_set_max_entries
(
    HANDLE device_handle, 
    int    max_entries
)
{
	return 0;	
}
// Sets the maximum number of entries in the queue
//
// Input: 
//          device_handle                           from getDeviceHandle() 
//          max_entries                             limits the size of the queue
//                                                  default is 100
// Output: 
//          None
//
// Return:
//                                      0           Successful
//                                     -3           Invalid Device Handle
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_si114x_irq_queue_get_max_entries
(
    HANDLE device_handle
)
{
	return 0;	
}
// Returns the current setting for maximum number of entries in the queue
// Input: 
//          device_handle                           from getDeviceHandle() 
//
// Return:
//                                     >0           Current setting for max
//                                                  number of entries in queue
//                                     -3           Invalid Device Handle
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_si114x_irq_queue_flush
(
    HANDLE device_handle
)
{
	return 0;	
}
// Empties the queue to get rid of 'old samples'. Note that the queue will 
// continue filling as long as the EVB MCU is servicing interrupts 
// from the Si114x.
//
// Input: 
//          device_handle                           from getDeviceHandle() 
// Return:
//                                      0           Successful
//                                     -3           Invalid Device Handle
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API PT_RESULT PT_si114x_irq_queue_pop
(
    HANDLE             device_handle, 
    SI114X_IRQ_SAMPLE *samples
)
{
	return 0;	
}
// Removes an entry from the interrupt service queue, and copies the values 
// into a host-allocated sample structure.
//
// Input: 
//          device_handle                           from getDeviceHandle() 
//
// Output:
//          samples                                 SI114X_IRQ_SAMPLE is 
//                                                  written. 
// Return:
//                                      0           Successful
//                                     -3           Invalid Device Handle
//                                     -5           The queue is empty.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
SI114X_PGM_TOOLKIT_API void PT_setDebug(int flag);
// Internal DLL Debugging. Do not use without consultation with Silicon Labs:w
// -----------------------------------------------------------------------------

SI114X_PGM_TOOLKIT_API HANDLE PT_openSerialPort_br
(
    STRING serial_port_name,
    int    baudrate
)
{
	return 0;	
}

