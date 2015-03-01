//#include "global.h"
//#include "queue.h"
//#include "smb.h"
#include "Si114x_functions.h"
#include "Si114x_PGM_Toolkit.h"

//-----------------------------------------------------------------------------
// Globals defined in main.c, used in this file.
//-----------------------------------------------------------------------------
//
extern U8  xdata               CMD_CNT;
extern bit                     PM_INT;
extern xdata SI114X_IRQ_SAMPLE Si114xsamples;
extern bit                     sw_reset;
extern U16                     Timestamp; 

//-----------------------------------------------------------------------------
// Globals defined in smb.c, used in this file.
//-----------------------------------------------------------------------------
//
#if 0
extern U8 xdata     i2c_addr[];
extern U8 xdata     i2c_bus[];


//-----------------------------------------------------------------------------
// Misc_options
//-----------------------------------------------------------------------------

static U8 idata     misc_options;
#define MISC_OPTIONS_0 0//(misc_options&0x01)  // Defines whether or not to 
                                            // reduce communications traffic
                                            // based on irqstatus

//-----------------------------------------------------------------------------
// Port Write Masks
//-----------------------------------------------------------------------------
//
#if ((BDNUM==4) || (BDNUM == 5))
static U8 code P0_WRITEMASK = 0xFF;
static U8 code P1_WRITEMASK = 0x00;
static U8 code P2_WRITEMASK = 0x00;
#elif ((BDNUM == 2) || (BDNUM == 3)|| (BDNUM == 0) || (BDNUM == 6))
static U8 code P0_WRITEMASK = 0x7C;
static U8 code P1_WRITEMASK = 0x00;
static U8 code P2_WRITEMASK = 0x00;
#elif BDNUM == 1
static U8 code P0_WRITEMASK = 0xF0;
static U8 code P1_WRITEMASK = 0x01;
static U8 code P2_WRITEMASK = 0x00;
#endif

//-----------------------------------------------------------------------------
// Function Prototypes 
//-----------------------------------------------------------------------------
void SMBus_Init (U8 i2c_speed);         // From smb.c

//-----------------------------------------------------------------------------
// Master Device ID 
//-----------------------------------------------------------------------------
// 
// Identifies which Si114x the interrupt handler is supposed to service.
//
U8 xdata master = 1;    
                        // 0 = Not used since it is associated with virtual MCU
                        // 1 = First Si114x is the interrupt master
                        // 2 = Second Si114x is the interrupt master
                        // 3 = Third Si114x is the interrupt master
                        // etc
                        // up to 8
                        
//-----------------------------------------------------------------------------
// Slave Device ID 
//-----------------------------------------------------------------------------
//
// Identifies the second Si114x in which the master is responsible for 
// initiating measurements.
//
static U8 xdata slave = 0;     
                        // Master will be responsible for executing psalsforce
                        // command. Valid slave values are 1 to 8. If zero
                        // master assumes no slave.

//-----------------------------------------------------------------------------
// Function Prototypesfrom main.c, used in this file
//-----------------------------------------------------------------------------
//

void StartTX(void);         // From main.c 
#endif

void ProcessSi114xSamples(HANDLE si114x_handle, SI114X_IRQ_SAMPLE *samples);
                            // From si114x_handler.c
#if 0
//-----------------------------------------------------------------------------
// Converts an ASCII value into a hex value
// ie.- 'D' = 0xD
//-----------------------------------------------------------------------------
//
U8 ASCII_HEX( U8 ascii )
{
   if( ascii < 0x3A )
   {
      return (ascii& 0x0F);
   } else
   {
      return (0x09 + (ascii& 0x0F)) ;
   }
}
#endif
//-----------------------------------------------------------------------------
// Process_Device
//-----------------------------------------------------------------------------
//
// Defines the actions of the 'virtual mcu'
//
#if 0
//
// i2c_speed defaults to whatever is in global.h. However, it can be dynamically
// set by the main application to speed it up.
//
static U8 i2c_speed=I2CSPEED/10000;

void Process_Device( U8 fg_virtual_addr, U8 numberOfBytesToRead, U8 numberOfBytesToWrite, U8 idata *buffer_data) 
{
   static U8 device_addr = 0;
   U8 out_count = 1;

   if( fg_virtual_addr& 0x01 )                       // If device read, perform read
   {
      {
         switch(device_addr){
            case 0x00: 
               TXQ_insert_hex(P0);
               break;
            case 0x01:
               TXQ_insert_hex(P1);
               break;
            case 0x02:
               TXQ_insert_hex(P2);
               break;
            case 0x03:
               TXQ_insert_hex(master);
               break;
            case 0x04:
               TXQ_insert_hex(slave);
               break;
            case 0x05:
               TXQ_insert_hex(misc_options);
               break;
            case 0x08:
               TXQ_insert_hex(BDNUM);
               break;
            case 0x09:
               TXQ_insert_hex(FWID);
            case 0x0a:
               TXQ_insert_hex(i2c_speed);
               break;
            case 0x10:
            case 0x11:
            case 0x12:
            case 0x13:
            case 0x14:
            case 0x15:
            case 0x16:
            case 0x17:
               TXQ_insert_hex( i2c_addr[device_addr-0x10] );
               break;
            case 0x20:
            case 0x21:
            case 0x22:
            case 0x23:
            case 0x24:
            case 0x25:
            case 0x26:
            case 0x27:
               TXQ_insert_hex( i2c_bus[device_addr-0x20] );
               break;
            default:
               TXQ_insert_hex(0);
               break;
         }
         device_addr++;
         numberOfBytesToRead--;
      }
   } else                                       // Device write, perform write
   {
      device_addr = buffer_data[0];
      numberOfBytesToWrite--;
      out_count = 1;
      while(numberOfBytesToWrite > 0)
      {
         switch(device_addr){
            case 0x00: 
               P0 |= buffer_data[out_count] & ~P0_WRITEMASK;
               P0 &= buffer_data[out_count] | P0_WRITEMASK;
               break;
            case 0x01:
               P1 |= buffer_data[out_count] & ~P1_WRITEMASK;
               P1 &= buffer_data[out_count] | P1_WRITEMASK;
               break;
            case 0x02:
               #if ((BDNUM == 2) || (BDNUM == 3))
               P2 |= buffer_data[out_count] & ~P2_WRITEMASK;
               P2 &= buffer_data[out_count] | P2_WRITEMASK;
               #endif
               break;
            case 0x03:
               master = buffer_data[out_count];
               break;
            case 0x04:
               slave = buffer_data[out_count];
               break;
            case 0x05:
               misc_options = buffer_data[out_count];
               break;
            case 0x0a:
               i2c_speed = buffer_data[out_count];
               EA = 0;
               SMBus_Init(i2c_speed);
               EA = 1;
               break;
            case 0x10:
            case 0x11:
            case 0x12:
            case 0x13:
            case 0x14:
            case 0x15:
            case 0x16:
            case 0x17:
               i2c_addr[device_addr-0x10] = buffer_data[out_count];
               break;
            case 0x20:
            case 0x21:
            case 0x22:
            case 0x23:
            case 0x24:
            case 0x25:
            case 0x26:
            case 0x27:
               i2c_bus[device_addr-0x20] = buffer_data[out_count];
               break;
            case 0xFF: // Command received
               if( buffer_data[out_count] == 0xFF)
               {
                   HANDLE si114x_handle;

                   si114x_handle = 0xB4+((master-1)<<1);
                   Si114xReset(si114x_handle);
                   if(slave != 0) 
                   {
                       si114x_handle = 0xB4+((slave-1)<<1);
                       Si114xReset(si114x_handle);
                       //delay_10ms();
                   }
                   RSTSRC = 0x10;
               }
               break;
            default:
               break;
         }         
         device_addr++;
         out_count++;
         numberOfBytesToWrite--;
      }
   }
}

//-----------------------------------------------------------------------------
// Process_CMD
//-----------------------------------------------------------------------------
//
// Processes incoming bytes from the UART
//
void Process_CMD(void)
{
   U8 input_count = 0;     // This counter keeps track of the number of bytes being processed from UART RX Queue
   U8 idata byte_process = 0;    // This is a temporary holder for the input byte from UART RX Queue
   U8 input_hex = 0;       // Holds the converted hex value of the ascii input
   U8 read_count = 0;      // Temporary counter for the processing of the I2C input array

   U8 fg_virtual_addr;
   U8 idata buffer_data[16];
   U8 numberOfBytesToWrite = 0; 
   U8 numberOfBytesToRead  = 0; 

   S16 retval;
   S16 error = 0;

   while( byte_process != 0x0D ) // stop processing command when carriage return 0x0D is received
   {
      RXQ_delete(&byte_process);                // Remove first element from RX queue                     
      if( byte_process == 0x0D )
      break;                                    // If element is 0x0D, break loop
      input_hex = ASCII_HEX(byte_process) << 4; // Convert input from ascii to hex for processing
                                                // This ascii char is the MSB of the hex input
      
      RXQ_delete(&byte_process);                // Get next element from RX queue
      if( byte_process == 0x0D )                // If element is 0x0D, break loop
      break;
      input_hex += ASCII_HEX(byte_process);     // This ascii char is the LSB of the hex input

      if( input_count == 0 )                    // If first byte being processed, expect I2C slave address
      {
         fg_virtual_addr = input_hex;           // Set I2C slave address

         numberOfBytesToWrite = 0;              // Reset Write counter for new transaction         

         TXQ_insert_hex(fg_virtual_addr);       // Add virtual addr to UART TX response
      } else
      {
         if( fg_virtual_addr& 0x01 && input_count==1)// Means I2C read
         { 
            numberOfBytesToRead = input_hex;    // Set number of bytes to read
         } else
         {
            numberOfBytesToWrite++;             // Increment number of bytes to write counter
            buffer_data[input_count-1] = input_hex; // Add new byte to write into output array
         }
      }         

      input_count++;                            // increment input_counter for every UART byte processed
   }


   if( (fg_virtual_addr& 0xFE) == 0x00 )
   {
      Process_Device( fg_virtual_addr, numberOfBytesToRead, numberOfBytesToWrite, buffer_data );
   }
   else if( fg_virtual_addr& 0x01 )             // If I2C read, perform read
   {
      retval = i2c_smbus_read_block_idata( fg_virtual_addr, numberOfBytesToRead, buffer_data );

      if ( retval==0 )
      {
          while( read_count < numberOfBytesToRead )
          {
              TXQ_insert_hex( buffer_data[read_count] );
              read_count++ ;
          }
      }

   } else
   { 
      retval = i2c_smbus_write_block_idata( fg_virtual_addr, numberOfBytesToWrite, buffer_data );
   }
   
   if ( error )
   {
       TXQ_insert(-error);
   }

   TXQ_insert(0x0D);                         // Add carriage return to UART TX to end transmission

   StartTX();

   EA = 0;
   CMD_CNT--;
   EA = 1;
   
}
#endif
//-----------------------------------------------------------------------------
// Process_INT()
//-----------------------------------------------------------------------------
//
// Function is called as a foreground process as a result of Port Match 
// This function will read all data from the Si114x then service/clear the 
// SI114x IRQSTAT register. Then it will send the data upstream to the host
//
void Process_INT()
{

    U8 /*idata*/ Data_Buffer[13];
    //U8  /*idata*/ counter;
    S16 /*idata*/ smb_data_irqstatus;
    S16 /*idata*/ error;
    HANDLE si114x_handle;
    U8 device = 0;

    //for( device=0; device < 2; device++ )
    {
        //
        // Set up the handle to point to the right device
        // whether it is the master or slave device
        //
        #if 0
        if( device == 0 )
        {
            si114x_handle = 0xB4+((master-1)<<1);
        }
        else
        {
            if( slave == 0 ) break;
            si114x_handle = 0xB4+((slave-1)<<1);
        }
        #endif
        //
        // MISC_OPTIONS_0 is used to decrease communications traffic for the 
        // purpose of improving sample rate. It does this by examining the
        // irqstatus bits.
        //
        // It is not 100% certain that this option can also be used in 
        // combination with setting a slave since the irqstatus information 
        // may not be correct. The slave must be set up with interrupts 
        // enabled, MEASRATE=0, and INT_OE = 0 so that the INT* hardware does 
        // not assert, but the irqstatus bits are still set. 
        //
        #if 0
        if (MISC_OPTIONS_0)
        {
            // This is speed-optimized for proximity cases. 
            error = 0;
            smb_data_irqstatus = i2c_smbus_optimized( si114x_handle, Data_Buffer );
            PM_INT = 0;
        }
        else 
		#endif
        {
            error  = PT_i2c_smbus_read_i2c_block_data( si114x_handle, 0x21, 13, Data_Buffer );
            error += PT_i2c_smbus_write_byte_data( si114x_handle, 0x21, 0xff ); 
            smb_data_irqstatus = Data_Buffer[0];
            PM_INT = 0;
        }

        if( error == 0 )
        {
            //
            // Begin filling the transmit queue. If processing the slave, force the
            // next measurement at this time since the measurements have already
            // been picked up.
            //
            #if 0
            if( device == 0 )
            {
                TXQ_insert_hex( (master<<1) + 1);  
            }
            else
            {
                // A slave device needs to be 'forced' to make a measurement
                Si114xPsAlsForce( si114x_handle );
                TXQ_insert_hex( (slave<<1) + 1);  
            }
            
            StartTX();
         
            TXQ_insert_hex_sixteen(Timestamp);           // Insert timestamp into output
         
            TXQ_insert_hex(smb_data_irqstatus);          // Insert IRQSTAT into queue

            if( MISC_OPTIONS_0 )
            {
                // Send VIS and IR if either of the ALS bits are set
                if( smb_data_irqstatus & 0x03 )
                {
                    TXQ_insert_hex(Data_Buffer[2]);
                    TXQ_insert_hex(Data_Buffer[1]);
         
                    TXQ_insert_hex(Data_Buffer[4]);
                    TXQ_insert_hex(Data_Buffer[3]);
                }
         
                // Send PS1 conditionally
                if( smb_data_irqstatus & 0x04 )
                {
                    TXQ_insert_hex(Data_Buffer[6]);
                    TXQ_insert_hex(Data_Buffer[5]);
                }
         
                // Send PS2 conditionally
                if( smb_data_irqstatus & 0x08 )
                {
                    TXQ_insert_hex(Data_Buffer[8]);
                    TXQ_insert_hex(Data_Buffer[7]);
                }
         
                // Send PS3 conditionally
                if( smb_data_irqstatus & 0x10 )
                {
                    TXQ_insert_hex(Data_Buffer[10]);
                    TXQ_insert_hex(Data_Buffer[9]);
                }
         
                // Send AUX if either of the ALS bits are set
                if( smb_data_irqstatus & 0x03 )
                {
                    TXQ_insert_hex(Data_Buffer[12]);
                    TXQ_insert_hex(Data_Buffer[11]);
                }
            }
            else
            {
                for( counter = 1; counter < 13; counter+=2 )
                {      
                   TXQ_insert_hex(Data_Buffer[counter+1]);
                   TXQ_insert_hex(Data_Buffer[counter]);     // Add data into output
                }
            }
         
            TXQ_insert(0x0D);                            // Add carriage return to UART TX to end transmission           
         
            StartTX();                                   // Start TX again just to be sure
			#endif
            if( !sw_reset )
            {
                if( device == 0 ) 
                {
                   Si114xsamples.timestamp = Timestamp;
                   Si114xsamples.irqstat   = Data_Buffer[0];
                   Si114xsamples.vis       = Data_Buffer[1]  +  ( Data_Buffer[2]  << 8 );
                   Si114xsamples.ir        = Data_Buffer[3]  +  ( Data_Buffer[4]  << 8 );             // IR
                   Si114xsamples.ps1       = Data_Buffer[5]  +  ( Data_Buffer[6]  << 8 );            // PS1
                   Si114xsamples.ps2       = Data_Buffer[7]  +  ( Data_Buffer[8]  << 8 );            // PS2
                   Si114xsamples.ps3       = Data_Buffer[9]  +  ( Data_Buffer[10] << 8 );            // PS3
                   Si114xsamples.aux       = Data_Buffer[11] +  ( Data_Buffer[12] << 8 );            // AUX
                   
                   ProcessSi114xSamples(si114x_handle, &Si114xsamples);
                }
            }
        } // if( error == 0 )
    } // for( device=0; device < 2; device++ )
}

