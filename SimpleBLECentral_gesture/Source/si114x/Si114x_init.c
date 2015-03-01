#include "Si114x_functions.h"

s16 Si114xConfigure(HANDLE si114x_handle, s16 *current, s16 tasklist, s16 measrate, s16 adcgain, s16 adcmisc, s16 *adcmux);

//
// standardize on this function to more easily hook up to main routine
//
s16 si114x_init(HANDLE si114x_handle)
{
    s16 code current[3]= {0x0f, 0x0f, 0x0f};    // 359 mA, 359 mA, 359 mA
    //s16 code current[3]= {0, 0, 0};    // 359 mA, 359 mA, 359 mA
    s16 code tasklist=0x37;    // VIS, IR, PS1, PS2, PS3
    s16 code measrate=0x84;    // 0xa0 every 30.0 ms
                          // 0x94 every 20.0 ms
                          // 0x84 every 10.4 ms
                          // 0x74 every  5.2 ms
                          // 0x70 every  4.2 ms
                          // 0x60 every  3.2 ms
    s16 code adcgain= 0x00;    // PS_ADC_GAIN to default div by 1

#ifdef GENERAL_CLEAR_OVERLAY
    s16 code adcmisc= 0x24;    // PS_ADC_MISC to high signal range
    s16 code adcmux[3]=  {0x00, 0x00, 0x00};    // PS1_ADCMUX, PS2_ADCMUX, PS3_ADCMUX to small photodiode
#endif

#ifdef GENERAL_INFRARED_OVERLAY
    s16 code adcmisc= 0x24;    // PS_ADC_MISC to high signal range
    s16 code adcmux[3]=  {0x03, 0x03, 0x03};    // PS1_ADCMUX, PS2_ADCMUX, PS3_ADCMUX to large photodiode
#endif

#ifdef INDOORS
    //s16 code adcmisc= 0x04;    // PS_ADC_MISC to default signal range
   //s16 code adcmux[3]=  {0x03, 0x03, 0x03};    // PS1_ADCMUX, PS2_ADCMUX, PS3_ADCMUX to default large photodiode
	s16 code adcmisc= 0x4;    // PS_ADC_MISC to default signal range
	s16 code adcmux[3]=  {0x03, 0x03, 0x03};	// PS1_ADCMUX, PS2_ADCMUX, PS3_ADCMUX to default large photodiode
   
#endif
	
    return Si114xConfigure(si114x_handle, current, tasklist, measrate, adcgain, adcmisc, adcmux);
}


s16 Si114xConfigure(HANDLE si114x_handle, s16 *current, s16 tasklist, s16 measrate, s16 adcgain, s16 adcmisc, s16 *adcmux)
{ 
   s16  retval=0;
   s16 tmp;

   // Note that the Si114xReset() actually performs the following functions:
   //     1. Pauses all prior measurements
   //     2. Clear  i2c registers that need to be cleared
   //     3. Clears irq status to make sure INT* is negated
   //     4. Delays 10 ms
   //     5. Sends HW Key
   retval+=Si114xReset(si114x_handle);
   NPI_WriteStrValue("====reset ret read:%d,\r\n",retval);   

   // Get the LED current passed from the caller
   // If the value passed is negative, just pick a current... 202 mA
   {
       u8 i21, i3; 
       if ( current<0 ) 
       {
           i21 = 0xbb; // current = LEDI_202;
           i3  = 0x0b; 
       }
       else
       {
           i21 = (current[1]<<4) + current[0];
           i3  = current[2];
       }
       retval+=Si114xWriteToRegister(si114x_handle, REG_PS_LED21, i21);
       retval+=Si114xWriteToRegister(si114x_handle, REG_PS_LED3 , i3);
   }

   // Initialize CHLIST Parameter from caller to enable measurement
   // Valid Tasks are: ALS_VIS_TASK, ALS_IR_TASK, PS1_TASK
   //                  PS2_TASK, PS3_TASK and AUX_TASK
   // However, if we are passed a 'negative' task, we will
   // turn on ALS_IR, ALS_VIS and PS1. Otherwise, we will use the 
   // task list specified by the caller.
   if (tasklist < 0 ) tasklist = ALS_IR_TASK + ALS_VIS_TASK + PS1_TASK;
   retval+=Si114xParamSet(si114x_handle, PARAM_CH_LIST, tasklist);
   #if 1
   tmp = Si114xParamRead(si114x_handle, PARAM_CH_LIST);  
   NPI_WriteStrValue("====1ram read:%d,\r\n",tmp);   
    NPI_WriteStrValue("====tasklist:%d,\r\n",tasklist);  
    #endif
   // Set IRQ Modes and INT CFG to interrupt on every sample
   retval+=Si114xWriteToRegister(si114x_handle, REG_INT_CFG, ICG_INTOE);  
   retval+=Si114xWriteToRegister(si114x_handle, REG_IRQ_ENABLE, 
         IE_ALS_EVRYSAMPLE + 
         IE_PS1_EVRYSAMPLE + 
         IE_PS2_EVRYSAMPLE + 
         IE_PS3_EVRYSAMPLE );  

   retval+=Si114xWriteToRegister(si114x_handle, REG_IRQ_MODE1, 
         IM1_ALS_EVRYSAMPLE + 
         IM1_PS1_EVRYSAMPLE + 
         IM1_PS2_EVRYSAMPLE );

   retval+=Si114xWriteToRegister(si114x_handle, REG_IRQ_MODE2, 
         IM2_PS3_EVRYSAMPLE);

   retval += Si114xParamSet(si114x_handle, PARAM_PS_ADC_GAIN, adcgain);
   retval += Si114xParamSet(si114x_handle, PARAM_PS_ADC_MISC, adcmisc);
   retval += Si114xParamSet(si114x_handle, PARAM_PS1_ADC_MUX, adcmux[0]);
   retval += Si114xParamSet(si114x_handle, PARAM_PS2_ADC_MUX, adcmux[1]);
   retval += Si114xParamSet(si114x_handle, PARAM_PS3_ADC_MUX, adcmux[2]);
#if 1
   tmp = Si114xParamRead(si114x_handle, PARAM_PS_ADC_MISC);  
   NPI_WriteStrValue("====2ram read:%d,\r\n",tmp);   
    NPI_WriteStrValue("====tasklist:%d,\r\n",tasklist);  
 #endif

   // Configure the ALS IR channel for the same settings as PS
   retval += Si114xParamSet(si114x_handle, PARAM_ALSIR_ADC_GAIN, adcgain);
   retval += Si114xParamSet(si114x_handle, PARAM_ALSIR_ADC_MISC, adcmisc &0x20);
   retval += Si114xParamSet(si114x_handle, PARAM_IR_ADC_MUX, adcmux[0]);

   // If the caller passes a negative measrate, it means that it does not
   // want to start autonomous measurements.
   if( measrate > 0 )
   {
       // Set up how often the device wakes up to make measurements
       // measrate, for example can be the following values:
       //    0xa0 = Device Wakes up every ~30 ms
       //    0x94 = Device Wakes up every ~20 ms
       //    0x84 = Device Wakes up every ~10 ms
       //    0xB9 = Device Wakes up every ~100 ms
       //    0xFF = Device Wakes up every ~2 sec
       retval+=Si114xWriteToRegister(si114x_handle, REG_MEAS_RATE, measrate);

       // if 0x08, PS1, PS2 and PS3 made every time device wakes up.
       retval+=Si114xWriteToRegister(si114x_handle, REG_PS_RATE,   0x08);

       // if 0x08, VIS, IR, AUX Measurements every time device wakes up.
       retval+=Si114xWriteToRegister(si114x_handle, REG_ALS_RATE,  0x08);

       // Enable Autonomous Operation
       retval+=Si114xPsAlsAuto(si114x_handle);
   
       // If nothing went wrong after all of this time, the value
       // returned will be 0. Otherwise, it will be some negative
       // number
   }
   return retval;
}
u8 values[32];
extern u8 astr[];

s16 si114x_disPlayCfg(HANDLE si114x_handle)
{
	s16 retval, i, j;

	for(i = 0; i <= 0x3e; i += 12)
	{
		retval = Si114xBlockRead(si114x_handle, i, 12, values);
		sprintf(astr, "1 reg of %d to %d:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\r\n",\
	            i, i + 11, values[0],values[1],\
			    values[2],values[3],values[4],\
	    		values[5],values[7],values[7],\
	    		values[8],values[9],values[10],\
	    		values[11]);
		NPI_WriteString(astr);	
	}
	NPI_WriteString("\r\n");	

	for(i = 0; i <= 0x1f; i += 12)
	{
		for(j = 0; j < 12;j ++)
		{
			values[j] = Si114xParamRead(si114x_handle, i+j);
		}
		sprintf(astr, "2 ram value of %d to %d:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\r\n",\
	            i, i + 11, values[0],values[1],\
			    values[2],values[3],values[4],\
	    		values[5],values[7],values[7],\
	    		values[8],values[9],values[10],\
	    		values[11]);		
		NPI_WriteString(astr);	

	}


	return retval;
}

