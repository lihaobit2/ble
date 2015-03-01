#include "Si114x_functions.h"
#include "Si114x_debug.h"


// Function Prototypes
void IRCompensation(
        u8 proxChannel,   // 0 - PS1 (using LED1)
                          // 1 - PS2 (using LED2)
                          // 2 - PS3 (using LED3)
        SI114X_IRQ_SAMPLE *samples, // raw samples
                                // PSx overwritten
        u8 ircorrection[] //  IR Correction Factors:
                          //  Lg PD, PSRANGE = 0 
                          //    use { 17, 35, 29 }
                          //  Lg PD, PSRANGE = 1 
                          //    use { 11, 33, 33 }
                          //  Sm PD, PSRANGE = 1 
                          //    use { 23, 64, 56 }
                          );
    
char isIRStable( SI114X_IRQ_SAMPLE *samples);

void ProcessStableSamples(HANDLE si114x_handle, SI114X_IRQ_SAMPLE *samples);



void ProcessSi114xSamples(HANDLE si114x_handle, SI114X_IRQ_SAMPLE *samples)
{
#ifdef GENERAL_CLEAR_OVERLAY
    u8 code ircorrection[3] =  { 23, 64, 56 };  
#endif

#ifdef GENERAL_INFRARED_OVERLAY
    u8 code ircorrection[3] =  { 11, 33, 33 };  
#endif

#ifdef INDOORS
    u8 code ircorrection[3] =  { 17, 35, 29 };  
#endif

    IRCompensation( 0, samples, ircorrection );  // IR Ambient Compensation for PS1 Channel
    IRCompensation( 1, samples, ircorrection );  // IR Ambient Compensation for PS2 Channel
    IRCompensation( 2, samples, ircorrection );  // IR Ambient Compensation for PS2 Channel

    // UDPWaveformViewer Debug
    REPLACE_0_PS1( samples->ps1 );
    REPLACE_0_PS2( samples->ps2 );
    REPLACE_0_PS3( samples->ps3 );

    if( isIRStable( samples ) )
    {
        ProcessStableSamples(si114x_handle, samples);
    }
}

char isIRStable( SI114X_IRQ_SAMPLE *samples)
{
    static u32 xdata IRlast = 0;              // Contains the ALS IR reading from 
                                        //     the previous measurement cycle
                                        //
   // u32 xdata IR = (u32) samples->ir; // Make a copy of ALS IR Reading
    
    // Initialize IRlast to the ALS IR reading if it does not contain 
    // information from a previous measurement cycle
    if( IRlast == 0) IRlast = samples->ir; 

    // If ALS IR is saturated, return false immediately
    else if( samples->ir > 48000 ) return 0; 

    if( samples->ir < (IRlast + IRlast/2) && samples->ir > (IRlast - IRlast/2))
    {
        // If IR is within 3 dB, declare it stable
        // and perform some averaging
        IRlast = (samples->ir + (IRlast*3)) / 4;
        return 1;
    }
    else
    {
        // Otherwise, return false and update
        // new IRlast value for next time
        IRlast = samples->ir;
        return 0;
    }
    //return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Function: IRCompensation
//
// This function will correct each IR proximity channel based on ALS IR
//
// Inputs:
//    proxChannel - indicates to function which proxChannel to compensate
//          0 - PS1 (using LED1)
//          1 - PS2 (using LED2)
//          2 - PS3 (using LED3)
//    samples - raw samples
//    ircorrection - IR Correction Factors:
//          Lg PD, PSRANGE = 0 
//             use { 17, 35, 29 }
//          Lg PD, PSRANGE = 1 
//             use { 11, 33, 33 }
//          Sm PD, PSRANGE = 1 
//             use { 23, 64, 56 }
////////////////////////////////////////////////////////////////////////////////
void IRCompensation(u8 proxChannel, SI114X_IRQ_SAMPLE *samples, u8 ircorrection[])
{
    u32 xdata Correction;             // IR Correction factor

    // Calculate correction factor based on the ALS IR + PS reading 
    // subtracting out the 256 count offset contained in both measurements
    // Full equation comes out to:
    // Correction = ( (ALS_IR - 256) + (PS - 256) ) * IRcorrectionfactor
    //  where IRcorrectionfactor = IRCORRECTION / 10000    
    switch(proxChannel)
    {
        case 0:
            Correction = ((u32)samples->ir + (u32)(samples->ps1) - 512) * (u32) ircorrection[proxChannel];
            Correction = Correction/10000;
            samples->ps1 = samples->ps1 + Correction;    
            break;
        case 1:
            Correction = ((u32)samples->ir + (u32)(samples->ps2) - 512) * (u32) ircorrection[proxChannel];
            Correction = Correction/10000;
            samples->ps2 = samples->ps2 + Correction;            
            break;
        case 2:
            Correction = ((u32)samples->ir + (u32)(samples->ps3) - 512) * (u32) ircorrection[proxChannel];
            Correction = Correction/10000;
            samples->ps3 = samples->ps3 + Correction;            
            break;
        default:
            break;
    }
}


void ProcessStableSamples(HANDLE si114x_handle, SI114X_IRQ_SAMPLE *samples)
{
    u16 xdata swipe_speed;
    u16 xdata ps[3];
    static u16 xdata ps_max[3]        = {0,0,0};
    static u16 xdata ps_entry_time[3] = {0,0,0};
    static u16 xdata ps_exit_time[3]  = {0,0,0};
    static u16 xdata ps_max_time[3]   = {0,0,0};
    static u8  xdata ps_state[3]      = {0,0,0};

#ifdef GENERAL_CLEAR_OVERLAY
    const  u16 code ps_threshold[3]  = {273, 271, 273};
#endif

#ifdef GENERAL_INFRARED_OVERLAY
    const  u16 code ps_threshold[3]  = {300, 295, 300};
#endif

#ifdef INDOORS
    const  u16 code ps_threshold[3]  = {700, 600, 700};
#endif
    u8 xdata array_counter = 0;
    u16 xdata diff_x = 0;
    u16 xdata diff_y = 0;
    u16 xdata ps_time[3] = {0,0,0};
    u32 xdata ps_avg;
    
    // Save new samples into ps array
    ps[0] = samples->ps1;
    ps[1] = samples->ps2;
    ps[2] = samples->ps3;

    // Check state of all three measurements
    for ( array_counter = 0; array_counter < 3; array_counter++)
    {
        // If measurement higher than the ps_threshold value, 
        //   record the time of entry and change the state to look for the exit time
        if ( ps[array_counter] >= ps_threshold[array_counter] )
        {
            if ( ps_state[array_counter] == 0 )
            {
                ps_state[array_counter] = 1;
                ps_entry_time[array_counter] = samples->sequence;
            }
            if ( ps[array_counter] > ps_max[array_counter] )
            {         
                ps_max[array_counter] = ps[array_counter];
                ps_max_time[array_counter] = samples->sequence;
            }
        } else 
        {
            if ( ps_state[array_counter] == 1 )
            {
                ps_state[array_counter] = 0;
                ps_exit_time[array_counter] = samples->sequence;
            }
        }
    }

    // If there is no object in front of the board, look at history to see if a gesture occured
    if ( (ps[0] < ps_threshold[0]) && (ps[1] < ps_threshold[1]) && (ps[2] < ps_threshold[2]) )
    {
        // If the ps_max values are high enough and there exit entry and exit times,
        //   then begin processing gestures
        if( (ps_entry_time[0] != 0) && (ps_entry_time[1] != 0) && (ps_entry_time[2] != 0)
            && (ps_exit_time[0] != 0) && (ps_exit_time[1] != 0) && (ps_exit_time[2] != 0))
        {
            // Make sure no timestamps overflowed, indicated possibility if any of them are close to overflowing
            if( (ps_exit_time[0] > 0xFC00) || (ps_exit_time[1] > 0xFC00) || (ps_exit_time[2] > 0xFC00) 
                || (ps_entry_time[0] > 0xFC00) || (ps_entry_time[1] > 0xFC00) || (ps_entry_time[2] > 0xFC00) )
            {   // If any of them are close to overflowing, overflow them all so they all have the same reference
                ps_exit_time[0] += 0x1FFF;
                ps_exit_time[1] += 0x1FFF;
                ps_exit_time[2] += 0x1FFF;

                ps_entry_time[0] += 0x1FFF;
                ps_entry_time[1] += 0x1FFF;
                ps_entry_time[2] += 0x1FFF;
            }

            // Calculate the midpoint (between entry and exit times) of each waveform
            //  the order of these midpoints helps determine the gesture
            ps_time[0] = (ps_exit_time[0] - ps_entry_time[0]) / 2;
            ps_time[0] = ps_time[0] + ps_entry_time[0];

            ps_time[1] = (ps_exit_time[1] - ps_entry_time[1]) / 2;
            ps_time[1] = ps_time[1] + ps_entry_time[1];

            ps_time[2] = (ps_exit_time[2] - ps_entry_time[2]) / 2;
            ps_time[2] = ps_time[2] + ps_entry_time[2];

            // The diff_x and diff_y values help determine a gesture by comparing the
            //  LED measurements that are on a single axis
            if( ps_time[0] > ps_time[2] )
            {
                diff_x = ps_time[0] - ps_time[2];
            } else 
            {
                diff_x = ps_time[2] - ps_time[0];
            }

            if( ps_time[0] > ps_time[1] )
            {
                diff_y = ps_time[0] - ps_time[1];
            } else 
            {
                diff_y = ps_time[1] - ps_time[0];
            }

            // Take the average of all three midpoints to make a comparison point for each midpoint
            ps_avg = (u32)ps_time[0] + (u32)ps_time[1] + (u32)ps_time[2];
            ps_avg = ps_avg / 3;

            if( (ps_exit_time[0]-ps_entry_time[0]) > 10 || (ps_exit_time[1]-ps_entry_time[1]) > 10 || (ps_exit_time[2] - ps_entry_time[2])> 10 )
            {
                // If the midpoints are all very close together, a tap gesture occured
                if( (ps_time[0] < (ps_avg+2)) && (ps_time[0] > (ps_avg-2))
                    && (ps_time[1] < (ps_avg+2)) && (ps_time[1] > (ps_avg-2))
                    && (ps_time[2] < (ps_avg+2)) && (ps_time[2] > (ps_avg-2)))
                {
                    swipe_speed = ps_exit_time[0] - ps_entry_time[1];
                    printf("tap \n");
                    printf("1 %04x, 2 %04x, 3 %04x, speed %04x\n", ps_time[0], ps_time[1], ps_time[2], swipe_speed);
                } else if( (ps_time[1] < ps_time[0]) && (ps_time[1] < ps_time[2]) && (diff_y > diff_x))
                {   // An up gesture occured if the bottom LED had its midpoint first
                    swipe_speed = (ps_time[0] - ps_time[1]);
                    printf("swipe up \n");
                    printf("1 %04x, 2 %04x, 3 %04x, speed %04x\n", ps_time[0], ps_time[1], ps_time[2], swipe_speed);
                } else if( (ps_time[1] > ps_time[0]) && (ps_time[1] > ps_time[2]) && (diff_y > diff_x))
                {   // A down gesture occured if the bottom LED had its midpoint last
                    swipe_speed = (ps_time[1] - ps_time[0]);
                    printf("swipe down \n");
                    printf("1 %04x, 2 %04x, 3 %04x, speed %04x\n", ps_time[0], ps_time[1], ps_time[2], swipe_speed);
                } else if((ps_time[0] < ps_time[2]) && (ps_time[1] < ps_time[2]) && (diff_x > diff_y))
                {  // A right gesture occured if the right LED had its midpoint last
                   swipe_speed = (ps_time[2] - ps_time[0]);
                   printf("swipe right \n");
                   printf("1 %04x, 2 %04x, 3 %04x, speed %04x\n", ps_time[0], ps_time[1], ps_time[2], swipe_speed);
                } else if( (ps_time[2] < ps_time[1]) && (ps_time[2] < ps_time[0])  && (diff_x > diff_y))
                {   // A left gesture occured if the left two LEDs had their midpoints later than the right LED
                    swipe_speed = (ps_time[0] - ps_time[2]);
                    printf("swipe left \n");
                    printf("1 %04x, 2 %04x, 3 %04x, speed %04x\n", ps_time[0], ps_time[1], ps_time[2], swipe_speed);
                }
            }
        }
        for ( array_counter = 0; array_counter < 3; array_counter++)
        {
            ps_max[array_counter] = 0;
            ps_exit_time[array_counter] = 0;
            ps_entry_time[array_counter] = 0;
            ps_max_time[array_counter] = 0;
        }

    }

    // Add the following line back into the code for debugging output
    /*printf("seq %04x,ts %04x,ps1 %04x,ps2 %04x,ps3 %04x, max1 %04x, max2 %04x, max3 %04x\n", 
            samples->sequence, 
            samples->timestamp, 
            samples->ps1, 
            samples->ps2, 
            samples->ps3,
            ps_time[0],
            ps_time[1],
            ps_time[2]
          );*/

}

