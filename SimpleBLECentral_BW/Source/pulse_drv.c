#include "bcomdef.h"
#include "OSAL.h"
#include "hal_mcu.h"
#include "hal_led.h"
#include "i2c.h"
#include "pulse_drv.h"
#include "uart_app.h"
#include "hal_adc.h"
#include "simpleBLECentral.h"

extern void bleApp_DumpPulseDataReq(uint8 *pDat, uint8 len);

#define PLUSE_ADC_CHN         HAL_ADC_CHANNEL_6
static int16 g_vpp = 0;
static int16 g_dc = 0;
int16 g_prec = 0;
static int16 g_adMax = 0;
static int16 g_adMin = 4095;

static uint8 g_badFlag = FALSE;
int16 g_pluseCnt = 0;
uint16 g_PulseTime = 0;
static uint16 g_InvalCnt=0;		//记录时间间隔数

static uint16 g_pulseState;
static int16 g_highTh,g_lowTh;
static int16 g_preState, g_currState, g_oldData, g_newData;
uint8 g_pulseStartFlag = FALSE;
uint16 g_pulseValue = 0;
static uint16 g_pulseMean = 0;
static uint8 g_validDataCnt = 0;
uint16 g_pulseReportNum = 0;
uint8 g_firstMeasureFlag = TRUE;
static uint16 g_reportLen;

static uint16 g_sampling = 0;

uint16 g_timeRec[REPORT_LEN];
uint16 g_IntvRec[REPORT_LEN];


//启动读脉搏
void pulse_startRead(void)
{
   g_pulseStartFlag = TRUE;
   osal_start_timerEx( simpleBLETaskId, BLE_PERIOD_PULSE_EVT, BLE_PLUSE_TIMER_5MS_CNT );

   return;

}

//上报脉搏数
void pluse_report(void)
{
  static int32 reportFilter = (int32)75<<16;
  
  if((g_pulseValue < MIN_PULSE_REPORT_VAL) || (g_pulseValue > MAX_PULSE_REPORT_VAL))
  {
	   g_pulseReportNum = 0;
  }
  else
  {
	
	if(g_firstMeasureFlag)
	{
	    g_firstMeasureFlag = FALSE;
	    g_pulseReportNum = g_pulseValue;
	    reportFilter = (int32)g_pulseReportNum<<16; //滤波赋初值
	}
	else
	{
      
	    //基于前一次滤波
	  reportFilter = FILTER(g_pulseValue, reportFilter, 1);
	  g_pulseReportNum = LIMIT((reportFilter>>16), MIN_PULSE_REPORT_VAL, MAX_PULSE_REPORT_VAL);
	}
	
    if(!LICENSE_IS_GOOD())
    {
        g_pulseReportNum = 1;
    }
  }

  //g_pulseReportNum = 60;
  

  uart_send_pulse(g_pulseReportNum);
}




//结束上报
void pulse_endRead(void)
{
    //led off
    HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF); 
    g_pulseStartFlag = FALSE;

    g_vpp = 0;
    g_dc = 0;
    g_prec = 0;
    g_adMax = 0;
    g_adMin = 4095;
    g_highTh = 0;
    g_lowTh = 0;
    
    g_badFlag = FALSE;
    g_pluseCnt = 0;
    g_PulseTime = 0;
    g_InvalCnt=0;	   //记录时间间隔数
    
    g_pulseState = PULSE_BEGIN;
   
    g_preState = PULSE_LOW;
    g_currState = PULSE_LOW;
    g_oldData = 0;
    g_newData = 0;
    g_pulseValue = 0;
    g_validDataCnt = 0;
    g_pulseReportNum = 0;
    g_firstMeasureFlag = TRUE;
    
    g_sampling = 0;
    
    memset(&g_timeRec[0], 0, sizeof(g_timeRec));
    memset(&g_IntvRec[0], 0, sizeof(g_IntvRec));

    return;
}


void pulse_UpdatePara(int16 adc)
{
    //计算最大和最小值
    g_adMax = MAX(adc, g_adMax);
    g_adMin = MIN(adc, g_adMin);
	g_reportLen = (TRUE == g_firstMeasureFlag) ? REPORT_LEN : (REPORT_LEN2);

    g_sampling += 1;
    if(g_sampling > PULSE_PARA_UPDATE_PERIOD)
    {
        g_sampling = 0;
		g_vpp = g_adMax - g_adMin;
		g_dc = (g_adMax + g_adMin)/2;
		g_highTh = g_dc + (((g_adMax - g_dc) *1) >> 4);
		g_lowTh = g_dc - ((g_dc - g_adMin) >> 1);		 

		g_badFlag = FALSE;	
		
		//vpp参?
	    if (g_vpp >= MIN_PULSE_VPP)
		{
		  g_prec = (g_vpp*11)>>4;
		} 
		else 
		{ 
		  g_prec = 4095;
		  g_badFlag = TRUE;
		}

        /*按的很紧*/ 
		if(g_adMin > 350)
		{
		    g_badFlag = TRUE;
		}

	    g_adMax = -4096;
		g_adMin = 4095;
    }

	return;
}
void pulse_CntJudge(uint16 adAvg)
{
   uint8 findFlagTmp;
   uint16 index, indexPre;
   
   if(adAvg > g_highTh)
   {
	   g_preState = g_currState;
	   g_currState = PULSE_HIGH;
	   /*从一个状态切换到另外一个状态，则寄存器值移位*/ 
	   if(g_preState != g_currState)
	   {
		   g_oldData= g_newData;
	   }
	   
	   /*记录正最大值*/
	   if(adAvg > g_newData)
	   {
		   g_newData = adAvg;
	   }
   }
   else if(adAvg < g_lowTh)
   {
	   g_preState = g_currState;
	   g_currState = PULSE_LOW;

	   /*从一个状态切换到另外一个状态，则寄存器值移位*/ 
	   if(g_preState!= g_currState)
	   {
		   g_oldData= g_newData;
	   }
	   
	   /*记录负最大值*/
	   if(adAvg < g_newData)
	   {
		   g_newData = adAvg;
	   }
	}
			 
	findFlagTmp= FALSE;
	if((g_oldData - g_newData)> g_prec)
	{
		findFlagTmp = TRUE;
		g_oldData =  g_newData;
	} 


   if(( findFlagTmp == TRUE)&&(g_badFlag == FALSE))
   {
		//findFlag = TimeWindow();
		
		index = g_pluseCnt % g_reportLen;
		indexPre = (index + g_reportLen - 1) % g_reportLen;

		g_timeRec[index] = g_PulseTime;
		g_IntvRec[index] = g_timeRec[index] - g_timeRec[indexPre];
		g_InvalCnt = 0;
		
		g_pluseCnt ++;

   }
 
   else 
   {
	   g_InvalCnt ++;  
   }


   return;
}

void pulse_stateTran()
{
	switch(g_pulseState)
	{
        /*开始检测脉搏*/ 
	    case PULSE_BEGIN:
	  	    if(g_pluseCnt > MIN_PULSE_FIND_CNT)
	        {
				g_pulseState = PULSE_RUN;
	    	}

	  		break;

        /*正常脉搏计数*/ 
	  	case PULSE_RUN:
		  	if(g_InvalCnt >= MAX_PULSE_BREAK_INTERVAL1)
		  	{
				g_pulseState = PULSE_PAUSE;
			  }

		  	break;
	  	/*暂停脉搏计数*/
		case PULSE_PAUSE:
	  
			if(0 == g_InvalCnt)
			{
				g_pulseState = PULSE_RUN;
			}
			
	  		break;
	  	default:
	  	    break;
	}

	return;
}

void pulse_stateProc()
{
	uint8 i;
	uint16 pulseValTmp;
	
    switch(g_pulseState)
	{
		 case PULSE_BEGIN:

			 break;
		 case PULSE_RUN:
			 //连续若干次检测到脉搏，则上报结果
			
			 if(g_reportLen == g_pluseCnt)
			 {
 				 /*60表示60s,100是1/10ms； */
				 g_validDataCnt = 0;
				 pulseValTmp = 0;
				 
				 for(i = 0; i < g_reportLen; i ++)
				 {
				    /*连续两次的时间间隔合理性判断*/
				 	if((g_IntvRec[i] < MAX_PULSE_INTV)&&(g_IntvRec[i] > MIN_PULSE_INTV))
				 	{
				 	    pulseValTmp += g_IntvRec[i];
				 	    g_validDataCnt ++;
				 	}
				 }
                 g_pulseMean = pulseValTmp/g_validDataCnt;
                 
				 for(i = 0; i < g_reportLen; i ++)
				 {
				    /*连续两次的时间间隔合理性判断*/
				    
				 	if((g_IntvRec[i] < MAX_PULSE_INTV)&&(g_IntvRec[i] > MIN_PULSE_INTV))
					{
					 	if((g_IntvRec[i] < ((g_pulseMean >> 13) >> 4))||(g_IntvRec[i] > ((g_pulseMean * 19) >> 4)))
					 	{
					 	    pulseValTmp -= g_IntvRec[i];
					 	    g_validDataCnt --;
					 	}
				 	}
				 }
				 
				 if((g_validDataCnt != 0)&&(g_validDataCnt >= ((g_reportLen * 1>> 2))))
				 {
				    g_pulseValue = (60 * (1000/INTR_INTERVAL) * g_validDataCnt/pulseValTmp);
					pluse_report();
    				g_InvalCnt = 0;

				 }
				 else
				 {
				 	//g_pulseValue = 0;
				 }
	
				 // 复位计数器
				 g_pluseCnt = 0;
	
	  	 }
	  	   
			 break;
		 case PULSE_PAUSE:
		     
		    if(g_InvalCnt >= MAX_PULSE_BREAK_INTERVAL2)
		 	{
			   //如果超过一定时间间隔仍然没有找到下一个脉搏，则重新开始计数
      		    if(g_PulseTime & (REPORT_PERIOD - 1))
			     {
			         g_pulseValue = 0;
			         pluse_report();
			     }	
			     
			    g_pluseCnt = 0;
			    g_firstMeasureFlag = TRUE;

                //防止溢出
			    g_InvalCnt = MAX(20000,g_InvalCnt);
		     }
			 break;
		 default:
			 break;
	 }

	return;
}
//脉搏更新算法
void pulse_update(void)
{
    static int16 loops = 0;
    static int32 adFilter = 0;
    static int16 adc, adcPre;
    static int16 adAvg;

    g_PulseTime ++; 
    //检测是否启动
    if( g_pulseStartFlag != TRUE ) 
    {
        loops = 0;
        g_sampling = 0;
        return;
    }
    
    //小于1.2内等待闪灯
    loops ++;
    if(loops <= 120) //12s
    {
        //led flash
	    if(loops == 1) 
	    {
	       HalSetFlashPeriod(250);
	       HalLedSet( HAL_LED_1, HAL_LED_MODE_FLASH); 
	    }  
        
	    if(loops == 120)
	    {
	        //启动脉搏测量，要求led常亮
	       HalLedSet( HAL_LED_1, HAL_LED_MODE_ON); 
	    }

        return;
    }
    else
    {
        loops = 120;
    }
    
    //采样
    adcPre = adc;
    adc = HalAdcRead (PLUSE_ADC_CHN, HAL_ADC_RESOLUTION_12);
 
	

    adFilter = FILTER(adc, adFilter, 2);
    adAvg = LIMIT((adFilter>>16), 0, 4095);
    bleApp_DumpPulseDataReq((uint8 *)&adc, sizeof(adc));


    pulse_UpdatePara(adAvg);

    /*异常跳变判断*/
    if(ABS(adc - adcPre) > MAX_JUMP_TH)
    {
    	 g_badFlag = TRUE;
    }
    /////////////////////

    pulse_CntJudge(adAvg);
    
    pulse_stateTran();
    pulse_stateProc();

    
	return;  
}
