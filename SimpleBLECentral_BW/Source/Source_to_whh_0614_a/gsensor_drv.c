#include <string.h>
#include "bcomdef.h"
#include "OnBoard.h"
#include "i2c.h"
#include "gsensor_drv.h"
#include "mma8452q.h"
#include "uart_app.h"
#include "hal_uart.h"
#include "simpleBLECentral.h"
#include "simpleBLEApp.h"

/*----------------------------------------------------------------------------*/

#define BT4_SYNC1 (0)
#define BT4_SYNC2 (1)
#define BT4_CMD   (2)
#define BT4_DATA1 (3)
#define BT4_DATA2 (4)
#define BT4_DATA3 (5)
#define BT4_DATA4 (6)
#define BT4_DATA5 (7)
#define BT4_DATA6 (8)

#define BT4_DATA7 (9)
#define BT4_DATA8 (10)
#define BT4_DATA9 (11)
#define BT4_DATA10 (12)

struct scale_factor{
    uint8  whole;
    uint8  fraction;
};

struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};

static struct data_resolution mma8452q_data_resolution[] = {
 /*8 combination by {FULL_RES,RANGE}*/
    {{ 1, 0}, 1024},   /*+/-2g  in 12-bit resolution:  3.9 mg/LSB*/
    {{ 2, 0}, 512},   /*+/-4g  in 12-bit resolution:  7.8 mg/LSB*/
    {{3, 9},  256},   /*+/-8g  in 12-bit resolution: 15.6 mg/LSB*/
    {{ 15, 6}, 64},   /*+/-2g  in 8-bit resolution:  3.9 mg/LSB (full-resolution)*/
    {{ 31, 3}, 32},   /*+/-4g  in 8-bit resolution:  3.9 mg/LSB (full-resolution)*/
    {{ 62, 5}, 16},   /*+/-8g  in 8-bit resolution:  3.9 mg/LSB (full-resolution)*/            
};

/*----------------------------------------------------------------------------*/

static int MMA8452Q_CheckDeviceID()
{
	uint8 databuf[10];    
	int res = 0;

	osal_memset(databuf, 0, sizeof(uint8)*10);    
	databuf[0] = MMA8452Q_REG_DEVID;    

	res = i2c_read(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_DEVID, databuf, 1); 
	 
	if(databuf[0]!=MMA8452Q_FIXED_DEVID)
	{
		return MMA8452Q_ERR_IDENTIFICATION;
	}

	if (res)
	{
		return MMA8452Q_ERR_I2C;
	}
	
	return MMA8452Q_SUCCESS;
}

static bool sensor_power = false;
static int MMA8452Q_SetPowerMode( bool enable)
{
	uint8 databuf[2];    
	int res = 0;
	uint8 addr = MMA8452Q_REG_CTL_REG1;

	if(enable == sensor_power)
	{
		//SerialPrintString("Sensor power status need not to be set again!!!\n");
		return MMA8452Q_SUCCESS;
	}

	res = i2c_read(MMA8452Q_I2C_SLAVE_ADDR, addr, databuf, 1); 
	if(res) {
		//SerialPrintString("read power ctl register err!\n");
		return MMA8452Q_ERR_I2C;
	}

	databuf[0] &= ~MMA8452Q_MEASURE_MODE;
	if(enable == true) {
		databuf[0] |= MMA8452Q_MEASURE_MODE;
	}

	res = i2c_write(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_CTL_REG1, databuf, 1); 
	if(res)
	{
		//SerialPrintString("fwq set power mode failed!\n");
		return MMA8452Q_ERR_I2C;
	}

	//SerialPrintString("fwq set power mode ok!\n");
	
	sensor_power = enable;
	return MMA8452Q_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
//set detect range

static int MMA8452Q_SetDataFormat(uint8 dataformat)
{
	uint8 databuf[10];    
	int res = 0;

	osal_memset(databuf, 0, sizeof(uint8)*10);     
	databuf[0] = dataformat;

	res = i2c_write(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_XYZ_DATA_CFG, databuf, 1); 
	if(res) {
		return MMA8452Q_ERR_I2C;
	}

	return MMA8452Q_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int MMA8452Q_SetBWRate(uint8 bwrate)
{
	uint8 databuf[10];    
	int res = 0;

	osal_memset(databuf, 0, sizeof(uint8)*10);    
	databuf[0] = MMA8452Q_REG_CTL_REG1;    
	//databuf[1] = bwrate;

	res = i2c_read(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_CTL_REG1, databuf, 1); 
	if( res ) {
		//SerialPrintString("read power ctl register err!\n");
		return MMA8452Q_ERR_I2C;
	}
	//SerialPrintString("fwq read MMA8452Q_REG_CTL_REG1\n");

	databuf[0] &=0xC7;//clear original  data rate 	
	databuf[0] |= bwrate; //set data rate

 	res = i2c_write(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_CTL_REG1, databuf, 1); 
	if(res) {
		return MMA8452Q_ERR_I2C;
	}
	
	return MMA8452Q_SUCCESS;    
}

/*----------------------------------------------------------------------------*/

static int MMA8452Q_SetDataResolution(uint8 dataresolution)
{
	uint8  dat, reso;
    uint8 databuf[10];    
    int res = 0;
	
	res = i2c_read(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_CTL_REG2, databuf, 1); 
	if(res) {
		//SerialPrintString("read power ctl register err!\n");
		return -1;
	}
	
	if(dataresolution == MMA8452Q_12BIT_RES){
		databuf[0] |= MMA8452Q_12BIT_RES;
	}else{
		databuf[0] &= (~MMA8452Q_12BIT_RES);//8 bit resolution
	}
	
	res = i2c_write(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_CTL_REG2, databuf, 1); 
	if(res){
		//SerialPrintString("set resolution  failed!\n");
		return -1;
	}else{
		//SerialPrintString("set resolution mode ok %x!\n", databuf[1]);
	}
	
    //choose sensitivity depend on resolution and detect range
	//read detect range
	res = i2c_read(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_XYZ_DATA_CFG, &dat, 1); 
	if(res) {
		//SerialPrintString("read detect range  fail!!\n");
		return res;
	}
	reso  = (dataresolution & MMA8452Q_12BIT_RES) ? (0x00) : (0x03);
	
	
	if(dat & MMA8452Q_RANGE_2G)
	{
		reso = reso + MMA8452Q_RANGE_2G;
	}
	if(dat & MMA8452Q_RANGE_4G)
	{
		reso = reso + MMA8452Q_RANGE_4G;
	}
	if(dat & MMA8452Q_RANGE_8G)
	{
		reso = reso + MMA8452Q_RANGE_8G;
	}

	if(reso < sizeof(mma8452q_data_resolution)/sizeof(mma8452q_data_resolution[0]))
	{        
		//SerialPrintString("reso=%x!! OK \n",reso);
		return 0;
	}
	else
	{   
		//SerialPrintString("choose sensitivity  fail!!\n");
		return MMA8452Q_ERR_STATUS;
	}
}

/*----------------------------------------------------------------------------*/

static int MMA8452Q_ResetCalibration()
{
	uint8 ofs[3] = {0x00, 0x00, 0x00};
	int err;

	//goto standby mode to clear cali
	MMA8452Q_SetPowerMode(false);
	err = i2c_write(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_OFSX, ofs, 3); 
	if(err)
	{
		//SerialPrintString("error: %d\n", err);
	}
  
	MMA8452Q_SetPowerMode(true);
	return err;    
}

/*----------------------------------------------------------------------------*/

static int MMA8452Q_Init(int reset_cali)
{
	int res = 0;
	
	res = MMA8452Q_CheckDeviceID(); 
	if(res != MMA8452Q_SUCCESS)
	{
	  //SerialPrintString("fwq mma8452q check id error\n");
		return res;
	}	

	res = MMA8452Q_SetPowerMode( false);
	if(res != MMA8452Q_SUCCESS)
	{
	  //SerialPrintString("fwq mma8452q set power error\n");
		return res;
	}
	

	res = MMA8452Q_SetBWRate( MMA8452Q_BW_100HZ);
	if(res != MMA8452Q_SUCCESS ) 
	{
	  //SerialPrintString("fwq mma8452q set BWRate error\n");
		return res;
	}

	res = MMA8452Q_SetDataFormat(MMA8452Q_RANGE_2G);
	if(res != MMA8452Q_SUCCESS)
	{
	  //SerialPrintString("fwq mma8452q set data format error\n");
		return res;
	}

	res = MMA8452Q_SetDataResolution(MMA8452Q_12BIT_RES);
	if(res != MMA8452Q_SUCCESS) 
	{
	  //SerialPrintString("fwq mma8452q set data reslution error\n");
		return res;
	}

	if(0 != reset_cali)
	{ 
		/*reset calibration only in power on*/
		res = MMA8452Q_ResetCalibration();
		if(res != MMA8452Q_SUCCESS)
		{
		  //SerialPrintString("fwq mma8452q set cali error\n");
			return res;
		}
	}

	return MMA8452Q_SUCCESS;
}

/*----------------------------------------------------------------------------*/

#define MMA8452Q_AXIS_X          0
#define MMA8452Q_AXIS_Y          1
#define MMA8452Q_AXIS_Z          2
#define MMA8452Q_AXES_NUM        3
#define MMA8452Q_DATA_LEN        6

int16 g_acc[MMA8452Q_AXES_NUM]; 	//运动传感器保存的数据

static int mma8452q_read_data(int16 data[MMA8452Q_AXES_NUM])
{      
	uint8 buf[MMA8452Q_DATA_LEN] = {0};
	int err = 0;
	
	err = i2c_read(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_DATAX0, buf, 2); 
	if( err ) {
		return err;
	}

	err = i2c_read(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_DATAY0, &buf[MMA8452Q_AXIS_Y*2], 2); 
	if( err ) {
		return err;
	}

	err = i2c_read(MMA8452Q_I2C_SLAVE_ADDR, MMA8452Q_REG_DATAZ0, &buf[MMA8452Q_AXIS_Z*2], 2); 
	if( err ) {
		return err;
	}	

	data[MMA8452Q_AXIS_X] = (int16)((buf[MMA8452Q_AXIS_X*2] << 8) | (buf[MMA8452Q_AXIS_X*2+1]));
	data[MMA8452Q_AXIS_Y] = (int16)((buf[MMA8452Q_AXIS_Y*2] << 8) | (buf[MMA8452Q_AXIS_Y*2+1]));
	data[MMA8452Q_AXIS_Z] = (int16)((buf[MMA8452Q_AXIS_Z*2] << 8) | (buf[MMA8452Q_AXIS_Z*2+1]));
   
	data[MMA8452Q_AXIS_X] = data[MMA8452Q_AXIS_X]>>4;
	data[MMA8452Q_AXIS_Y] = data[MMA8452Q_AXIS_Y]>>4;
	data[MMA8452Q_AXIS_Z] = data[MMA8452Q_AXIS_Z]>>4;

	return err;
}

/*----------------------------------------------------------------------------*/

static int mma8452q_read()
{
	uint8 databuf[20];
	
	int res = 0;
	osal_memset(databuf, 0, sizeof(uint8)*20);
	
	if(sensor_power == false)
	{
		res = MMA8452Q_SetPowerMode( true);
		if(res){
		    //SerialPrintString("Power on mma8452q error %d!\n", res);
		}
	}

	if(res = mma8452q_read_data(g_acc))
	{        
		//SerialPrintString("I2C error: ret value=%d", res);
		return -3;
	}

	return 0;
}

/*********************************************************************/
/*********************************************************************/


//ADC采样通道
#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2

#define TIMEWINDOW_MIN    10   	//时间窗，单位20ms
#define TIMEWINDOW_MAX    200		//时间窗，单位20ms
#define REGULATION	      3			//认为找到稳定规律所需要的步数
#define INVALID		        4			//认为失去稳定规律所需要的步数


uint32 STEPS;	//总步数

uint8 itemp,jtemp;								
uint8 _bad_flag[3];									
uint8  sampling_counter;								
int16 _adresult[3];									
int16 _max[3]={0,0,0};
int16 _min[3]={1000,1000,1000};
int16 _dc[3]={500,500,500};
int16 _vpp[3]={30,30,30};	
int16  _precision[3]={5,5,5};	
int16 _old_fixed[3];
int16 _new_fixed[3];

uint8 Interval=0;		//记录时间间隔数
uint8 TempSteps=0;		//记步缓存
uint8 InvalidSteps=0;	//无效步缓存
uint8 ReReg=2;			//记录是否重新开始寻找规律
						//	2-新开始
						//	1-已经开始，但是还没有找到规律
						//	0-已经找到规律
int16 highTh[3] = {0};
int16 lowTh[3] = {0};
uint8 currState[3] = {STATE_IDLE};
uint8 preState[3] = {STATE_IDLE};
uint8 findStepFlag[3] = {0};

/*------------------------------------------------------------------------------------------------------------------------
*Name: 		TimeWindow()
*Function:	实现"时间窗"算法,认为只有在有效"时间窗"内的记步才有效,而且起始时需要连续出现有效步才认为开始
*Input:		void
*Output: 	void
*------------------------------------------------------------------------------------------------------------------------*/
uint8 TimeWindow()
{
    uint8 ret = FIND_IDLE;
    
	if(ReReg==2)		//如果是新开始的第一步，直接在记步缓存中加1
	{
		TempSteps++;
		Interval=0;
		ReReg=1;
		InvalidSteps=0;	
		ret = FIND_NOT_REACH_REGULATION;
	}
	else				//如果不是新开始的第一步
	{
		if((Interval>=TIMEWINDOW_MIN)&&(Interval<=TIMEWINDOW_MAX))	//如果时间间隔在有效的时间窗内
		{
			InvalidSteps=0;	
			if(ReReg==1)					//如果还没有找到规律
			{
				TempSteps++;				//记步缓存加1
				if(TempSteps>=REGULATION)	//如果记步缓存达到所要求的规律数
				{
					ReReg=0;				//已经找到规律
					STEPS=STEPS+TempSteps;	//更新显示
					TempSteps=0;
					ret = FIND_OK;
				}
				else
				{
				    ret = FIND_NOT_REACH_REGULATION;
                }
				Interval=0;
			}
			else if(ReReg==0)				//如果已经找到规律，直接更新显示
			{
				STEPS++;
				TempSteps=0;
				Interval=0;
				ret = FIND_OK;
			}
		}
		else if(Interval<TIMEWINDOW_MIN)	//如果时间间隔小于时间窗下限
		{	
			if(ReReg==0)					//如果已经找到规律
			{
				if(InvalidSteps<255) 	InvalidSteps++;	//无效步缓存加1
				if(InvalidSteps>=INVALID)				//如果无效步达到所要求的数值，则重新寻找规律
				{	
					InvalidSteps=0;
					ReReg=1;
					TempSteps=1;
					Interval=0;
				}
				else					    //否则，只丢弃这一次的记步，但是继续记步，不需要重新寻找规律
				{
					Interval=0;
				}
			}
			else if(ReReg==1)				//如果还没有找到规律，则之前的寻找规律过程无效，重新寻找规律
			{
				InvalidSteps=0;	
				ReReg=1;
				TempSteps=1;
				Interval=0;
			}

			ret = WIN_TOO_SMALL;
		}
		else if(Interval>TIMEWINDOW_MAX)	//如果时间间隔大于时间窗上限，记步已经间断，重新寻找规律
		{
			InvalidSteps=0;	
			ReReg=1;						
			TempSteps=1;
			Interval=0;

			ret = WIN_TOO_LARGE;
		}
	}		

	return ret;
}

/*------------------------------------------------------------------------------------------------------------------------
*Name: 		step_counter()
*Function:	实现Pedometer计步的基本算法.
*Input:		void
*Output: 	void
*------------------------------------------------------------------------------------------------------------------------*/
void step_counter()
{
	uint8 findStepFlagTmp[3];
	Interval++;
	
	//----------------------------------------------ADC采样----------------------//
	for(jtemp=X_CHANNEL; jtemp<=Z_CHANNEL; jtemp++)
	{
		_adresult[X_CHANNEL] = g_acc[X_CHANNEL];
		_adresult[Y_CHANNEL] = g_acc[Y_CHANNEL];
		_adresult[Z_CHANNEL] = g_acc[Z_CHANNEL];
		
		if (_adresult[jtemp]>_max[jtemp])    {_max[jtemp]=_adresult[jtemp];}
		if (_adresult[jtemp]<_min[jtemp])    {_min[jtemp]=_adresult[jtemp];}
	}

    sampling_counter=sampling_counter+1;
	
	//----------------------------------计算动态门限和动态精度-----------------------//
    if (sampling_counter==20)
    {               
        sampling_counter=0;
				
        for(jtemp=X_CHANNEL; jtemp<=Z_CHANNEL; jtemp++)
        {
          _vpp[jtemp]=_max[jtemp]-_min[jtemp];
          _dc[jtemp]=_min[jtemp]+(_vpp[jtemp]>>1);
          highTh[jtemp] = _dc[jtemp] + ((_max[jtemp] - _dc[jtemp]) >> 1);
          lowTh[jtemp] = _dc[jtemp] - ((_dc[jtemp] - _min[jtemp]) >> 1);
          _max[jtemp]=-5000;
          _min[jtemp]=5000;
          _bad_flag[jtemp]=0;

          //调整门限参数
          if (_vpp[jtemp]>=2000) 
          {
            _precision[jtemp]=1800; 
          } 
          else if (_vpp[jtemp]>=500)
          {            
            _precision[jtemp] = (_vpp[jtemp]*13)>>4;
          } 
          else { 
            _precision[jtemp]=100;
            _bad_flag[jtemp]=1;
          }
        }
  }
		
	//--------------------------线性移位寄存器--------------------------------------
	#if 0
	for(jtemp=X_CHANNEL; jtemp<=Z_CHANNEL; jtemp++)
	{
		_old_fixed[jtemp]=_new_fixed[jtemp];

	  if (_adresult[jtemp]>=_new_fixed[jtemp]) {   
	   	if((_adresult[jtemp]-_new_fixed[jtemp])>=_precision[jtemp]) {
				_new_fixed[jtemp]=_adresult[jtemp];
      }
	  }
			
	  if (_adresult[jtemp]<_new_fixed[jtemp]) {   
	  	if((_new_fixed[jtemp]-_adresult[jtemp])>=_precision[jtemp]) {
				_new_fixed[jtemp]=_adresult[jtemp];
      }
	  }
	}
    #endif
    
#if 1
	for(jtemp=X_CHANNEL; jtemp<=Z_CHANNEL; jtemp++)
	{
	   

        if(_adresult[jtemp] > highTh[jtemp])
        {
             preState[jtemp] = currState[jtemp];
        	currState[jtemp] = STATE_HIGH;
	        /*从一个状态切换到另外一个状态，则寄存器值移位*/ 
	        if(preState[jtemp] != currState[jtemp])
	        {
	            _old_fixed[jtemp] =  _new_fixed[jtemp];
	        }
	        /*记录正最大值*/
        	if(_adresult[jtemp] > _new_fixed[jtemp])
        	{
        	     _new_fixed[jtemp] = _adresult[jtemp];
        	}

        	
        }
        else if(_adresult[jtemp] < lowTh[jtemp])
        {
            preState[jtemp] = currState[jtemp];
        	currState[jtemp] = STATE_LOW;
	        if(preState[jtemp] != currState[jtemp])
	        {
	            _old_fixed[jtemp] =  _new_fixed[jtemp];
	        }
	         /*记录负最大值*/
        	if(_adresult[jtemp] < _new_fixed[jtemp])
        	{
        	     _new_fixed[jtemp] = _adresult[jtemp];
        	}

        }
        	
     	findStepFlagTmp[jtemp] = FALSE;

       
        if((_old_fixed[jtemp]-_new_fixed[jtemp])>=_precision[jtemp]) 
        {
             findStepFlagTmp[jtemp] = TRUE;
             _old_fixed[jtemp] =  _new_fixed[jtemp];
        } 
        else if((_new_fixed[jtemp]-_old_fixed[jtemp])>=_precision[jtemp]) 
        {
             findStepFlagTmp[jtemp] = TRUE;
             _old_fixed[jtemp] =  _new_fixed[jtemp];
        }       
      
	}
#endif
    memset(findStepFlag, FIND_IDLE, sizeof(findStepFlag));
    

	//------------------------- 动态门限判决 ----------------------------------
	if ((_vpp[X_CHANNEL]>=_vpp[Y_CHANNEL])&&(_vpp[X_CHANNEL]>=_vpp[Z_CHANNEL])) {
		if ((findStepFlagTmp[X_CHANNEL] == TRUE) &&
        (_bad_flag[X_CHANNEL]==0)) {
			findStepFlag[X_CHANNEL] =  TimeWindow();
			//STEPS=STEPS+1;
		} 
	}
	else if ((_vpp[Y_CHANNEL]>=_vpp[X_CHANNEL])&&(_vpp[Y_CHANNEL]>=_vpp[Z_CHANNEL])) {
		if ((findStepFlagTmp[Y_CHANNEL] == TRUE)&&
			(_bad_flag[Y_CHANNEL]==0)) {
			findStepFlag[Y_CHANNEL] = TimeWindow();
			//STEPS=STEPS+1;
		}
	}
	else if ((_vpp[Z_CHANNEL]>=_vpp[Y_CHANNEL])&&(_vpp[Z_CHANNEL]>=_vpp[X_CHANNEL])){
		if ((findStepFlagTmp[Z_CHANNEL] == TRUE)&&
			(_bad_flag[Z_CHANNEL]==0)){
			findStepFlag[Z_CHANNEL] = TimeWindow();
			//STEPS=STEPS+1;
		}
	}
}

//计步初始化
void step_init()
{
	sampling_counter=0;
	STEPS=0;	
	memset(_new_fixed, 0, sizeof(_new_fixed));
	memset(highTh, 0, sizeof(highTh));
	memset(lowTh, 0, sizeof(lowTh));	
	
}

/*********************************************************************/
/*********************************************************************/

//gsensor初始化
void gsensor_init()
{
	MMA8452Q_Init(1);
	
	step_init();
}

//主机读取结果
void gsensor_read()
{
	uint8 uartbuf[20];
	uint8 *gbuf = (uint8*)g_acc; 

  if(!LICENSE_IS_GOOD())
  {
    if(STEPS > 0)
    {
      STEPS = 1;
    }
  }
  
	uint8 step_0 = (STEPS&0xff000000)>>24;
	uint8 step_1 = (STEPS&0x00ff0000)>>16;
	uint8 step_2 = (STEPS&0x0000ff00)>>8;
	uint8 step_3 = (STEPS&0x000000ff);
	
	uartbuf[BT4_SYNC1] = UART2_SYNC_CMD;
	uartbuf[BT4_SYNC2] = UART2_SYNC_CMD;
	uartbuf[BT4_CMD]   = UART2_CMD_GSENSOR;		
	uartbuf[BT4_DATA1] = gbuf[0];	//x
	uartbuf[BT4_DATA2] = gbuf[1];	//x
	uartbuf[BT4_DATA3] = gbuf[2];	//y
	uartbuf[BT4_DATA4] = gbuf[3];	//y
	uartbuf[BT4_DATA5] = gbuf[4];	//z
	uartbuf[BT4_DATA6] = gbuf[5];	//z

	uartbuf[BT4_DATA7] = step_0;
	uartbuf[BT4_DATA8] = step_1;
	uartbuf[BT4_DATA9] = step_2;
	uartbuf[BT4_DATA10] = step_3;

	(void)HalUARTWrite(0, uartbuf, 13);
}

//GSensor定期更新处理
void gsensor_update()
{
	mma8452q_read();

  bleApp_DumpDataReq((uint8 *)g_acc, 6);

	step_counter();
}


