#ifndef _BT4_GSENSOR_H_
#define _BT4_GSENSOR_H_

//�߲��˶�������
#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2

#define TIMEWINDOW_MIN    15  	//ʱ�䴰����λ20ms
#define TIMEWINDOW_MAX    150		//ʱ�䴰����λ20ms
#define REGULATION	      3			//��Ϊ�ҵ��ȶ���������Ҫ�Ĳ���
#define INVALID		        3			//��Ϊʧȥ�ȶ���������Ҫ�Ĳ���

#define VPP_MIN 200


#define ADC_MIM_VALUE (-4096)
#define ADC_MAX_VALUE 4095

/*��ǰֵ�����ĸ�����*/
#define STATE_IDLE 0
#define STATE_LOW 1
#define STATE_HIGH 2

/*�ҵ�һ����û�ҵ���ԭ��*/
#define FIND_IDLE 0
#define FIND_OK 1
#define WIN_TOO_SMALL 2
#define WIN_TOO_LARGE 3
#define FIND_NOT_REACH_REGULATION 4
void gsensor_read();
void gsensor_init();
void gsensor_update();
void step_init();

#endif