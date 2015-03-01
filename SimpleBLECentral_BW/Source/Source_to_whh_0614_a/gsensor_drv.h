#ifndef _BT4_GSENSOR_H_
#define _BT4_GSENSOR_H_

//走步运动传感器

#define STATE_IDLE 0
#define STATE_LOW 1
#define STATE_HIGH 2

/*找到一步或没找到的原因*/
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