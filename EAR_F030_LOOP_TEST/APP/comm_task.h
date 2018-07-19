#ifndef __COMM_TASK_H
#define __COMM_TASK_H	    
//////////////////////////////////////////////////////////////////////////////////	 							  
//////////////////////////////////////////////////////////////////////////////////

#include "datatype.h"

//#define SEND_BUF_LEN  255
#define SEND_BUF_LEN  248

//434是数据，2是两个校验位
#define PARAMETER_BUF_LEN 436

#define _DEBUG_TEST_CYCLES

//局部变量
typedef enum
{
	LOAD_PARA,  //加载参数
//	GET_MODE,
	CHECK_PRESSURE,
	CHECK_PRESSURE_AGAIN,
	PREV_OUTPUT_PWM,
	OVER_THRESHOLD_SAFETY,
	CPY_PARA_TO_BUFFER,
	OUTPUT_PWM,
	CHECK_BAT_VOL,
	LED_RED_BLINK
}CHCKMODE_OUTPUT_PWM;

typedef enum
{
	PWM_NONE,
	PWM_START,
	PWM_PERIOD,
	PWM_WAIT_BETWEEN,
	PWM_WAIT_AFTER,
	PWM_OUTPUT_FINISH
}PWM_STATE;

typedef enum
{
	THERMAL_NONE,
	//THERMAL_DELAY,
//	THERMAL_CFG,
//	THERMAL_GET_VALUE,
//	THERMAL_START_CHECK,
	THERMAL_OVER_HEATING,
}THERMAL_STATE;

void init_PWMState(void);

void TaskDataSend (void);
void CMD_ProcessTask (void);
void CalcCheckSum(UINT8* pPacket);
void check_selectedMode_ouputPWM(void);
void PaintPWM(unsigned char num,unsigned char* pwm_buffer);
//void PaintPWM(unsigned char num );
void CheckFlashData(unsigned char* buffer);
void ResetParameter(unsigned char* buffer);
void get_switch_mode(void);
void thermal_check(void);
void adc_value_sample(void);
void bat_check(void);
#endif
