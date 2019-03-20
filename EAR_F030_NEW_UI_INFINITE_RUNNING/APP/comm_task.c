//////////////////////////////////////////////////////////////////////////////////	 
			  
//////////////////////////////////////////////////////////////////////////////////
#include "comm_task.h"
#include "fifo.h"
#include "CMD_Receive.h"
#include "os_cfg.h"
#include "stdio.h"
#include "delay.h"
#include "string.h"
#include "app.h"
#include "serial_port.h"
#include "protocol_module.h"
#include "key_led_task.h"
#include "Motor_pwm.h"
#include "i2c.h"
#include "key_led_task.h"
#include "hardware.h"
#include "iwtdg.h"

#define HONEYWELL_RATE			11110   //斜率
extern uint32_t HONEYWELL_ZERO_POINT;
extern uint32_t trans_xmmHg_2_adc_value(uint8_t xmmHg);
extern BOOL b_getHoneywellZeroPoint;

//#define PRESSURE_RATE 70
//#define PRESSURE_RATE (FlashReadWord(FLASH_PRESSURE_RATE_ADDR))
uint16_t PRESSURE_RATE;

#define PRESSURE_SAFETY_THRESHOLD 160   //0xA0表示10.0，对应十进制160
//y=ax+b
#define PRESSURE_SENSOR_VALUE(x) ((int16_t)(((PRESSURE_RATE)*(x))+zero_point_of_pressure_sensor))


//全局变量
CMD_Receive g_CmdReceive;  // 命令接收控制对象
FIFO_TYPE send_fifo;//發送數據FIFO
UINT8 send_buf[SEND_BUF_LEN];

//用来接收上位机传过来的参数
UINT8 parameter_buf[PARAMETER_BUF_LEN]; 

UINT8 buffer[PARAMETER_BUF_LEN];

UINT16 check_sum;
extern BOOL b_Is_PCB_PowerOn;
extern MCU_STATE mcu_state;
extern BOOL rcvParameters_from_PC;
extern KEY_STATE key_state;
extern const uint8_t default_parameter_buf[PARAMETER_BUF_LEN];

extern uint16_t RegularConvData_Tab[2];

extern uint32_t os_ticks;
extern BOOL b_check_bat;

extern LED_STATE led_state;

extern int16_t zero_point_of_pressure_sensor;

static BOOL PWM1_timing_flag=TRUE;
static BOOL PWM2_timing_flag=TRUE; 
static BOOL PWM3_timing_flag=TRUE;
static BOOL waitBeforeStart_timing_flag=TRUE;
static BOOL* b_timing_flag;

uint32_t prev_WaitBeforeStart_os_tick;
uint32_t prev_PWM1_os_tick;
uint32_t prev_PWM2_os_tick;
uint32_t prev_PWM3_os_tick;
uint32_t* p_prev_os_tick;

PWM_STATE pwm1_state=PWM_START;
PWM_STATE pwm2_state=PWM_START;
PWM_STATE pwm3_state=PWM_START;


static PWM_STATE* p_pwm_state;
static uint16_t* p_PWM_period_cnt;
static uint16_t* p_PWM_waitBetween_cnt;
static uint16_t* p_PWM_waitAfter_cnt;
static uint8_t* p_PWM_numOfCycle;
static uint8_t* p_PWM_serial_cnt;

uint16_t PWM_waitBeforeStart_cnt=0;

uint16_t PWM1_period_cnt=0;
uint16_t PWM2_period_cnt=0;
uint16_t PWM3_period_cnt=0;

uint16_t PWM1_waitBetween_cnt=0;
uint16_t PWM2_waitBetween_cnt=0;
uint16_t PWM3_waitBetween_cnt=0;

uint16_t PWM1_waitAfter_cnt=0;
uint16_t PWM2_waitAfter_cnt=0;
uint16_t PWM3_waitAfter_cnt=0;

uint8_t PWM1_numOfCycle=0;
uint8_t PWM2_numOfCycle=0;
uint8_t PWM3_numOfCycle=0;

uint8_t PWM1_serial_cnt=0;
uint8_t PWM2_serial_cnt=0;
uint8_t PWM3_serial_cnt=0;

//volatile CHCKMODE_OUTPUT_PWM state=LOAD_PARA;
CHCKMODE_OUTPUT_PWM state=LOAD_PARA;
uint8_t mode=1;                      
//uint8_t prev_mode;

//uint8_t pwm_buffer[144]={0};
//uint16_t	mode;
static uint8_t pwm1_buffer[49];
static uint8_t pwm2_buffer[49];
static uint8_t pwm3_buffer[49];
uint8_t pressure;
uint16_t checkPressAgain_cnt=0;
uint8_t wait_cnt=0;


//THERMAL_STATE thermal_state=THERMAL_NONE;

//uint32_t adc_value[2]={0xFFFF,0x00}; //[0]对应NTC，[1]对应pressure
uint32_t adc_pressure_value=0;
//uint8_t adc_state=1;

uint8_t led_high_cnt=0;
uint8_t led_low_cnt=0;
uint8_t flash_cnt=0;

extern BOOL b_check_BAT_ok;

uint8_t switch_mode_cnt=0;
BOOL b_check_bnt_release=FALSE;
uint8_t release_btn_cnt=0;
/*******************************************************************************
*                                内部函数声明
*******************************************************************************/
static BOOL ModuleUnPackFrame(void);
static BOOL ModuleProcessPacket(UINT8 *pData);
static UINT8 CheckCheckSum(UINT8* pData, UINT8 nLen);

void init_PWMState(void)
{
	PWM1_timing_flag=TRUE;
	PWM2_timing_flag=TRUE; 
	PWM3_timing_flag=TRUE;
	waitBeforeStart_timing_flag=TRUE;
	
	pwm1_state=PWM_NONE;
	pwm2_state=PWM_NONE;
	pwm3_state=PWM_NONE;

	PWM_waitBeforeStart_cnt=0;

	PWM1_period_cnt=0;
	PWM2_period_cnt=0;
	PWM3_period_cnt=0;

	PWM1_waitBetween_cnt=0;
	PWM2_waitBetween_cnt=0;
	PWM3_waitBetween_cnt=0;

	PWM1_waitAfter_cnt=0;
	PWM2_waitAfter_cnt=0;
	PWM3_waitAfter_cnt=0;

	PWM1_numOfCycle=0;
	PWM2_numOfCycle=0;
	PWM3_numOfCycle=0;

	PWM1_serial_cnt=0;
	PWM2_serial_cnt=0;
	PWM3_serial_cnt=0;
}

void CalcCheckSum(UINT8* pPacket)
{
	UINT16 dataLen = pPacket[1];
	UINT16 checkSum = 0;
	UINT16 i;

	for (i = 1; i < dataLen; i++)
	{
		checkSum += pPacket[i];
	}

	pPacket[dataLen] = checkSum >> 8;
	pPacket[dataLen+1] = checkSum&0xFF;
}

/*******************************************************************************
** 函数名称: ModuleUnPackFrame
** 功能描述: 命令接收处理
** 输　  入: 无
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
 UINT8 sDataBuff[CMD_BUFFER_LENGTH] = {0};	
 UINT8 sBackBuff[CMD_BUFFER_LENGTH] = {0};
BOOL ModuleUnPackFrame(void)
{
	static BOOL sPacketHeadFlag = FALSE;
	static BOOL sPacketLenFlag = FALSE;
	static UINT8 sCurPacketLen = 0;
	static UINT8 sResetByte = 0;
	//static UINT8 sDataBuff[CMD_BUFFER_LENGTH] = {0};	
	//static UINT8 sBackBuff[CMD_BUFFER_LENGTH] = {0};

	UINT8 *pBuff = (UINT8 *)sDataBuff;
	UINT16 dwLen = 0;
	UINT8 byCurChar;

	// 从串口缓冲区中读取接收到的数据
	dwLen = GetBuf2Length(&g_CmdReceive);

	// 对数据进行解析
	while(0 < dwLen)
	{
		byCurChar = Buf2Read(&g_CmdReceive);

		if (sPacketHeadFlag)
		{
			// 解析到包头
			if(sPacketLenFlag)
			{
				// 解析到包长度
				pBuff[sCurPacketLen] = byCurChar;
				sCurPacketLen ++;
				sResetByte --;

				if (0 >= sResetByte)
				{
					// 接收完毕
					// 进行校验和比较
					if (CheckCheckSum(pBuff, pBuff[1]))
					{
						// 解析到一个有效数据包
						memcpy(sBackBuff, sDataBuff, CMD_BUFFER_LENGTH);
						ModuleProcessPacket(sBackBuff);//命令解包*********************************
						//防止连续接收多条命令时，命令应答信号出故障
						delay_ms(2);
					}

					sPacketHeadFlag = FALSE;
					sPacketLenFlag = FALSE;
					memset(&sDataBuff, 0x00, CMD_BUFFER_LENGTH);
				}													
			}
			else
			{
				if((CMD_BUFFER_LENGTH-1 > byCurChar) && (0 < byCurChar ))// 容错处理，防止数据包长为49或0时溢出 并且X5最长的主机发送包为15
				{
					// 解析到模块的长度
					sDataBuff[sCurPacketLen] = byCurChar;
					sResetByte = byCurChar;			
					sPacketLenFlag = TRUE;
					sCurPacketLen ++;
				}
				else
				{
					//没有解析到模块的长度, 重新解析
					sPacketHeadFlag = FALSE;
					sPacketLenFlag = FALSE;					
				}
			}
		}
		
		else if (PACK_HEAD_BYTE == byCurChar)		
		{
			// 解析到包头
			sDataBuff[0] = byCurChar;
			sPacketHeadFlag = TRUE;
			sPacketLenFlag = FALSE;			
			sCurPacketLen = 1;
			sResetByte = 0;
		}

		//pData ++;
		dwLen --;
	}
	return TRUE;
}


/*******************************************************************************
** 函数名称: CheckCheckSum
** 功能描述: 包校验
** 输　  入: pData 数据 nLen长度
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
UINT8 CheckCheckSum(UINT8* pData, UINT8 nLen)
{
	UINT16 bySum = 0;
	int i;
	// 计算数据的校验和	
	for(i = 1; i < nLen; i++)
	{
		bySum += pData[i];
	}		

	if (bySum == (pData[nLen] << 8)+ pData[nLen + 1])
	{
		return TRUE;
	}
	else
	{
		return FALSE;	
	}
}

/*******************************************************************************
** 函数名称: ModuleProcessPacket
** 功能描述: 命令解包
** 输　  入: pData 命令
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
BOOL ModuleProcessPacket(UINT8 *pData)
{	
	protocol_module_process(pData);

	return TRUE;     	
}


/*******************************************************************************
* 函数名称 : TaskDataSend
* 功能描述 : 数据发送任务，5ms执行一次
* 输入参数 : arg  创建任务时传递的参数
* 输出参数 : 无
* 返回参数 : 无
*******************************************************************************/
void TaskDataSend (void)
{
    UINT8 send_data_buf[SEND_DATA_BUF_LENGTH] = {0};
    UINT16  len;
		
		//protocol_module_send_exp_flag(1);
		#ifdef _DEBUG
		#else
		if(mcu_state==POWER_ON)
		#endif
		{
				//循環
			len = fifoReadData(&send_fifo, send_data_buf, SEND_DATA_BUF_LENGTH);
			if(len)
			{
					UartSendNBytes(send_data_buf, len);
					delay_ms(30);
			}
		}
		
		os_delay_ms(SEND_TASK_ID, 30);  //mark一下
		//os_delay_ms(SEND_TASK_ID, 28);
}

//定时x毫秒,n_ms最大就255s，255000
BOOL Is_timing_Xmillisec(uint32_t n_ms,uint8_t num)
{
	switch(num)
	{
		case 1:      //PWM1
			b_timing_flag=&PWM1_timing_flag;
			p_prev_os_tick=&prev_PWM1_os_tick;
			break;
		case 2:     //PWM2
			b_timing_flag=&PWM2_timing_flag;
			p_prev_os_tick=&prev_PWM2_os_tick;
			break;
		case 3:    //PWM3
			b_timing_flag=&PWM3_timing_flag;
			p_prev_os_tick=&prev_PWM3_os_tick;
			break;
//		case 4:    //PWM4
//			b_timing_flag=&PWM4_timing_flag;
//			p_prev_os_tick=&prev_PWM4_os_tick;
//			break;
//		case 5:   //PWM5
//			b_timing_flag=&PWM5_timing_flag;
//			p_prev_os_tick=&prev_PWM5_os_tick;
//			break;
		case 6:   //wait before start
			b_timing_flag=&waitBeforeStart_timing_flag;
			p_prev_os_tick=&prev_WaitBeforeStart_os_tick;
			break;
//		case 7:   //模式开关的按键时间 ,不适合用这个代码
//			b_timing_flag=&switch_bnt_timing_flag;
//			p_prev_os_tick=&prev_switchBtn_os_tick;
//			break;
		default:
			break;
	}
	if(*b_timing_flag==TRUE)
	{
		*p_prev_os_tick=os_ticks;
		*b_timing_flag=FALSE;
	}
	if(os_ticks+n_ms<os_ticks) //如果os_ticks+n_ms溢出了，那么os_ticks+n_ms必然小于os_ticks
	{
		//*p_prev_os_tick=os_ticks;
		if(os_ticks==os_ticks+n_ms)
		{
			*b_timing_flag=TRUE;
			*p_prev_os_tick=0;
			return TRUE;
		}
	}
	else
	{
		if(os_ticks-*p_prev_os_tick>=n_ms)
		{
			*b_timing_flag=TRUE;
			*p_prev_os_tick=0;
			return TRUE;
		}
	}
	
	return FALSE;
}

void bat_check()
{
	if(mcu_state==POWER_ON)
	{
		if(led_state==LED_RED_SOLID_NO_POWER) //关机
		{
			set_led(LED_CLOSE,TRUE);
			set_led(LED_YELLOW,TRUE);
//			
////			//记录关机
////			record_dateTime(CODE_NO_POWER); //没电开机，不用记录
			for(uint8_t i=0;i<5;i++)
			{
				Motor_PWM_Freq_Dudy_Set(1,100,20);
				Motor_PWM_Freq_Dudy_Set(2,100,20);
				Delay_ms(500);
				//IWDG_Feed();
				Motor_PWM_Freq_Dudy_Set(1,100,0);
				Motor_PWM_Freq_Dudy_Set(2,100,0);
				Delay_ms(500);
				IWDG_Feed();
			}
			EnterStopMode();
			init_system_afterWakeUp();
		}
		
		if(led_state==LED_RED_FLASH_LOW_POWER)
		{
			if(led_high_cnt==5)
			{
				set_led(LED_CLOSE,TRUE);
				if(led_low_cnt==5)
				{
					led_high_cnt=0;
					led_low_cnt=0;
					if(flash_cnt==4)  //黄灯闪5下然后常量
					{
						led_state=LED_NONE;
						flash_cnt=0;
						show_mode_LED();  //显示模式灯	
						
						Motor_PWM_Freq_Dudy_Set(1,100,50);
						Motor_PWM_Freq_Dudy_Set(2,100,50);
						Delay_ms(500);
						Motor_PWM_Freq_Dudy_Set(1,100,0);
						Motor_PWM_Freq_Dudy_Set(2,100,0);
						
						b_check_BAT_ok=TRUE;
						//记录开机时间
						record_dateTime(CODE_SYSTEM_POWER_ON);
					}
					else
					{
						flash_cnt++;
					}
				}
				else
				{
					led_low_cnt++;
				}
			}
			else
			{
				set_led(LED_CLOSE,TRUE);
				set_led(LED_YELLOW,TRUE);
				led_high_cnt++;
			}
		}
		
		if(led_state==LED_GREEN_SOLID)
		{
			show_mode_LED();  //显示模式灯
			led_state=LED_NONE;
			
			//记录开机时间
			record_dateTime(CODE_SYSTEM_POWER_ON);
			
			//马达震动
			Motor_PWM_Freq_Dudy_Set(1,100,80);
			Motor_PWM_Freq_Dudy_Set(2,100,80);
			Delay_ms(500);
			Motor_PWM_Freq_Dudy_Set(1,100,0);
			Motor_PWM_Freq_Dudy_Set(2,100,0);
			
			b_check_BAT_ok=TRUE;
		}
	}
	os_delay_ms(TASK_BAT_CHECK, 50);
}
#if 0
////采集ADS115的ADC值
//void adc_value_sample()
//{
//	if(thermal_state==THERMAL_NONE)
//	{
//		switch(adc_state)
//		{
//			case 1:
//				ADS115_cfg4ThermalCheck();
//				adc_state=2;
//				break;
//			case 2:
//				adc_value[0]=ADS115_readByte(0x90);  //温度
//				ADS115_Init();
//				if(adc_value[0]<=7247)
//				//if(adc_value[0]<=22500)  //debug
//				{
//					thermal_state=THERMAL_OVER_HEATING;
//				}
//				adc_state=3;
//				break;
//			case 3:
//				adc_value[1]=ADS115_readByte(0x90);
//				ADS115_cfg4ThermalCheck();
//				adc_state=2;
//				break;
//		}
//		
//		if(thermal_state==THERMAL_OVER_HEATING)
//		{
//			Motor_PWM_Freq_Dudy_Set(1,100,0);
//			Motor_PWM_Freq_Dudy_Set(2,100,0);
//			Motor_PWM_Freq_Dudy_Set(3,100,0);
//			//橙色LED闪3s
////			for(int i=0;i<3;i++)
////			{
////				set_led(LED_RED);
////				Delay_ms(500);
////				set_led(LED_CLOSE);
////				Delay_ms(500);
////				IWDG_Feed();   //喂狗
////			}
////			EnterStopMode();
////			init_system_afterWakeUp();
//			set_led(LED_RED);
//	
//			for(uint8_t i=0;i<5;i++)
//			//for(uint8_t i=0;i<6;i++)  //debug
//			{
//				Motor_PWM_Freq_Dudy_Set(1,100,0);
//				Motor_PWM_Freq_Dudy_Set(2,100,0);
//				Motor_PWM_Freq_Dudy_Set(3,100,0);
//				Delay_ms(500);
//				//IWDG_Feed();
//				Motor_PWM_Freq_Dudy_Set(1,100,50);
//				Motor_PWM_Freq_Dudy_Set(2,100,50);
//				//Motor_PWM_Freq_Dudy_Set(2,100,50);
//				Motor_PWM_Freq_Dudy_Set(3,100,50);
//				Delay_ms(500);
//				IWDG_Feed();
//			}
//			EnterStopMode();
//			init_system_afterWakeUp();
//		}
//	}
//	os_delay_ms(TASK_ADC_VALUE_SAMPLE, 15);
//}

//void thermal_check()
//{
//	static uint16_t test;
//	ADS115_cfg4ThermalCheck();
//	delay_ms(15);
//	test=ADS115_readByte(0x90);
//	if(test<=7248)
//	{
//		//橙色LED闪3s
//		for(int i=0;i<3;i++)
//		{
//			set_led(LED_RED);
//			Delay_ms(500);
//			set_led(LED_CLOSE);
//			Delay_ms(500);
//			IWDG_Feed();   //喂狗
//		}

//		EnterStopMode();
//		init_system_afterWakeUp();
//	}
//	else
//	{
//		ADS115_Init();
//	}

//	os_delay_ms(TASK_THERMAL_CHECK, 100);
//}
#endif

/*******************************************************************************
** 函数名称: get_switch_mode
** 功能描述: 获取按键所对应的模式
** 输　  入: 无
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
void get_switch_mode()
{
	if(mcu_state==POWER_ON||b_check_BAT_ok==TRUE)
	{
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)==0)  //按键按下了
		{
			if(switch_mode_cnt==3)
			{
				b_check_bnt_release=TRUE;
				switch_mode_cnt=0;
			}
			else
			{
				switch_mode_cnt++;
			}
		}
		else
		{
			switch_mode_cnt=0;
			if(b_check_bnt_release==TRUE)
			{
				if(release_btn_cnt==2)
				{
					b_check_bnt_release=FALSE;
					release_btn_cnt=0;

					//波形重新初始化，关闭马达
					init_PWMState();
					state=LOAD_PARA;
					Motor_PWM_Freq_Dudy_Set(1,100,0);
					Motor_PWM_Freq_Dudy_Set(2,100,0);
					
						//选择模式灯
					switch(mode)
					{
						case 1:   //当前是模式1，按下之后就变成了模式2，所以mode=2，亮两盏灯
							mode=2;
							set_led(LED_CLOSE,TRUE);
							set_led(LED_GREEN_1,TRUE);
							set_led(LED_GREEN_2,TRUE);
							break;
						case 2:
							mode=3;
							set_led(LED_CLOSE,TRUE);
							set_led(LED_GREEN_1,TRUE);
							set_led(LED_GREEN_2,TRUE);
							set_led(LED_GREEN_3,TRUE);
							break;
						case 3:
							mode=1;
							set_led(LED_CLOSE,TRUE);
							set_led(LED_GREEN_1,TRUE);
							break;
						default:
							break;
					}	
					
					
				}
				else
				{
					release_btn_cnt++;
				}
			}
		}
	}
	os_delay_ms(TASK_GET_SWITCH_MODE, 20);
}


/*******************************************************************************
* 函数名称 : TaskDataSend
* 功能描述 : 数据发送任务，5ms执行一次
* 输入参数 : arg  创建任务时传递的参数
* 输出参数 : 无
* 返回参数 : 无
*******************************************************************************/
void PaintPWM(unsigned char num,unsigned char* buffer)
{
	uint8_t ELEMENTS_CNT=8;
	uint8_t FREQ=2;
	uint8_t DUTY_CYCLE=3;
	uint8_t PERIOD=4;
//	uint8_t NUM_OF_CYCLES=5;
	uint8_t WAIT_BETWEEN=6;
	uint8_t WAIT_AFTER=7;
	switch(num)
	{
		case 1:
			p_pwm_state=&pwm1_state;
			p_PWM_period_cnt=&PWM1_period_cnt;
			p_PWM_waitBetween_cnt=&PWM1_waitBetween_cnt;
			p_PWM_waitAfter_cnt=&PWM1_waitAfter_cnt;
			p_PWM_numOfCycle=&PWM1_numOfCycle;
			p_PWM_serial_cnt=&PWM1_serial_cnt;
			break;
		case 2:
			p_pwm_state=&pwm2_state;
			p_PWM_period_cnt=&PWM2_period_cnt;
			p_PWM_waitBetween_cnt=&PWM2_waitBetween_cnt;
			p_PWM_waitAfter_cnt=&PWM2_waitAfter_cnt;
			p_PWM_numOfCycle=&PWM2_numOfCycle;
			p_PWM_serial_cnt=&PWM2_serial_cnt;
			break;
		case 3:
			p_pwm_state=&pwm3_state;
			p_PWM_period_cnt=&PWM3_period_cnt;
			p_PWM_waitBetween_cnt=&PWM3_waitBetween_cnt;
			p_PWM_waitAfter_cnt=&PWM3_waitAfter_cnt;
			p_PWM_numOfCycle=&PWM3_numOfCycle;
			p_PWM_serial_cnt=&PWM3_serial_cnt;
			break;
		default:
			break;
	}
	#ifdef _DEBUG
	#else
	if(b_Is_PCB_PowerOn==FALSE)
	{
//		ResetAllState();
		mcu_state=POWER_OFF;
		state=LOAD_PARA;
		*p_pwm_state=PWM_START;
		*p_PWM_period_cnt=0;
		*p_PWM_waitBetween_cnt=0;
		*p_PWM_waitAfter_cnt=0;
		*p_PWM_numOfCycle=0;
		*p_PWM_serial_cnt=0;
		//PWM_waitBeforeStart_cnt=0;
		Motor_PWM_Freq_Dudy_Set(num,100,0);
	}
	else
	#endif
	{
		if(*p_pwm_state==PWM_START)
		{
			if(*p_PWM_serial_cnt>buffer[0]-1)
			{
				Motor_PWM_Freq_Dudy_Set(num,buffer[1+8*(*p_PWM_serial_cnt)+2],0);
				*p_pwm_state=PWM_OUTPUT_FINISH;
				*p_PWM_serial_cnt=0;
			}
			else
			{
				Motor_PWM_Freq_Dudy_Set(num,buffer[1+8*(*p_PWM_serial_cnt)+2],buffer[1+8*(*p_PWM_serial_cnt)+3]);
				*p_pwm_state=PWM_PERIOD;
			}
		}
		
		if(*p_pwm_state==PWM_PERIOD)
		{
//			if((*p_PWM_period_cnt)*CHECK_MODE_OUTPUT_PWM==buffer[1+8*(*p_PWM_serial_cnt)+4]*1000)
//			{
//				++(*p_PWM_numOfCycle);
//				*p_PWM_period_cnt=0;
//				*p_pwm_state=PWM_WAIT_BETWEEN;
//				Motor_PWM_Freq_Dudy_Set(num,buffer[1+8*(*p_PWM_serial_cnt)+2],0);
//			}
//			else
//			{
//				++(*p_PWM_period_cnt);
//			}
			if(Is_timing_Xmillisec(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+PERIOD]*1000,num))
			{
				++(*p_PWM_numOfCycle);
				*p_PWM_period_cnt=0;
				*p_pwm_state=PWM_WAIT_BETWEEN;
				Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
			}
			else
			{
				++(*p_PWM_period_cnt);
			}
		}
		
		if(*p_pwm_state==PWM_WAIT_BETWEEN)
		{
			if(*p_PWM_numOfCycle==buffer[1+8*(*p_PWM_serial_cnt)+5])
			{
				*p_pwm_state=PWM_WAIT_AFTER;
				*p_PWM_numOfCycle=0;
			}
			else
			{
//				if((*p_PWM_waitBetween_cnt)*CHECK_MODE_OUTPUT_PWM==buffer[1+8*(*p_PWM_serial_cnt)+6]*1000)
//				{ 
//					Motor_PWM_Freq_Dudy_Set(num,buffer[1+8*(*p_PWM_serial_cnt)+2],buffer[1+8*(*p_PWM_serial_cnt)+3]); 
//					*p_PWM_waitBetween_cnt=0;
//					*p_pwm_state=PWM_PERIOD;
//				}
//				else
//				{
//					++(*p_PWM_waitBetween_cnt);
//				}
				if(Is_timing_Xmillisec(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+WAIT_BETWEEN]*1000,num))
				{
					Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+DUTY_CYCLE]); 
					//*p_PWM_waitBetween_cnt=0;
					*p_pwm_state=PWM_PERIOD;
				}
			}
		}
		
		if(*p_pwm_state==PWM_WAIT_AFTER)
		{
//			if((*p_PWM_waitAfter_cnt)*CHECK_MODE_OUTPUT_PWM==buffer[1+8*(*p_PWM_serial_cnt)+7]*1000)
//			{
//				*p_PWM_numOfCycle=0;
//				Motor_PWM_Freq_Dudy_Set(num,buffer[1+8*(*p_PWM_serial_cnt)+2],0);
//				++(*p_PWM_serial_cnt);
//				*p_pwm_state=PWM_START;
//				*p_PWM_waitAfter_cnt=0;
//			}
//			else	
//			{
//				++(*p_PWM_waitAfter_cnt);
//			}
			if(Is_timing_Xmillisec(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+WAIT_AFTER]*1000,num))
			{
				state=OUTPUT_PWM;
				*p_PWM_numOfCycle=0;
				Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
				++(*p_PWM_serial_cnt);
				*p_pwm_state=PWM_START;
				//*p_PWM_waitAfter_cnt=0;
			}
		}
	}

}

void ResetParameter(unsigned char* buffer)
{
	//将代码段中flash数据拷贝到buffer中
	for(int i=0;i<PARAMETER_BUF_LEN;i++)
	{
		buffer[i]=default_parameter_buf[i];
	}
	//更新flash
	FlashWrite(FLASH_WRITE_START_ADDR,buffer,PARAMETER_BUF_LEN/4);
}

void CheckFlashData(unsigned char* buffer)
{
	uint16_t j=0;
	//如果数据出错就用默认的数据
	if(buffer[0]>249)
	{
		ResetParameter(buffer);
		return;
	}
	for(int i=0;i<54;i++)
	{
		j++;                 //1.跳过第一个
		if(buffer[2+j++]>1) //2.enable
		{
			ResetParameter(buffer);
			return;
		}
		if(buffer[2+j++]==0)  //3.freq
		{
			ResetParameter(buffer);
			return;
		}
		if(buffer[2+j]<5||buffer[2+j]>100) //4.duty cycle
		{
			ResetParameter(buffer);
			return;
		}
		j++;
		if(buffer[2+j++]==0)            //5.period
		{
			ResetParameter(buffer);
			return;
		}
		if(buffer[2+j]<1||buffer[2+j]>250)            //6.number of cycle
		{
			ResetParameter(buffer);
			return;
		}
		j++;
		
		j++;                                  //7.wait between
		j++;																	//8.wait after
	}
}

/*******************************************************************************
** 函数名称: FillUpPWMbuffer
** 功能描述: 按照serial的有和无来填充pwm1_buffer,pwm2_buffer,pwm3_buffer
** 输　  入: 无
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
void FillUpPWMbuffer(uint8_t* dest,uint8_t* src)
{
	uint8_t serial_cnt=0;
	uint8_t j=1;
	for(int i=0;i<6;i++)
	{
		if(src[8*i+1]==0x01)
		{
			uint8_t k;
			for(k=0;k<8;k++)
			{
				dest[j++]=src[8*i+k];
			}
			serial_cnt++;
		}
	}
	dest[0]=serial_cnt;
}

//容错，如果读到的sensor斜率值在[10,200]之间，为ok，否则读10次之后就给sensor默认值20
void get_pressure_sensor_rate()
{
	uint8_t readCnt=0;
	do
	{
		if(readCnt==10) //如果读10次都不在[10,200]之间，认为有问题
		{
			readCnt=0;
			PRESSURE_RATE=60;
			return;
		}
		PRESSURE_RATE=FlashReadWord(FLASH_PRESSURE_RATE_ADDR);
		readCnt++;
	}while(PRESSURE_RATE<10||PRESSURE_RATE>200);
}




/*******************************************************************************
** 函数名称: check_selectedMode_ouputPWM
** 功能描述: 检查模式，并对应的输出PWM波形
** 输　  入: 无
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
void check_selectedMode_ouputPWM()
{
//	static uint16_t pressure_result; 
	#ifdef _DEBUG
	#else
//	if(mcu_state==POWER_ON&&b_check_BAT_ok==TRUE)
	if(mcu_state==POWER_ON)
//	if(1==0)
	#endif
	{
		//1.从flash中加载参数到内存
		if(state==LOAD_PARA)      
		{
			uint8_t len=PARAMETER_BUF_LEN/4;  
			uint32_t tmp[PARAMETER_BUF_LEN/4]={0};   		
			
			//读取flash数据到buffer中
			FlashRead(FLASH_WRITE_START_ADDR,tmp,len);
			memcpy(buffer,tmp,PARAMETER_BUF_LEN);
			CheckFlashData(buffer);
			
//			get_pressure_sensor_rate();  //获取压力sensor的斜率
//			//PRESSURE_RATE=FlashReadWord(FLASH_PRESSURE_RATE_ADDR);
//			//state=GET_MODE;
			state=CPY_PARA_TO_BUFFER;
		}
//		//2.获得开关对应的模式
//		if(state==GET_MODE)    //flash参数加载内存之后，获取开关对应的模式
//		{
//			state=CPY_PARA_TO_BUFFER;
//		}
		
		//3.根据选择的模式将数据拷贝到pwm_buffer
		if(state==CPY_PARA_TO_BUFFER)  //根据选择的模式，将para填充到pwm_buffer中
		{
			uint8_t pwm_buffer[144];
			
			memset(pwm1_buffer,0,49);
			memset(pwm2_buffer,0,49);
			memset(pwm3_buffer,0,49);
			
			switch(mode)
			{
				case 1:
					memcpy(pwm_buffer,buffer+2,144);            
					break;
				case 2:
					memcpy(pwm_buffer,buffer+146,144);
					break;
				case 3:
					memcpy(pwm_buffer,buffer+290,144);
					break;
				default:
					break;
			}
			FillUpPWMbuffer(pwm1_buffer,pwm_buffer);
			FillUpPWMbuffer(pwm2_buffer,pwm_buffer+48);
			FillUpPWMbuffer(pwm3_buffer,pwm_buffer+96);
//			state=CHECK_PRESSURE;
			state=PREV_OUTPUT_PWM;   //直接跳过压力，让系统一直跑
		}
		
		//4.检测压力
		if(state==CHECK_PRESSURE) //检测压力
		{
//			if(b_getHoneywellZeroPoint)
			{
				if(CHECK_MODE_OUTPUT_PWM*checkPressAgain_cnt==60*1000)   //连续60s检测不到，进入POWER_OFF
				{
					checkPressAgain_cnt=0;
					mcu_state=POWER_OFF;
					state=LOAD_PARA;
					set_led(LED_CLOSE,TRUE);
					
					//60s内没触发，记录
					record_dateTime(CODE_NO_TRIGGER_IN_60S);
					
					EnterStopMode();
					init_system_afterWakeUp();
				}
				else
				{
					if(b_getHoneywellZeroPoint)
					{
						if(adc_pressure_value<=trans_xmmHg_2_adc_value(buffer[0]))
						{
							checkPressAgain_cnt++;
						}
						else	
						{
							checkPressAgain_cnt=0;

							if(adc_pressure_value>trans_xmmHg_2_adc_value(PRESSURE_SAFETY_THRESHOLD))  
							{
								state=OVER_THRESHOLD_SAFETY;
							}

							else if(adc_pressure_value>=trans_xmmHg_2_adc_value(buffer[0])&&adc_pressure_value<=trans_xmmHg_2_adc_value(PRESSURE_SAFETY_THRESHOLD))
							{
								//触发成功，记录治疗开始时间
								record_dateTime(CODE_SYSTEM_BEEN_TRIGGERED);
								
								state=PREV_OUTPUT_PWM;
								checkPressAgain_cnt=0;
								waitBeforeStart_timing_flag=TRUE;
								prev_WaitBeforeStart_os_tick=0;
							}
							else
							{
								//do nothing
							}
						}
					}
					else  //如果得不到honeywell零点，说明sensor坏了，不用在进行数据比较了，直接60s倒计时
					{
						checkPressAgain_cnt++;
					}
					
				}
			}
		}

		//5.检测压力Ok,则预备输出波形，先定时waitBeforeStart这么长时间
		if(state==PREV_OUTPUT_PWM)  //开始预备输出PWM波形
		{
			//如果不加if(b_Is_PCB_PowerOn==FALSE)会导致开关重新开机waitbeforestart定时不到想要的秒数
			#ifdef _DEBUG
			#else
			if(b_Is_PCB_PowerOn==FALSE)
			{
				PWM_waitBeforeStart_cnt=0;
			}
			else
			#endif
			{
				if(Is_timing_Xmillisec(buffer[1]*1000,6))
				{
					state=OUTPUT_PWM;
					pwm1_state=PWM_START;
					pwm2_state=PWM_START;
					pwm3_state=PWM_START;
				}
			}  
		}
		
		//6.开始输出波形
		if(state==OUTPUT_PWM) //按照设定的参数，输出PWM1,PWM2,PWM3
		{			
			if(pwm1_state==PWM_OUTPUT_FINISH&&pwm2_state==PWM_OUTPUT_FINISH)
			{
				PWM1_serial_cnt=0;
				PWM2_serial_cnt=0;
				PWM3_serial_cnt=0;
				state=CHECK_BAT_VOL;
				
				//一次有效触发的治疗结束
				record_dateTime(CODE_ONE_CYCLE_FINISHED);
			}		
			else
			{
				//if(adc_value[1]<=PRESSURE_SAFETY_THRESHOLD*PRESSURE_RATE)
//				if(adc_value[1]<=PRESSURE_SENSOR_VALUE(PRESSURE_SAFETY_THRESHOLD))
	
				if(adc_pressure_value<=trans_xmmHg_2_adc_value(PRESSURE_SAFETY_THRESHOLD))
				{
					PaintPWM(1,pwm1_buffer); 
					PaintPWM(2,pwm2_buffer);
					//PaintPWM(3,pwm3_buffer);
				}
				else
				{
					state=OVER_THRESHOLD_SAFETY;
				}
			}
		}
		
		//如果超过了threshold的安全值，关闭设备，保护耳朵
		if(state==OVER_THRESHOLD_SAFETY)
		{
	//		Motor_shake_for_sleep();
			
			//记录过压
			record_dateTime(CODE_OVER_PRESSURE);
			
			set_led(LED_CLOSE,TRUE);
			set_led(LED_YELLOW,TRUE);
			for(uint8_t i=0;i<5;i++)
			{
				//set_led(LED_CLOSE);
				Motor_PWM_Freq_Dudy_Set(1,100,50);
				Motor_PWM_Freq_Dudy_Set(2,100,50);
//				Motor_PWM_Freq_Dudy_Set(3,100,0);
				Delay_ms(500);
				//IWDG_Feed(); 
				//set_led(LED_RED);
				Motor_PWM_Freq_Dudy_Set(1,100,0);
				Motor_PWM_Freq_Dudy_Set(2,100,0);
				Delay_ms(500);
				IWDG_Feed();
			}
			Motor_PWM_Freq_Dudy_Set(1,100,0);
			Motor_PWM_Freq_Dudy_Set(2,100,0);
			
			EnterStopMode();
			init_system_afterWakeUp();
		}
		
		//7.波形输出完毕，检测电池电压
		if(state==CHECK_BAT_VOL) 
		{
			#ifdef _DEBUG
			#else
//			led_state=Check_Bat();  //没有必要检测电池电压，因为开机已经检测过了
			#endif
			
			state=LOAD_PARA;
			pwm1_state=PWM_START;
			pwm2_state=PWM_START;
			pwm3_state=PWM_START;
			#if 0
//			uint16_t result;
//			result=RegularConvData_Tab[0];
//			if(result<2730) //如果电压小于2.2v,（基准3.3v）
//			{
//				//闪灯，进入POWER_OFF
//				state=LED_RED_BLINK;
//			}
//			else
//			{
//				state=LOAD_PARA;
//				pwm1_state=PWM_START;
//				pwm2_state=PWM_START;
//				pwm3_state=PWM_START;
//			}
//		}
	#endif
#if 0		
//		//对应4，压力检测，如果检测压力不ok，则再次检测压力
//		if(state==CHECK_PRESSURE_AGAIN) //再次检测压力
//		{
//			if(CHECK_MODE_OUTPUT_PWM*checkPressAgain_cnt==60*1000)   //连续60s检测不到，进入POWER_OFF
//			{
//				checkPressAgain_cnt=0;
//				mcu_state=POWER_OFF;
//				state=LOAD_PARA;
//				set_led(LED_CLOSE);
//				
//				EnterStopMode();
//				init_system_afterWakeUp();
//			}
//			else
//			{
//				//pressure_result=ADS115_readByte(0x90);
//				//adc_value[1]=ADS115_readByte(0x90);
//				//特别注意，这里不能用全局变量buffer,而应该用parameter_buf
//				//理由：如果进入60s倒计时状态，此时的buffer的值在CHECK_PRESSURE_AGAIN状态已经固定了
//				//如果此时上位机更新了参数，parameter_buf[0]会改变，应该用这个变化了的值来判断
//				if(adc_value[1]<parameter_buf[0]*70) 
//				{
//					checkPressAgain_cnt++;
//				}
//				else	
//				{
//					checkPressAgain_cnt=0;
//					state=LOAD_PARA;
//					
//				}
//			}
//		}
#endif

//		//对应7，如果检测电池电压小于2.2V，则闪灯
//		if(state==LED_RED_BLINK)
//		{
#if 0
//			//橙色LED闪3s
//			for(int i=0;i<3;i++)
//			{
//				set_led(LED_RED);
//				Delay_ms(500);
//				set_led(LED_CLOSE);
//				Delay_ms(500);
//				IWDG_Feed();   //喂狗
//			}
//			//Delay_ms(10);
//			state=LOAD_PARA;
//			pwm1_state=PWM_START;
//			pwm2_state=PWM_START;
//			pwm3_state=PWM_START;
//			mcu_state=POWER_OFF;
//			
//			EnterStopMode();
//			init_system_afterWakeUp();
#endif
		}
	}
//	else
//	{
//		//进入低功耗模式
////		EnterStopMode();
////		init_system_afterWakeUp();
////		Motor_PWM_Init();
//	}
	IWDG_Feed();   //喂狗
	os_delay_ms(TASK_OUTPUT_PWM, CHECK_MODE_OUTPUT_PWM);
}
/*******************************************************************************
** 函数名称: CMD_ProcessTask
** 功能描述: 命令解析任务
** 输　  入: arg  创建任务时传递的参数
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
void CMD_ProcessTask (void)
{
	//循環
	#ifdef _DEBUG
	#else
	if(mcu_state==POWER_ON)
	#endif
	{
		ReceiveData(&g_CmdReceive);//接收数据到缓冲区
		ModuleUnPackFrame();//命令处理
	}
	
	os_delay_ms(RECEIVE_TASK_ID, 100);
	//os_delay_ms(RECEIVE_TASK_ID, 300);
}
