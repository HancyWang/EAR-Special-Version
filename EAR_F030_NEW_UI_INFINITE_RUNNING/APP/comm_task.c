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

#define HONEYWELL_RATE			11110   //б��
extern uint32_t HONEYWELL_ZERO_POINT;
extern uint32_t trans_xmmHg_2_adc_value(uint8_t xmmHg);
extern BOOL b_getHoneywellZeroPoint;

//#define PRESSURE_RATE 70
//#define PRESSURE_RATE (FlashReadWord(FLASH_PRESSURE_RATE_ADDR))
uint16_t PRESSURE_RATE;

#define PRESSURE_SAFETY_THRESHOLD 160   //0xA0��ʾ10.0����Ӧʮ����160
//y=ax+b
#define PRESSURE_SENSOR_VALUE(x) ((int16_t)(((PRESSURE_RATE)*(x))+zero_point_of_pressure_sensor))


//ȫ�ֱ���
CMD_Receive g_CmdReceive;  // ������տ��ƶ���
FIFO_TYPE send_fifo;//�l�͔���FIFO
UINT8 send_buf[SEND_BUF_LEN];

//����������λ���������Ĳ���
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

//uint32_t adc_value[2]={0xFFFF,0x00}; //[0]��ӦNTC��[1]��Ӧpressure
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
*                                �ڲ���������
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
** ��������: ModuleUnPackFrame
** ��������: ������մ���
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
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

	// �Ӵ��ڻ������ж�ȡ���յ�������
	dwLen = GetBuf2Length(&g_CmdReceive);

	// �����ݽ��н���
	while(0 < dwLen)
	{
		byCurChar = Buf2Read(&g_CmdReceive);

		if (sPacketHeadFlag)
		{
			// ��������ͷ
			if(sPacketLenFlag)
			{
				// ������������
				pBuff[sCurPacketLen] = byCurChar;
				sCurPacketLen ++;
				sResetByte --;

				if (0 >= sResetByte)
				{
					// �������
					// ����У��ͱȽ�
					if (CheckCheckSum(pBuff, pBuff[1]))
					{
						// ������һ����Ч���ݰ�
						memcpy(sBackBuff, sDataBuff, CMD_BUFFER_LENGTH);
						ModuleProcessPacket(sBackBuff);//������*********************************
						//��ֹ�������ն�������ʱ������Ӧ���źų�����
						delay_ms(2);
					}

					sPacketHeadFlag = FALSE;
					sPacketLenFlag = FALSE;
					memset(&sDataBuff, 0x00, CMD_BUFFER_LENGTH);
				}													
			}
			else
			{
				if((CMD_BUFFER_LENGTH-1 > byCurChar) && (0 < byCurChar ))// �ݴ���������ֹ���ݰ���Ϊ49��0ʱ��� ����X5����������Ͱ�Ϊ15
				{
					// ������ģ��ĳ���
					sDataBuff[sCurPacketLen] = byCurChar;
					sResetByte = byCurChar;			
					sPacketLenFlag = TRUE;
					sCurPacketLen ++;
				}
				else
				{
					//û�н�����ģ��ĳ���, ���½���
					sPacketHeadFlag = FALSE;
					sPacketLenFlag = FALSE;					
				}
			}
		}
		
		else if (PACK_HEAD_BYTE == byCurChar)		
		{
			// ��������ͷ
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
** ��������: CheckCheckSum
** ��������: ��У��
** �䡡  ��: pData ���� nLen����
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
UINT8 CheckCheckSum(UINT8* pData, UINT8 nLen)
{
	UINT16 bySum = 0;
	int i;
	// �������ݵ�У���	
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
** ��������: ModuleProcessPacket
** ��������: ������
** �䡡  ��: pData ����
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
BOOL ModuleProcessPacket(UINT8 *pData)
{	
	protocol_module_process(pData);

	return TRUE;     	
}


/*******************************************************************************
* �������� : TaskDataSend
* �������� : ���ݷ�������5msִ��һ��
* ������� : arg  ��������ʱ���ݵĲ���
* ������� : ��
* ���ز��� : ��
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
				//ѭ�h
			len = fifoReadData(&send_fifo, send_data_buf, SEND_DATA_BUF_LENGTH);
			if(len)
			{
					UartSendNBytes(send_data_buf, len);
					delay_ms(30);
			}
		}
		
		os_delay_ms(SEND_TASK_ID, 30);  //markһ��
		//os_delay_ms(SEND_TASK_ID, 28);
}

//��ʱx����,n_ms����255s��255000
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
//		case 7:   //ģʽ���صİ���ʱ�� ,���ʺ����������
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
	if(os_ticks+n_ms<os_ticks) //���os_ticks+n_ms����ˣ���ôos_ticks+n_ms��ȻС��os_ticks
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
		if(led_state==LED_RED_SOLID_NO_POWER) //�ػ�
		{
			set_led(LED_CLOSE,TRUE);
			set_led(LED_YELLOW,TRUE);
//			
////			//��¼�ػ�
////			record_dateTime(CODE_NO_POWER); //û�翪�������ü�¼
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
					if(flash_cnt==4)  //�Ƶ���5��Ȼ����
					{
						led_state=LED_NONE;
						flash_cnt=0;
						show_mode_LED();  //��ʾģʽ��	
						
						Motor_PWM_Freq_Dudy_Set(1,100,50);
						Motor_PWM_Freq_Dudy_Set(2,100,50);
						Delay_ms(500);
						Motor_PWM_Freq_Dudy_Set(1,100,0);
						Motor_PWM_Freq_Dudy_Set(2,100,0);
						
						b_check_BAT_ok=TRUE;
						//��¼����ʱ��
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
			show_mode_LED();  //��ʾģʽ��
			led_state=LED_NONE;
			
			//��¼����ʱ��
			record_dateTime(CODE_SYSTEM_POWER_ON);
			
			//������
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
////�ɼ�ADS115��ADCֵ
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
//				adc_value[0]=ADS115_readByte(0x90);  //�¶�
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
//			//��ɫLED��3s
////			for(int i=0;i<3;i++)
////			{
////				set_led(LED_RED);
////				Delay_ms(500);
////				set_led(LED_CLOSE);
////				Delay_ms(500);
////				IWDG_Feed();   //ι��
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
//		//��ɫLED��3s
//		for(int i=0;i<3;i++)
//		{
//			set_led(LED_RED);
//			Delay_ms(500);
//			set_led(LED_CLOSE);
//			Delay_ms(500);
//			IWDG_Feed();   //ι��
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
** ��������: get_switch_mode
** ��������: ��ȡ��������Ӧ��ģʽ
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
void get_switch_mode()
{
	if(mcu_state==POWER_ON||b_check_BAT_ok==TRUE)
	{
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)==0)  //����������
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

					//�������³�ʼ�����ر�����
					init_PWMState();
					state=LOAD_PARA;
					Motor_PWM_Freq_Dudy_Set(1,100,0);
					Motor_PWM_Freq_Dudy_Set(2,100,0);
					
						//ѡ��ģʽ��
					switch(mode)
					{
						case 1:   //��ǰ��ģʽ1������֮��ͱ����ģʽ2������mode=2������յ��
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
* �������� : TaskDataSend
* �������� : ���ݷ�������5msִ��һ��
* ������� : arg  ��������ʱ���ݵĲ���
* ������� : ��
* ���ز��� : ��
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
	//���������flash���ݿ�����buffer��
	for(int i=0;i<PARAMETER_BUF_LEN;i++)
	{
		buffer[i]=default_parameter_buf[i];
	}
	//����flash
	FlashWrite(FLASH_WRITE_START_ADDR,buffer,PARAMETER_BUF_LEN/4);
}

void CheckFlashData(unsigned char* buffer)
{
	uint16_t j=0;
	//������ݳ�������Ĭ�ϵ�����
	if(buffer[0]>249)
	{
		ResetParameter(buffer);
		return;
	}
	for(int i=0;i<54;i++)
	{
		j++;                 //1.������һ��
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
** ��������: FillUpPWMbuffer
** ��������: ����serial���к��������pwm1_buffer,pwm2_buffer,pwm3_buffer
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
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

//�ݴ������������sensorб��ֵ��[10,200]֮�䣬Ϊok�������10��֮��͸�sensorĬ��ֵ20
void get_pressure_sensor_rate()
{
	uint8_t readCnt=0;
	do
	{
		if(readCnt==10) //�����10�ζ�����[10,200]֮�䣬��Ϊ������
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
** ��������: check_selectedMode_ouputPWM
** ��������: ���ģʽ������Ӧ�����PWM����
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
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
		//1.��flash�м��ز������ڴ�
		if(state==LOAD_PARA)      
		{
			uint8_t len=PARAMETER_BUF_LEN/4;  
			uint32_t tmp[PARAMETER_BUF_LEN/4]={0};   		
			
			//��ȡflash���ݵ�buffer��
			FlashRead(FLASH_WRITE_START_ADDR,tmp,len);
			memcpy(buffer,tmp,PARAMETER_BUF_LEN);
			CheckFlashData(buffer);
			
//			get_pressure_sensor_rate();  //��ȡѹ��sensor��б��
//			//PRESSURE_RATE=FlashReadWord(FLASH_PRESSURE_RATE_ADDR);
//			//state=GET_MODE;
			state=CPY_PARA_TO_BUFFER;
		}
//		//2.��ÿ��ض�Ӧ��ģʽ
//		if(state==GET_MODE)    //flash���������ڴ�֮�󣬻�ȡ���ض�Ӧ��ģʽ
//		{
//			state=CPY_PARA_TO_BUFFER;
//		}
		
		//3.����ѡ���ģʽ�����ݿ�����pwm_buffer
		if(state==CPY_PARA_TO_BUFFER)  //����ѡ���ģʽ����para��䵽pwm_buffer��
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
			state=PREV_OUTPUT_PWM;   //ֱ������ѹ������ϵͳһֱ��
		}
		
		//4.���ѹ��
		if(state==CHECK_PRESSURE) //���ѹ��
		{
//			if(b_getHoneywellZeroPoint)
			{
				if(CHECK_MODE_OUTPUT_PWM*checkPressAgain_cnt==60*1000)   //����60s��ⲻ��������POWER_OFF
				{
					checkPressAgain_cnt=0;
					mcu_state=POWER_OFF;
					state=LOAD_PARA;
					set_led(LED_CLOSE,TRUE);
					
					//60s��û��������¼
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
								//�����ɹ�����¼���ƿ�ʼʱ��
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
					else  //����ò���honeywell��㣬˵��sensor���ˣ������ڽ������ݱȽ��ˣ�ֱ��60s����ʱ
					{
						checkPressAgain_cnt++;
					}
					
				}
			}
		}

		//5.���ѹ��Ok,��Ԥ��������Σ��ȶ�ʱwaitBeforeStart��ô��ʱ��
		if(state==PREV_OUTPUT_PWM)  //��ʼԤ�����PWM����
		{
			//�������if(b_Is_PCB_PowerOn==FALSE)�ᵼ�¿������¿���waitbeforestart��ʱ������Ҫ������
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
		
		//6.��ʼ�������
		if(state==OUTPUT_PWM) //�����趨�Ĳ��������PWM1,PWM2,PWM3
		{			
			if(pwm1_state==PWM_OUTPUT_FINISH&&pwm2_state==PWM_OUTPUT_FINISH)
			{
				PWM1_serial_cnt=0;
				PWM2_serial_cnt=0;
				PWM3_serial_cnt=0;
				state=CHECK_BAT_VOL;
				
				//һ����Ч���������ƽ���
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
		
		//���������threshold�İ�ȫֵ���ر��豸����������
		if(state==OVER_THRESHOLD_SAFETY)
		{
	//		Motor_shake_for_sleep();
			
			//��¼��ѹ
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
		
		//7.���������ϣ�����ص�ѹ
		if(state==CHECK_BAT_VOL) 
		{
			#ifdef _DEBUG
			#else
//			led_state=Check_Bat();  //û�б�Ҫ����ص�ѹ����Ϊ�����Ѿ�������
			#endif
			
			state=LOAD_PARA;
			pwm1_state=PWM_START;
			pwm2_state=PWM_START;
			pwm3_state=PWM_START;
			#if 0
//			uint16_t result;
//			result=RegularConvData_Tab[0];
//			if(result<2730) //�����ѹС��2.2v,����׼3.3v��
//			{
//				//���ƣ�����POWER_OFF
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
//		//��Ӧ4��ѹ����⣬������ѹ����ok�����ٴμ��ѹ��
//		if(state==CHECK_PRESSURE_AGAIN) //�ٴμ��ѹ��
//		{
//			if(CHECK_MODE_OUTPUT_PWM*checkPressAgain_cnt==60*1000)   //����60s��ⲻ��������POWER_OFF
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
//				//�ر�ע�⣬���ﲻ����ȫ�ֱ���buffer,��Ӧ����parameter_buf
//				//���ɣ��������60s����ʱ״̬����ʱ��buffer��ֵ��CHECK_PRESSURE_AGAIN״̬�Ѿ��̶���
//				//�����ʱ��λ�������˲�����parameter_buf[0]��ı䣬Ӧ��������仯�˵�ֵ���ж�
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

//		//��Ӧ7���������ص�ѹС��2.2V��������
//		if(state==LED_RED_BLINK)
//		{
#if 0
//			//��ɫLED��3s
//			for(int i=0;i<3;i++)
//			{
//				set_led(LED_RED);
//				Delay_ms(500);
//				set_led(LED_CLOSE);
//				Delay_ms(500);
//				IWDG_Feed();   //ι��
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
//		//����͹���ģʽ
////		EnterStopMode();
////		init_system_afterWakeUp();
////		Motor_PWM_Init();
//	}
	IWDG_Feed();   //ι��
	os_delay_ms(TASK_OUTPUT_PWM, CHECK_MODE_OUTPUT_PWM);
}
/*******************************************************************************
** ��������: CMD_ProcessTask
** ��������: �����������
** �䡡  ��: arg  ��������ʱ���ݵĲ���
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
void CMD_ProcessTask (void)
{
	//ѭ�h
	#ifdef _DEBUG
	#else
	if(mcu_state==POWER_ON)
	#endif
	{
		ReceiveData(&g_CmdReceive);//�������ݵ�������
		ModuleUnPackFrame();//�����
	}
	
	os_delay_ms(RECEIVE_TASK_ID, 100);
	//os_delay_ms(RECEIVE_TASK_ID, 300);
}