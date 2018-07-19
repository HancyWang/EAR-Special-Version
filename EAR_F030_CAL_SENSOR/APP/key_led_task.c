

/**
********************************************************************************
* ��ࣺ
* ģ�����ƣ�key_led_task.c
* ģ�鹦�ܣ�
* �������ڣ�
* �� �� �ߣ�
* ��    ����
********************************************************************************
**/

/***********************************
* ͷ�ļ�
***********************************/

#include "app.h"
#include "datatype.h"
#include "hardware.h"
#include "fifo.h"
#include "key_led_task.h"
#include "protocol_module.h"

#include "i2c.h"
#include "Motor_pwm.h"
#include "stm32f0xx_pwr.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_dma.h"
#include "serial_port.h"
#include "CMD_receive.h"
#include "app.h"
#include "delay.h"
#include "comm_task.h"
#include "iwtdg.h"
/**********************************
*�궨��
***********************************/

/***********************************
* ȫ�ֱ���
***********************************/

/***********************************
* �ֲ�����
***********************************/
extern uint32_t os_ticks;
extern CMD_Receive g_CmdReceive;  // ������տ��ƶ���

extern FIFO_TYPE send_fifo;
extern uint8_t send_buf[SEND_BUF_LEN];
extern CHCKMODE_OUTPUT_PWM state;

extern PWM_STATE pwm1_state;
extern PWM_STATE pwm2_state;
extern PWM_STATE pwm3_state;

extern uint8_t led_high_cnt;
extern uint8_t led_low_cnt;
//KEYֵ������㰴Ϊȷ����������
typedef enum {
	NO_KEY,
	BLUE_CHECK
}KEY_VAL;

MCU_STATE mcu_state=POWER_OFF;
//mcu_state=POWER_OFF;

//extern uint8_t OUTPUT_FINISH;
BOOL b_Is_PCB_PowerOn=FALSE;
//BOOL b_check_bat=FALSE;
volatile KEY_STATE key_state=KEY_STOP_MODE;

extern uint16_t RegularConvData_Tab[2];
extern uint8_t adc_state;
extern THERMAL_STATE thermal_state;


LED_STATE led_state=LED_NONE;
/***********************************
* �ֲ�����
***********************************/

/*******************************************************************************
** ��������: EnterStopMode
** ��������: ����͹���ģʽ
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
void CfgPA0ASWFI()
{
	//ʱ��ʹ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOF,ENABLE);  
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); 
	
	
	//�ⲿ����GPIOA��ʼ��,PA0  
	GPIO_InitTypeDef GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;  
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  
	//GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;  
	GPIO_Init(GPIOA,&GPIO_InitStructure);  

	//��EXTI0ָ��PA0  
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);  
	//EXTI0�ж�������
	EXTI_InitTypeDef EXTI_InitStructure;  
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;  
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;  
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
	EXTI_Init(&EXTI_InitStructure);  

	//EXTI0�ж���������  
	NVIC_InitTypeDef NVIC_InitStructure;  
	NVIC_InitStructure.NVIC_IRQChannel=EXTI0_1_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
}


//���PA0(WAKE_UP)�Ƿ񱻰���
BOOL Check_wakeUpKey_pressed(void)
{
	//uint32_t cnt=0;
	while(TRUE)
	{
		//��ȡPA0�ĵ�ƽ
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
		{
			//cnt++;
			delay_ms(30);
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
			{
				while(TRUE)
				{
					delay_ms(10);
					if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1)
					{
						return TRUE;
					}
				}
			}
			//delay_ms(5);
//			if(cnt>20)
//			{
//				return TRUE;
//			}
		}
		else
		{
			return FALSE;
		}
	}
}

void EXTI0_1_IRQHandler(void)
{  
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)  
	{ 
		if(Check_wakeUpKey_pressed())
		{
			b_Is_PCB_PowerOn=!b_Is_PCB_PowerOn;		//ÿ��һ�Σ�b_Is_PCB_PowerOn��תһ��״̬
			if(b_Is_PCB_PowerOn==TRUE)
			{
				mcu_state=POWER_ON;	
				key_state=KEY_WAKE_UP;		
				state=LOAD_PARA;
				init_PWMState();
			}
			else
			{
				mcu_state=POWER_OFF;	
				key_state=KEY_STOP_MODE;
				state=LOAD_PARA;
				init_PWMState();
			}
		}
	}  
	//EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_ClearFlag(EXTI_Line0);
} 


void init_system_afterWakeUp()
{
	Init_iWtdg(4,1250);  //4*2^4=64��Ƶ��1250(�����1250*1.6ms=2s)
	os_ticks = 0;
	//os_ticks = 4294967290;
	
	delay_init();
	os_init();
	
	SystemInit();
	
	init_task();
	//os_start();	
	#if 0
//	delay_init();
//	os_init();
//	SystemInit();
//	
//	
//	//��ʼ��ͨ�����
//	fifoInit(&send_fifo,send_buf,SEND_BUF_LEN);
//	UARTInit(g_CmdReceive.m_Buf1, BUF1_LENGTH);	
//	Init_Receive(&g_CmdReceive);
//	
//	
//	GPIO_InitTypeDef  GPIO_InitStructure;

//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);

//	//������
//	GPIO_InitStructure.GPIO_Pin = EXP_DETECT_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_Init(EXP_DETECT_PORT, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = KEY_DETECT_PIN;
//	GPIO_Init(KEY_DETECT_PORT, &GPIO_InitStructure);
//	
//	//�������
//	GPIO_InitStructure.GPIO_Pin = GREEN_LED_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GREEN_LED_PORT, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = RED_LED_PIN;
//	GPIO_Init(RED_LED_PORT, &GPIO_InitStructure);
//	//GPIO_ResetBits(GPIOF, GPIO_Pin_0|GPIO_Pin_1);
//	
//	//��ԴPWR_SAVE
//	GPIO_InitStructure.GPIO_Pin = KEY_PWR_SAVE_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(KEY_PWR_SAVE_PORT, &GPIO_InitStructure);
//	GPIO_ResetBits(KEY_PWR_SAVE_PORT,KEY_PWR_SAVE_PIN);
//	
//	GPIO_InitStructure.GPIO_Pin = RED_LED_PIN;
//	GPIO_Init(RED_LED_PORT, &GPIO_InitStructure);
//	set_led(LED_CLOSE);
//	
//	//��ʼ��ADC
//	ADC1_Init();
//	//��ʼ��ADS115,I2C
//	ADS115_Init();
//	
//	Motor_PWM_Init();
	
#endif
}


void CfgALLPins4StopMode()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);
	
	//PF0,PF1,����Ϊ���
	GPIO_InitTypeDef GPIO_InitStructure_PF;
	GPIO_InitStructure_PF.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;                       
	GPIO_InitStructure_PF.GPIO_Speed = GPIO_Speed_50MHz;     
	GPIO_InitStructure_PF.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PF.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PF.GPIO_PuPd=GPIO_PuPd_UP;
	//GPIO_InitStructure_PF.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure_PF);
	GPIO_SetBits(GPIOF, GPIO_Pin_0|GPIO_Pin_1);
	
	
	//����ADC1��ADC4
	GPIO_InitTypeDef GPIO_InitStructure_PA_1_4;
	GPIO_InitStructure_PA_1_4.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4;
  GPIO_InitStructure_PA_1_4.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure_PA_1_4.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure_PA_1_4);	
	
	//�ر�ADC
	DMA_Cmd(DMA1_Channel1, DISABLE);/* DMA1 Channel1 enable */			
  ADC_DMACmd(ADC1, DISABLE);
	ADC_Cmd(ADC1, DISABLE);  
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , DISABLE);		
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , DISABLE);
	
	
	//PA2,PA3
	GPIO_InitTypeDef GPIO_InitStructure_PA_2_3;
	GPIO_InitStructure_PA_2_3.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;                       
	GPIO_InitStructure_PA_2_3.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA_2_3.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure_PA_2_3.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PA_2_3.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure_PA_2_3.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_2_3);
	
	//�رմ���
	DMA_Cmd(UART_DMA_RX_CHANNEL, DISABLE);
	DMA_Cmd(UART_DMA_TX_CHANNEL, DISABLE);
	USART_Cmd(UART, DISABLE);
	
	//PA9,PA10
	GPIO_InitTypeDef GPIO_InitStructure_UART;
	GPIO_InitStructure_UART.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;                       
	GPIO_InitStructure_UART.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_UART.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure_UART.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_UART.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_UART);
	
	//PA5
	GPIO_InitTypeDef GPIO_InitStructure_PA5;
	GPIO_InitStructure_PA5.GPIO_Pin = GPIO_Pin_5;                       
	GPIO_InitStructure_PA5.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA5.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure_PA5.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PA5.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure_PA5.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA5);
	//GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	
	//PWM1,PWM2
	GPIO_InitTypeDef GPIO_InitStructure_PWM_1_2;
	GPIO_InitStructure_PWM_1_2.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;                       
	GPIO_InitStructure_PWM_1_2.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PWM_1_2.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PWM_1_2.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PWM_1_2.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure_PWM_1_2.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PWM_1_2);
	//GPIO_SetBits(GPIOA,GPIO_Pin_6|GPIO_Pin_7);
	
	//PWM3
	GPIO_InitTypeDef GPIO_InitStructure_PWM3;
	GPIO_InitStructure_PWM3.GPIO_Pin = GPIO_Pin_1;                       
	GPIO_InitStructure_PWM3.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PWM3.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PWM3.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PWM_1_2.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure_PWM_1_2.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PWM3);
	//GPIO_SetBits(GPIOB,GPIO_Pin_1);
}

//����stopģʽ�������жϻ���
void EnterStopMode()
{
//	b_check_bat=FALSE;
	led_state=LED_NONE;
	led_high_cnt=0;
 led_low_cnt=0;
	thermal_state=THERMAL_NONE;
	adc_state=1;
	mcu_state=POWER_OFF;
	state=LOAD_PARA;
	init_PWMState();
	//�����ж�
	CfgPA0ASWFI();
	//I2CоƬADS115����power-downģʽ
	ADS115_enter_power_down_mode();

	CfgALLPins4StopMode();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);  
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}


LED_STATE Check_Bat()
{
	uint16_t result;
	result=RegularConvData_Tab[0];
	if(result<3003) //�����ѹС��2.2v,û���� ��ֱ�ӽ���͹���
	{
		//led_state=LED_RED_SOLID;
		return LED_RED_SOLID;
	}
	else if(result>=3003&&result<3549)  //2.2-2.6 �������û�����������
	{
		//led_state=LED_RED_FLASH;
		return LED_RED_FLASH;
	}
	else if(result>=3549)
	{
		//solid green,�����̵ƣ���ʾ������ֵ
		//led_state=LED_GREEN_SOLID;
		return LED_GREEN_SOLID;
	}
	else
	{
		//do nothing
		return LED_NONE;
	}
}


void key_led_task(void)
{
	if(key_state==KEY_STOP_MODE)
	{
		EnterStopMode();
		init_system_afterWakeUp();
	}
	
	//���������£�����ص�ѹ
	if(key_state==KEY_WAKE_UP)
	{
		//b_check_bat=TRUE;
		//if(b_Is_PCB_PowerOn)
		{
			if(RegularConvData_Tab[0]>3003)
			{
				//����
				//set_led(LED_GREEN);
				if(RegularConvData_Tab[0]>3549)
				{
					set_led(LED_GREEN);
				}
				Motor_PWM_Freq_Dudy_Set(1,100,80);
				Motor_PWM_Freq_Dudy_Set(2,100,80);
				Motor_PWM_Freq_Dudy_Set(3,100,80);
				Delay_ms(500);
				Motor_PWM_Freq_Dudy_Set(1,100,0);
				Motor_PWM_Freq_Dudy_Set(2,100,0);
				Motor_PWM_Freq_Dudy_Set(3,100,0);
				
				key_state=KEY_UPING;
				mcu_state=POWER_ON;
				
				led_state=Check_Bat();
			}
			else
			{
//				//��ɫLED��3s���ػ�
//				for(int i=0;i<3;i++)
//				{
//					set_led(LED_RED);
//					Delay_ms(500);
//					set_led(LED_CLOSE);
//					Delay_ms(500);
//					IWDG_Feed();   //ι��
//				}
//				//key_state=KEY_UPING;
				//key_state=KEY_STOP_MODE;
				set_led(LED_RED);
				for(int i=0;i<5;i++)
				{
					Motor_PWM_Freq_Dudy_Set(1,100,0);
					Motor_PWM_Freq_Dudy_Set(2,100,0);
					Motor_PWM_Freq_Dudy_Set(3,100,0);
					Delay_ms(500);
					Motor_PWM_Freq_Dudy_Set(1,100,50);
					Motor_PWM_Freq_Dudy_Set(2,100,50);
					Motor_PWM_Freq_Dudy_Set(3,100,50);
					Delay_ms(500);
					IWDG_Feed();
				}
				mcu_state=POWER_OFF;
				
				//����stopģʽ
				EnterStopMode();
				//����֮�����³�ʼ��
				init_system_afterWakeUp();
			}
		}

	}

	//IWDG_Feed();   //ι��
	os_delay_ms(KEY_LED_TASK_ID, KEY_LED_PERIOD);
}