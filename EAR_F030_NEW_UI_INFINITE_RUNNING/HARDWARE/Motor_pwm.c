/**
********************************************************************************
* 版權：
* 模块名称：hardware.c
* 模块功能：
* 创建日期：
* 创 建 者：
* 声    明：
********************************************************************************
**/

/***********************************
* 头文件
***********************************/
#include "Motor_pwm.h"
#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_pwr.h"
#include "stm32f0xx_rtc.h"

#include "delay.h"
#include "os_cfg.h"
#include "datatype.h"

#include "time.h"

/**********************************
*宏定义
***********************************/

/***********************************
* 全局变量
***********************************/

//PWM输出
#define PWM1_PIN    GPIO_Pin_6
#define PWM1_PORT   GPIOA

//#define PWM2_PIN    GPIO_Pin_7
//#define PWM2_PORT   GPIOA

//#define PWM3_PIN    GPIO_Pin_1
//#define PWM3_PORT   GPIOB

#define PWM2_PIN    GPIO_Pin_1
#define PWM2_PORT   GPIOB

static void Motor_PWM_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
  /* GPIOA and GPIOB clock enable */
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOF, ENABLE); 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB, ENABLE); 

//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;


	GPIO_PinAFConfig(PWM1_PORT, GPIO_PinSource6,GPIO_AF_1);  //PA6  TIM3
	GPIO_InitStructure.GPIO_Pin = PWM1_PIN;
  GPIO_Init(PWM1_PORT, &GPIO_InitStructure);
	
//	GPIO_PinAFConfig(PWM2_PORT, GPIO_PinSource7,GPIO_AF_5);
//	GPIO_InitStructure.GPIO_Pin = PWM2_PIN;
//  GPIO_Init(PWM2_PORT, &GPIO_InitStructure);

//	GPIO_PinAFConfig(PWM3_PORT, GPIO_PinSource1,GPIO_AF_0);
//	GPIO_InitStructure.GPIO_Pin = PWM3_PIN;
//  GPIO_Init(PWM3_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(PWM2_PORT, GPIO_PinSource1,GPIO_AF_0); //PB1 TIM14
	GPIO_InitStructure.GPIO_Pin = PWM2_PIN;
	GPIO_Init(PWM2_PORT, &GPIO_InitStructure);
}

static void Motor_PWM_Config(void)  //分频
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
	
	/* TIM3CLK enable*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 	
	/* TIM14CLK enable*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE); 	
//	/* TIM17CLK enable*/
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE); 	
	

	//TIM3定时周期为
  /* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 48000-1;       //周期为48000,即1Hz 
  TIM_TimeBaseStructure.TIM_Prescaler = 1000-1;	    //1000预分频
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//设置时钟分频系数：不分频(这里用不到)
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
//	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
	
//	TIM_BDTRInitStruct.TIM_AutomaticOutput = ENABLE;
//	TIM_BDTRConfig(TIM17, &TIM_BDTRInitStruct);
	

//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//OUT ENABLE
//  TIM_OCInitStructure.TIM_Pulse = 0;//12000;	   //当计数器计数到这个值时，电平发生跳变
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //有效为高电平输出

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//OUT ENABLE
  TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;  //@修改PWM2
	TIM_OCInitStructure.TIM_Pulse = 0;//12000;	   //当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //有效为高电平输出
//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;  //@修改PWM2
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);	 //使能TIM3CH1
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);	//自动重载	

	
//	TIM_OC1Init(TIM17, &TIM_OCInitStructure);	 //使能TIM17CH1
//  TIM_OC1PreloadConfig(TIM17, TIM_OCPreload_Enable);	//自动重载	
	
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);	 //使能TIM14CH1
  TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);	//自动重载

	
  TIM_ARRPreloadConfig(TIM3, ENABLE);			 // 使能TIM3重载寄存器ARR
	TIM_ARRPreloadConfig(TIM14, ENABLE);			 // 使能TIM3重载寄存器ARR
//	TIM_ARRPreloadConfig(TIM17, ENABLE);			 // 使能TIM3重载寄存器ARR

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);                   //使能定时器3	
	TIM_Cmd(TIM14, ENABLE);                   //使能定时器14
//	TIM_Cmd(TIM17, ENABLE);                   //使能定时器17
	
//	TIM17->BDTR  = TIM_BDTR_MOE;
}


void Motor_PWM_Init(void)
{
	Motor_PWM_GPIO_Config();
	Motor_PWM_Config();
	
//	Motor_PWM_Freq_Dudy_Set(1,50,30);
//	Motor_PWM_Freq_Dudy_Set(2,100,50);
//	delay_ms(500);
//	Motor_PWM_Freq_Dudy_Set(1,50,0);
//	Motor_PWM_Freq_Dudy_Set(2,100,0);

}

void Motor_PWM_Freq_Dudy_Set(UINT8 PWM_NUMBER, UINT16 Freq,UINT16 Duty)			//PWM1-2-3,FREQ,DUFY
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	UINT32 i;	
	
	if((Freq >=1) && (Freq <=3000)// Frequency  1 - 255Hz
		&& (Duty <= 100))//Duty cycle 10 - 90
	{
		TIM_TimeBaseStructure.TIM_Period = 48000/Freq - 1;       //
		TIM_TimeBaseStructure.TIM_Prescaler = 1000-1;	    //1000预分频
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//设置时钟分频系数：不分频(这里用不到)
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
		//TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	
		
		i = TIM_TimeBaseStructure.TIM_Period + 1;
		i *= Duty;
		i /= 100;
		
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//OUT ENABLE
		TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;  //@修改PWM2
		TIM_OCInitStructure.TIM_Pulse = i;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //有效为高电平输出
		//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;  //@修改PWM2
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
		
//		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
//		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
//		TIM_OCInitStructure.TIM_Pulse = i;//0;	
//		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为低电平
		if(PWM_NUMBER == 1)
		{
			TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	
			TIM_OC1Init(TIM3, &TIM_OCInitStructure);	 //使能TIM3CH1
			TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);	//		
		}
//		else if(PWM_NUMBER == 2)
//		{
//			TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);	
//			TIM_OC1Init(TIM17, &TIM_OCInitStructure);	 //使能TIM3CH1
//			TIM_OC1PreloadConfig(TIM17, TIM_OCPreload_Enable);	//		
//		}
		else if(PWM_NUMBER == 2)
		{
			TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);	
			TIM_OC1Init(TIM14, &TIM_OCInitStructure);	 
			TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);		
		}		
		//TIM_SetCompare1(TIM3, Wire_Temp);
	}
}


  



