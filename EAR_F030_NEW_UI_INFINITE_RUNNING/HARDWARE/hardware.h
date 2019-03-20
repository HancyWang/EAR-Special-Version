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
#ifndef __HARDWARE_
#define __HARDWARE_

/***********************************
* 头文件
***********************************/
#include "datatype.h"
#include "stm32f0xx.h"
/**********************************
*宏定义
***********************************/
////呼吸检测
//#define EXP_DETECT_PIN    GPIO_Pin_9
//#define EXP_DETECT_PORT   GPIOA
////LED 绿色    
//#define GREEN_LED_PIN    GPIO_Pin_9
//#define GREEN_LED_PORT   GPIOB
////LED 红色    
//#define RED_LED_PIN    GPIO_Pin_8
//#define RED_LED_PORT   GPIOB
//////LED 蓝色    
////#define BLUE_LED_PIN    GPIO_Pin_1
////#define BLUE_LED_PORT   GPIOB

#define GREEN_LED_PIN_1 		GPIO_Pin_8
#define GREEN_LED_PORT_1   	GPIOB

#define GREEN_LED_PIN_2 		GPIO_Pin_9
#define GREEN_LED_PORT_2   	GPIOB

#define GREEN_LED_PIN_3 		GPIO_Pin_10
#define GREEN_LED_PORT_3   	GPIOB

#define YELLO_LED_CTRL_PIN	GPIO_Pin_11
#define YELLO_LED_PORT			GPIOB


//开机按键	PA0
#define KEY_DETECT_PIN    GPIO_Pin_0
#define KEY_DETECT_PORT   GPIOA
//PWR_SAVE	PA5
#define KEY_PWR_SAVE_PIN  GPIO_Pin_5
#define KEY_PWR_SAVE_PORT   GPIOA
//按键模式	PA4
#define KEY_MODE_PIN			GPIO_Pin_4
#define KEY_MODE_PORT			GPIOA

#define ADC1_DR_Address                0x40012440

/***********************************
* 全局变量
***********************************/

/***********************************
* 变量定义
***********************************/
typedef enum{
	LED_CLOSE,
	LED_GREEN_1,
	LED_GREEN_2,
	LED_GREEN_3,
	LED_YELLOW,
}LED_COLOR;

//时间结构体
typedef struct 
{
		//公历日月年周
	uint16_t w_year;
	uint8_t  w_month;
	uint8_t  w_date;
	
	uint8_t hour;
	uint8_t min;
	uint8_t sec;			 
}_calendar_obj;
/***********************************
* 外部函数
***********************************/
void init_hardware(void);
//void init_hardware_byWakeUpOrNot(BOOL bWakeUp);
void init_rtc(void);
void init_tim(void);
void convert_rtc(_calendar_obj* calendar, uint32_t rtc);
void set_rtc(uint32_t rtc);
void set_led(LED_COLOR color,BOOL ON_OFF);
uint32_t get_rtc(void);
BOOL get_exp_status(void);
BOOL get_key_status(void);
BOOL save_one_page_to_flash(uint32_t Address, uint8_t* buf, uint16_t len);
void read_one_page_from_flash(uint32_t Address, uint8_t* buf, uint16_t len);
BOOL save_half_word_to_flash(uint32_t Address, uint16_t data);
void read_half_word_from_flash(uint32_t Address, uint16_t* pdata);
BOOL save_half_word_buf_to_eeprom(uint32_t Address, uint16_t* buf, uint16_t len);
void read_half_word_buf_from_eeprom(uint32_t Address, uint16_t* buf, uint16_t len);
uint8_t get_bat_vol_per(void);
void show_mode_LED(void);

//void Init_LED(void);

void Init_PWRSAVE(void);

//配置按键key_wakeup
void Key_WakeUp_Init(void);

////初始化flash中的参数
//void Init_parameter_in_Flash(void);

//ADC
void Init_ADC1(void);
//uint16_t Adc_Switch(uint32_t ADC_Channel);

//I2C
void ADS115_Init(void);


void Calibrate_pressure_sensor(int16_t* p_zeroPoint);
#endif
