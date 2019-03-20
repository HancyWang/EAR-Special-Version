/**
********************************************************************************
* 版權：
* 模块名称：protocol.c
* 模块功能：跟上位機進行通信
* 创建日期：
* 创 建 者：
* 声    明：
********************************************************************************
**/

/***********************************
* 头文件
***********************************/
#include "stm32f0xx_usart.h"
#include "stm32f0xx.h"
#include "datatype.h"
#include "serial_port.h"
#include "hardware.h"
#include "fifo.h"
#include "protocol_module.h"
#include "comm_task.h"
#include "os_core.h"
#include "app.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f0xx_flash.h"
#include "key_led_task.h"

#include "common.h"
#include "rtc.h"
/**********************************
*宏定义
***********************************/

/***********************************
* 全局变量
***********************************/
uint32_t pageBuff[512];
//BOOL rcvParameters_from_PC=FALSE;
uint8_t rcvParaSuccess=0x00;
//發送數據FIFO
extern FIFO_TYPE send_fifo;
extern uint8_t send_buf[SEND_BUF_LEN];

extern UINT8 parameter_buf[PARAMETER_BUF_LEN];  //长度为434+2，用来接收上位机发送来的参数设置
extern UINT16 check_sum;
//当接收到上位机发送数据命令时，该变量置为TRUE,发送完毕置为FALSE
//extern uint8_t send_exp_train_data_status;s
extern MCU_STATE mcu_state;
extern uint16_t RegularConvData_Tab[2];

uint8_t arr_mmgH_value[3];
uint16_t arr_adc_value[3];

//extern INT16U ADS115_readByte(INT8U slaveaddr);
//extern int16_t zero_point_of_pressure_sensor;

//uint32_t RATE;
typedef struct POINT
{
	uint8_t mmgh_value;
	uint16_t adc_value;
}POINT;



/***********************************
* 局部变量
***********************************/

/***********************************
* 局部函数
***********************************/
////發送有效呼吸检测
//void protocol_module_send_exp_flag(uint8_t flag)
//{
//	uint8_t buffer[CMD_BUFFER_LENGTH];
//	
//	buffer[0] = PACK_HEAD_BYTE;
//	buffer[1] = 0x05;
//	buffer[2] = MODULE_CMD_TYPE;
//	buffer[3] = SEND_VOILD_EXP_FLAG_ID;
//	buffer[4] = flag;
//	
//	CalcCheckSum(buffer);
//	
//	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
//}

////發送有效呼吸检测
//void protocol_module_send_train_data_one_page(uint8_t* buf, uint8_t len)
//{
//	uint8_t buffer[CMD_BUFFER_LENGTH];
//	uint8_t cnt,i = 0;
//	
//	buffer[0] = PACK_HEAD_BYTE;
//	buffer[1] = 0x05;
//	buffer[2] = MODULE_CMD_TYPE;
//	buffer[3] = SEND_EXP_TRAIN_DATA_ID;
//	for(cnt = 4; cnt < len+4; cnt ++)
//	{
//		buffer[cnt] = buf[i++];
//	}
//	
//	CalcCheckSum(buffer);
//	
//	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
//}

//發送数据校验和
void protocol_module_send_train_data_check_sum(uint32_t check_sum)
{
	uint8_t buffer[CMD_BUFFER_LENGTH];
	
	buffer[0] = PACK_HEAD_BYTE;
	buffer[1] = 0x08;
	buffer[2] = MODULE_CMD_TYPE;
	buffer[3] = SEND_EXP_TRAIN_DATA_CHECK_SUM_ID;
	buffer[4] = check_sum & 0xff;
	buffer[5] = (check_sum >> 8) & 0xff;
	buffer[6] = (check_sum >> 16) & 0xff;
	buffer[7] = (check_sum >> 24) & 0xff;
	
	CalcCheckSum(buffer);
	
	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
}

////發送数据校验和
//void protocol_module_send_bat_per(uint8_t bat_per)
//{
//	uint8_t buffer[CMD_BUFFER_LENGTH];
//	
//	buffer[0] = PACK_HEAD_BYTE;
//	buffer[1] = 0x08;
//	buffer[2] = MODULE_CMD_TYPE;
//	buffer[3] = SEND_BAT_PER_ID;
//	buffer[4] = bat_per;
//	
//	CalcCheckSum(buffer);
//	
//	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
//}

void get_comm_para_to_buf(uint8_t* pdata)
{
	memset(parameter_buf,0,PARAMETER_BUF_LEN);
	check_sum=0;
	UINT8* pPos=(UINT8*)&parameter_buf;
	memset(parameter_buf,0,PARAMETER_BUF_LEN);
	memcpy(pPos,pdata+4,1);
	memcpy(pPos+1,pdata+5,1);
	check_sum+=*pPos+*(pPos+1);
}

void get_parameter_to_buf_by_frameId(uint8_t* pdata,char frameId)
{
	int pos_mode1_pwm1=2;
	int pos_mode1_pwm2=50;
	int pos_mode1_pwm3=98;
	int pos_mode2_pwm1=146;
	int pos_mode2_pwm2=194;
	int pos_mode2_pwm3=242;
	int pos_mode3_pwm1=290;
	int pos_mode3_pwm2=338;
	int pos_mode3_pwm3=386;
	if(0x11==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode1_pwm1;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x12==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode1_pwm2;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x13==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode1_pwm3;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x21==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode2_pwm1;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x22==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode2_pwm2;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x23==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode2_pwm3;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x31==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode3_pwm1;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x32==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode3_pwm2;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
	}
	else if(0x33==frameId)
	{
		uint8_t* pstart=pdata+4;
		uint8_t* pDest=(uint8_t*)(&parameter_buf)+pos_mode3_pwm3;
		char buf[48]={0};
		for(int i=0;i<48;i++)
		{
			buf[i]=*pstart++;
			check_sum+=buf[i];
		}
		//memcpy(pDest,buf,sizeof(buf));
		memcpy(pDest,buf,48);
		
		//收到最后一帧数据完成后，写入flash
		//填充check_sum
		uint8_t tmp1=(uint8_t)(check_sum>>8);
		uint8_t tmp2=(uint8_t)(check_sum&0xFF);
		*(parameter_buf+pos_mode3_pwm3+48)=tmp1;
		*(parameter_buf+pos_mode3_pwm3+48+1)=tmp2;
		//将parameter_buf中的数据写入flash中
		
		//FlashWrite(FLASH_WRITE_START_ADDR,(uint32_t*)&parameter_buf,PARAMETER_BUF_LEN/4);
		FlashWrite(FLASH_WRITE_START_ADDR,parameter_buf,PARAMETER_BUF_LEN/4);
		rcvParaSuccess=0x01;
	}
	else
	{
		//do nothing
	}
}

void send_para_rcv_result()
{
	uint8_t buffer[7];
	buffer[0] = PACK_HEAD_BYTE;       //0xFF，头
	buffer[1] = 0x05;            			//长度
	buffer[2] = MODULE_CMD_TYPE;      //0x00，下位机像上位机发送命令的标志
	buffer[3] = SEND_PARA_RCV_RESULT; //0x08，FrameID
	buffer[4]	=	rcvParaSuccess;       //0x01表示接收数据完成，0x00表示未完成接收
	CalcCheckSum(buffer);
	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
	rcvParaSuccess=0x00;   //复位，为下次接收
}

void send_prameter_fram1_to_PC()
{
	//CMD_BUFFER_LENGTH定义为255的时候，PWM2波形老是不见了，也不知道为什么
	//现在将CMD_BUFFER_LENGTH长度定义为300，就OK了，原因不知道
	uint8_t buffer[CMD_BUFFER_LENGTH];

	//读取flash数据到buffer中
	//CheckFlashData(parameter_buf); //检测flash数据是否是正确的，第一次会检测flash时，会将默认的数据填充到flash中
	
	//memset(parameter_buf,0,PARAMETER_BUF_LEN);  //清空parameter_buf
	//填充parameter_buf
	uint8_t len=PARAMETER_BUF_LEN/4;                          
	uint32_t tmp[PARAMETER_BUF_LEN/4]={0};
	FlashRead(FLASH_WRITE_START_ADDR,tmp,len);
	
	memcpy(parameter_buf,tmp,len*4);
	CheckFlashData(parameter_buf);
	
	//发送第一帧
	//公共信息2Bytes, (Mode1-PWM1, Mode1-PWM2, Mode1-PWM3),Mode2-PWM1,Mode2-PWM2
	buffer[0] = PACK_HEAD_BYTE;       //0xFF
	buffer[1] = 0x04+0xF2;            //0xF2=242,数据长度
	buffer[2] = MODULE_CMD_TYPE;      //0x00
	buffer[3] = SEND_FLASH_DATA_1_ID; //0x06
	//填充公共信息
	buffer[4] = *parameter_buf;       //exhalation threshold
	buffer[5] = *(parameter_buf+1);   //wait before after
	
	unsigned char* pstart=parameter_buf+2;
	for(int i=2;i<242;i++)
	{
		buffer[i+4]=*pstart++;
	}
	CalcCheckSum(buffer);
	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
//	UartSendNBytes(buffer, buffer[1]+2);
	//Delay_ms(30);
}


void send_prameter_fram2_to_PC()
{
	//发送第二帧
	//Mode2-PWM3, (Mode3-PWM1,MODE3-PWM2,MODE3-PWM3)
	uint8_t buffer1[CMD_BUFFER_LENGTH];
	
	buffer1[0] = PACK_HEAD_BYTE;       //0xFF
	buffer1[1] = 0x04+0xC0;            //0xC0=192，数据长度
	buffer1[2] = MODULE_CMD_TYPE;      //0x00
	buffer1[3] = SEND_FLASH_DATA_2_ID; //0x07
	
	unsigned char* pstart=parameter_buf+242; //将指针播到第二帧的位置
	for(int i=242;i<434;i++)
	{
		buffer1[i-238]=*pstart++;
	}
	CalcCheckSum(buffer1);
	fifoWriteData(&send_fifo, buffer1, buffer1[1]+2);
//	UartSendNBytes(buffer1, buffer1[1]+2);
//	Delay_ms(30);
}

uint16_t FlashWrite(uint32_t addr, uint8_t *p_data, uint16_t len)
{
	uint16_t i = 0;
	//uint32_t tmp	= 0;
	uint32_t address = addr;
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

	FLASH_ErasePage(address);
	
	if(len<1024/4)
	{
		for (i=0;i<len;i++)
		{
			//小端
			uint32_t ndata=p_data[4*i]+p_data[4*i+1]*256+p_data[4*i+2]*256*256+p_data[4*i+3]*256*256*256;
			FLASH_ProgramWord(address,ndata);
			//FLASH_Status st=FLASH_ProgramWord(address, 0x11223344);				
			address += 4;
		}
//		for (i=0;i<1024/4-len;i++)
//		{
//			FLASH_ProgramWord(address, 0);	
//			address += 4;
//		}
	}

	FLASH_Lock();
	#if 0
	//读数据验证
//	address = addr;
//	for (i=0;i<len;i++)
//	{
//		tmp	= FlashReadWord(address);
//		address	+= 4;
//		if (tmp != p_data[i])
//			break;
//	}
	
	//先不做校验
//	uint16_t sum=0;
//	for(int j=0;j<4*len-2;j++)
//	{
//		sum+=FlashReadByte(address);
//		address++;
//	}
//	//如果检验的数据和不对，返回-1,表示写入失败
//	if(sum!=(*(char*)addr)*256+*(char*)(addr+1))
//	{
//		return -1;
//	}
#endif 
	return i;
}

void FlashRead(uint32_t addr, uint32_t *p_data, uint16_t len)
{
	UINT16 i = 0;
	UINT32 address = addr;
	
	if(p_data == NULL)
		return;
	
	for(i = 0; i < len; i ++)
	{
		p_data[i] = FlashReadWord(address);
		address += 4;
	}
}

uint32_t FlashReadWord(uint32_t addr)
{
	uint32_t data = 0;
	uint32_t address = addr;

	data = *(uint32_t*)address;
	return data;
}

uint8_t FlashReadByte(uint32_t addr)
{
	return (uint8_t)(*(uint8_t*)addr);
}

////得到按键模式
//uint8_t GetModeSelected(void)
//{
//	uint16_t res;
//	res=RegularConvData_Tab[1];
////	for(uint8_t i=0;i<3;i++)
////	{
////		res=Adc_Switch(ADC_Channel_4);
////	}
//	
//	if(res>=1650)
//	{
//		return 1;  //返回模式1
//	}
//	else if(res>=700&&res<1650)
//	//else if(res>=mod2_base_vol-200&&res<=mod2_base_vol+200)
//	{
//		return 2;	//返回模式2
//	}
//	//else if(res>=138&&res<=538)
//	else
//	{
//		return 3; //返回模式3
//	}
//}

//uint8_t mmgH_adcValue[3][2];
uint16_t abs_(uint16_t a,uint16_t b)
{
	if(a>b)
		return a-b;
	else
		return b-a;
}

uint32_t cal_pressure_rate(POINT point_1,POINT point_2,POINT point_3)
{
	uint16_t rate1=abs(point_2.adc_value-point_1.adc_value)/abs(point_2.mmgh_value-point_1.mmgh_value);
	uint16_t rate2=abs(point_3.adc_value-point_1.adc_value)/abs(point_3.mmgh_value-point_1.mmgh_value);
	return (rate1+rate2)/2;
}

//void send_cal_reslut_2_PC()
//{
//	uint8_t buffer[4+9+2+2];
//	POINT point_1;
//	POINT point_2;
//	POINT point_3;
//	
//	//1.发送给上位机
//	buffer[0] = PACK_HEAD_BYTE;       //0xFF
//	buffer[1] = 0x04+11;            
//	buffer[2] = MODULE_CMD_TYPE;      //0x00
//	buffer[3] = CAL_SENSOR_SEND_TO_PC; //0x60
//	
//	point_1.mmgh_value=arr_mmgH_value[0];
//	point_1.adc_value=arr_adc_value[0];

//	point_2.mmgh_value=arr_mmgH_value[1];
//	point_2.adc_value=arr_adc_value[1];

//	point_3.mmgh_value=arr_mmgH_value[2];
//	point_3.adc_value=arr_adc_value[2];

//	 
//	//填数值1
//	buffer[4]=point_1.mmgh_value;
//	buffer[5]=point_1.adc_value/256;
//	buffer[6]=point_1.adc_value%256;

//	//填数值2
//	buffer[7]=point_2.mmgh_value;
//	buffer[8]=point_2.adc_value/256;
//	buffer[9]=point_2.adc_value%256;

//	//填数值3
//	buffer[10]=point_3.mmgh_value;
//	buffer[11]=point_3.adc_value/256;
//	buffer[12]=point_3.adc_value%256;

//	buffer[buffer[1]-1]=((uint16_t)zero_point_of_pressure_sensor)%256;
//	buffer[buffer[1]-2]=((uint16_t)zero_point_of_pressure_sensor)/256;
//	CalcCheckSum(buffer);
//	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
//	
//	//2.将斜率存起来
//	//计算斜率
//	uint32_t rate=cal_pressure_rate(point_1,point_2,point_3);
//	FlashWrite(FLASH_PRESSURE_RATE_ADDR,(uint8_t*)&rate,1);
//}

//void calibrate_sensor_by_ID(uint8_t* pdata,uint8_t ID)
//{
//	
//	
//	switch(ID)
//	{
//		case 1:
//			arr_mmgH_value[0]=*(pdata+4);
//			arr_adc_value[0]=ADS115_readByte(0x90);
//			break;
//		case 2:
//			arr_mmgH_value[1]=*(pdata+4);
//			arr_adc_value[1]=ADS115_readByte(0x90);
//			break;
//		case 3:
//			arr_mmgH_value[2]=*(pdata+4);
//			arr_adc_value[2]=ADS115_readByte(0x90);
//			
//			send_cal_reslut_2_PC();
//			break;
//		default:
//			break;
//	}
//}

#if 1
uint16_t FlashWriteUIntBuffer(uint32_t addr, uint32_t *p_data, uint16_t len)
{
	uint16_t i = 0;
	uint32_t address = addr;
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

	FLASH_ErasePage(addr);
	
	for(i=0;i<len;i++)
	{
		FLASH_ProgramWord(address,p_data[i]);			
		address += 4;
	}
	
	FLASH_Lock();
	return i;
}

void reset_dateTime()
{
	uint32_t pageInfo[3];
	memset(pageInfo,0,3*4);
	FlashRead(FLASH_RECORD_PAGE_FROM_TO,pageInfo,3);
	
	uint16_t len=FLASH_PAGE_STEP/4;   
	memset(pageBuff,0xFF,len*4);
	
	for(uint32_t addr=FLASH_RECORD_PAGE_FROM_TO;addr<=pageInfo[1];)
	{
		FlashWriteUIntBuffer(addr,pageBuff,len);
		addr+=2048;
	}
//	FlashWriteUIntBuffer(FLASH_RECORD_PAGE_FROM_TO,pageBuff,len);
//	FlashWriteUIntBuffer(FLASH_RECORD_DATETIME_START,pageBuff,len);
}

//初始化PAGE_from_to,记录从XXX页到xxx页，有多少条数据
void Init_RecordPage()
{
	//如果在FLASH_RECORD_PAGE_FROM_TO中有数据，说明已经初始化过了
	uint32_t data = *(uint32_t*)FLASH_RECORD_PAGE_FROM_TO;
	if(data==FLASH_RECORD_DATETIME_START)
	{
		return;
	}
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	
	uint32_t address=FLASH_RECORD_PAGE_FROM_TO;
	FLASH_ErasePage(address);
	
	FLASH_ProgramWord(address,FLASH_RECORD_DATETIME_START);     //page from FLASH_RECORD_DATETIME_START
	FLASH_ProgramWord(address+4,FLASH_RECORD_DATETIME_START);  //page to  FLASH_RECORD_DATETIME_START
	FLASH_ProgramWord(address+8,0);        											//record numbers，记录数据条数

	FLASH_Lock();
}



//记录开关机时间
void record_dateTime(SYSTEM_CODE code)
{
	//获取page信息，1.从XXX页 2.到XXX页 3.记录条数
	//pageInfo[0]  从xxx页
	//pageInfo[1]  到xxx页
	//pageInfo[2]  记录条数，一条记录8字节
	uint32_t pageInfo[3];
	memset(pageInfo,0,3*4);
	FlashRead(FLASH_RECORD_PAGE_FROM_TO,pageInfo,3);
	
	//根据获取的page信息，确定在哪一页进行操作
	uint32_t address=pageInfo[1];															//定位操作的页面
	if(address>=FLASH_RECORD_DATETIME_UPLIMIT)  //不允许超过128K
	{
		return;
	}
	
	uint16_t len=FLASH_PAGE_STEP/4;   //len=512
//	static uint32_t pageBuff[512]={0};
	memset(pageBuff,0xFF,len*4);
	
//#ifdef	_DEBUG_FLASH_RECORD_DATETIME
//	FlashWriteUIntBuffer(FLASH_RECORD_PAGE_FROM_TO,pageBuff,len);
//	FlashWriteUIntBuffer(FLASH_RECORD_DATETIME_START,pageBuff,len);
//#else
	FlashRead(address,pageBuff,len);    											//读取该页面的数据到pageBuff中
	
	//根据pageInfo[2](信息条数)来加入新的一条的数据
	RTC_DateTypeDef date_struct;
	RTC_TimeTypeDef time_struct;
	Get_DataTime(&date_struct,&time_struct);
	
	pageBuff[(pageInfo[2]%256)*2]=code+(date_struct.RTC_Year<<16)+(date_struct.RTC_Month<<24);
	pageBuff[(pageInfo[2]%256)*2+1]=(time_struct.RTC_Seconds<<24)+(time_struct.RTC_Minutes<<16)+(time_struct.RTC_Hours<<8)+date_struct.RTC_Date;
	
	pageInfo[2]++;          //完成了一条数据的记录
	if(pageInfo[2]%256==0)  //如果刚好更新满一页了
	{
		//更新pageInfo中的数据
		pageInfo[0]=FLASH_RECORD_DATETIME_START;
		pageInfo[1]+=2048;
	}
	FlashWriteUIntBuffer(FLASH_RECORD_PAGE_FROM_TO,pageInfo,3);
	
	//写入数据
	FlashWriteUIntBuffer(address,pageBuff,len);
//#endif
}

uint32_t get_rtc_record_number()
{
	uint32_t pageInfo[3];
	memset(pageInfo,0,3*4);
	FlashRead(FLASH_RECORD_PAGE_FROM_TO,pageInfo,3);
	
	return pageInfo[2];
}

void send_RTC_SYN_finish(BOOL success)
{
	uint8_t buffer[7];
	
	buffer[0] = PACK_HEAD_BYTE;       //0xFF
	buffer[1] = 0x05;            
	buffer[2] = MODULE_CMD_TYPE;      //0x00
	buffer[3] = RTC_SYN_FINISHED; //0x66
	
	buffer[4]=success;
	
	CalcCheckSum(buffer);
	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
}

void send_RTC_record_numbers()
{
	uint8_t buffer[10];
	
	buffer[0] = 0xFF;       //0xFF
	buffer[1] = 0x08;            
	buffer[2] = MODULE_CMD_TYPE;      //0x00
	buffer[3] = SENT_RTC_BYTES; //0x69
	
	uint32_t tmp=get_rtc_record_number();
	
	buffer[4]=tmp/256/256/256;
	buffer[5]=tmp/256/256%256;
	buffer[6]=tmp%(256*256)/256;
	buffer[7]=tmp%(256*256)%256;
	
	CalcCheckSum(buffer);
	fifoWriteData(&send_fifo, buffer, buffer[1]+2);
}

uint16_t get_page_num(uint16_t frameX)
{
	uint16_t tmp=0;
	tmp=frameX/9;
	if(frameX%9!=0)
	{
		tmp++;
	}
	return tmp;
}


uint16_t send_one_frame_data(uint8_t frame_length,uint8_t* p_data,uint16_t pack_No)
{
//	memset(send_buf,0,SEND_BUF_LEN);
	
		uint8_t bufferSend[30*8+6+2];
//	uint8_t bufferSend[frame_length];
//	memset(bufferSend,0,frame_length);
//	static uint8_t what;
//		what=frame_length;
	
	uint16_t cnt=0;
	bufferSend[0] = 0xFF;       //0xFF
	bufferSend[1] = frame_length-2; 
//	if(pack_No%9==0)
//	{
//		bufferSend[1] = 128+2+4;
//	}
//	else
//	{
//		bufferSend[1] = frame_length-2; 
//	}         
	bufferSend[2] = MODULE_CMD_TYPE;      //0x00
	bufferSend[3] = SEND_RTC_INFO; //0x71
	
	//填充数据
	bufferSend[4]=pack_No/256;   //每一个包都做一个标记，表示这是第几包
	bufferSend[5]=pack_No%256;
	
	for(int m=6;m<=bufferSend[1]-1;m++) 
	{
		bufferSend[m]=*p_data++;
		cnt++;
	}
	
	CalcCheckSum(bufferSend);
	fifoWriteData(&send_fifo, bufferSend, bufferSend[1]+2);
	memset(bufferSend,0,frame_length);
	return cnt;
}

void send_rtc_info(uint16_t frameX)
{
	
	int recordNums=get_rtc_record_number();  //获取记录数据的条数
	uint16_t pages_numbers=recordNums/256; //一共有多少页
	uint16_t page_rest=recordNums%256;//还剩下多少条记录
	uint16_t DATA_RECORD_CNT=30;  //一次发送30条数据
	
	uint16_t tmp_cnt,tmp_rest;
		
	tmp_cnt=page_rest/DATA_RECORD_CNT;  //满包要发送的次数
	tmp_rest=page_rest%DATA_RECORD_CNT; //非满包的记录条数
	
	if(frameX<=9*pages_numbers)
	{
		uint8_t* p=(uint8_t*)pageBuff;
		//读取当前页面数据
		memset(pageBuff,0xFF,512*4);
		FlashRead(FLASH_RECORD_DATETIME_START+FLASH_PAGE_STEP*(get_page_num(frameX)-1),pageBuff,FLASH_PAGE_STEP/4);    											//读取该页面的数据到pageBuff中
		
		if(frameX%9==0)
		{
			send_one_frame_data(6+16*8+2,p+8*240,frameX);
		}
		else
		{
			send_one_frame_data(6+DATA_RECORD_CNT*8+2,p+30*8*(frameX-9*(get_page_num(frameX)-1)-1),frameX);
		}
	}
	else
	{
		uint8_t* p=(uint8_t*)pageBuff;
		//读取当前页面数据
		memset(pageBuff,0xFF,512*4);
		FlashRead(FLASH_RECORD_DATETIME_START+FLASH_PAGE_STEP*(get_page_num(frameX)-1),pageBuff,FLASH_PAGE_STEP/4);  
		
		if(frameX<=9*pages_numbers+tmp_cnt)
		{
			send_one_frame_data(6+DATA_RECORD_CNT*8+2,p+30*8*(frameX-9*(get_page_num(frameX)-1)-1),frameX);  //发送满包的数据
		}
		else
		{
			send_one_frame_data(6+tmp_rest*8+2,p+240*(frameX-9*(get_page_num(frameX)-1)-1),frameX);  //发送非满包的
		}
	}
}

uint16_t getFrameNo(uint8_t* pdata)
{
	if(pdata==NULL)
		return 0;
	return pdata[4]*256+pdata[5];
}
#endif
//解析上位机命令
void protocol_module_process(uint8_t* pdata)
{
	uint8_t *pCmdPacketData = (uint8_t *)pdata;
	uint8_t byFrameID = pCmdPacketData[3];

//	uint8_t bat_per;//电池电量
	
//	//如果没有上电，直接返回
//	if(mcu_state!=POWER_ON)
//	{
//		return;
//	}
	
	//pCmdPacketData = pdata;
	//byFrameID = pCmdPacketData[3];
	//byFrameID = *(pdata+3);

	//byFrameID = GET_BAT_PER_ID;
	switch(byFrameID)
	{

//	case GET_EXP_TRAIN_DATA_ID:
//			//发送存储数据
//			send_exp_train_data_status = TRUE;//是能数据发送
//			
//			//挂起任务
//			os_pend_task(KEY_LED_TASK_ID);
//			os_pend_task(EXP_DETECT_SAVE_TASK_ID);
//			break;

//	case GET_BAT_PER_ID:
//		//得到电池电压
//		bat_per = get_bat_vol_per();
//		//发送给上位机
//		protocol_module_send_bat_per(bat_per);
//		break;
//	
//	case PWM_VALUE_SET_ID:
//		//得到电池电压
//		bat_per = get_bat_vol_per();
//		//发送给上位机
//		protocol_module_send_bat_per(bat_per);
//		break;
	case COMM_PARAMETER_ID:
		get_comm_para_to_buf(pdata);
		break;
	case MODE1_PWM1_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE1_PWM1_ID);
		break;
	case MODE1_PWM2_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE1_PWM2_ID);
		break;
	case MODE1_PWM3_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE1_PWM3_ID);
		break;
	case MODE2_PWM1_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE2_PWM1_ID);
		break;
	case MODE2_PWM2_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE2_PWM2_ID);
		break;
	case MODE2_PWM3_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE2_PWM3_ID);
		break;
	case MODE3_PWM1_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE3_PWM1_ID);
		break;
	case MODE3_PWM2_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE3_PWM2_ID);
		break;
	case MODE3_PWM3_ID:
		get_parameter_to_buf_by_frameId(pdata,MODE3_PWM3_ID);
		break;
	case IS_RCV_PARA_FINISHED:
		send_para_rcv_result();
		break;
	case GET_FLASH_DATA_1_ID:
		send_prameter_fram1_to_PC();
		break;
	case GET_FLASH_DATA_2_ID:
		send_prameter_fram2_to_PC();
		break;
	//现在使用honeywell 代替ADS115,不需要校验了
//	case CAL_SENSOR_MMGH_1:   //新增的专门用来校验sensor
//		calibrate_sensor_by_ID(pdata,1);
//		break;
//	case CAL_SENSOR_MMGH_2:
//		calibrate_sensor_by_ID(pdata,2);
//		break;
//	case CAL_SENSOR_MMGH_3:
//		calibrate_sensor_by_ID(pdata,3);  //在3中回传值
//		break;
	case RTC_SYN_CMD:
		if(Set_RTC(pdata)==TRUE)
		{		
			reset_dateTime();
			Init_RecordPage();
			record_dateTime(CODE_PC_SYN_RTC);
			send_RTC_SYN_finish(1);
		}
		else
		{
			send_RTC_SYN_finish(0);
		}
		break;
	case GET_RTC_RECORD_NUMBERS:
		send_RTC_record_numbers();
		break;
	case GET_RTC_INFO:
		send_rtc_info(getFrameNo(pdata));
	default:
		break;
	}
}
