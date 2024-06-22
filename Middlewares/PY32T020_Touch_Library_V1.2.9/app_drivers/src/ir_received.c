#include "ir_received.h"

#if APP_IR_RECEIVED_ENABLE
#define ir_state_bit(n) ((Bits8_TypeDef *)(&(ir_state)))->bit##n
#define		Queue_Size		12
typedef struct 
{
	uint8_t level_In;				//Fifo入口
	uint8_t level_Out;				//Fifo出口
	uint8_t level_count;			//高电平计数
	uint8_t level_fifo[Queue_Size];	//Fifo
}ir_Queue;
static uint8_t ir_state;
static ir_Queue ir;  
static uint8_t ir_Count;		//按键次数
static uint8_t ir_Rec[4];		//接收BIT缓存
static uint8_t ir_Cnt;			//接收BIT计数
static uint16_t ir_release;		//释放计数
#define ir_last			ir_state_bit(0)
#define SYNC			ir_state_bit(1)
#define RX_OVER			ir_state_bit(2)
/********************************************************
**	函数名	void IR_Received_Init(void)
**	描述	红外扫描解码初始化
**	传入	：无
**	返回	：无
*********************************************************/
void IR_Received_Init(void)
{
	ir_state = 0;
	ir.level_Out = ir.level_In = 0;
	ir_Cnt = 0;
	GPIO_Init(IR_GPIO,INPUT|PULL_UP);
}
/********************************************************
**	函数名	void IR_Received_Scan(void)
**	描述	红外解码电平扫描函数，放在定时器中断内
**	传入	：无
**	返回	：无
*********************************************************/
void IR_Received_Scan(void)
{
	if(GPIO_ReadBit(IR_GPIO))	  //高电平
	{
		ir.level_count++;
		ir_last = 1;			 //标记高电平已经被捕获
	}
	else	  					//低电平
	{   
		if(ir_last)				//保存高电平时间
		{
			ir.level_fifo[ir.level_In] = ir.level_count;
			ir.level_In++;
			if(ir.level_In >= Queue_Size)
				ir.level_In = 0;
		}	
		ir.level_count = 0;
		ir_last = 0;		
	}
	if(ir_release)
		ir_release--;
}
/********************************************************
**	函数名	uint8_t IR_Press(Ir_TypeDef *remote)
**	描述	红外解码函数
**	传入	：remote 数据接收结构体
**	返回	：	0：无信号输入
				1：收到正确的红外信号
*********************************************************/
uint8_t IR_Press(Ir_TypeDef *remote)
{
	uint8_t i;
	uint8_t hight_level;
	while(ir.level_In != ir.level_Out)
	{
		hight_level = ir.level_fifo[ir.level_Out];
		ir.level_Out++;
		if(ir.level_Out >= Queue_Size)
			ir.level_Out = 0;
		if(SYNC)//接收到了引导码
		{
			i = (ir_Cnt >> 3);
			if(hight_level >= DATA0_MIN_TIME && hight_level < DATA0_MAX_TIME)			
			{
				ir_Rec[i] <<= 1;	//左移一位.
				ir_Rec[i] &= 0XFE;	//接收到0
				ir_Cnt++;	
			}
			else if(hight_level >= DATA1_MIN_TIME && hight_level < DATA1_MAX_TIME)	
			{
				ir_Rec[i] <<= 1;	//左移一位.
				ir_Rec[i] |= 0X01;	//接收到1
				ir_Cnt++;
			}
			else
			{	
				SYNC = 0;
				ir_Cnt = 0; 
			}
			/*	地址 16bit 数据 16bit*/
			if(ir_Cnt == 32)
			{
				RX_OVER = 1;
				SYNC = 0;		//清除接收到了引导码
				ir_Cnt = 0;		//清除按键次数计数器
				ir_Count = 1;
				ir_release = RELEASE_TIME;		//标记收到数据
				break;
			}
		}
		else if(hight_level >= SYNC_MIN_TIME && hight_level < SYNC_MAX_TIME)	
		{
			SYNC = 1;		//标记成功接收到了引导码
			ir_Cnt = 0;		//清除按键次数计数器
			ir_Count = 0;
			ir_release = 0;
		}
		else if(hight_level >= LONG_MIN_TIME && hight_level < LONG_MAX_TIME)	
		{
			if(ir_release)						//前面已经收到数据
			{
				ir_release = RELEASE_TIME;
				if(ir_Count < 255)
					ir_Count++;
				RX_OVER = 1;
				SYNC = 0;		//清除接收到了引导码
				ir_Cnt = 0;		//清除按键次数计数器
				break;
			}
		}
		else
		{
			SYNC = 0;
			ir_Cnt = 0; 
		}
	}
	if(RX_OVER)
	{
		RX_OVER = 0;
		/*	数据校验		*/
		if((ir_Rec[0] == (uint8_t)~ir_Rec[1]) && (ir_Rec[2] == (uint8_t)~ir_Rec[3]))
		{
			remote->ir_address = ir_Rec[0];
			remote->ir_command = ir_Rec[2]; 
			remote->ir_count = ir_Count;
			return 1;
		}
		else
		{
			SYNC = 0;
			ir_Cnt = 0; 
			ir_release = 0;
		}
	}
	return 0;
}
#endif
