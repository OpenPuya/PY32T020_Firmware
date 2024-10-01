#include "ir_received.h"

#if APP_IR_RECEIVED_ENABLE
#define ir_state_bit(n) ((Bits8_TypeDef *)(&(ir_state)))->bit##n
#define		Queue_Size		12
typedef struct 
{
	uint8_t level_In;				//Fifo���
	uint8_t level_Out;				//Fifo����
	uint8_t level_count;			//�ߵ�ƽ����
	uint8_t level_fifo[Queue_Size];	//Fifo
}ir_Queue;
static uint8_t ir_state;
static ir_Queue ir;  
static uint8_t ir_Count;		//��������
static uint8_t ir_Rec[4];		//����BIT����
static uint8_t ir_Cnt;			//����BIT����
static uint16_t ir_release;		//�ͷż���
#define ir_last			ir_state_bit(0)
#define SYNC			ir_state_bit(1)
#define RX_OVER			ir_state_bit(2)
/********************************************************
**	������	void IR_Received_Init(void)
**	����	����ɨ������ʼ��
**	����	����
**	����	����
*********************************************************/
void IR_Received_Init(void)
{
	ir_state = 0;
	ir.level_Out = ir.level_In = 0;
	ir_Cnt = 0;
	GPIO_Init(IR_GPIO,INPUT|PULL_UP);
}
/********************************************************
**	������	void IR_Received_Scan(void)
**	����	��������ƽɨ�躯�������ڶ�ʱ���ж���
**	����	����
**	����	����
*********************************************************/
void IR_Received_Scan(void)
{
	if(GPIO_ReadBit(IR_GPIO))	  //�ߵ�ƽ
	{
		ir.level_count++;
		ir_last = 1;			 //��Ǹߵ�ƽ�Ѿ�������
	}
	else	  					//�͵�ƽ
	{   
		if(ir_last)				//����ߵ�ƽʱ��
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
**	������	uint8_t IR_Press(Ir_TypeDef *remote)
**	����	������뺯��
**	����	��remote ���ݽ��սṹ��
**	����	��	0�����ź�����
				1���յ���ȷ�ĺ����ź�
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
		if(SYNC)//���յ���������
		{
			i = (ir_Cnt >> 3);
			if(hight_level >= DATA0_MIN_TIME && hight_level < DATA0_MAX_TIME)			
			{
				ir_Rec[i] <<= 1;	//����һλ.
				ir_Rec[i] &= 0XFE;	//���յ�0
				ir_Cnt++;	
			}
			else if(hight_level >= DATA1_MIN_TIME && hight_level < DATA1_MAX_TIME)	
			{
				ir_Rec[i] <<= 1;	//����һλ.
				ir_Rec[i] |= 0X01;	//���յ�1
				ir_Cnt++;
			}
			else
			{	
				SYNC = 0;
				ir_Cnt = 0; 
			}
			/*	��ַ 16bit ���� 16bit*/
			if(ir_Cnt == 32)
			{
				RX_OVER = 1;
				SYNC = 0;		//������յ���������
				ir_Cnt = 0;		//�����������������
				ir_Count = 1;
				ir_release = RELEASE_TIME;		//����յ�����
				break;
			}
		}
		else if(hight_level >= SYNC_MIN_TIME && hight_level < SYNC_MAX_TIME)	
		{
			SYNC = 1;		//��ǳɹ����յ���������
			ir_Cnt = 0;		//�����������������
			ir_Count = 0;
			ir_release = 0;
		}
		else if(hight_level >= LONG_MIN_TIME && hight_level < LONG_MAX_TIME)	
		{
			if(ir_release)						//ǰ���Ѿ��յ�����
			{
				ir_release = RELEASE_TIME;
				if(ir_Count < 255)
					ir_Count++;
				RX_OVER = 1;
				SYNC = 0;		//������յ���������
				ir_Cnt = 0;		//�����������������
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
		/*	����У��		*/
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
