
#ifndef __SPI_H
#define __SPI_H
#include "at32f4xx.h"


#define   CW	  0		// Counter Clock Wise direction
#define   CCW	 1		// Clock Wise direction

#define   TXRX_ZH_DYValue(x)          ((u16)((u16)((u16)(x)*5)+50))
#define   DY_ZH_TXRXValue(x)          ((x-50)/5)

#define 	SPIhigh 						  1
#define 	SPIlow 							  0
#define 	R_PIN_MOSI()				 GPIO_ReadInputDataBit(GPIOB,GPIO_Pins_5)?1:0
#define 	R_PIN_MOSI_CLK()		 GPIO_ReadInputDataBit(GPIOB,GPIO_Pins_3)?1:0
//#define 	c_max_num						  8    //by jason in 20180326  //����CRCУ��
#define 	bit_idle						  0
#define 	bit_rxing						  1
#define   bit_endok						  2
#define 	txrx_idle						  0
#define 	txrx_rxing						  1
#define   txrx_endok						  2
#define   C_DATA_IDLE						  0XFF
#define   C_max_data_jg_time		          3


/******************************************************************************/
//by jason in 20180326
//����CRCУ��
#define c_max_num						             9	          //by jason in 20180326  //����CRCУ��
/******************************************************************************/
//com
#define c_normal                       0
#define c_dir_com                      1
#define c_SW_com                       2      //by jason in 20180330
#define c_BitChange_com                3      // //by jason in 20180806
#define c_DYLimtLZChange_com           4      //
#define c_ParmReg1Change_com           5      //
/*****************************************END************************************/
//��������汾��
//BY JASON IN 20180330 
//V503��V504 �޸ģ� 1.���ӹ��ٺ͹��س���3���Ժ��ٱ�����   2.���й��ϳ��������θ�Ϊ���� 2020��10��26��14:01:35
#define c_MB_SW_Num                      504    //�޸�OB_BW��600��Ϊ300��������ر���ȶ���������ȡ���¶ȱ�����ȡ����ѹǷѹ��������Ϊ5.01
#define c_003F3P6HW_Num                  1        //С��16
#define c_76E003HW_Num                   2
#define c_030C8T6HW_Num                  3
#define c_005K6t6HW_Num                  4
#define c_MB_Version_Num                 (u8)((u16)((u16)(c_005K6t6HW_Num<<12)|(u16)(c_MB_SW_Num))) 


#define c_MotoType_Max              20    
#define c_MotoType_Min              1    
#define c_MotoType_init             1  

#define c_SDMode_QY_Mask                 1
#define c_SDMode_GY_Mask                 2
#define c_SDMode_QY                      1
#define c_SDMode_GY                      2 

#define c_QYsd_Max                   190    //AC190
#define c_QYsd_Min                   50     
#define c_QYsd_Init                  80
#define c_GYsd_Max                   300      //AC300
#define c_GYsd_Min                   120     
#define c_GYsd_Init                  270      
#define c_dianya_hf_cha              10       //10V  2020��7��28��17:20:31


typedef struct
{
	  u8 tx_buf[c_max_num];
		u8 rx_buf[c_max_num];
	  u8 bit_state;
	  u8 bit_num;
	  u8 state;
		u8 tx_index;
		u8 tx_data;
		u8 rx_index;
		u8 rx_data;
		u8 tx_data_jg_time;                 
}spi_synch;


typedef union 
{
	struct 
	{ 
		u8 f_dir_ack                     : 1;       //����ת����Ӧ��                     //1��Ч��0��Ч
		u8 f_dir_pro                     : 1;       //������Ҫ�ı䴦����                 //1��Ч��0��Ч	
		u8 f_SW_ack                      : 1; 	    //����汾Ӧ��                       //1��Ч��0��Ч	
		u8 f_BChange_ack                 : 1;       //λ�ı�����Ӧ��                     //1��Ч��0��Ч	
		u8 f_DYLimtLZChange_ack          : 1;       //��ѹ��ֵ��������������Ӧ��         //1��Ч��0��Ч	
		u8 f_REG1Change_ack              : 1;       //�Ĵ���1�ı�Ӧ��                    //1��Ч��0��Ч	
		u8 f_rsvd        				         : 2;
	}bits;
	u8 value;
}txflag;

typedef struct 
{ 
		u16 QYSD_Limt;
		u16 GYSD_Limt;
}DYLimt_str;   

typedef union 
{
	struct 
	{ 
/*		u8 f_StopMode                    : 2;       //ͣ����ʽ               0������ͣ��   1������ͣ��  2������ͣ��+DC�ƶ�    //ˮ����Ч
		u8 f_GZ_Clear                    : 1;       //�����Ƿ����           0�������     1: ���	                          //ˮ����Ч
		u8 f_SDMode        				       : 2;       //     
		u8 f_DriverPWM        				   : 1;       //��������ѡ��           0��SPWM       1: SVPWM
	  u8 f_M_OP              				   : 1;       //����ȱ������           0����������ȱ��  1������ȱ��                  //��������������Ч    //c_AddParam403_en
	  u8 f_INS              				   : 1;       //�����Դѡ��           0��������     1����ֱ��ͨ�ã���Ҫ��̫���ܹ��ܣ�
	  	
	  u8 f_CM              				     : 1;       //������Ʒ�ʽѡ��       0���ٶȿ���   1���㹦�ʿ��ƣ������ŵ����Ч��	
		u8 f_rsvd1        				       : 7;*/
		
		
	  	
	  u8 f_CM              				     : 1;       //������Ʒ�ʽѡ��       0���ٶȿ���   1���㹦�ʿ��ƣ������ŵ����Ч��	
		u8 f_rsvd1        				       : 7;	
		
		u8 f_StopMode                    : 2;       //ͣ����ʽ               0������ͣ��   1������ͣ��  2������ͣ��+DC�ƶ�    //ˮ����Ч
		u8 f_GZ_Clear                    : 1;       //�����Ƿ����           0�������     1: ���	                          //ˮ����Ч
		u8 f_SDMode        				       : 2;       //     
		u8 f_DriverPWM        				   : 1;       //��������ѡ��           0��SPWM       1: SVPWM
	  u8 f_M_OP              				   : 1;       //����ȱ������           0����������ȱ��  1������ȱ��                  //��������������Ч    //c_AddParam403_en
	  u8 f_INS              				   : 1;       //�����Դѡ��           0��������     1����ֱ��ͨ�ã���Ҫ��̫���ܹ��ܣ�
	}bits;
	u16 value;
}BChange_str;

typedef struct 
{ 
		u8 D0BUF;
		u8 D1BUF;
		u8 D2BUF;
}ComBUF_str; 

extern spi_synch   spi_synch_blx;
extern txflag      TXStr_BLX;
extern DYLimt_str  DYLimt_blx;  
extern BChange_str BChange_blx;
extern ComBUF_str  ComBuf_blx;


extern u16  uwKeySpeedUartOrg,uwKeySpeedUartOrg1,uwKeySpeedUartOrg2,uwKeySpeedRefRec,uwKeySpeedRefFil; 
extern u32  ulKeySpeedUart;
extern u32  ulKeySpeedRefFil;
extern u8   uwMotorNumSPINum;
extern u16  TEMP_uACVol,TEMP_uwDisSpdFdb,TEMP_uwIqFdb;


extern volatile u8  f_txgz;



extern void spi_synch_init(void);
extern void PIN_MISO_O(u8 temp);
extern void spi_ctr(void);
extern u16 crc16_update (u16 crc, u8 date);
extern u16 calcrc16(u8 *ptr, u8 count);
extern void synch_receive_proc(void);
extern void synch_send_proc(void);
extern void txgz_check(void);
extern void spi_synch_tongxun_check(void);







#endif

