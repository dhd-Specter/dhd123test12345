

#include "at32f4xx.h"


#define 	C_NO_ERR              0  // 0:�޹���  //
#define 	C_GUOLIU              1  // 1:Ӳ������
#define 	C_GUOZAI              2  // 2:����
#define 	C_QIANYALI            3  // 3:Ƿѹ��
#define 	C_GUOYALI             4  // 4:��ѹ��
#define 	C_QIANYA              5  // 5:Ƿѹ
#define 	C_GUOYA               6  // 6:��ѹ
#define 	C_QUEXIANG            7  // 7:��Դȱ��
#define 	C_GUOWEN              8  // 8:����
#define 	C_SUN_LOWERPOWER      9  // 9:̫���ܹ��ʲ���
#define 	C_SOFT_GUOLIU         10 // 10:�������
#define 	C_TONGXUN_GZ          11 // 11:ͨѶ����
#define 	C_STALL_GZ            13 // 13:�����ת
#define 	C_MOTORPHASE_GZ       14 // 14:���ȱ��
#define 	C_OVERSPD_GZ          15 // 15:�������
#define 	C_FLASH_GZ            16 // 16:FLASH����


#define DC_LVLimt   2750   //DC275V
#define DC_OVLimt   7415   //DC741.5V

typedef struct
{
    u16 FlashInterTime;
    u16 FlashCnt;
    u16 FlashStopInterTime;
    enum  {NORMAL1,FAULT1}state;
}LED_FLASH_STRU;

extern LED_FLASH_STRU  g_hLedFlash;


extern volatile s16 DelayTime;


extern u16  g_uErrorCode; //test3
extern u32  ulIrate150Cnt;
extern u16  uwVolBusCom;
extern u16  uwTemper;
extern u16  g_lTemper;
extern u16  g_lBusCur;
extern u16  h_ADCBusvolt;
extern u16  AVI1;
extern u16  AVI2;
extern u16 SpeedSourceTemp;



extern void Ext_Led(void);
extern void Fault_Management(void);
extern void moto_idle_check(void);
extern void power_on_count(void);
extern void Speed_vInit(void);
extern void Motor_Ctr(void);


















