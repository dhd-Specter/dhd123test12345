

#include "at32f4xx.h"


#define 	C_NO_ERR              0  // 0:无故障  //
#define 	C_GUOLIU              1  // 1:硬件过流
#define 	C_GUOZAI              2  // 2:过载
#define 	C_QIANYALI            3  // 3:欠压力
#define 	C_GUOYALI             4  // 4:过压力
#define 	C_QIANYA              5  // 5:欠压
#define 	C_GUOYA               6  // 6:过压
#define 	C_QUEXIANG            7  // 7:电源缺相
#define 	C_GUOWEN              8  // 8:过温
#define 	C_SUN_LOWERPOWER      9  // 9:太阳能功率不足
#define 	C_SOFT_GUOLIU         10 // 10:软件过流
#define 	C_TONGXUN_GZ          11 // 11:通讯故障
#define 	C_STALL_GZ            13 // 13:电机堵转
#define 	C_MOTORPHASE_GZ       14 // 14:电机缺相
#define 	C_OVERSPD_GZ          15 // 15:电机过速
#define 	C_FLASH_GZ            16 // 16:FLASH故障


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


















