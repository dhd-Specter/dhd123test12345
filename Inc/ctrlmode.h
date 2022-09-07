/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    10-October-2011
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CTRLMODE_H
#define __CTRLMODE_H
#include "at32f4xx.h"


#define FAULT_STRU_DEFAULTS   { {0,0},0,0,0,{{0},0,0,0,0}}



struct FAULT_FLAG0_BITS {     // bits  description
   u16 CurOC:1;            	//1  软件过流
   u16 FoError:1;           //2  硬件过流
   u16 VolLow:1;    	      //3  欠压
   u16 VolOve:1;    			  //4  过压
   u16 HallError:1;         //5  HALL
   u16 FALSH:1;   		      //6  FLASH出错
   u16 FALSHFir:1;    			//7  First
   u16 ForceStall:1;        //8  Hall堵转
   u16 HIGHSPDERROR:1;      //9  高速
   u16 PhaseError:1;  		  //10 缺相
   u16 ADCERROR:1;			//11
   u16 QEPErrorFlag:1;   	//12     //the flag of QEP error
   u16 TemperErorr:1;  		//13the flag of temper error
   u16 BusErorr:1;  			//14the flag of temper error
   u16 FOErorr:1;			//the flag of FO
   u16 LoadOverErorr:1;      // 15:15   reserved
};
struct FAULT_FLAG1_BITS {     // bits  description

   u16 OpenTimeLimError:1;    		//0   The flag of error with opendoor time limit
   u16 ClsTimeLimError:1;    		//1   The flag of error with closedoor time limit
  // u16 LowOpenClsTimeLimError:1;    //2   The flag of error with low open/colse door time limit
   u16 ThreeWireError:1;            			//2   reserved
   u16 LossVel:1;
   u16 rsvd:12;            			// 15:3   reserved
};
union FAULT_FLAG0 {
   u16                all;
   struct FAULT_FLAG0_BITS   bit;
};
union FAULT_FLAG1 {
   u16                all;
   struct FAULT_FLAG1_BITS   bit;
};
typedef struct{
	struct
	{
		union FAULT_FLAG0     FautFlag0;
		union FAULT_FLAG1     FautFlag1;
	}fault;
	u32 FaultAll;
	u32 FaultPre;
	u32 FaultLast;
	struct
	{
		u16 FaultSave[5];       //the fault save
		u16 VolSave;
		u16 IdcSave;  //5678
		u16 FreSave;
		u16 PosSave;  //1234
	}ParSave;
}FAULT_STRU;





extern u16 cf_uwWebRad,cf_uwtbSec,cf_uwPbWa;
extern u16 uwRunSpdDec;
extern u16 uwTBSPowerCnt;
extern u32 cf_ulTbNm,cf_ulJbKgm;
extern s32 kw_slFunActFlxAlfa,kw_slFunActFlxBeta;
extern s32 kw_slIqMaxBase,kw_slIqMaxCal;
extern u16 kw_uwStaticLq;
extern s32 kw_slTorRef;
extern s16 att_swIsRef;
extern u16 att_uwIN;
extern u16 att_uwSmplLength;
extern u32 att_ulTBSCt;
extern u16 att_uwTBCSmplCt;
extern s32 att_slAUTOTunIdc;
extern s32 att_slAUTOTunUdc;
extern s32 att_slAUTOTunUdcSum;
extern s16 swIFFocSpdPu;
extern s16 swIdCmd;
extern s32 flw_slIdCmdInte;
extern u16 uwIFtoFunCnt;
extern s16 RunCmd;
extern u16 uwMotPowWattCal,uwMotPowWattDis;
extern u16 uwSpdRefRadio;
extern u16 uwDisSpdFdb,uwDisIRef;
extern u16 uwUlimt,uwAcrUDlim,uwAcrUQlim;
extern u32 ulAcrUQlim;
extern u16 uwErrClear;
extern u16 uwKeySpeedRef,kw_uwPlugSpdHz,uwKeyPowerRef;
extern s16 swOVCnt,swLVCnt,swLVNomCnt,swOVNomCnt;
extern u16 uwIrateOver;
extern u16 uwIdDecCom;
extern u32 mpd_ulPMSALCnt;
extern s16 swPowerActive;
extern s16 swPowerTmp;
extern s16 IFtoFunSign;
extern u16 uwIfSign;
extern s32 slThetaComTBC;
extern u16 uwSTAVOLCnt;
extern u32 ulSTACOF;
extern u16 uwSpdMechHz,uwSpdMechCf;
extern s16 swCurOCCnt;
extern s32 slHighSpdCnt;
extern u16 flw_uwUout;
extern s16 flw_swUerr,flw_swUerrDiff,flw_swUerrLast;
extern u16 uwRs,uwDTCDu;
extern s16 swUdOut;
extern s16 swSpeedFdbLPF;
extern u16 adAverageCurrent;
extern u8  sampleVddDivideTwo_bit;
extern u16 uwTBCCnt;




void ctm_CtrlModeSchdResv(void);

static void ctm_Resv(void);
static void ctm_IFFOC(void);
static void ctm_AutoTuning(void);

void com_DevResv(void);




void  mpd_IFIniDegreeInit(void);
u8 mpd_IFIniDegreeResv(void);
void  mpd_IFIniDegreeTBC(void);




void ctm_IFFocInit(void);  //FOC initializer.
void ctm_IFFocResv(void);  //FOC Resv loop task scheduler.
void ctm_IFFocTBS(void);	  //FOC TBC task scheduler.
void ctm_IFFocTBC(void);	  //FOC TBC task scheduler.



void ctm_TBC(void);
void ctm_TBSIsr(void);
void ctm_TBCIsr(void);

#endif /* __STM32F4xx_IT_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
