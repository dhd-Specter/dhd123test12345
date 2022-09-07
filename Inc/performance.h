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
#ifndef __PERFORMANCE_H
#define __PERFORMANCE_H


#include "at32f4xx.h"





/////////////////////// PWM Peripheral Input clock ////////////////////////////
#define CKTIM	((u32)120000000uL) 	/* Silicon running at 72MHz Resolution: 1Hz */
#define PWM_PRSC ((u8)0)                          
#define PWM_PERIOD ((u16) (CKTIM / (u32)(2 * PWM_FREQ *(PWM_PRSC+1)))) 
#define DEADTIME  (u16)((unsigned long long)CKTIM/2*(unsigned long long)DEADTIME_NS/1000000000uL) 
#define PWM_FREQ ((u16)16000) // in Hz  (N.b.: pattern type is center aligned)
#define DEADTIME_NS	((u16) 2000)  //in nsec; range is [0...3500] 
#define REP_RATE (1)  
#define GLB_TBS_CST_CNT      (PWM_FREQ/1000)
#define GLB_TBS_FREQ_CST_HZ  1000   //SYSTICK÷–∂œ ±º‰
#define GLB_TBC_FREQ_CST_HZ  PWM_FREQ //(pr[PWMFRE]*20)

					
#define MLPF_FMIN_CST  55	//Min. freq. of MLPF.
#define MLPF_FSIGNPOS_CST  1	//Freq. sign positive of MLPF.
#define MLPF_FSIGNNEG_CST  0	//Freq. sign negative of MLPF.					
#define PLL_SIM_CST  0		//0: Simulation off; 1: Simulation on.
#define PLL_FRE_INTE_LIM_CST 0x7FFFFFFF //Freq. integral overflow limiting.
#define PLL_FRE_OUT_LIM_CST 0x7FFFFFFF //Freq. output overflow limiting.
#define SINTABSIZE  256+1
#define TQC_RAD2RPM_CST  250329080	//60*100/2/3.14(Q18)
#define TQC_START_DELAY_MS 100          //torque add delay ms on start-up stage
#define SLR_HI_EFF_MACRO 1 //1: High Efficiency Calc; 0: Low Efficiency Calc

#define SLR_JL_RATIO_DEF_CST 10 //Default of Load Inertia Ratio. (D1)
#define SLR_KP_MAX_CST 1073741824 //Limit of Kv. 2^30.
#define SLR_FC_MIN_CST_HZ 1 //Minimal Cut-off Freq. Unit: 0.1Hz.
#define SLR_VARJ_CST 10 //Variation Rate of Cut-off Freq to System Inertia. (D2)
//#define SLR_TG_PMMIN_CST 5773 //Tangent of Minimal Acceptable Phase Margin. (D4)
#define SLR_TVI_MIN_CST_SEC 4 //Minimal Acceptable Integral Time. Unit: 0.001s
#define SLR_TRAN_WIDTH_CST_HZ 500 //Transition Width of ASR Switching. Unit: 0.01Hz
#define SLR_ANTINOISE_DEF_CST 0 //Default of Anti-Noise Coefficient. (D3)


#define LPFCF_DEFAULT 0
#define LPFOUT_DEFAULT {0, 0}
#define LONGLPFOUT_DEFAULT {0, 0}//
#define TRACKCF_DEFAULT {0, 0 }
#define TRACKOUT_DEFAULT { 0, 0, 0 }
#define CDT_CLARKE_OUT_DEFAULT { 0,0 }	//Default value of CLARKEOUT
#define CDT_INVCLARKE_OUT_DEFAULT { 0,0,0 }	//Default value of CDT_INVCLARKE_OUT
#define CDT_PARK_OUT_DEFAULT { 0,0 }	//Default value of PARKOUT
#define CDT_INVPARK_OUT_DEFAULT { 0,0 }	//Default value of INVPARKOUT
#define SAL_DEGREE1_CST 5965232//1 degree in Q31
#define SAL_DEGREE90_CST 536870912//90 degree in Q31
#define PIT_TONSTATUS_CST   1//
#define PIT_TOFFSTATUS_CST  0//

#define   SVGENOUT_DEFAULT { 0,0,0 }  //Default value of SVGENOUT
#define   PWM_2OVERSQRT3_CST    18919            /* Q14 2/sqrt(3), Q15 1/sqrt(3) */
#define   PWM_4OVER3_CST   21845     /* Q14 4/3, Q15 2/3 */
#define   T123MAX     0x3FFFFFFF       /* Q30 */
#define   T123MIN     -0x3FFFFFFF      /* Q30 */
#define   HALFSQRT3   28378            /* Q15 sqrt(3)/2 */

//Time delays of d,q axis of ACR for PG.
#define CLR_DAMP_RATIO_CST 707  //Xi=sqrt(2)/2.(D3)

#define SLR_VARJ_CST 10 //Variation Rate of Cut-off Freq to System Inertia. (D2)
#define CLR_PI_OVERLIM_CST 6553500 //PI coefficient Overflow limiting



#define CLR_SAT_LIT_CST 8192       //Saturation limiting.(Q15)
#define PWM_ONE_CST_Q15	 0x7FFF


#define FLW_UDCLIMIT_CONST_PU 16384


#define CTM_GROUP 0
// 00: Resv mode.
#define CTM_RESV_CST (CTM_GROUP+0)
// 01: VF control mode.
#define CTM_VVVF_CST (CTM_GROUP+1)
// 02: CS.
#define CTM_CS_CST (CTM_GROUP+2)
// 03: Reserved.
#define CTM_PGFOC_CST (CTM_GROUP+3)
// 04: Reserved.
//#define  (CTM_GROUP+4)
// 05: Reserved.
//#define  (CTM_GROUP+5)
// 06: Reserved.
//#define  (CTM_GROUP+6)
// 07: IF Sensorless  control.
#define CTM_IFFOC_CST (CTM_GROUP+7)
// 08: Reserved.
#define CTM_RotFOC_CST (CTM_GROUP+8)
// 09: Reserved.
#define CTM_PulFOC_CST (CTM_GROUP+9)
// 10: Auto-tuning mode.
#define CTM_ATUN_ROT_CST (CTM_GROUP+10)
// 11: Reserved.

#define CTM_MAX_MODES_CST (CTM_GROUP+19)  // Total control modes.

//==== Run mode ID
#define CTM_RM_GROUP 0
// 00: Resv mode.
#define CTM_RM_RESV_CST (CTM_RM_GROUP+0)
// 01: VF control mode.
#define CTM_RM_VVVF_CST (CTM_RM_GROUP+1)
// 02: CS control mode.
#define CTM_RM_CS_CST (CTM_RM_GROUP+2)
// 03: Reserved.
#define CTM_RM_PGFOC_CST (CTM_RM_GROUP+3)

#define CTM_RM_IFFOC_CST (CTM_RM_GROUP+7)
// 08: Reserved.
#define CTM_RM_RotFOC_CST (CTM_RM_GROUP+8)
// 09: Reserved.
#define CTM_RM_PulFOC_CST (CTM_RM_GROUP+9)
// 10: Reserved.
#define CTM_RM_TUNING_CST (CTM_RM_GROUP+10)
// 11: IniDegree testing.
#define CTM_RM_IFIniDegree_CST (CTM_RM_GROUP+11)
// 12: Reserved.
#define CTM_RM_HFIIniDegree_CST (CTM_RM_GROUP+12)
// 13: Reserved.
//#define  (CTM_RM_GROUP+13)
// 14: Reserved.
//#define  (CTM_RM_GROUP+14)
// 15: Reserved.
//#define  (CTM_RM_GROUP+15)
// 16: Reserved
//#define   (CTM_RM_GROUP+16)
// 10: Reserved.
//#define  (CTM_RM_GROUP+17)
// 10: Reserved.
//#define  (CTM_RM_GROUP+18)
#define CTM_RM_MAX_MODES_CST (CTM_RM_GROUP+19)  // Total run modes.

// Frequency below which control mode allowed to be changed.
#define CTM_CHANGE_FREQ_CST 328  // (Q15), 6Hz.

// Control mode initialization enabled.
#define CTM_INIT_EN_CST 0
// Control mode initialization finished.
#define CTM_INIT_OK_CST 1



#define CURMODCF_DEFAULT { 0 }	    //Default value of CURMODCF


#define SLR_FC_MIN_CST_HZ 1 //Minimal Cut-off Freq. Unit: 0.1Hz.

#define TonStatus        0
#define ToffStatus       1
#define MAGPOLE_15       0x555
#define MAGPOLE_7P5      0x2ab
#define MAGPOLE_3P75     0x155
#define MAGPOLE_1P875    0xab

#define NB_CONVERSIONS 1024

#define TBSPowerLim 2
#define GET_PHASE_CURRENTS SVPWM_3ShuntGetPhaseCurrentValues



typedef struct 
{
  s16 qV_Component1;
  s16 qV_Component2;
} Volt_Components;


typedef struct
{
  u16 uwFc; //Unit: 0.1Hz
  u16 uwTctrl; //Q15
}
LPFCFIN;
extern  LPFCFIN sal_stLpfCfIn;
typedef s16   LPFIN;	//Input, Q15
typedef u16   LPFCF; //Coef, Q15
extern  LPFCF   sal_uwLpfCfPowerCal,sal_uwLpfCfPowerDis;
extern  LPFCF   sal_uwLpfCfIqFdb;
extern  LPFCF   sal_uwLpfCfVdc;
extern  LPFCF   sal_uwLpfCfSpdDis;
extern  LPFCF   sal_uwLpfCfSpdFun;
extern  LPFIN   sal_swLpfIn;



typedef struct
{
  s16 swLpfOut;//Q15
  s32 slTmp;//Q30
}
LPFOUT;
extern  LPFOUT  sal_stLpfOutIqFdb;
extern  LPFOUT  sal_stLpfOutVdc;
extern  LPFOUT  sal_stLpfOutSpdDis;
extern  LPFOUT  sal_stLpfOutSpdFun;
extern  LPFOUT  sal_stLpfOutPowerCal,sal_stLpfOutPowerDis;

typedef struct
{
  s32 slVelRef; //Velocity Reference. (Q31)
  s32 slVelFdb; //Velocity Feedback.  (Q31)
  s32 slTorFed; //Torque Offset. (Q27)
  u32 ulTorMax; //Torque Limit.   (Q27)
	u32 ulErrIntMax; //Err Int Limit.   (Q15)
}
SLR_LOOP_TBS_IN;
extern SLR_LOOP_TBS_IN slr_stLoopTBSIn;

typedef struct
{
	u32 ulP; //Proportional Coefficient for Velocity Regulation. (Q11)
	u32 ulI; //Integral Coefficient for Velocity Regulation. (Q17)
	u32 ulKpKi; //Integral Coefficient for Velocity Regulation. (Q12)
	u16 uwKfErr; //Anti-Noise Filter Coefficient. (Q15)
	u16 uwKfCmd; //Velocity Setting Filter Coefficient. (Q15)
	u16 uwKfTFED; //Feedforward Filter Coefficient. (Q15)
	u32 ulKff; //Torque Feedforward Coefficient. (Q13)
  u16 uwErrLpfCf;//Q13,Error Lpf Coefficient
}
SLR_CF;
extern SLR_CF slr_stCf;

typedef struct
{
  s32 slErrOld; //Last Regulated Velocity Error. (if SLR_HI_EFF_MACRO=1, Q30; else Q31)
  s32 slTorFedOld; //Last Torque Offset. (Q27)
}
SLR_LOOP_TBS_PRT;
extern SLR_LOOP_TBS_PRT slr_stLoopTBSPrt;

typedef struct
{
  s32 slTorquRef; //Torque Reference. (Q27)
	s32 slErrInt;   //Q15
}
SLR_LOOP_TBS_OUT;
extern SLR_LOOP_TBS_OUT slr_stLoopTBSOut;

typedef struct
{
  s16 swFsCMD;
  s16 swFsFDB;
  u16 uwTdACR; //Equivalent current control delay. (Q10)
}
SLR_CF_IN;
extern SLR_CF_IN slr_stCfIn;

typedef struct
{
  u16 uwVpuCnv;//voltage  coefficient
  u16 uwVarCnv;// voltage ratio coefficient,D3*Q12
}
VOL_VDC_CF;
extern  VOL_VDC_CF   vol_stVdcCf;

typedef struct
{
  u16 uwVdc;  // voltage pu value,Q14
  u16 uwVdcVt;  // real dc bus voltage value,D1
  u16 uwVdcAd;  // voltage ad value
  u16 uwVdcPct; // voltage ratio, D3
}
VOL_VDC_TBC_OUT;
extern VOL_VDC_TBC_OUT  vol_stVdcTBCOut;

typedef struct
{
  s16 swUAlfaRef; //Alfa-axis voltage reference. (Q15)
  s16 swUBetaRef; //Beta-axis voltage reference. (Q15)
  u16 uwUdcPct; //DC bus voltage in permillage. (D3)
}
VLR_VOL_TO_DUTY_TBC_IN;
extern VLR_VOL_TO_DUTY_TBC_IN vlr_stVolToDutyTBCIn;


typedef struct
{
  s16 swTAlfaRef; //Alfa-axis duty ratio reference. (Q15)
  s16 swTBetaRef; //Beta_axis duty ratio reference. (Q15)
}
VLR_VOL_TO_DUTY_TBC_OUT;
extern VLR_VOL_TO_DUTY_TBC_OUT   vlr_stVolToDutyTBCOut;

typedef struct
{
  s16 swTa; //A phase duty ratio. (Q15)
  s16 swTb; //B phase duty ratio. (Q15)
  s16 swTc; //C phase duty ratio. (Q15)
  u16 uwUdcPct; //DC bus voltage in permillage. (D3)
}
VLR_DUTY_TO_VOL_TBC_IN;
extern VLR_DUTY_TO_VOL_TBC_IN vlr_stDutyToVolTBCIn;

typedef struct
{
  s16 swUa;  //A phase voltage. (Q15)
  s16 swUb;  //B phase voltage. (Q15)
  s16 swUc;  //C phase voltage. (Q15)
}
VLR_DUTY_TO_VOL_TBC_OUT;
extern VLR_DUTY_TO_VOL_TBC_OUT   vlr_stDutyToVolTBCOut;

typedef struct
{
  u16 uwKUdcCf1;  //Coefficient for volt-to-duty.
  u16 uwKUdcCf2;  //Coefficient for duty-to-volt.
}
VLR_CF;
extern VLR_CF   vlr_stCf;


typedef union
{
  struct{
    u32 SVPWM : 1;		// SVPWM / SPWM.
    u32 DPWM : 1;		// DPWM / CPWM.
    u32 SyncPWM : 1;		// Synchronous / Asynchronous PWM.
    u32 DBCMode : 2;		// DBC mode selection.
    u32 RSRV : 3;		// Reserved bits.
  } bit;
  u8 ub;
}
INV_PWM_FLAG_UN;	// PWM inverter flag.
typedef struct
{
  s16 swIdRef; //D-axis current reference. (Q15)
  s16 swIdFdb; //D-axis current feedback. (Q15)
  s16 swUdDcp; //D-axis  feedforward decoupled voltage.(Q15)
}
CLR_ID_LOOP_TBC_IN;
extern CLR_ID_LOOP_TBC_IN clr_stIdLoopTBCIn;

typedef struct
{
  s16 swIdErrOld; //Last value of d-axis current error between reference and feedback. (Q15)
  s16 swUdDcpOld; //Last value of d-axis feedforward decoupled voltage. (Q15)
  s32 slUdRefOld; //Last value of d-axis voltage reference old. (Q27)
}
CLR_ID_LOOP_TBC_PRT;
extern CLR_ID_LOOP_TBC_PRT   clr_stIdLoopTBCPrt;

typedef struct
{
  s16 swUdRef; //D-axis PI regulator's voltage reference output. (Q15)
}
CLR_ID_LOOP_TBC_OUT;
extern CLR_ID_LOOP_TBC_OUT   clr_stIdLoopTBCOut;

typedef struct
{
  u16 uwUdcPct; //DC bus voltage permillage. (D3)
}
CLR_ID_LOOP_CF_IN;

typedef struct
{
  u16 uwKp;    //Proportional gain. (Q14)
  u16 uwKi;    //Integral gain. (Q17)
  u16 uwUdMax; //Limits of PI regulator's output. (Q15)
}
CLR_ID_LOOP_CF;
extern CLR_ID_LOOP_CF       clr_stIdLoopTBCCf;

typedef struct
{
  u16 uwKUdMax;    // (Q15)
}
CLR_ID_LOOP_CF_PRT;
extern CLR_ID_LOOP_CF_PRT   clr_stIdLoopTBCCfPrt;

typedef struct
{
  s16 swIqRef; //Q-axis current reference. (Q15)
  s16 swIqFdb; //Q-axis current feedback. (Q15)
  s16 swUqDcp; //Q-axis  feedforward decoupled voltage.(Q15)
}
CLR_IQ_LOOP_TBC_IN;
extern CLR_IQ_LOOP_TBC_IN clr_stIqLoopTBCIn;

typedef struct
{
  s16 swIqErrOld; //Last value of q-axis current error between reference and feedback. (Q15)
  s16 swUqDcpOld; //Last value of q-axis feedforward decoupled voltage. (Q15)
  s32 slUqRefOld; //Last value of q-axis voltage reference old. (Q27)
}
CLR_IQ_LOOP_TBC_PRT;
extern CLR_IQ_LOOP_TBC_PRT   clr_stIqLoopTBCPrt;

typedef struct
{
  s16 swUqRef; //Q-axis PI regulator's voltage reference output. (Q15)
}
CLR_IQ_LOOP_TBC_OUT;
extern CLR_IQ_LOOP_TBC_OUT   clr_stIqLoopTBCOut;

typedef struct
{
  s16 swIqRef;   //Q-axis current reference. (Q15)
  u16 uwUdcPct;   //DC bus voltage permillage. (D3)
}
CLR_IQ_LOOP_CF_IN;

typedef struct
{
  u16 uwKp;    //Proportional gain. (Q14)
  u16 uwKi;    //Integral gain. (Q17)
  u16 uwUqMax; //Maximum of PI regulator's output. (Q15)
}
CLR_IQ_LOOP_CF;
extern CLR_IQ_LOOP_CF       clr_stIqLoopTBCCf;

typedef struct
{
  u16 uwDamp; // (D3)
  u16 uwKUqMax; //(Q15)
}
CLR_IQ_LOOP_CF_PRT;

typedef struct
{
  s16 swId;  //D-axis current of PM. (Q15)
  s16 swIq;  //Q-axis current of PM. (Q15)
  u16 uwFluxRef;  //Magnetizing flux reference of PM. (Q10)
  s32 slVel; //Motor velocity of PM. (Q31)
}
CLR_UDQ_DCP_TBC_IN;

typedef struct
{
  s16 swUdDcp; //Q-axis feedforward decoupled voltage of PM. (Q15)
  s16 swUqDcp; //D-axis feedforward decoupled voltage of PM. (Q15)
}
CLR_UDQ_DCP_TBC_OUT;
typedef struct {	//Coef. of CURMOD
    u32 ulKDegree; //Q32.
}	CURMODCF;
extern CURMODCF ccs_stModCf;

typedef struct {	//Input of CURMODCFIN
		u16 uwTctrl;
}	CURMODCFIN;
extern CURMODCFIN ccs_stModCfIn;

typedef struct
{
  s16 swDegreeComp;  //Q15
  s32 slFs; //Q31
}
CCS_MOD_INTE_TBC_IN;
extern CCS_MOD_INTE_TBC_IN ccs_stInteTBCIn;

typedef struct
{
  SQWORD_UNION FlxDegree; //Q(31+32)
}
CCS_MOD_INTE_TBC_OUT;
extern CCS_MOD_INTE_TBC_OUT ccs_stInteTBCOut;


typedef struct {
		s32 slVelRef;	//Input1, Velocity command, Q31;
		u16 uwVdcVt;		//Input2, Dc_bus voltage PU value;
		u16 uwIrmsAd;	//Input3, Current RMS
}	VELREFIN;
extern VELREFIN srg_stVelRefIn;

typedef struct {
		s32 slIFAccel;	//Cf1
		s32 slIFDecel;	//Cf2
	  s32 slAccel;	//Cf1
		s32 slDecel;	//Cf2
}	VELREFCF;
extern VELREFCF srg_stVelRefCf;
typedef s32	VELREFOUT;
extern VELREFOUT srg_slVelRef;


typedef struct
{
  s32 slPowerRef; //Velocity Reference. (Q31)
  s32 slPowerFdb; //Velocity Feedback.  (Q31)
  u32 ulSpdMax;   //Spd Limit.   (Q27)
	u32 ulSpdMin;   //Spd Limit.   (Q27)
}
POWER_LOOP_TBS_IN;
extern POWER_LOOP_TBS_IN Power_stLoopTBSIn;

typedef struct
{
	u32 ulP; //Proportional Coefficient for Velocity Regulation. (Q11)
	u32 ulI; //Integral Coefficient for Velocity Regulation. (Q17)
	u16 uwKfErr; //Anti-Noise Filter Coefficient. (Q15)
	u16 uwKfCmd; //Velocity Setting Filter Coefficient. (Q15)
	u16 uwKfTFED; //Feedforward Filter Coefficient. (Q15)
	u32 ulKff; //Torque Feedforward Coefficient. (Q13)
  u16 uwErrLpfCf;//Q16,Error Lpf Coefficient
}
POWER_CF;
extern POWER_CF Power_stCf;

typedef struct
{
  s32 slErrOld; //Last Regulated Velocity Error. (if SLR_HI_EFF_MACRO=1, Q30; else Q31)
}
POWER_LOOP_TBS_PRT;
extern POWER_LOOP_TBS_PRT Power_stLoopTBSPrt;

typedef struct
{
  s32 slSpdOut; //Spd Reference. (Q27)
}
POWER_LOOP_TBS_OUT;
extern POWER_LOOP_TBS_OUT Power_stLoopTBSOut;

typedef s32	POWERIN;
extern POWERIN Power_stPowerRefIn;

typedef struct {
		s32 slIFAccel;	//Cf1
		s32 slIFDecel;	//Cf2
	  s32 slAccel;	//Cf1
		s32 slDecel;	//Cf2
}	POWERCF;
extern POWERCF Power_stPowerRefCf;

typedef s32	POWEROUT;
extern POWEROUT Power_slPowerRef;

typedef struct
{//Input of 'sal_HFGen'
  u32 ulFreq;	//Input1, Q31
  u16 uwUMag;	//Input2, Q16
}
HFGENIN;



typedef struct
{	//Output of 'sal_HFGen'
  s16 swUcos;	//Output1, Q15
  s16 swUsin;	//Output2, Q15
  u32 ulDegreeh;	//Output3,Q31
}
HFGENOUT;

/***********************************************************************
 TypeDefs & Structure defines for 'sal_Track' & 'sal_TrackCf'
*************************************************************************/
typedef s32 TRACKIN;	//SAL PLL Err Input, Q31
typedef u32 TRACKCFIN; //Cf Input

typedef struct
{
  u32 ulVelKp;  //Q15 SAL PLL Proportional gain
  u32 ulVelKi;  //Q22 SAL PLL Integral gain
}
TRACKCF;

typedef struct
{//Output of 'sal_Track'
  u32 ulDegree;	//Q31 PLL degree output
  u32 ulDegreeCal;	//Q31 PLL degree output
  s32 slVelocity;	//Q31 PLL velocity output
  SQWORD sqKiInte;  //Q58
}
TRACKOUT;

/**************************************************************************************************
TYPEDEFS AND STRUCTURES
**************************************************************************************************/
typedef struct {    //Outputs of sin-cos table
    s16 swSin;    //Output SIN
    s16 swCos;    //Output COS
}    SINCOS;    //Struct of sin&cos

typedef struct
{
  s16 swUAlfa; //Alfa-axis stator voltage. (Q15)
  s16 swUBeta; //Beta-axis stator voltage. (Q15)
  s16 swMotIAlfa; //Alfa-axis stator current. (Q15)
  s16 swMotIBeta; //Beta-axis stator current. (Q15)
  s16 swFc; //Stator flux freq. obtained by PLL. (Q15)
}
FUN_LPF_TBC_IN;
extern FUN_LPF_TBC_IN fun_stMLpfTBCIn;


typedef struct
{
  s32 slStaFlxAlfa; //Alfa-axis stator flux output of MLPF. (Q10)
  s32 slStaFlxBeta; //Beta-axis stator flux output of MLPF. (Q10)
}
FUN_LPF_TBC_OUT;
extern FUN_LPF_TBC_OUT   fun_stMLpfTBCOut;

typedef struct
{
  u16 uwPLLINMag; //variable magnitude of PLL input. (Q10)
}
PLL_CF_IN;
extern PLL_CF_IN pll_stPllCfIn;

typedef struct
{
  u16 uwKp; //Kp of PI regulator coefficient. (Q14)
  u16 uwKi; //Ki of PI regulator coefficient. (Q18)
}
PLL_CF;
extern  PLL_CF      pll_stPllCf;

typedef struct
{
  SQWORD sqIntiVel; //Velocity integral. (Q43)
}
PLL_TBC_PRT;
extern PLL_TBC_PRT   pll_stActFlxTBCPrt;

typedef struct
{
  s32 slPLLINAlfa; //Alfa-axis input variable of PLL. (Q14)
  s32 slPLLINBeta; //Beta-axis input variable of PLL. (Q14)
}
PLL_TBC_IN;
extern  PLL_TBC_IN pll_stPllTBCIn;

typedef struct
{
  s32 slFreq; //Freq. output of PLL. (Q31)
  u32 ulDegree; //Phase output of PLL. (Q31)
}
PLL_TBC_OUT;
extern  PLL_TBC_OUT  pll_stActFlxTBCOut;


typedef struct {	//Input of COORDNT
		s16 swa;		//Input1, Q15
		s16 swb;		//Input2, Q15
		s16 swc;       //Input3, Q15
}	CDT_CLARKE_IN;
extern CDT_CLARKE_IN cdt_stClarkeIn;

typedef struct {	//Integrator,exchange & outputs of COORDNT
		s16 swAlfa;		//Input1, Q15
		s16 swBeta;		//Input2, Q15
}	CDT_CLARKE_OUT;
extern CDT_CLARKE_OUT cdt_stClarke;


typedef struct
{ //Input of COORDNT
  s16 swAlfa;    //Input1, Q15
  s16 swBeta;    //Input2, Q15
}
CDT_INVCLARKE_IN;
typedef struct
{
  s16 swa;   //Output1, Q15
  s16 swb;   //Output1, Q15
  s16 swc;   //Output1, Q15
}
CDT_INVCLARKE_OUT;
/************************************************************************
 function Park TypeDefs & Structure defines
*************************************************************************/

typedef struct {	//Input of COORDNT
		s16 swAlfa;		//Input1, Q15
		s16 swBeta;		//Input2, Q15
        u32 uldegree;   //Input3, Q31

}	CDT_PARK_IN;
extern CDT_PARK_IN cdt_stParkIn;

typedef struct {	//Integrator,exchange & outputs of COORDNT
		s16 swd;		//Input1, Q15
		s16 swq ;		//Input2, Q15
}	CDT_PARK_OUT;
extern CDT_PARK_OUT cdt_stPark;

/************************************************************************
 function Inverse park TypeDefs & Structure defines
*************************************************************************/

typedef struct {	//Input of COORDNT
		s16 swd;		//Input1, Q15
		s16 swq;		//Input2, Q15
    u32 uldegree;   //Input3, Q31

}	CDT_INVPARK_IN;
extern CDT_INVPARK_IN  cdt_stInvParkIn;


typedef struct {	//Integrator,exchange & outputs of COORDNT
		s16 swAlfa;		//Input1, Q15
		s16 swBeta;		//Input2, Q15
}	CDT_INVPARK_OUT;
extern CDT_INVPARK_OUT cdt_stInvPark;

typedef struct {	//Input of SVGEN
			s16 swUAlfa;		//Input1, Q15
			s16 swUBeta;		//Input2, Q15
		}	SVGENIN;
extern SVGENIN pwm_stPWMIn;

typedef struct {	//Integrator,exchange & outputs of SVGEN
			s16 swTa;		//Output1, Q15
			s16 swTb;		//Output2, Q15
			s16 swTc;		//Output3, Q15
                }	SVGENOUT;
extern SVGENOUT pwm_stPWMOut;



typedef struct  {
             u16 uwDTCLim;//Q15
             }DTC_CF;	//Coef. of  DBC
extern DTC_CF dtc_stDtcCf;

typedef struct {	//Input of DBC
			s16 swIaDtc;		//Input1, Q15
			s16 swIbDtc;	    //Input2, Q15
			s16 swIcDtc;		//Input3, Q15
            }DTC_IN;
extern DTC_IN dtc_stDtcIn;

typedef struct {	// output of DBC
			s16 swUaDtc;		// Phase A's DBC value for transfer,Q15
			s16 swUbDtc;		// Phase B's DBC value for transfer,Q15
			s16 swUcDtc;		// Phase C's DBC value for transfer,Q15
            }DTC_OUT;
extern DTC_OUT dtc_stDtcOut;
						
typedef struct {	//Coef. of  DBCCF
			u16 uwVdc;		                  //dc bus Value,Q14
			}DTC_CF_IN;
extern DTC_CF_IN dtc_stDtcCfIn;

typedef struct
{
			  u8 ubDeadtimeCt;  //Dead-time counter value
			  u16 uwHalfPeriodCt;  //Full period countervalue
}
PWM_COMP_PWM_CONFIG_IN;

typedef struct
{
			  u16 uwHalfPeriodCt;  //Full period countervalue
			  s16 swMfunc1;		//Voltage duty value for TIOC3B/D, Q15
			  s16 swMfunc2;		//Voltage duty value for TIOC4A/C, Q15
			  s16 swMfunc3;		//Voltage duty value for TIOC4B/D, Q15
			  s16 swDtcv1;		//Dead-time duty value for TIOC3B/D, Q15
			  s16 swDtcv2;		//Dead-time duty value for TIOC4A/C, Q15
			  s16 swDtcv3;		//Dead-time duty value for TIOC4B/D, Q15
}
PWM_COMP_PWM_WG_IN;  //wave-form generator

typedef struct
{
			  u16 uwHalfPeriodCt;  //Full period countervalue
			  s16 swMfunc1;		//Voltage duty value for GU/GX, Q15
			  s16 swMfunc2;		//Voltage duty value for GV/GY, Q15
			  s16 swMfunc3;		//Voltage duty value for GW/GZ, Q15
			  s16 swDtcv1;		//Dead-time duty value for GU/GX, Q15
			  s16 swDtcv2;		//Dead-time duty value for GV/GY, Q15
			  s16 swDtcv3;		//Dead-time duty value for GW/GZ, Q15
}PWM_NORM_PWM_GEN_IN;  //wave-form generator
extern PWM_NORM_PWM_GEN_IN pwm_stPwmGenIn;

typedef union
{
			  u16 uw;
			  struct
			  {
			    u32 Sect:4;  // Section flag in a tuning step.
			    u32 Step:4;  // Step flag in a tuning mode.
			    u32 Smpl:1;  // Sampling flag in a tuning step.
			    u32 Done:1;  // Done flag of a tuning step.
			    u32 Erro:2;  // Error flag of auto-tuning.
			    u32 Rsvd:4;  // Reserved flag bits.
			  } bit;            // Bit  Access
} ATT_STATUS_FLAG_UN;  // status word (bit-accessable).
extern  ATT_STATUS_FLAG_UN att_unStatusFlag;

typedef struct
{

	 u8 RunMode;  // Run mode flag.
   u8 CtrlMode; // Control mode flag.
   u8 CtrlModeInit; // Control mode initialization status flag.
 
} CTM_RUN_FLAG_UN;
extern CTM_RUN_FLAG_UN ctm_unRunSign;

typedef struct
{
	s16 swIa;//phase A current
	s16 swIb;//phase B current
	s16 swIc;//phase C current
}
CSV_CUR_TBC_OUT;


typedef struct
{
  s32 slVelRef; //Velocity Command. (Q31)
}
SLR_PREFILT_TBS_IN;


typedef struct
{
  s32 slVelRef; //Velocity Reference. (Q31)
}
SLR_PREFILT_TBS_OUT;
extern SLR_PREFILT_TBS_OUT slr_stPreLPFTBSOut;

typedef struct
{
  s32 slVelRef; //Velocity Command. (Q31)
  u32 ulTorMax; //Torque Limit. (Q27)
}
SLR_TFED_TBS_IN;

typedef struct
{
  s32 slVelOld; //Last Filtered Velocity. (if SLR_HI_EFF_MACRO=1, Q30; else Q31)
}
SLR_TFED_TBS_PRT;
extern SLR_TFED_TBS_PRT slr_stTFEDTBSPrt;

typedef struct
{
  s32 slTorFed; //Feedforward Torque. (Q27)
}
SLR_TFED_TBS_OUT;
extern SLR_TFED_TBS_OUT slr_stTFEDTBSOut;



typedef struct
{
  u8 ubTaskFlag; //Flag for Task Assignment, Range: 1~6
                                    //1,3,5 --- Coefficient calculation for control modules
	                                //2 --- Check on HF-PI pr
	                                //4 --- Check on LF-PI pr
	                                //6 --- Check on ZF-PI pr
  u32 ulPDef; //Proportional Coefficient Default. (Q11)
}
SLR_CF_PRT;

/* Includes ------------------------------------------------------------------*/
extern s32 hPhaseAOffset;
extern s32 hPhaseBOffset;
extern s32 hPhaseCOffset;
extern u16 RegularConvData_Tab[5];
extern u16 InjectedConvData_Tab[4];

extern u16 cf_uwUBVt,cf_uwIBAp,cf_uwRbOm,cf_uwLbHu,cf_uwTBSLast,cf_uwPolePairs;
extern s16 g_lBusVol,g_lDisBusVol;
extern u16 g_uVdcDis;
extern u32 kw_ulFunDegreeEst;
extern ULONG_UNION	fcmd;
extern s32 slIFFocSpd,slIFCurAutoSpd,slSTALLSpdTh,slSTALLSpdFun;
extern u16 Umax,Vmax,Wmax;

extern u32 PowerFst;
extern s32 kw_slFunVelEst;
extern s16 MPDChkOkFlag;
extern s16 UCur,VCur,WCur;
extern s16 swSALFUNDegreeErr,swIfDegreeErr,swIfDegreeErrLast,swIfDegreeErrDiff;
extern s32 slIfIqCmdInte;
extern u16 uwSector;
extern u16 fun_uwFcSignFlag;
extern u16 cf_uwTBCLast,cf_uwThetaTLast;
extern s16 swMotIAlfa,swMotIBeta;//Q15
extern s16 kw_swIqRef, kw_swIdRef;
extern s16 pwm_swUAlfaRef;
extern s16 pwm_swUBetaRef;
extern s16 pwm_swUAlfa;
extern s16 pwm_swUBeta;
extern u16 kw_uwFluxMag,kw_uwPowerCalFlux;
extern u16 cf_uwPMRs,cf_uwPMRsOm,cf_uwPMLd,cf_uwPMLq;
extern s16 swIdTmp2;
extern s16 swIqTmp2;
extern s16 swIdMax0;
extern s16 swIdMax1;
extern s16 swIqMin0;
extern s16 swIqMin1;
extern s16 swMagIdMax1;
extern s16 swMagIdMax2;
extern s16 swInitTheta0Pu;
extern s16 swInitTheta1Pu;
extern s16 swInitThetaBase0Pu;
extern s16 swInitThetaBase1Pu;
extern s16 swInitThetaOut0Pu;
extern s16 swInitThetaOut1Pu;
extern s32 slInitThetaSum0;
extern s32 slInitThetaSum1;
extern u16 uwMagVoltCmdPu;
extern u16 uwTonCnt;
extern u16 uwToffCnt;
extern u16 uwMagPoleStepIndex;
extern u16 uwCheckCnt;
extern u16 uwCheckStatus;
extern u16 uwRepeatedCnt;
extern u16 uwMaxRepeatedCnt;
extern s16 swInitTheatTab[20];
extern u16 uwVelAcc;
extern s32 slVdcPWMCf;
extern u16 uwPllFreCof;
extern u16 uwAccTime;
extern u8  bSector; 
extern u16 uwSectorCur;

extern u8 MotorDir;



void clr_Init (void);
void clr_IdLoopCf (CLR_ID_LOOP_CF *cf, CLR_ID_LOOP_CF_PRT *prt);
void clr_IdLoopTBC (CLR_ID_LOOP_TBC_IN *in, CLR_ID_LOOP_CF *cf, CLR_ID_LOOP_TBC_PRT *prt, CLR_ID_LOOP_TBC_OUT *out);
void clr_IqLoopTBC (CLR_IQ_LOOP_TBC_IN *in, CLR_IQ_LOOP_CF *cf, CLR_IQ_LOOP_TBC_PRT *prt, CLR_IQ_LOOP_TBC_OUT *out);

void vlr_Init (void);
void vlr_Cf (VLR_CF *cf);
void vol_VdcCf (VOL_VDC_CF *cf);
void vol_VdcTBC (VOL_VDC_CF *cf, VOL_VDC_TBC_OUT *out);


void vlr_VolToDutyTBC (VLR_VOL_TO_DUTY_TBC_IN *in, VLR_CF *cf, VLR_VOL_TO_DUTY_TBC_OUT *out);
void vlr_DutyToVolTBC (VLR_DUTY_TO_VOL_TBC_IN *in, VLR_CF *cf, VLR_DUTY_TO_VOL_TBC_OUT *out);
void pwm_voSVGen(SVGENIN *in, SVGENOUT *out);
void pwm_PWMInvTBC (void);
void pwm_PwmGenTBC(PWM_NORM_PWM_GEN_IN *stIn);
void pwm_CompPwmOutput(PWM_COMP_PWM_WG_IN *in);

void dtc_voDtcIni(void);
void dtc_voDtcCf(DTC_CF_IN *in,DTC_CF *out);
void dtc_voDtcCls(DTC_IN *in,DTC_OUT *out);

void srg_voVelRefCf(VELREFCF *out);
void srg_voVelRef(VELREFIN *in,VELREFCF *cf,VELREFOUT *out);
u32  srg_ulVelRefGen(u32 x);

void slr_Init (void);
void slr_CfSensorless (SLR_CF_IN *in, SLR_CF *cf);
void slr_LoopTBS (SLR_LOOP_TBS_IN *in, SLR_CF *cf, SLR_LOOP_TBS_PRT *prt, SLR_LOOP_TBS_OUT *out);



void ccs_voModIni(void);
void ccs_voModCf(CURMODCFIN *in,CURMODCF *out);
void ccs_voInteTBC(CCS_MOD_INTE_TBC_IN *in, CURMODCF *cf, CCS_MOD_INTE_TBC_OUT *out);

void pll_Init (void);
void pll_PllCf (PLL_CF_IN *in, PLL_CF *cf);
void pll_PllTBC (PLL_TBC_IN *in, PLL_CF *cf, PLL_TBC_PRT *prt, PLL_TBC_OUT *out);
void fun_Init (void);
void fun_MLpfTBC (FUN_LPF_TBC_IN *in, FUN_LPF_TBC_OUT *out);

void POWERRefCf(POWERCF *out);
u32  ulPowRefGen(u16 x);
void POWERRef(POWERIN *in,POWERCF *cf,POWEROUT *out);

void Power_Init (void);
void Power_Cf (POWER_CF *cf);
void Power_LoopTBS (POWER_LOOP_TBS_IN *in, POWER_CF *cf, POWER_LOOP_TBS_PRT *prt, POWER_LOOP_TBS_OUT *out);

void sal_LPF (LPFIN *in, LPFCF *cf, LPFOUT *out);
void sal_LPFCf (LPFCFIN *in, LPFCF *out);

void mpd_MagPoleDetectIni(void);
void mpd_MagPoleDetectTbc(void);
void SVPWM_3ShuntCurrentReadingCalibration(void);
//Curr_Components SVPWM_3ShuntGetPhaseCurrentValues(void);
#endif /* __STM32F4xx_IT_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
