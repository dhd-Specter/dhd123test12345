
#include "SciToOprt.h"
#include "performance.h"
#include "function.h"
#include "ctrlmode.h"



//-----------------------------------------------------------------------
//   0    30      60     90     120     150
//   0   0xaaa  0x1555 0x2000  0x2aaa  0x3555
//  180   210     240    270    300     330
//0x4000 0x4aaa 0x5555 0x6000  0x6aaa  0x7555
s16 swInitTheatTab[20] = { 0,        //0
                                        0x4000,        //180 degree
                                        0xaab,         //30  degree
                                        0x4aab,        //210 degree
                                        0x1555,        //60  degree
                                        0x5555,        //240 degree
                                        0x2000,        //90  degree
                                        0x6000,        //270 degree
                                        0x2aab,        //120 degree
                                        0x6aab,        //300 degree
                                        0x3555,        //150 degree
                                        0x7555,        //330 degree
                                        0,             //
                                        0,             //
                                        0,             //
                                        0,
                                        0,
                                        0,
                                        0,
                                        0};            //

																				

s32 hPhaseAOffset;
s32 hPhaseBOffset;
s32 hPhaseCOffset;
u16 RegularConvData_Tab[5] = { 0 };	
u16 InjectedConvData_Tab[4] = { 0 };	
																				
FUN_LPF_TBC_OUT   fun_stMLpfTBCOut;
FUN_LPF_TBC_IN    fun_stMLpfTBCIn;

LPFCF   sal_uwLpfCfPowerCal = LPFCF_DEFAULT,sal_uwLpfCfPowerDis = LPFCF_DEFAULT;
LPFCF   sal_uwLpfCfIqFdb = LPFCF_DEFAULT;
LPFCF   sal_uwLpfCfVdc = LPFCF_DEFAULT;
LPFCF   sal_uwLpfCfSpdDis = LPFCF_DEFAULT;
LPFCF   sal_uwLpfCfSpdFun = LPFCF_DEFAULT;
LPFCFIN sal_stLpfCfIn;

LPFIN  sal_swLpfIn;
LPFOUT sal_stLpfOutIqFdb;
LPFOUT sal_stLpfOutVdc;
LPFOUT sal_stLpfOutSpdDis;
LPFOUT sal_stLpfOutSpdFun;
LPFOUT sal_stLpfOutPowerCal,sal_stLpfOutPowerDis;


POWERIN   Power_stPowerRefIn;
POWEROUT  Power_slPowerRef=0;
POWERCF   Power_stPowerRefCf;
SVGENIN   pwm_stPWMIn;
SVGENOUT  pwm_stPWMOut =  SVGENOUT_DEFAULT;
PWM_NORM_PWM_GEN_IN pwm_stPwmGenIn;

PLL_TBC_PRT   pll_stActFlxTBCPrt;
PLL_TBC_OUT   pll_stActFlxTBCOut;
PLL_CF        pll_stPllCf;
PLL_TBC_IN    pll_stPllTBCIn;
PLL_CF_IN     pll_stPllCfIn;


CDT_CLARKE_IN cdt_stClarkeIn;
CDT_PARK_IN cdt_stParkIn;
CDT_INVPARK_IN cdt_stInvParkIn;
CDT_CLARKE_OUT cdt_stClarke = CDT_CLARKE_OUT_DEFAULT;
CDT_PARK_OUT cdt_stPark = CDT_PARK_OUT_DEFAULT;
CDT_INVPARK_OUT cdt_stInvPark = CDT_INVPARK_OUT_DEFAULT;

DTC_CF dtc_stDtcCf;
DTC_OUT dtc_stDtcOut;
DTC_IN dtc_stDtcIn;
DTC_CF_IN dtc_stDtcCfIn;


VELREFOUT srg_slVelRef=0;
VELREFCF  srg_stVelRefCf;
VELREFIN  srg_stVelRefIn;


CLR_ID_LOOP_CF        clr_stIdLoopTBCCf;
CLR_ID_LOOP_CF_PRT    clr_stIdLoopTBCCfPrt;
CLR_IQ_LOOP_CF        clr_stIqLoopTBCCf;
CLR_ID_LOOP_TBC_OUT   clr_stIdLoopTBCOut;
CLR_ID_LOOP_TBC_PRT   clr_stIdLoopTBCPrt;
CLR_IQ_LOOP_TBC_OUT   clr_stIqLoopTBCOut;
CLR_IQ_LOOP_TBC_PRT   clr_stIqLoopTBCPrt;
CLR_ID_LOOP_TBC_IN    clr_stIdLoopTBCIn;
CLR_IQ_LOOP_TBC_IN    clr_stIqLoopTBCIn;


CTM_RUN_FLAG_UN ctm_unRunSign={0};
ATT_STATUS_FLAG_UN att_unStatusFlag;
s32 slIFFocSpd,slIFCurAutoSpd,slSTALLSpdTh,slSTALLSpdFun;
ULONG_UNION	fcmd={0};



CURMODCF ccs_stModCf = CURMODCF_DEFAULT;
CCS_MOD_INTE_TBC_OUT ccs_stInteTBCOut;
CURMODCFIN ccs_stModCfIn;
CCS_MOD_INTE_TBC_IN ccs_stInteTBCIn;
SLR_CF slr_stCf;
SLR_PREFILT_TBS_OUT slr_stPreLPFTBSOut;
SLR_TFED_TBS_PRT slr_stTFEDTBSPrt;
SLR_TFED_TBS_OUT slr_stTFEDTBSOut;
SLR_LOOP_TBS_PRT slr_stLoopTBSPrt;
SLR_LOOP_TBS_OUT slr_stLoopTBSOut;
SLR_LOOP_TBS_IN slr_stLoopTBSIn;
SLR_CF_IN slr_stCfIn;


VLR_VOL_TO_DUTY_TBC_OUT   vlr_stVolToDutyTBCOut;
VLR_DUTY_TO_VOL_TBC_OUT   vlr_stDutyToVolTBCOut;
VLR_CF   vlr_stCf;

VLR_VOL_TO_DUTY_TBC_IN vlr_stVolToDutyTBCIn;
VLR_DUTY_TO_VOL_TBC_IN vlr_stDutyToVolTBCIn;
VOL_VDC_CF      vol_stVdcCf;
VOL_VDC_TBC_OUT  vol_stVdcTBCOut;
PWM_COMP_PWM_WG_IN pwm_stCompPwmIn;

u32 kw_ulFunDegreeEst;
u16 g_uVdcDis = 3100;
u16 Umax=0,Vmax=0,Wmax=0;
s16 swSALFUNDegreeErr,swIfDegreeErr,swIfDegreeErrLast,swIfDegreeErrDiff;
s32 slIfIqCmdInte;
s16 g_lBusVol = 0,g_lDisBusVol = 0;
u16 uwSector=4;
u16 fun_uwFcSignFlag;
u16 cf_uwTBCLast,cf_uwThetaTLast;
u16 cf_uwUBVt,cf_uwIBAp,cf_uwRbOm,cf_uwLbHu,cf_uwTBSLast,cf_uwPolePairs;
s16 swMotIAlfa,swMotIBeta;//
s16 kw_swIqRef, kw_swIdRef;
u16 kw_uwFluxMag,kw_uwPowerCalFlux;
s32 kw_slFunVelEst;
u16 cf_uwPMRs,cf_uwPMRsOm,cf_uwPMLd,cf_uwPMLq;
u32 PowerFst = 1;

s16 UCur,VCur,WCur;
s16 pwm_swUAlfaRef;
s16 pwm_swUBetaRef;
s16 pwm_swUAlfa;
s16 pwm_swUBeta;
s16 kw_swIqRef;
s16 MPDChkOkFlag;
s16 swIdTmp2 = 0;
s16 swIqTmp2 = 0;
s16 swIdMax0 = 0;
s16 swIdMax1 = 0;
s16 swIqMin0 = 0x7fff;
s16 swIqMin1 = 0x7fff;


s16 swMagIdMax1 = 0;      //没参与运算           
s16 swMagIdMax2 = 0;      //没参与运算  


s16 swInitTheta0Pu = 0;           
s16 swInitTheta1Pu = 0;
s16 swInitThetaBase0Pu = 0;          
s16 swInitThetaBase1Pu = 0;
s16 swInitThetaOut0Pu = 0;
s16 swInitThetaOut1Pu = 0;
s32 slInitThetaSum0 = 0;
s32 slInitThetaSum1 = 0;
u16 uwMagVoltCmdPu;            
u16 uwTonCnt;                  
u16 uwToffCnt;
u16 uwMagPoleStepIndex = 0;
u16 uwCheckCnt = 0;
u16 uwCheckStatus = 0;
u16 uwRepeatedCnt = 0;
u16 uwMaxRepeatedCnt = 1;
u16 uwVelAcc = 0;
u16 uwSpdRefSta = 0;
s32 slVdcPWMCf;
u16 uwPllFreCof;
u16 uwAccTime = 400;
u8  bSector; 
u16 uwSectorCur = 2;

u8 MotorDir = 1;

void clr_Init (void)
{
  u16 uwTemp;

  clr_stIdLoopTBCOut.swUdRef = 0;

  clr_stIdLoopTBCPrt.swIdErrOld = 0;
  clr_stIdLoopTBCPrt.slUdRefOld = 0;


  clr_stIqLoopTBCOut.swUqRef = 0;

  clr_stIqLoopTBCPrt.swIqErrOld = 0;
  clr_stIqLoopTBCPrt.slUqRefOld = 0;

  uwTemp = (u32)pr[VMAX] * 30894 / cf_uwUBVt;
  
  uwTemp = (u32)uwTemp * 1024 / 1000;
  clr_stIdLoopTBCCfPrt.uwKUdMax = uwTemp;

}

void clr_IdLoopCf (CLR_ID_LOOP_CF *cf, CLR_ID_LOOP_CF_PRT *prt)
{
  u32 ulTemp;
  u16 uwTemp,uwPMRs;
  u16 uwDamp,uwTf;
  u32 ulTsInv;

  
  uwTf = cf_uwTBCLast>>4;
  uwDamp = CLR_DAMP_RATIO_CST;
  uwTemp = (u32)uwDamp * uwDamp / 100;
  uwTemp = uwTemp << 2;
  uwTemp = (u32)uwTemp * uwTf / 10000;
  uwTemp = (u32)cf_uwPMLd * 4096 / uwTemp;
  ulTemp = (u32)uwTemp * pr[CLR_D_KP];
  if (ulTemp >= CLR_PI_OVERLIM_CST)
  ulTemp=CLR_PI_OVERLIM_CST;
  cf->uwKp = ulTemp / 100;

  uwPMRs = cf_uwPMRs;
  ulTsInv = (u32)uwPMRs * 8192 / cf_uwPMLd;
  ulTemp = (u32)ulTsInv * cf_uwTBCLast >> 15;
  ulTemp = (u32)ulTemp * pr[CLR_D_KI];
  if (ulTemp >= CLR_PI_OVERLIM_CST)
    ulTemp = CLR_PI_OVERLIM_CST;
  cf->uwKi = ulTemp / 100;

  cf->uwUdMax = (u32)prt->uwKUdMax * vol_stVdcTBCOut.uwVdcPct >> 10;

  clr_stIqLoopTBCCf.uwKp = (u32)cf->uwKp;
  clr_stIqLoopTBCCf.uwKi = (u32)cf->uwKi;
}

void clr_IdLoopTBC (CLR_ID_LOOP_TBC_IN *in, CLR_ID_LOOP_CF *cf,
                   CLR_ID_LOOP_TBC_PRT *prt, CLR_ID_LOOP_TBC_OUT *out)
{
  s32 slCurErr, slOut;
  s32 slTemp;
  u16 uwTmp;

  slCurErr = (s32)in->swIdRef - in->swIdFdb; 

  if (slCurErr > CLR_SAT_LIT_CST)
  {
    slCurErr = CLR_SAT_LIT_CST;
  }
  else if (slCurErr < -CLR_SAT_LIT_CST)
  {
    slCurErr = -CLR_SAT_LIT_CST;
  }

  uwTmp = (u32)cf->uwKp*cf->uwKi>>15;
  slOut = prt->slUdRefOld + ((s32)slCurErr * uwTmp)//
        + ((s32)cf->uwKp * ((s32)slCurErr - prt->swIdErrOld) * 16);  // (Q30).

  //Saturate the output.
  slTemp = (s32)uwAcrUDlim<<15;
  if ((slOut - slTemp) > 0)
  {
    slOut = slTemp;
    prt->swIdErrOld = 0;
  }
  else if ((slOut + slTemp) < 0)
  {
    slOut = 0 - slTemp;
    prt->swIdErrOld = 0;
  }
  else
  {
    prt->swIdErrOld = slCurErr;
  }

  out->swUdRef = slOut >> 15;
  prt->slUdRefOld = slOut;
}

void clr_IqLoopTBC (CLR_IQ_LOOP_TBC_IN *in, CLR_IQ_LOOP_CF *cf,
                   CLR_IQ_LOOP_TBC_PRT *prt, CLR_IQ_LOOP_TBC_OUT *out)
{
  s32 slCurErr, slOut;
  s32 slTemp;
  u16 uwTmp;
  
	slCurErr = (s32)in->swIqRef - in->swIqFdb; 

  if (slCurErr > CLR_SAT_LIT_CST)
  {
    slCurErr = CLR_SAT_LIT_CST;
  }
  else if (slCurErr < -CLR_SAT_LIT_CST)
  {
    slCurErr = -CLR_SAT_LIT_CST;
  }

  uwTmp = (u32)cf->uwKp*cf->uwKi>>15;
  slOut = prt->slUqRefOld + ((s32)slCurErr*uwTmp)//
        + ((slCurErr - prt->swIqErrOld) * cf->uwKp * 16);

  slTemp = ulAcrUQlim;
	/*
  slTemp = ((s32)uwAcrUQlim * uwAcrUQlim) - ((s32)clr_stIdLoopTBCOut.swUdRef * clr_stIdLoopTBCOut.swUdRef);
  if (slTemp < 0)
  {
    slTemp = 0;
  }
  slTemp = mth_uwSqrt(slTemp);
  slTemp = slTemp<<15;
  */
  if ((slOut - slTemp) > 0)
  {
    slOut = slTemp;
    prt->swIqErrOld = 0;
  }
  else if ((slOut + slTemp) < 0)
  {
    slOut = 0 - slTemp;
    prt->swIqErrOld = 0;
  }
  else
  {
    prt->swIqErrOld = slCurErr;
  }

  out->swUqRef = slOut >> 15; //
  prt->slUqRefOld = slOut;
}



void vlr_Init (void)
{
  vlr_stVolToDutyTBCOut.swTAlfaRef = 0;
  vlr_stVolToDutyTBCOut.swTBetaRef = 0;
  vlr_stDutyToVolTBCOut.swUa = 0;
  vlr_stDutyToVolTBCOut.swUb = 0;
  vlr_stDutyToVolTBCOut.swUc = 0;
}

void vlr_Cf (VLR_CF *cf)
{
  u16 uwTemp;

  cf->uwKUdcCf1 = (u32)cf_uwUBVt * 1061 / pr[VMAX];

  uwTemp = (u32)pr[VMAX] * 11585 / cf_uwUBVt;
  cf->uwKUdcCf2 = (u32)uwTemp * 1024 / 1000;
}

void vol_VdcCf (VOL_VDC_CF *cf)
{
  cf->uwVpuCnv = 33553920 / cf_uwUBVt;//6700;//
  cf->uwVarCnv = 2896309 / pr[VMAX];//2586;//
}


void vol_VdcTBC (VOL_VDC_CF *cf, VOL_VDC_TBC_OUT *out)
{
  out->uwVdcVt = g_uVdcDis; //0.1V
  out->uwVdc = ((u32)out->uwVdcVt*cf->uwVpuCnv)>>11; //
  out->uwVdcPct = (((u32)cf->uwVarCnv*out->uwVdcVt)>>8)>>4;//
}



void pwm_PWMInvTBC (void)
{
  /*
  vlr_stVolToDutyTBCIn.swUAlfaRef = pwm_swUAlfaRef;
  vlr_stVolToDutyTBCIn.swUBetaRef = pwm_swUBetaRef;
  vlr_stVolToDutyTBCIn.uwUdcPct = vol_stVdcTBCOut.uwVdcPct;
  vlr_VolToDutyTBC (&vlr_stVolToDutyTBCIn, &vlr_stCf, &vlr_stVolToDutyTBCOut);
	*/
	
	s32 slTemp;

  slTemp = (s32)pwm_swUAlfaRef * slVdcPWMCf  >> 10;
  /* Maximum(UAlfa,Ubeta)=>Udc*2/3,
  Maximum of duty ratio=>(Udc*2/3)/(2*Udc/3)=1 */
  if (slTemp > 32767)
    vlr_stVolToDutyTBCOut.swTAlfaRef = 32767;
  else if (slTemp < -32767)
    vlr_stVolToDutyTBCOut.swTAlfaRef = -32767;
  else
    vlr_stVolToDutyTBCOut.swTAlfaRef = slTemp;

  slTemp= (s32)pwm_swUBetaRef * slVdcPWMCf >> 10;
  if (slTemp > 32767)
    vlr_stVolToDutyTBCOut.swTBetaRef = 32767;
  else if (slTemp < -32767)
    vlr_stVolToDutyTBCOut.swTBetaRef = -32767;
  else
    vlr_stVolToDutyTBCOut.swTBetaRef = slTemp;

	pwm_stPWMIn.swUAlfa = vlr_stVolToDutyTBCOut.swTAlfaRef;
  pwm_stPWMIn.swUBeta = vlr_stVolToDutyTBCOut.swTBetaRef;
  pwm_voSVGen (&pwm_stPWMIn, &pwm_stPWMOut);

  pwm_stPwmGenIn.swMfunc1 = pwm_stPWMOut.swTa;
  pwm_stPwmGenIn.swMfunc2 = pwm_stPWMOut.swTb;
  pwm_stPwmGenIn.swMfunc3 = pwm_stPWMOut.swTc;
  if(pr[DTC_EN]==1)
  {
    pwm_stPwmGenIn.swDtcv1 = dtc_stDtcOut.swUaDtc;//
    pwm_stPwmGenIn.swDtcv2 = dtc_stDtcOut.swUbDtc;//
    pwm_stPwmGenIn.swDtcv3 = dtc_stDtcOut.swUcDtc;//
  }
  else
  {
    pwm_stPwmGenIn.swDtcv1 = 0;//
    pwm_stPwmGenIn.swDtcv2 = 0;//
    pwm_stPwmGenIn.swDtcv3 = 0;//
  }
  if((MPDChkOkFlag == 0) && (pr[IniMode] == 2))
  {
    pwm_stPwmGenIn.swDtcv1 = 0;//
    pwm_stPwmGenIn.swDtcv2 = 0;//
    pwm_stPwmGenIn.swDtcv3 = 0;//
  }
  
	pwm_PwmGenTBC(&pwm_stPwmGenIn);

  dtc_stDtcIn.swIaDtc = UCur;
  dtc_stDtcIn.swIbDtc = VCur;
  dtc_stDtcIn.swIcDtc = WCur;
  dtc_voDtcCls (&dtc_stDtcIn,&dtc_stDtcOut);

  /*	
  vlr_stDutyToVolTBCIn.swTa = pwm_stPWMOut.swTa;
  vlr_stDutyToVolTBCIn.swTb = pwm_stPWMOut.swTb;
  vlr_stDutyToVolTBCIn.swTc = pwm_stPWMOut.swTc;
  vlr_stDutyToVolTBCIn.uwUdcPct = vol_stVdcTBCOut.uwVdcPct;
  vlr_DutyToVolTBC (&vlr_stDutyToVolTBCIn, &vlr_stCf, &vlr_stDutyToVolTBCOut);
  	
  cdt_stClarkeIn.swa = -vlr_stDutyToVolTBCOut.swUa;
  cdt_stClarkeIn.swb = -vlr_stDutyToVolTBCOut.swUb;
  cdt_stClarkeIn.swc = -vlr_stDutyToVolTBCOut.swUc;
  cdt_Clarke(&cdt_stClarkeIn, &cdt_stClarke);
  pwm_swUAlfa = cdt_stClarke.swAlfa;
  pwm_swUBeta = cdt_stClarke.swBeta;	
	*/
}


void vlr_VolToDutyTBC (VLR_VOL_TO_DUTY_TBC_IN *in, VLR_CF *cf,
                        VLR_VOL_TO_DUTY_TBC_OUT *out)
{
  s32 slTemp;

  slTemp = (s32)in->swUAlfaRef * slVdcPWMCf  >> 10;
  /* Maximum(UAlfa,Ubeta)=>Udc*2/3,
  Maximum of duty ratio=>(Udc*2/3)/(2*Udc/3)=1 */
  if (slTemp > 32767)
    out->swTAlfaRef = 32767;
  else if (slTemp < -32767)
    out->swTAlfaRef = -32767;
  else
    out->swTAlfaRef = slTemp;

  slTemp= (s32)in->swUBetaRef * slVdcPWMCf >> 10;
  if (slTemp > 32767)
    out->swTBetaRef = 32767;
  else if (slTemp < -32767)
    out->swTBetaRef = -32767;
  else
    out->swTBetaRef = slTemp;
}


void vlr_DutyToVolTBC (VLR_DUTY_TO_VOL_TBC_IN *in, VLR_CF *cf,
                        VLR_DUTY_TO_VOL_TBC_OUT *out)
{
  u16 uwTemp;

  uwTemp = (u32)cf->uwKUdcCf2 * in->uwUdcPct >> 8;  //

  out->swUa = (s32)in->swTa * uwTemp >> 16;  //
  out->swUb = (s32)in->swTb * uwTemp >> 16;  //
  out->swUc = (s32)in->swTc * uwTemp >> 16;  //
}

void pwm_voSVGen (SVGENIN *in, SVGENOUT *out)
{
  s16 Vol1,Vol2,Vol3;
  u8 sector1,sector2,sector3;
  u8 sector;
  s16 X,Y,Z,t1,t2,Ta,Tb,Tc;
  s32 slTmp; //

  Vol1=in->swUBeta;

  slTmp=(((s32)in->swUAlfa*HALFSQRT3)>>15)-(s32)(Vol1>>1); //
  if (slTmp>32767) Vol2=32767;
  else if (slTmp<-32767) Vol2=-32767;
  else Vol2=slTmp;

  slTmp=-slTmp-Vol1; //
  if (slTmp>32767) Vol3=32767;
  else if (slTmp<-32767) Vol3=-32767;
  else Vol3=slTmp;

  if (Vol1>0)
  {
    sector1=1;
  }
  else
  {
    sector1=0;
  }
  if (Vol2>0)
  {
    sector2=1;
  }
  else
  {
	  sector2=0;
  }
  if (Vol3>0)
  {
	  sector3=1;
  }
  else
  {
	  sector3=0;
  }
  sector=sector1+(sector2<<1)+(sector3<<2);

  uwSector = sector;

  /* X,Y,Z calculation.
  Conventional SVPWM algorithm:
    X = sqrt(3)/2*beta
    Y = 3/4*Alfa + sqrt(3)/4*beta
    Z = X-Y = -3/4*Alfa + sqrt(3)/4*beta
  Note: The gain of 4/3 is intentionally used below to cancel the preceeding block gain of 3/4.
  Therefore,
  the first-step calculation (* 2/sqrt(3)):
    X' = 2/sqrt(3)*X = beta
    Y' = 2/sqrt(3)*Y = sqrt(3)/2*Alfa + 1/2*beta
    Z' = 2/sqrt(3)*Z = X'-Y'
  the second-step calculation (* 2/sqrt(3)):
    X" = 2/sqrt(3)*X'
    Y" = 2/sqrt(3)*Y'
    Z" = 2/sqrt(3)*Z'
  */
  X=in->swUBeta;

  slTmp=(((s32)in->swUAlfa*HALFSQRT3)>>15)+(X>>1); //
  if (slTmp>32767) Y=32767;
  else if (slTmp<-32767) Y=-32767;
  else Y=slTmp;

  slTmp=X-slTmp; //
  if (slTmp>32767) Z=32767;
  else if (slTmp<-32767) Z=-32767;
  else Z=slTmp;

  slTmp=((s32)X*PWM_2OVERSQRT3_CST)>>15; //
  if (slTmp>16383) X=16383;
  else if (slTmp<-16383) X=-16383;
  else X=slTmp;

  slTmp=((s32)Y*PWM_2OVERSQRT3_CST)>>15; //
  if (slTmp>16383) Y=16383;
  else if (slTmp<-16383) Y=-16383;
  else Y=slTmp;

  slTmp=((s32)Z*PWM_2OVERSQRT3_CST)>>15; //
  if (slTmp>16383) Z=16383;
  else if (slTmp<-16383) Z=-16383;
  else Z=slTmp;

  switch (sector)
  {
    case 1:
	  {
      t1=Z;
	    t2=Y;
	    Tb=(16384-t1-t2)>>1;
      if (Tb<0) Tb=0; //
		  Ta=Tb+t1;
		  Tc=Ta+t2;
	    break;
	  }
	  case 2:
    {
	    t1=Y;
	    t2=-X;
	    Ta=(16384-t1-t2)>>1;
      if (Ta<0) Ta=0; //
      	Tc=Ta+t1;
		  Tb=Tc+t2;
	    break;
	  }
	  case 3:
    {
	    t1=-Z;
	    t2=X;
	    Ta=(16384-t1-t2)>>1;
      if (Ta<0) Ta=0; //
	    Tb=Ta+t1;
	    Tc=Tb+t2;
	    break;
	  }
	  case 4:
    {
	    t1=-X;
	    t2=Z;
	    Tc=(16384-t1-t2)>>1;
      if (Tc<0) Tc=0; //
	    Tb=Tc+t1;
	    Ta=Tb+t2;
	    break;
	  }
	  case 5:
    {
	    t1=X;
	    t2=-Y;
	    Tb=(16384-t1-t2)>>1;
      if (Tb<0) Tb=0; //
	    Tc=Tb+t1;
	    Ta=Tc+t2;
	    break;
    }
	  case 6:
    {
	    t1=-Y;
	    t2=-Z;
	    Tc=(16384-t1-t2)>>1;
	    if (Tc<0) Tc=0; //
	    Ta=Tc+t1;
	    Tb=Ta+t2;
	    break;
	  }
    default:
	  {
	    Ta = 8192;
	    Tb = 8192;
	    Tc = 8192;
			break;
	  }
  }

  if (Ta>16383) Ta=32767;
	 else Ta=Ta+Ta;
  if (Tb>16383) Tb=32767;
	 else Tb=Tb+Tb;
  if (Tc>16383) Tc=32767;
	 else Tc=Tc+Tc;
  Ta=(Ta-16384)*2+1;
  Tb=(Tb-16384)*2+1;
  Tc=(Tc-16384)*2+1;
  
	out->swTa=Ta;
  out->swTb=Tb;
  out->swTc=Tc;
}


void pwm_PwmGenTBC(PWM_NORM_PWM_GEN_IN *stIn)
{
  pwm_stCompPwmIn.swMfunc1 = -stIn->swMfunc1;
  pwm_stCompPwmIn.swMfunc2 = -stIn->swMfunc2;
  pwm_stCompPwmIn.swMfunc3 = -stIn->swMfunc3;
  pwm_stCompPwmIn.swDtcv1 = -stIn->swDtcv1;
  pwm_stCompPwmIn.swDtcv2 = -stIn->swDtcv2;
  pwm_stCompPwmIn.swDtcv3 = -stIn->swDtcv3;
  pwm_CompPwmOutput(&pwm_stCompPwmIn);
}

u16 pwm_uwPwmCmpBuf[3];//for pc pwm output monitor

void pwm_CompPwmOutput(PWM_COMP_PWM_WG_IN *in)
{
	s16 overflow1,overflow2;
	
	in->uwHalfPeriodCt=PWM_PERIOD;//get pwm period counter 8kHz 3000

	if(uwabs(in->swMfunc1) == PWM_ONE_CST_Q15)
  {
    if(in->swMfunc1 < 0)
      pwm_uwPwmCmpBuf[0] = 0;
    else
      pwm_uwPwmCmpBuf[0] = in->uwHalfPeriodCt;
  }
  else
  {
    overflow1 = ((u32)in->swMfunc1 + PWM_ONE_CST_Q15)>>1;
    overflow2 = ((u32)in->swMfunc1 - PWM_ONE_CST_Q15)>>1;
    if(overflow1 <= (in->swDtcv1 >> 1))
      pwm_uwPwmCmpBuf[0] = 0;
    else if(overflow2 >= (in->swDtcv1 >> 1))
      pwm_uwPwmCmpBuf[0] = in->uwHalfPeriodCt;
    else
      pwm_uwPwmCmpBuf[0] = (((u32)in->uwHalfPeriodCt*(overflow1 - (in->swDtcv1 >> 1)))>>15);
  }
	if(uwabs(in->swMfunc2) == PWM_ONE_CST_Q15)
  {
    if(in->swMfunc2 < 0)
      pwm_uwPwmCmpBuf[1] = 0;
    else
      pwm_uwPwmCmpBuf[1] = in->uwHalfPeriodCt;
  }
  else
  {
    overflow1 = ((u32)in->swMfunc2 + PWM_ONE_CST_Q15)>>1;
    overflow2 = ((u32)in->swMfunc2 - PWM_ONE_CST_Q15)>>1;
    if(overflow1 <= (in->swDtcv2 >> 1))
      pwm_uwPwmCmpBuf[1] = 0;
    else if(overflow2 >= (in->swDtcv2 >> 1))
      pwm_uwPwmCmpBuf[1] = in->uwHalfPeriodCt;
    else
      pwm_uwPwmCmpBuf[1] = (((u32)in->uwHalfPeriodCt*(overflow1 - (in->swDtcv2 >> 1)))>>15);
  }
	if(uwabs(in->swMfunc3) == PWM_ONE_CST_Q15)
  {
    if(in->swMfunc3 < 0)
      pwm_uwPwmCmpBuf[2] = 0;
    else
      pwm_uwPwmCmpBuf[2] = in->uwHalfPeriodCt;
  }
  else
  {
    overflow1 = ((u32)in->swMfunc3 + PWM_ONE_CST_Q15)>>1;
    overflow2 = ((u32)in->swMfunc3 - PWM_ONE_CST_Q15)>>1;
    if(overflow1 <= (in->swDtcv3 >> 1))
      pwm_uwPwmCmpBuf[2] = 0;
    else if(overflow2 >= (in->swDtcv3 >> 1))
      pwm_uwPwmCmpBuf[2] = in->uwHalfPeriodCt;
    else
      pwm_uwPwmCmpBuf[2] = (((u32)in->uwHalfPeriodCt*(overflow1 - (in->swDtcv3 >> 1)))>>15);
  }
	if(pwm_uwPwmCmpBuf[0]>PWM_PERIOD)
		pwm_uwPwmCmpBuf[0]=PWM_PERIOD;
	if(pwm_uwPwmCmpBuf[1]>PWM_PERIOD)
		pwm_uwPwmCmpBuf[1]=PWM_PERIOD;
	if(pwm_uwPwmCmpBuf[2]>PWM_PERIOD)
		pwm_uwPwmCmpBuf[2]=PWM_PERIOD;
	
	if(MotorDir)
	{
		TMR1->CC1 = pwm_uwPwmCmpBuf[1];
		TMR1->CC2 = pwm_uwPwmCmpBuf[2];
		TMR1->CC3 = pwm_uwPwmCmpBuf[0];
		TMR1->CC4 = PWM_PERIOD - 1;
	}
	else
	{
		TMR1->CC1 = pwm_uwPwmCmpBuf[2];
		TMR1->CC2 = pwm_uwPwmCmpBuf[1];
		TMR1->CC3 = pwm_uwPwmCmpBuf[0];
		TMR1->CC4 = PWM_PERIOD - 1;
	}

}




void dtc_voDtcIni(void)
{
	dtc_stDtcOut.swUaDtc = 0;
	dtc_stDtcOut.swUbDtc = 0;
	dtc_stDtcOut.swUcDtc = 0;
}

void dtc_voDtcCf(DTC_CF_IN *in,DTC_CF *out)
{
	out->uwDTCLim = ((u32)pr[DTCDu]<<11)/8192;//
}

void dtc_voDtcCls(DTC_IN *in,DTC_OUT *out)
{
	SLONG_UNION temp;
	u16 dtcratio;
	dtcratio = pr[DTCRATIO];
	temp.sl = (s32)in->swIaDtc * dtcratio;
	out->swUaDtc = temp.sw.hi;
	if (uwabs(out->swUaDtc) > dtc_stDtcCf.uwDTCLim)
	{
    if (out->swUaDtc > 0)
		  out->swUaDtc = dtc_stDtcCf.uwDTCLim;
    else
		  out->swUaDtc = -dtc_stDtcCf.uwDTCLim;
	}

	temp.sl = (s32)in->swIbDtc * dtcratio;
	out->swUbDtc = temp.sw.hi;
	if (uwabs(out->swUbDtc) > dtc_stDtcCf.uwDTCLim)
	{
    if (out->swUbDtc > 0)
		  out->swUbDtc = dtc_stDtcCf.uwDTCLim;
    else
		  out->swUbDtc = -dtc_stDtcCf.uwDTCLim;
	}

	temp.sl = (s32)in->swIcDtc * dtcratio;
	out->swUcDtc = temp.sw.hi;
	if (uwabs(out->swUcDtc) > dtc_stDtcCf.uwDTCLim)
	{
    if (out->swUcDtc > 0)
		  out->swUcDtc = dtc_stDtcCf.uwDTCLim;
    else
		  out->swUcDtc = -dtc_stDtcCf.uwDTCLim;
	}
}

void srg_voVelRefCf(VELREFCF *out)
{
  u16 FBase,uwTemp;

  uwTemp =  (u32)pr[RATED_RPM] * (u32)10 * (u32)cf_uwPolePairs/(u32)60;
  FBase = (u32)uwTemp * (u32)10 * 65536 / pr[FB];//
  if (uwAccTime != 0)
    out->slAccel = (u32)FBase * 32767 / uwAccTime / GLB_TBS_FREQ_CST_HZ * 10;
	else
	  out->slAccel = (u32)FBase * 32767 / GLB_TBS_FREQ_CST_HZ * 10;

	if(uwAccTime != 0)
    out->slDecel = (u32)FBase * 32767 / uwAccTime / GLB_TBS_FREQ_CST_HZ * 10;
	else
	  out->slDecel = (u32)FBase * 32767 / GLB_TBS_FREQ_CST_HZ * 10;
 	
	out->slIFAccel = (u32)FBase * 32767 / 400 / GLB_TBS_FREQ_CST_HZ * 10;
	out->slIFDecel = (u32)FBase * 32767 / 400 / GLB_TBS_FREQ_CST_HZ * 10;
}

void srg_voVelRef(VELREFIN *in,VELREFCF *cf,VELREFOUT *out)
{
	if(ulabs(srg_slVelRef) > slSTALLSpdTh)
	{
		if (in->slVelRef > *out)
	  {
		  *out += cf->slAccel;
		  if (*out > in->slVelRef)
			  *out = in->slVelRef;
		  uwVelAcc = 0;
		
		  uwSpdRefSta = 0;
	  }
	  else if (in->slVelRef < *out)
	  {
		  *out -= cf->slDecel;
		  if (*out < in->slVelRef)
			  *out = in->slVelRef;
		  if(kw_swIqRef<1000) 
			  uwVelAcc = 1;
		
		  uwSpdRefSta = 0;
	  }
	  else
	  {
		  uwVelAcc = 0;
		  uwSpdRefSta = 1;
	  }	
  }
	else
	{
		if (in->slVelRef > *out)
	  {
		  *out += cf->slIFAccel;
		  if (*out > in->slVelRef)
			  *out = in->slVelRef;
		  uwVelAcc = 0;
		
		  uwSpdRefSta = 0;
	  }
	  else if (in->slVelRef < *out)
	  {
		  *out -= cf->slIFDecel;
		  if (*out < in->slVelRef)
			  *out = in->slVelRef;
		  if(kw_swIqRef<1000) 
			  uwVelAcc = 1;
		
		  uwSpdRefSta = 0;
	  }
	  else
	  {
		  uwVelAcc = 0;
		  uwSpdRefSta = 1;
	  }	
  }
}

u32 srg_ulVelRefGen(u32 x)
{
	u32 tmp1,tmp2;

	tmp1 = ((u32)x<<15)/pr[FB];
	tmp2 = tmp1<<16;
	return (tmp2);	
}

u32 ulPowRefGen(u16 x)//UNIT:W
{
	u32 tmp1,tmp2;

	tmp1 = ((u32)x<<10)/cf_uwPbWa;//Q15
	tmp2 = tmp1<<16;
	return (tmp2);
}	

void POWERRefCf(POWERCF *out)
{
  u16 PowerBase,uwTemp;

  uwTemp = pr[RATED_POW];
  PowerBase = (u32)uwTemp * 102 / cf_uwPbWa;//

	if (uwAccTime != 0)
    out->slAccel = (u32)PowerBase * TBSPowerLim * 32767 / uwAccTime / GLB_TBS_FREQ_CST_HZ * 20;
	else
	  out->slAccel = (u32)PowerBase * TBSPowerLim * 32767 / GLB_TBS_FREQ_CST_HZ * 20;

	if(uwAccTime != 0)
    out->slDecel = (u32)PowerBase * TBSPowerLim * 32767 / uwAccTime / GLB_TBS_FREQ_CST_HZ * 20;
	else
	  out->slDecel = (u32)PowerBase * TBSPowerLim * 32767 / GLB_TBS_FREQ_CST_HZ * 20;
	//开环引导加减速度
	out->slIFAccel = (u32)PowerBase * TBSPowerLim * 32767 / 40 / GLB_TBS_FREQ_CST_HZ * 20;
	out->slIFDecel = (u32)PowerBase * TBSPowerLim * 32767 / 40 / GLB_TBS_FREQ_CST_HZ * 20;
}

void POWERRef(POWERIN *in,POWERCF *cf,POWEROUT *out)
{
	
	if (*in > *out)
	{
		*out += cf->slAccel<<4;
		if (*out > *in)
			*out = *in;
	}
	else if (*in < *out)
	{
		*out -= cf->slDecel<<4;
		if (*out < *in)
			*out = *in;
	}
	else
	{

	}	
	
}

POWER_CF Power_stCf;
POWER_LOOP_TBS_PRT Power_stLoopTBSPrt;
POWER_LOOP_TBS_OUT Power_stLoopTBSOut;
POWER_LOOP_TBS_IN Power_stLoopTBSIn;

void Power_Init (void)
{
  Power_stLoopTBSPrt.slErrOld = 0;
  Power_stLoopTBSOut.slSpdOut = pr[VELCMD]>>2;
}

void Power_Cf (POWER_CF *cf)
{
  u16 uwTemp;
	
	uwTemp = 64 + ((u32)pr[SLR_ERR_LPF] * GLB_TBS_FREQ_CST_HZ / 16);  // 
  cf->uwErrLpfCf = (u32)8192 * 64 / uwTemp;  //Q13

	cf->ulP = 1000; //Q5
	cf->ulI = 1000; //Q17

}

s16 LIJIN,LIJING;
void Power_LoopTBS (POWER_LOOP_TBS_IN *in, POWER_CF *cf, POWER_LOOP_TBS_PRT *prt, POWER_LOOP_TBS_OUT *out)
{
  s32 slTmp;
  s32 slErr;
  s32 slTemp;
  SQWORD sqTmp;


    slErr = (in->slPowerRef>>16) - (in->slPowerFdb>>16); //Q15
		if (slErr > 0x7fff)
      slErr = 0x7fff;
    else if (slErr < -0x7fff)
      slErr = -0x7fff;
    slErr = prt->slErrOld + ((s32)cf->uwErrLpfCf * (slErr-prt->slErrOld) >> 13);//Q15
    
	  //slTemp = ((s32)slErr*cf->ulI)>>17;	//Q15	
    slTemp = ((s32)slErr*10000)>>17;	//Q15	
		slTmp = slTemp + (slErr - prt->slErrOld); //Q15		
		if (slTmp > 0x7fff)
      slTmp = 0x7fff;
    else if (slTmp < -0x7fff)
      slTmp = -0x7fff;
		LIJIN = slTmp;
    //sqTmp = slTmp * cf->ulP + out->slSpdOut; //Q20
		sqTmp = slTmp * 10000 + out->slSpdOut; //Q20
		sqTmp = sqTmp >> 5;
		LIJING = sqTmp;
		if (sqTmp - ((s32)in->ulSpdMax) > 0)
    {
      sqTmp = (s32)in->ulSpdMax;
      prt->slErrOld = slErr;
    }		
		else if (sqTmp - ((s32)in->ulSpdMin) < 0)
    {
      sqTmp = (s32)in->ulSpdMin;
      prt->slErrOld = slErr;
    }
    else
    {
      prt->slErrOld = slErr; //
    }

    out->slSpdOut = sqTmp; //
}



void slr_Init (void)
{
  slr_stPreLPFTBSOut.slVelRef = 0;
  slr_stLoopTBSPrt.slErrOld = 0;
  slr_stLoopTBSPrt.slTorFedOld = 0;

  slr_stLoopTBSOut.slTorquRef = 0;
	slr_stLoopTBSOut.slErrInt = 0;
  slr_stTFEDTBSPrt.slVelOld = 0;
  slr_stTFEDTBSOut.slTorFed = 0;
}

void slr_CfSensorless (SLR_CF_IN *in, SLR_CF *cf)
{

  u16 uwTd,uwTdLpf,uwTemp;
  u32 ulJm,ulKvi,ulP,ulI;

  u16 uwFsHz; //Q17

    ulJm = (UQWORD)pr[J_MOTOR]*8796093/cf_ulJbKgm; //Q4=D6*Q43/1000000/Q39, 2^43/1000000 = 8796093
    if(pr[CTRLM]==7)
    {
	    uwTemp = uwabs(in->swFsCMD);
      uwFsHz = (u32)uwTemp*pr[FB]>>15;//0.1Hz
      if(uwFsHz < (pr[IFFOCSPD]*4))//40Hz*4
      {
        ulJm = ulJm; //
      }
      else
      {
        ulJm = ulJm;
      }
	  }
    
		uwTdLpf=(u32)pr[FB]*161*(pr[SLR_ERR_LPF])/1000;//161 = 2*3.14*0.1*256,Q8
    uwTemp = ((u32)pr[FB]<<8) / pr[OB_BW]; //(D1<<8)/D1=Q8
    uwTd = (in->uwTdACR>>2) + uwTdLpf + (cf_uwTBSLast>>3) + uwTemp;
    
    ulKvi=10485760L/((u32)pr[SLR_SHAPE]*uwTd);//Q12=2^22*10/(D1*Q8),10485760=2^20*10
    ulI = ((UQWORD)ulKvi*pr[SLR_KI]*cf_uwTBSLast>>5)/100; //Q17=(Q12*Q10*D2>>5)/100
    cf->ulI = ulI;
		ulP = (UQWORD)(pr[SLR_SHAPE]+10)*ulJm*16384L/((u32)pr[SLR_SHAPE]*uwTd);//Q11=(D1*Q4<<15)/(2*D1*Q8)=Q11,(2^15)/2=16384
    ulP = ulP*pr[SLR_KP]/100;
    if(ulP>SLR_KP_MAX_CST)
	    ulP = SLR_KP_MAX_CST;

		cf->ulP = ((UQWORD)ulP << 7) / kw_uwFluxMag; //Q11 + 7 - 10
    cf->ulKpKi = (UQWORD)cf->ulP * cf->ulI >> 13;//Q12
		
    uwTemp = 64 + ((u32)pr[SLR_ERR_LPF] * GLB_TBS_FREQ_CST_HZ / 16);  // 
    cf->uwErrLpfCf = (u32)8192 * 64 / uwTemp;  //Q13

}

s16 lli1,lli2,lli3,lli4;
u16 uwVdcIqLim = 3900;
void slr_LoopTBS (SLR_LOOP_TBS_IN *in, SLR_CF *cf, SLR_LOOP_TBS_PRT *prt, SLR_LOOP_TBS_OUT *out)
{
  s32 slTmp,slTemp;//,slTorMax;
  s32 slErr;
	u32 ulTemp;
	u16 uwTemp,uwFsHz;
	s32 slTorLow;
	
	if(g_uVdcDis > uwVdcIqLim)
		  slTorLow = 0;
    else			
		  slTorLow = 0;//(s32)1000 * 4096;//

			slErr = (in->slVelRef>>12) - (in->slVelFdb>>12); //Q19
		  if (slErr > 0x1ffff)
        slErr = 0x1ffff;
      else if (slErr < -0x1ffff)
        slErr = -0x1ffff;
		  lli1 = slErr>>4;
		
		  slTemp = (slErr - prt->slErrOld); //Q19

      if (slTemp > 0x1ffff)
        slTemp = 0x1ffff;
      else if (slTemp < -0x1ffff)
        slTemp = -0x1ffff;

      slTemp = (slTemp * cf->uwErrLpfCf>>13); //(Q19)*(Q13>>13)=Q19
      slErr =  slTemp + prt->slErrOld; //Q19
     
      if (slErr > 0x1ffff)
        slErr = 0x1ffff;
      else if (slErr < -0x1ffff)
        slErr = -0x1ffff;
	    lli2 = slErr>>4;
	    slTemp = (s32)slErr*((s32)cf->ulI)>>17;	//19
      lli3 = slTemp>>4;
      slTmp = slTemp + (slErr - prt->slErrOld); //Q19
      if (slTmp > 0x1ffff)
        slTmp = 0x1ffff;
      else if (slTmp < -0x1ffff)
        slTmp = -0x1ffff;
      lli4 = slTmp>>4;
			
			ulTemp = ulabs(srg_slVelRef);
			uwTemp = ulTemp >> 16;
      uwFsHz = (u32)uwTemp*pr[FB]>>15;//0.1Hz
      
			if(uwSpdRefSta == 0)
			{
				if(uwFsHz < (pr[IFFOCSPD]*2))//40Hz*2
        {
          slTmp = slTmp >> 3;
        }
        else if(uwFsHz < (pr[IFFOCSPD]*3))//40Hz*3
        {
          slTmp = slTmp >> 2;
        }
			  else
        {
          slTmp = slTmp >> 1;
        }
      }	
      else
      {				
			  if(uwFsHz < (pr[IFFOCSPD]*2))//40Hz*2
        {
          slTmp = slTmp >> 2;
        }
        else if(uwFsHz < (pr[IFFOCSPD]*3))//40Hz*3
        {
          slTmp = slTmp >> 2;
        }
			  else
        {
          slTmp = slTmp * 3 >> 3;
        }
		  }
      slTemp = slTmp * cf->ulP + out->slTorquRef;//Q19+8=Q27
      if (slTemp - ((s32)in->ulTorMax) > 0)
      {
        slTemp = (s32)in->ulTorMax;
        prt->slErrOld = 0;
      }
      else if (slTemp + slTorLow < 0)
      {
        slTemp = -slTorLow;
        prt->slErrOld = 0;
      }		
      else
      {
        prt->slErrOld = slErr; //
      }
      out->slTorquRef = slTemp; //

}

void ccs_voModIni(void)
{
  if(pr[IniMode] == 2)
  {
	  ccs_stInteTBCOut.FlxDegree.sl.hi = ((s32)swInitTheta1Pu<<16) - 0x1FFFFFFF;
	  ccs_stInteTBCOut.FlxDegree.sl.hi = ccs_stInteTBCOut.FlxDegree.sl.hi & 0x7FFFFFFF;
	  ccs_stInteTBCOut.FlxDegree.sl.low = 0;
  }
  else
  {
	  ccs_stInteTBCOut.FlxDegree.sq = 0;
  }
}

void ccs_voModCf(CURMODCFIN *in,CURMODCF *out)
{
	
  out->ulKDegree = (u32) in->uwTctrl * 20860; //

}

void ccs_voInteTBC(CCS_MOD_INTE_TBC_IN *in, CURMODCF *cf, CCS_MOD_INTE_TBC_OUT *out)
{
  
  s16 swTmp1;
  u16 uwTmp1;
  s32 slTmp1;
  swTmp1 = in->slFs>>16;
  uwTmp1 = cf->ulKDegree>>16;
  slTmp1 = (s32)swTmp1*uwTmp1;

  out->FlxDegree.sl.hi = (out->FlxDegree.sl.hi + slTmp1)& 0x7FFFFFFF;
}

void pll_Init (void)
{
  pll_stPllCf.uwKp = 0;
  pll_stPllCf.uwKi = 0;


  pll_stActFlxTBCPrt.sqIntiVel = 0;



  pll_stActFlxTBCOut.slFreq = 0;
  pll_stActFlxTBCOut.ulDegree = 0;
}


void pll_PllCf (PLL_CF_IN *in, PLL_CF *cf)
{
  u16 uwFreqTmp, uwKiTmp;

  uwFreqTmp = (((u32)pr[OB_BW] >> 1) * 16384 / pr[FB]); 
  cf->uwKp = ((u32)uwFreqTmp * 23170 / in->uwPLLINMag); 
  uwKiTmp = ((u32)uwFreqTmp * cf_uwTBCLast / in->uwPLLINMag); 
  cf->uwKi = ((u32)uwKiTmp * uwFreqTmp / 32768); 
}

void pll_PllTBC (PLL_TBC_IN *in, PLL_CF *cf, PLL_TBC_PRT *prt, PLL_TBC_OUT *out)
{
  SINCOS SincosValue;
  s32 slDegreeErr;
  SQWORD sqFreq;
  u32 ulDegreeTmp;
  SQWORD sqTemp;

  out->slFreq = out->slFreq>>16; 
  
	ulDegreeTmp = out->ulDegree + ((s32)out->slFreq * cf_uwThetaTLast);
  out->ulDegree = ulDegreeTmp & 0x7FFFFFFF;

  sin_voSinCos(out->ulDegree >> 16,&SincosValue);
  SincosValue.swCos = SincosValue.swCos;
  SincosValue.swSin = SincosValue.swSin;
  slDegreeErr = (s32)in->slPLLINBeta * SincosValue.swCos - (s32)in->slPLLINAlfa * SincosValue.swSin; 

  slDegreeErr = slDegreeErr>>12;
  sqTemp = (s32)slDegreeErr * (cf->uwKi); 
  prt->sqIntiVel = prt->sqIntiVel + sqTemp;
  if (prt->sqIntiVel > PLL_FRE_INTE_LIM_CST)
  {
    prt->sqIntiVel = PLL_FRE_INTE_LIM_CST;
  }
  else if (prt->sqIntiVel < - PLL_FRE_INTE_LIM_CST)
  {
    prt->sqIntiVel = - PLL_FRE_INTE_LIM_CST;
  }

  sqFreq = (prt->sqIntiVel) + ((s32)slDegreeErr * cf->uwKp);

  if ((sqFreq - PLL_FRE_OUT_LIM_CST) > 0)
  {
    sqFreq = PLL_FRE_OUT_LIM_CST;
  }
  else if ((sqFreq + PLL_FRE_OUT_LIM_CST) < 0)
  {
    sqFreq = - PLL_FRE_OUT_LIM_CST;
  }
  out->slFreq = sqFreq;
}

void fun_Init (void)
{
  fun_stMLpfTBCOut.slStaFlxAlfa = 0;
  fun_stMLpfTBCOut.slStaFlxBeta = 0;

  if(0 == pr[VELDIR])
  {
	  fun_uwFcSignFlag = MLPF_FSIGNNEG_CST;
  }
  else
  {
	  fun_uwFcSignFlag = MLPF_FSIGNPOS_CST;
  }
}

void fun_MLpfTBC (FUN_LPF_TBC_IN *in, FUN_LPF_TBC_OUT *out)
{
  s32 slSBemfMpcAlfa, slSBemfMpcBeta, slTemp;
  s32 slSBemfAlfa,slSBemfBeta;

  slSBemfAlfa = (in->swUAlfa) - ((s32)in->swMotIAlfa * cf_uwPMRs >> 15); 
  slSBemfBeta = (in->swUBeta) - ((s32)in->swMotIBeta * cf_uwPMRs >> 15);

  if ((in->swFc - MLPF_FMIN_CST) > 0)
  {
	  fun_uwFcSignFlag = MLPF_FSIGNPOS_CST;
  }
  else if ((in->swFc + MLPF_FMIN_CST) < 0)
  {
	  fun_uwFcSignFlag = MLPF_FSIGNNEG_CST;
  }

  if (MLPF_FSIGNNEG_CST == fun_uwFcSignFlag)
  {
    slSBemfMpcAlfa = slSBemfAlfa - slSBemfBeta*1;
    slSBemfMpcBeta = slSBemfAlfa + slSBemfBeta*1;
  }
  else
  {
    slSBemfMpcAlfa = slSBemfBeta + slSBemfAlfa*1;  
    slSBemfMpcBeta = slSBemfBeta - slSBemfAlfa*1;
  }

  slTemp = ((s32)slSBemfMpcAlfa * cf_uwTBCLast >> 4) + out->slStaFlxAlfa;
  slTemp = 	slTemp>>10;
  out->slStaFlxAlfa = slTemp * uwPllFreCof;
 
  slTemp = ((s32)slSBemfMpcBeta * cf_uwTBCLast >> 4) + out->slStaFlxBeta;
  slTemp = 	slTemp>>10;
  out->slStaFlxBeta = slTemp * uwPllFreCof;
}

void sal_LPFCf (LPFCFIN *in, LPFCF *out)
{
  u16 uwTemp;
  u32 ulTemp1,ulTemp2;
  ulTemp1 = (u32) in->uwFc * in->uwTctrl;
  uwTemp =  ulTemp1 / pr[FB];
  ulTemp1 = (u32)uwTemp>>1<<16;
  ulTemp2 = uwTemp + 32767;
  if	(ulTemp2 > 65535)		ulTemp2 = 65535;
  uwTemp = ulTemp2;
  *out =  ulTemp1 / uwTemp;
}

void sal_LPF (LPFIN *in, LPFCF *cf, LPFOUT *out)
{
  s32 slTmp;
  slTmp = (s32)((*in) - (out -> swLpfOut)) * (*cf);
  out -> slTmp = (out -> slTmp) + slTmp;
  out -> swLpfOut = (out -> slTmp) >> 15;
}

void mpd_MagPoleDetectIni(void)
{
  u16 uwTmp;

  MPDChkOkFlag  = 0;
  uwRepeatedCnt = 0;
  uwMagPoleStepIndex = 0;
  swIdMax0 = 0;               
  swIdMax1 = 0;               
  swIqMin0 = 0x7fff;
  swIqMin1 = 0x7fff;
  swInitTheta0Pu = 0;
  swInitTheta1Pu = 0;
  swInitThetaBase0Pu = 0;
  swInitThetaBase1Pu = 0;
  slInitThetaSum0 = 0;
  slInitThetaSum1 = 0;
  swInitThetaOut0Pu = 0;
  swInitThetaOut1Pu = 0;

  uwCheckStatus = ToffStatus;
  swMagIdMax1 = 0;    
  swMagIdMax2 = 0;    
  uwTmp = ((u32)pr[MAG_VOLT]<<15)/cf_uwUBVt;  
  uwMagVoltCmdPu = uwTmp; 

  uwTonCnt  = pr[MAG_TIME];
  uwToffCnt = uwTonCnt << 1;


  dtc_voDtcIni();
  vlr_Init();
  dtc_voDtcCf(&dtc_stDtcCfIn, &dtc_stDtcCf);

}

void mpd_MagPoleDetectTbc(void)
{
  s16 swIdDiff,swIqDiff,swIdTmp,swIqTmp,swTmp; 

  if(MPDChkOkFlag == 1) return;

  if(Umax < uwabs(UCur))
		Umax = uwabs(UCur);
	if(Vmax < uwabs(VCur))
		Vmax = uwabs(VCur);
	if(Wmax < uwabs(WCur))
		Wmax = uwabs(WCur);
	
  cdt_stParkIn.swAlfa = swMotIAlfa;
  cdt_stParkIn.swBeta = swMotIBeta;
  if(uwMagPoleStepIndex > 0)
	{
    cdt_stParkIn.uldegree = ((u32)swInitTheatTab[uwMagPoleStepIndex-1])<<16;
  }
  else
	{
    cdt_stParkIn.uldegree = 0;
  }
  cdt_Park(&cdt_stParkIn, &cdt_stPark);

  
  swIdTmp = cdt_stPark.swd;
  swIqTmp = cdt_stPark.swq;


  swIdDiff = swIdTmp - swIdTmp2;
  swIqDiff = swIqTmp - swIqTmp2;
  if(swIdDiff < 0) swIdDiff = -swIdDiff;
  if(swIqDiff < 0) swIqDiff = -swIqDiff;

  uwCheckCnt++;
  if(uwCheckStatus == TonStatus)
  {
    if(uwCheckCnt > uwTonCnt)
    {
      PWM_DIS_DUTY();
      if( uwMagPoleStepIndex < 13)
			{         
        if(swIdDiff > swIdMax0)
				{    
          swIdMax0 = swIdDiff;
          swIqMin0 = swIqDiff;
          swInitTheta0Pu = swInitTheatTab[uwMagPoleStepIndex-1];
        }
      }
      else
			{
        swTmp = swIdDiff - swIqDiff;
        if( swTmp > swIdMax1)
				{
          swIdMax1 = swTmp;
          swInitTheta1Pu = swInitTheatTab[uwMagPoleStepIndex-1];
        }

      }
      uwCheckStatus = ToffStatus;
      uwCheckCnt = 0;
      swMagIdMax1 = uwabs(swIdTmp);  
      swMagIdMax2 = uwabs(swIdDiff); 
    }
    else
		{   
      if(uwCheckCnt == 1)        	{PWM_EN_DUTY();}
      else if(uwCheckCnt == 2)
			{
        swIdTmp2 = swIdTmp;
        swIqTmp2 = swIqTmp;
      }
    }
  }
  else
	{
    if(uwCheckCnt > uwToffCnt)
		{
      uwCheckCnt = 0;                     
      uwCheckStatus = TonStatus;            

      uwMagPoleStepIndex++;
      if(uwMagPoleStepIndex == 13) 
			{
        swIqMin1 = swIqMin0;
        swIdMax1 = swIdMax0 - swIqMin0;  
        swInitTheta1Pu = swInitTheta0Pu;
        MPDChkOkFlag = 1; 
        PWM_DIS_DUTY();
        swInitTheatTab[12] = (swInitTheta1Pu + MAGPOLE_15)&0x7fff;
        swInitTheatTab[13] = (swInitTheta1Pu - MAGPOLE_15)&0x7fff;
      }
      else if(uwMagPoleStepIndex == 15)
			{
        swInitTheatTab[14] = (swInitTheta1Pu + MAGPOLE_7P5)&0x7fff;
        swInitTheatTab[15] = (swInitTheta1Pu - MAGPOLE_7P5)&0x7fff;
      }
      else if(uwMagPoleStepIndex == 17)
			{
        swInitTheatTab[16] = (swInitTheta1Pu + MAGPOLE_3P75)&0x7fff;
        swInitTheatTab[17] = (swInitTheta1Pu - MAGPOLE_3P75)&0x7fff;
      }
      else if(uwMagPoleStepIndex == 19)
			{
        swInitTheatTab[18] = (swInitTheta1Pu + MAGPOLE_1P875)&0x7fff;
        swInitTheatTab[19] = (swInitTheta1Pu - MAGPOLE_1P875)&0x7fff;
      }
      else if(uwMagPoleStepIndex >= 21)
			{

        if (uwMagPoleStepIndex>=21)
				{
//        if(uwRepeatedCnt != 0)
          {
            slInitThetaSum0 += swInitTheta0Pu;
            swInitThetaBase0Pu = swInitTheta0Pu;
            slInitThetaSum1 += swInitTheta1Pu;
            swInitThetaBase1Pu = swInitTheta1Pu;
          }
          uwRepeatedCnt ++;
          if(uwRepeatedCnt >= uwMaxRepeatedCnt)
          {
            MPDChkOkFlag = 1; //
            uwRepeatedCnt = 0;
            swInitThetaOut0Pu = slInitThetaSum0/(uwMaxRepeatedCnt);
            swInitThetaOut1Pu = slInitThetaSum1/(uwMaxRepeatedCnt);
            PWM_DIS_DUTY();
          }
          else
					{
            uwMagPoleStepIndex = 1;
            swIdMax0=0;
            swIdMax1=0;
            swIqMin0=0;
            swIqMin1=0;
          }
        }
      }
    }
  }
  if(uwMagPoleStepIndex > 0)
  {
     cdt_stInvParkIn.uldegree = ((u32)swInitTheatTab[uwMagPoleStepIndex-1])<<16;
     cdt_stInvParkIn.swd = uwMagVoltCmdPu;
     cdt_stInvParkIn.swq = 0;
     cdt_InvPark(&cdt_stInvParkIn, &cdt_stInvPark);

     pwm_swUAlfaRef = cdt_stInvPark.swAlfa;
     pwm_swUBetaRef = cdt_stInvPark.swBeta;
		
     pwm_PWMInvTBC();
  }
}




void SVPWM_3ShuntCurrentReadingCalibration(void)
{
	static u16 bIndex=0;
	if(bIndex==0)
	{
		hPhaseAOffset = 0;
		hPhaseBOffset = 0;
		hPhaseCOffset = 0;
	}
  if(bIndex < NB_CONVERSIONS)
  {
    hPhaseAOffset += InjectedConvData_Tab[0];
	  hPhaseBOffset += InjectedConvData_Tab[1];
	  hPhaseCOffset += InjectedConvData_Tab[2];
		bIndex++;
  }
	else
	{
		hPhaseAOffset = hPhaseAOffset/64;
		hPhaseBOffset = hPhaseBOffset/64;
		hPhaseCOffset = hPhaseCOffset/64;
		sampleVddDivideTwo_bit = 1;										//33040
	}
}

