
#include "SciToOprt.h"
#include "performance.h"
#include "function.h"
#include "ctrlmode.h"
#include "application.h"
#include "spi.h"


u16 cf_uwWebRad,cf_uwtbSec,cf_uwPbWa;
u16 uwRunSpdDec = 0;
u16 uwTBSPowerCnt = 0;
u32 cf_ulTbNm,cf_ulJbKgm;
s32 kw_slFunActFlxAlfa,kw_slFunActFlxBeta;
s32 kw_slIqMaxBase,kw_slIqMaxCal;
u16 kw_uwStaticLq;
s32 kw_slTorRef;
s16 att_swIsRef;
u16 att_uwIN;
u16 att_uwSmplLength;
u32 att_ulTBSCt;
u16 att_uwTBCSmplCt;
s32 att_slAUTOTunIdc;
s32 att_slAUTOTunUdc;
s32 att_slAUTOTunUdcSum;
s16 swIFFocSpdPu;
s16 swIdCmd;
s32 flw_slIdCmdInte;
u16 uwIFtoFunCnt = 0;
s16 RunCmd = 0;
u16 uwMotPowWattCal = 0,uwMotPowWattDis = 0;
u16 uwSpdRefRadio = 1500;
u16 uwDisSpdFdb,uwDisIRef;
u16 uwUlimt,uwAcrUDlim,uwAcrUQlim;
u32 ulAcrUQlim;
u16 uwErrClear = 0;
u16 uwKeySpeedRef = 0,kw_uwPlugSpdHz = 0,uwKeyPowerRef = 0;
s16 swOVCnt = 0,swLVCnt = 0,swLVNomCnt = 0,swOVNomCnt = 0;
u16 uwIrateOver = 0;
u16 uwIdDecCom = 0;
u32 mpd_ulPMSALCnt;
s16 swPowerActive = 0;
s16 swPowerTmp = 0;
s16 IFtoFunSign = 0;
u16 uwIfSign = 0;
s32 slThetaComTBC = 0;
u16 uwSTAVOLCnt;
u32 ulSTACOF;
u16 uwSpdMechHz = 0,uwSpdMechCf = 0;
s16 swCurOCCnt = 0;
s32 slHighSpdCnt = 0;
u16 flw_uwUout;
s16 flw_swUerr,flw_swUerrDiff,flw_swUerrLast;
u16 uwRs,uwDTCDu;
s16 swUdOut;
s16 swSpeedFdbLPF = 0;
u16 adAverageCurrent=0;
u8  sampleVddDivideTwo_bit = 0;
u16 uwTBCCnt = 0;




//main函数while循环中调用
void ctm_CtrlModeSchdResv(void)
{
  if(RunCmd)
  {
    if (ctm_unRunSign.CtrlMode != 1)
    {
      ctm_unRunSign.CtrlMode = 1;
      ctm_unRunSign.CtrlModeInit = 0;
    }
  }
  else
  {
		ctm_unRunSign.CtrlMode = 0;
  }
  com_DevResv();
  
  switch (ctm_unRunSign.CtrlMode)
  {
		case 0:  
			      ctm_Resv();
			break;
    case 1:	
		        ctm_IFFOC();
		  break;

		default: break;  
	}	
}


//SysTick_Handler中调用
void ctm_TBSIsr(void)
{
  switch (ctm_unRunSign.RunMode)
  {
		case 0: ctm_Resv();
			break;

    case 2:	
		         ctm_IFFocTBS();
		  break;
		default:  break;	
	}	
	
}

//DMA中断中ctm_TBC调用
void ctm_TBCIsr(void)
{
	switch (ctm_unRunSign.RunMode)
  {
		case 0: ctm_Resv();
			break;
	  case 1:	
		        mpd_IFIniDegreeTBC();
		 break;
    case 2:	
		        ctm_IFFocTBC();
		  break;

		default: break;
	}	
}

static void ctm_Resv(void)
{
  PWM_DIS_DUTY();
  ctm_unRunSign.RunMode = 0;
}



static void ctm_IFFOC(void)
{
  u16 ModeSwitch;

  if (ctm_unRunSign.CtrlModeInit == 0) //
  {
    mpd_IFIniDegreeInit();
    ctm_unRunSign.RunMode = 1;//
    ctm_unRunSign.CtrlModeInit = 1;
  }
  else 
  {
    if(ctm_unRunSign.RunMode == 1)
	  {
	    ModeSwitch = mpd_IFIniDegreeResv();
	    if(ModeSwitch==1)
	    {
		    ctm_IFFocInit();
	      ctm_unRunSign.RunMode = 2;	  
        PWM_EN_DUTY();				
	    }
	  }
	  else if(ctm_unRunSign.RunMode == 2)
    {
      ctm_IFFocResv();
    }
  } 
}


void mpd_IFIniDegreeInit(void)
{
  u16 uwTemp;
	uwTemp = (u32)pr[I_RATE] * 46334 / cf_uwIBAp; //
	uwIrateOver = (u32)uwTemp * 4 >> 1;//200%
	
	mpd_MagPoleDetectIni();
	mpd_ulPMSALCnt = 0;
	Umax = 0;
	Vmax = 0;
	Wmax = 0;
}

u8 mpd_IFIniDegreeResv(void)
{
  if(MPDChkOkFlag == 1)
  {
	  TMR1->CC1 = PWM_PERIOD >> 1;
	  TMR1->CC2 = PWM_PERIOD >> 1;
	  TMR1->CC3 = PWM_PERIOD >> 1;
	  TMR1->CC4 = PWM_PERIOD - 1;
    return 1;
  }
	else
  {
    return 0;
  }  
}


void mpd_IFIniDegreeTBC(void)
{
	mpd_ulPMSALCnt++;
	if(mpd_ulPMSALCnt < 2000)  //此处以前是2000，修改为20
	{
		mpd_STOP();
	}
	else
	{
		mpd_MagPoleDetectTbc();
	}
}


void ctm_IFFocInit(void)
{
  u16 uwTemp, uwWr, uwE0;
  u32 ulTemp;

  slr_Init();	 
  Power_Init();
  clr_Init();	  
  dtc_voDtcIni();
  vlr_Init(); 

  srg_voVelRefCf(&srg_stVelRefCf);
	POWERRefCf(&Power_stPowerRefCf);
		
	uwTemp = pr[RATED_RPM] * 10 * cf_uwPolePairs;
  uwWr = (u32)uwTemp * 5461 / pr[FB]; //
  uwE0 = (u32)pr[BEMF] * 32768 / cf_uwUBVt; //
  kw_uwFluxMag = (u32)uwE0 * 1024 / uwWr; //
  
  uwTemp = (u32)pr[I_RATE] * 46334 / cf_uwIBAp; //
	ulTemp = (u32)uwTemp * pr[TE_LIM] / 100;

  kw_slIqMaxBase = (u32)ulTemp << 12;//Q27
	kw_slIqMaxCal = kw_slIqMaxBase >> 1;

	uwIrateOver = (u32)uwTemp * 4 >> 1;//200%
	  
	
  uwTemp = pr[RATED_RPM] * 10 * cf_uwPolePairs/60;


 
	clr_IdLoopCf(&clr_stIdLoopTBCCf, &clr_stIdLoopTBCCfPrt);
  dtc_voDtcCf(&dtc_stDtcCfIn, &dtc_stDtcCf);
		
	slr_stCfIn.uwTdACR=cf_uwTBCLast>>4;//
  slr_stCfIn.swFsCMD = 0;
  slr_stCfIn.swFsFDB = 0;
  slr_CfSensorless(&slr_stCfIn,&slr_stCf);
	Power_Cf(&Power_stCf);
  slr_stLoopTBSIn.ulErrIntMax = kw_slIqMaxBase/slr_stCf.ulKpKi;//Q15 = 27 - 12
	
	kw_swIdRef = 0;
  kw_swIqRef = 0;
  clr_stIdLoopTBCIn.swIdRef = 0;
  clr_stIqLoopTBCIn.swIqRef = 0;
	pwm_swUAlfaRef = 0;
  pwm_swUBetaRef = 0;
  swIdCmd = 0;


	swIFFocSpdPu = (s32)pr[IFFOCSPD] * 32768  / pr[FB];
  slIFFocSpd = swIFFocSpdPu<<16;
	slSTALLSpdTh = slIFFocSpd * 14 >> 3;
	slSTALLSpdFun = slIFFocSpd * 7 >> 3;
  slIFCurAutoSpd = (s32)pr[IFCURAUTOSPD] * 32768  / pr[FB];
  slIFCurAutoSpd = slIFCurAutoSpd<<16;

  IFtoFunSign=0;
	uwIfSign=0;

  srg_slVelRef = 0;  

  kw_slFunVelEst = 0;
  kw_ulFunDegreeEst = 0;

	PowerFst = 0;

	fun_Init ();
  pll_Init ();

	pll_stActFlxTBCPrt.sqIntiVel = 0;//(s32)swSpeedFdbLPF<<16;;
	pll_stActFlxTBCOut.ulDegree = (u32)swInitTheta1Pu<<16;
	pll_stPllCfIn.uwPLLINMag = kw_uwFluxMag;//
  pll_PllCf (&pll_stPllCfIn, &pll_stPllCf);
	swPowerActive = 0;
	
	sal_stLpfOutPowerCal.swLpfOut = 0;
	sal_stLpfOutPowerCal.slTmp = 0;
  sal_stLpfOutPowerDis.swLpfOut = 0;
	sal_stLpfOutPowerDis.slTmp = 0;
	
	sal_stLpfCfIn.uwFc = 50*10; 
  sal_stLpfCfIn.uwTctrl = cf_uwTBSLast;
  sal_LPFCf (&sal_stLpfCfIn, &sal_uwLpfCfPowerCal);
	
	sal_stLpfCfIn.uwFc = 50*10; 
  sal_stLpfCfIn.uwTctrl = cf_uwTBSLast;
  sal_LPFCf (&sal_stLpfCfIn, &sal_uwLpfCfPowerDis);
	
	sal_stLpfOutSpdFun.swLpfOut = 0;
	sal_stLpfOutSpdFun.slTmp = 0;
	
	sal_stLpfCfIn.uwFc = 50*10; 
	sal_stLpfCfIn.uwTctrl = cf_uwTBSLast;
  sal_LPFCf (&sal_stLpfCfIn, &sal_uwLpfCfSpdFun);
	
	sal_stLpfOutIqFdb.swLpfOut = 0;
	sal_stLpfOutIqFdb.slTmp = 0;
	
	sal_stLpfCfIn.uwFc = 5*10; 
	sal_stLpfCfIn.uwTctrl = cf_uwTBSLast;
  sal_LPFCf (&sal_stLpfCfIn, &sal_uwLpfCfIqFdb);
	
	sal_stLpfCfIn.uwFc = 25*10; 
	sal_stLpfCfIn.uwTctrl = cf_uwTBSLast;
  sal_LPFCf (&sal_stLpfCfIn, &sal_uwLpfCfVdc);
	
	sal_stLpfOutSpdDis.swLpfOut = 0;
	sal_stLpfOutSpdDis.slTmp = 0;
	
	sal_stLpfCfIn.uwFc = 15*10; 
	sal_stLpfCfIn.uwTctrl = cf_uwTBSLast;
  sal_LPFCf (&sal_stLpfCfIn, &sal_uwLpfCfSpdDis);
	
	ccs_voModIni();
	ccs_stModCfIn.uwTctrl = cf_uwTBCLast;
  ccs_voModCf(&ccs_stModCfIn, &ccs_stModCf);
	
	flw_swUerrLast = 0;
  flw_slIdCmdInte = 0;
	
	uwTemp = (u32)pr[RATED_RPM] * 100 * cf_uwPolePairs/60;//0.1Hz
	ulSTACOF = (u32)uwTemp*65536*2/3000;//300HZ/1000HZ=150V/500V
	
	slHighSpdCnt = 0;
  uwSTAVOLCnt = 0;
  uwIFtoFunCnt = 0;
  swCurOCCnt = 0;
  swOVCnt = 0;
  swLVCnt = 0;
	

	uwIdDecCom = 0;
	swIfDegreeErr = 0;
  slIfIqCmdInte = 0;
	


	
	if(MPDChkOkFlag == 1)
	{	
	  if((Umax < 2000) || (Vmax < 2000) || (Wmax < 2000))	
	  {				           
			g_uErrorCode = C_MOTORPHASE_GZ;
	  }
  }
}



void ctm_IFFocResv(void)
{
	u16 uwFcTmp, uwTemp;
	u32 ulTmp;
	
	uwTemp = pll_stActFlxTBCOut.slFreq>>16;
	uwFcTmp = uwabs(uwTemp)>>0;
	if(uwFcTmp<1638/2) uwFcTmp = 1638/2;
  uwTemp = 16384 + ((u32)uwFcTmp * cf_uwTBCLast >> 16);
  ulTmp = 268435456/uwTemp;
  uwPllFreCof = ulTmp>>4;
	
}




void ctm_IFFocTBS(void)
{
  u16 uwTmp,uwTemp,uwUlimTemp;
  s32 slVelRef;
  s16 swTemp,swSpdTmp,swTmp;
  u32 ulSpdVdccf,ulTemp,ulTorque;
	s32 slTmp;
	
	if(IFtoFunSign == 1)
	{	
		swTemp = kw_slFunVelEst>>16;	
	  uwTemp = uwabs(swTemp);	
	  kw_uwPlugSpdHz = (u32)uwTemp * pr[FB] >> 15;//0.1Hz
  }

  if(IFtoFunSign == 1)//FOC Sensorless
	{	  
    if(uwRunSpdDec == 0)
		{
		    fcmd.uw.hi = uwKeySpeedRef;
				kw_slIqMaxCal = kw_slIqMaxBase;
		}	
		else
		{	
			fcmd.uw.hi = pr[IFFOCSPD]>>1;
		}	
		
    if(fcmd.uw.hi < (pr[IFFOCSPD]>>1))
			fcmd.uw.hi = pr[IFFOCSPD]>>1; 
		if(fcmd.uw.hi > pr[VELCMD])
		  fcmd.uw.hi = pr[VELCMD];
    slVelRef = srg_ulVelRefGen(fcmd.uw.hi);	// 
    if(pr[VELDIR] == 0)
    {
      slVelRef = 0 - slVelRef;
    }
	   
    srg_stVelRefIn.slVelRef = slVelRef;
    srg_stVelRefIn.uwVdcVt = g_uVdcDis;
    srg_voVelRef(&srg_stVelRefIn, &srg_stVelRefCf, &srg_slVelRef);
  
    slr_stLoopTBSIn.slVelRef = srg_slVelRef;//slr_stPreLPFTBSOut.slVelRef;
    slr_stLoopTBSIn.slVelFdb = kw_slFunVelEst;
    slr_stLoopTBSIn.slTorFed = 0;//slr_stTFEDTBSOut.slTorFed;

	  slr_stLoopTBSIn.ulTorMax = kw_slIqMaxCal;
    slr_LoopTBS(&slr_stLoopTBSIn,&slr_stCf,&slr_stLoopTBSPrt,&slr_stLoopTBSOut);
    kw_slTorRef = slr_stLoopTBSOut.slTorquRef;//
    //Torque current calculation.
    ulTorque = ulabs(kw_slTorRef>>12); //

	  if(ulTorque > 32767)
	    ulTorque = 32767;
	  uwTmp = ulTorque;
    if(kw_slTorRef < 0)
    {
      kw_swIqRef = 0 -(s16)uwTmp;
    }
    else
    {
      kw_swIqRef = (s16)uwTmp;
    }
	  
    uwTmp = (u32)vol_stVdcTBCOut.uwVdc * 37838 >> 15;   //37838/32768 = 32768/28377
    uwUlimTemp = (u32)uwTmp * 217 >> 8;//85%
    flw_swUerr = flw_uwUout - (uwUlimTemp);//
    		
    if (flw_swUerr > FLW_UDCLIMIT_CONST_PU)
    {
      flw_swUerr = FLW_UDCLIMIT_CONST_PU;
    }
    else if (flw_swUerr <= -FLW_UDCLIMIT_CONST_PU)
    {
      flw_swUerr = -FLW_UDCLIMIT_CONST_PU;
    }
    flw_swUerrDiff=flw_swUerr-flw_swUerrLast;
    if (flw_swUerrDiff > FLW_UDCLIMIT_CONST_PU)
    {
      flw_swUerrDiff = FLW_UDCLIMIT_CONST_PU;
    }
    else if (flw_swUerrDiff <= -FLW_UDCLIMIT_CONST_PU)
    {
      flw_swUerrDiff = -FLW_UDCLIMIT_CONST_PU;
    }
    flw_swUerrLast=flw_swUerr;
    flw_slIdCmdInte = flw_slIdCmdInte + (s32)flw_swUerr * pr[FLW_ID_INTE] ;
    if(flw_slIdCmdInte>0)
      flw_slIdCmdInte = flw_slIdCmdInte + (s32)flw_swUerrDiff*pr[FLW_ID_GAIN];
    if(flw_slIdCmdInte>0)//Flux weaken
    {
      slTmp =  (flw_slIdCmdInte >> 10);
      if (slTmp > pr[ID_REF])
      {
        slTmp = pr[ID_REF];
				flw_slIdCmdInte = slTmp << 10;
      }
      swIdCmd = slTmp;
    }
    else
    {

    	swIdCmd = 0;
      flw_slIdCmdInte=0;
    }

    if(kw_swIdRef > 0)
			kw_swIdRef = kw_swIdRef - 50;
		else
			uwIdDecCom = 1;
    if(uwIdDecCom == 1)
      kw_swIdRef = -swIdCmd;
    
  }
  else             
  {
		if(uwIfSign==1)//IF
		{	        
			fcmd.uw.hi = pr[VELCMD]>>2;//uwKeySpeedRef;
	    if(fcmd.uw.hi < (pr[IFFOCSPD]>>1))
			  fcmd.uw.hi = pr[IFFOCSPD]>>1; 
		  if(fcmd.uw.hi > pr[VELCMD])
		    fcmd.uw.hi = pr[VELCMD];
			slVelRef = srg_ulVelRefGen(fcmd.uw.hi);	//
		
			if(pr[VELDIR] == 0)
			{
				slVelRef = 0 - slVelRef;
			}
				
			srg_stVelRefIn.slVelRef = slVelRef;
			srg_stVelRefIn.uwVdcVt = g_uVdcDis;
			srg_voVelRef(&srg_stVelRefIn, &srg_stVelRefCf, &srg_slVelRef);
		}	   
		else//Cur Start Up
		{
			srg_slVelRef=0;
		}
  }  
	swTemp = clr_stIdLoopTBCOut.swUdRef;
  ulTemp = (s32)swTemp * swTemp;
  swTemp = clr_stIqLoopTBCOut.swUqRef;
  ulTemp = ulTemp + (s32)swTemp * swTemp;
  flw_uwUout = mth_uwSqrt(ulTemp);
	
	if((ulabs(srg_slVelRef) > slSTALLSpdTh) && (uwVelAcc != 1))
	{
		ulTemp = ulSTACOF * flw_uwUout;
		uwTemp = ulabs(srg_slVelRef) >> 16;
		ulSpdVdccf = (u32)uwTemp * g_uVdcDis * 22;
		if(ulSpdVdccf > ulTemp)
		{	
      if(uwSTAVOLCnt<10000)
			  uwSTAVOLCnt++;
		}	
    else
    {			
			if(uwSTAVOLCnt>0)
				uwSTAVOLCnt--;
		}	
		if(uwSTAVOLCnt > 5000)
		{	
			if(g_uErrorCode == C_NO_ERR)
	    {    
				g_uErrorCode = C_STALL_GZ;
		  }
		}
	}	
	
	swSpdTmp = kw_slFunVelEst>>16;
	swTmp = (s32)swSpdTmp * cdt_stPark.swq >> 15;
	if(swTmp > 0x3FFF)
		swTmp = 0x3FFF;
	
	if(swTmp < -0x3FFF)
		swTmp = -0x3FFF;
	swPowerTmp = ((s32)swTmp * kw_uwPowerCalFlux) >> 10;
	swPowerActive = uwabs(swPowerTmp);
  
	sal_swLpfIn = swPowerActive;
  sal_LPF (&sal_swLpfIn, &sal_uwLpfCfPowerCal, &sal_stLpfOutPowerCal);//Q15
}

void ctm_IFFocTBC(void)
{  
  s16 swTemp,swTmp;

	if(IFtoFunSign == 1)//FOC Sensorless
	{	 
    cdt_stParkIn.swAlfa = swMotIAlfa;//
    cdt_stParkIn.swBeta = swMotIBeta;
    cdt_stParkIn.uldegree = kw_ulFunDegreeEst;
    cdt_Park(&cdt_stParkIn, &cdt_stPark);//for ACR

    clr_stIdLoopTBCIn.swIdRef = kw_swIdRef;//clr_stIdLoopTBCIn.swIdRef = kw_swIqRef*K;
    clr_stIdLoopTBCIn.swIdFdb = cdt_stPark.swd;
    clr_IdLoopTBC(&clr_stIdLoopTBCIn, &clr_stIdLoopTBCCf, &clr_stIdLoopTBCPrt, &clr_stIdLoopTBCOut);

    clr_stIqLoopTBCIn.swIqRef = kw_swIqRef;
    clr_stIqLoopTBCIn.swIqFdb = cdt_stPark.swq;
    clr_IqLoopTBC(&clr_stIqLoopTBCIn, &clr_stIqLoopTBCCf, &clr_stIqLoopTBCPrt, &clr_stIqLoopTBCOut);
 
    cdt_stInvParkIn.swd = clr_stIdLoopTBCOut.swUdRef;
    cdt_stInvParkIn.swq = clr_stIqLoopTBCOut.swUqRef;
    cdt_stInvParkIn.uldegree = kw_ulFunDegreeEst;
    cdt_InvPark(&cdt_stInvParkIn, &cdt_stInvPark);   
    pwm_swUAlfaRef = cdt_stInvPark.swAlfa;
    pwm_swUBetaRef = cdt_stInvPark.swBeta;

    pwm_PWMInvTBC();    
  }
	else                            //IF
  {	
		ccs_stInteTBCIn.swDegreeComp = 0;
    ccs_stInteTBCIn.slFs = srg_slVelRef;
    ccs_voInteTBC (&ccs_stInteTBCIn, &ccs_stModCf, &ccs_stInteTBCOut);

    cdt_stParkIn.swAlfa = swMotIAlfa;
    cdt_stParkIn.swBeta = swMotIBeta;
    cdt_stParkIn.uldegree = ccs_stInteTBCOut.FlxDegree.sl.hi;
    cdt_Park(&cdt_stParkIn, &cdt_stPark);

    clr_stIdLoopTBCIn.swIdRef = 0;
    clr_stIdLoopTBCIn.swIdFdb =  cdt_stPark.swd;
    clr_IdLoopTBC (&clr_stIdLoopTBCIn, &clr_stIdLoopTBCCf, &clr_stIdLoopTBCPrt, &clr_stIdLoopTBCOut);

    if(uwIfSign==0)	 
    {
      clr_stIqLoopTBCIn.swIqRef = clr_stIqLoopTBCIn.swIqRef+100;
      if(clr_stIqLoopTBCIn.swIqRef > pr[IsREF])
      {
        clr_stIqLoopTBCIn.swIqRef = pr[IsREF];
        uwIfSign=1;
      }
    }
    else
    {      
	    if(ulabs(srg_slVelRef) > slIFCurAutoSpd)//2Hz
      {
        if(pr[VELDIR]==1)
          swIfDegreeErr = swSALFUNDegreeErr;
        if(pr[VELDIR]==0)
        {
          swIfDegreeErr = 16384 - swSALFUNDegreeErr;
          if(swIfDegreeErr>16384)
        	  swIfDegreeErr= swIfDegreeErr - 32768;
          if(swIfDegreeErr<-16384)
            swIfDegreeErr= swIfDegreeErr + 32768;
        }
        if (swIfDegreeErr > 1365)//15
        {
          swIfDegreeErr = 1365;
        }
        else if (swIfDegreeErr <= -1365)
        {
          swIfDegreeErr = -1365;
        }
        swIfDegreeErrDiff=swIfDegreeErr-swIfDegreeErrLast;
        if (swIfDegreeErrDiff > 1365)
        {
          swIfDegreeErrDiff = 1365;
        }
        else if (swIfDegreeErrDiff <= -1365)
        {
          swIfDegreeErrDiff = -1365;
        }
        swIfDegreeErrLast=swIfDegreeErr;
        slIfIqCmdInte = slIfIqCmdInte + (s32)swIfDegreeErr * pr[IF_IQ_INTE] ;
        slIfIqCmdInte = slIfIqCmdInte + (s32)swIfDegreeErrDiff*pr[IF_IQ_GAIN];

        clr_stIqLoopTBCIn.swIqRef = ((s16)pr[IsREF]) - (slIfIqCmdInte>>16);
        
				swTemp = (pr[IsREF]*3)>>1;//150%
        if(clr_stIqLoopTBCIn.swIqRef>swTemp)
        {
          clr_stIqLoopTBCIn.swIqRef=swTemp;
          slIfIqCmdInte = ((s16)pr[IsREF]) - swTemp;
          slIfIqCmdInte = slIfIqCmdInte<<16;
        }
        
        swTemp = (pr[IsREF]*1)>>1;//50%
        if(clr_stIqLoopTBCIn.swIqRef<swTemp)
        {
          clr_stIqLoopTBCIn.swIqRef=swTemp;
          slIfIqCmdInte = ((s16)pr[IsREF]) - swTemp;
          slIfIqCmdInte = slIfIqCmdInte<<16;
        }
      
      }
    }
	
    clr_stIqLoopTBCIn.swIqFdb =  cdt_stPark.swq;
    clr_IqLoopTBC (&clr_stIqLoopTBCIn, &clr_stIqLoopTBCCf, &clr_stIqLoopTBCPrt, &clr_stIqLoopTBCOut);

    cdt_stInvParkIn.uldegree = ccs_stInteTBCOut.FlxDegree.sl.hi;
    cdt_stInvParkIn.swd = clr_stIdLoopTBCOut.swUdRef;
    cdt_stInvParkIn.swq = clr_stIqLoopTBCOut.swUqRef;
    cdt_InvPark(&cdt_stInvParkIn, &cdt_stInvPark);

    pwm_swUAlfaRef = cdt_stInvPark.swAlfa;
    pwm_swUBetaRef = cdt_stInvPark.swBeta;
	
    pwm_PWMInvTBC();

    swTmp = -((s32)cdt_stInvParkIn.uldegree >> 16) + ((s32)kw_ulFunDegreeEst >> 16);
    if((swTmp - 16384) > 0)
    {
      swSALFUNDegreeErr = swTmp - 0x7FFF;
    }
    else if((swTmp +16384) < 0)
    {
      swSALFUNDegreeErr = swTmp + 0x7FFF;
    }
    else
    {
      swSALFUNDegreeErr = swTmp;
    }
    if(((ulabs(srg_slVelRef)) > slIFFocSpd) && ((ulabs(pll_stActFlxTBCOut.slFreq)) > slSTALLSpdFun))// IF -->> FOC
    {
			if(uwIFtoFunCnt < 10000)
			  uwIFtoFunCnt ++;
		}
    else
    {
			if(uwIFtoFunCnt > 0)
				uwIFtoFunCnt --;
    }	
		
		if(ulabs(srg_slVelRef) > slSTALLSpdTh)
		{	
			if(uwIFtoFunCnt < 5)
			{	
				if(g_uErrorCode == C_NO_ERR)
	      {    
				  g_uErrorCode = C_STALL_GZ;
		    }
			}
    }
		
    if(uwIFtoFunCnt	> 10)
		{	
      IFtoFunSign=1;
      
      cdt_stParkIn.swAlfa = 0;
      cdt_stParkIn.swBeta = clr_stIqLoopTBCIn.swIqRef;
      cdt_stParkIn.uldegree = (s32)swSALFUNDegreeErr<<16;
      cdt_Park(&cdt_stParkIn, &cdt_stPark);
      kw_swIdRef = cdt_stPark.swd;
			kw_swIqRef = cdt_stPark.swq;
			slr_stLoopTBSOut.slTorquRef = (s32)kw_swIqRef << 12;

      cdt_stParkIn.swAlfa = clr_stIdLoopTBCPrt.slUdRefOld>>15;
      cdt_stParkIn.swBeta = clr_stIqLoopTBCPrt.slUqRefOld>>15;
      cdt_stParkIn.uldegree = (s32)swSALFUNDegreeErr<<16;
      cdt_Park(&cdt_stParkIn, &cdt_stPark);
      clr_stIdLoopTBCPrt.slUdRefOld = (s32)cdt_stPark.swd<<15;
      clr_stIqLoopTBCPrt.slUqRefOld = (s32)cdt_stPark.swq<<15;
    }	
  }
  
	if((ulabs(srg_slVelRef)) > (slIFCurAutoSpd>>1))//(slIFFocSpd>>1))
	{	
	  kw_uwStaticLq = cf_uwPMLq;
    fun_stMLpfTBCIn.swUAlfa = pwm_swUAlfaRef;
    fun_stMLpfTBCIn.swUBeta = pwm_swUBetaRef;
    fun_stMLpfTBCIn.swMotIAlfa = swMotIAlfa;
    fun_stMLpfTBCIn.swMotIBeta = swMotIBeta; 
    fun_stMLpfTBCIn.swFc = pll_stActFlxTBCOut.slFreq>>16;//swSpeedFdbLPF;//	  
    fun_MLpfTBC (&fun_stMLpfTBCIn, &fun_stMLpfTBCOut);
  
    kw_slFunActFlxAlfa = (fun_stMLpfTBCOut.slStaFlxAlfa >> 16) - ((s32)swMotIAlfa * kw_uwStaticLq >> 14); //Q10
    kw_slFunActFlxBeta = (fun_stMLpfTBCOut.slStaFlxBeta >> 16) - ((s32)swMotIBeta * kw_uwStaticLq >> 14); //Q10
	  
    pll_stPllTBCIn.slPLLINAlfa = kw_slFunActFlxAlfa;
    pll_stPllTBCIn.slPLLINBeta = kw_slFunActFlxBeta;
    pll_PllTBC (&pll_stPllTBCIn, &pll_stPllCf, &pll_stActFlxTBCPrt, &pll_stActFlxTBCOut);
  }
	else
	{	
		pll_stActFlxTBCOut.ulDegree = ccs_stInteTBCOut.FlxDegree.sl.hi;
	  pll_stActFlxTBCOut.slFreq = srg_slVelRef;
		pll_stActFlxTBCPrt.sqIntiVel = srg_slVelRef;
	}
	slThetaComTBC = (s32)sal_stLpfOutSpdFun.swLpfOut * cf_uwThetaTLast;
	kw_ulFunDegreeEst = pll_stActFlxTBCOut.ulDegree;//
	kw_ulFunDegreeEst = kw_ulFunDegreeEst&0x7FFFFFFF;     
	kw_slFunVelEst = (s32)sal_stLpfOutSpdFun.swLpfOut<<16;//	
}



void ctm_TBC(void)
{

  //软件过流，电流基值75%
  if((UCur>31129) || (VCur>31129) || (WCur>31129) || (UCur<-31129) || (VCur<-31129) || (WCur<-31129))
  {
	  if(RunCmd)
		{
			if(swCurOCCnt < 5) swCurOCCnt++;
			else
			{	
        g_uErrorCode = C_SOFT_GUOLIU;
			}	
		}	
  }
	else  swCurOCCnt = 0;

  
  cdt_stClarkeIn.swa = UCur;
  cdt_stClarkeIn.swb = VCur;
  cdt_stClarkeIn.swc = WCur;
  cdt_Clarke(&cdt_stClarkeIn, &cdt_stClarke);

  swMotIAlfa = cdt_stClarke.swAlfa;//
  swMotIBeta = cdt_stClarke.swBeta;//

  ctm_TBCIsr(); 
}




void com_DevResv(void)
{
  u16 uwIqFdb,uwSpdFdb;

	
////电阻2.2欧，电感8.46~8.91  380V          //220406
//     pr[MAG_VOLT] = 500;   
//     pr[RsPM] = 1200;    
////     pr[PMLD] = 470;                 //2196RPM 395V 3.9A  1350W
////     pr[PMLQ] = 530;                 //2195RPM 390V 3.8A  1345W
////     pr[PMLD] = 420;                 //2196RPM 392V 4.2A  1360W
////     pr[PMLQ] = 450; 
////     pr[PMLD] = 520;                 //2195RPM 392V 3.7A  1340W
////     pr[PMLQ] = 580;               //2195RPM 388V 3.7A  1340W
////     pr[PMLD] = 570;                   //2195RPM 389V 3.5A  1340W
////     pr[PMLQ] = 650;            //2195RPM 389V 3.5A  1340W  
//     pr[PMLD] = 590;                   
//     pr[PMLQ] = 650;            //2195RPM 395V 3.5A  1321W  
//     pr[I_RATE] = 50;
//     pr[RATED_POW]= 15000; 
//     pr[CLR_D_KP] = 100;//120;
//     pr[CLR_D_KI] = 100;//60; 	
//     kw_uwPowerCalFlux = 405;          //390对应输入功率1850w //AT32F421 370---输入1814W   440---输入1484W  420---1509W  //第一档是36.1，3.7A 第二档是37.6，4.0A第三档是38.9，4.3A
//     pr[IsREF] = 10500;
//     pr[ACCTIME] = 200;
//     pr[DECTIME] = 200; 
//     pr[IFFOCSPD] = 600;          
//     
////电阻0.6欧，电感2.92~3.01  220V
///*     pr[MAG_VOLT] = 320;   
//     pr[RsPM] = 330;
//     pr[PMLD] = 170;
//     pr[PMLQ] = 180; 
//     pr[I_RATE] = 75;
//     pr[RATED_POW]= 15000; 
//     pr[CLR_D_KP] = 150;//100;//120;
//     pr[CLR_D_KI] = 150;//100;//60; 
//     kw_uwPowerCalFlux = 220;   
//     pr[IsREF] = 12000;
//     pr[ACCTIME] = 200;
//     pr[DECTIME] = 200;
//     pr[IFFOCSPD] =600; */    
//                         
//     pr[VELCMD] = 7334;
//     pr[PM_POLES] = 4;
//     pr[J_MOTOR] = 400;
//     pr[RATED_RPM] = 3000;
//     pr[ID_REF] =0;// 2300;
//     pr[SLR_KP] = 100;
//     pr[SLR_KI] = 100;      
//     uwMeLowSpdHz = 70;
//     uwSpdMode1 = 7334;
//     uwSpdMode2 = 6667;
//     uwSpdMode3 = 3334;
//    uwSpdMode4 = 1667;
//     uwSpdRefRadio = pr[PM_POLES] * 250;//2000;
//     uwSpdDisRadio = 2;
//     swIqLowLim = 0;
//    uwSpdPhase = 5000;
//    RunDelayLim = 600;//3S
//    mpdulPMSALLimCof =2000;//0.5S	
	

	if(RunCmd==0)
	{
//电阻2.2欧，电感8.46~8.91  380V          //220406
     pr[MAG_VOLT] = 500;   
		 pr[MAG_TIME] = PWM_FREQ/2000;  //500uS
     pr[RsPM] = 1200;    
//     pr[PMLD] = 590;                   
//     pr[PMLQ] = 650;            //2195RPM 395V 3.5A  1321W  
		 pr[PMLD] =  430; //8.60mH
		 pr[PMLQ] =  460; //9.20mH
     pr[I_RATE] = 50;                   
     pr[VELCMD] = 3000;  //25000rpm
     pr[PM_POLES] = 4;
 
		uwSpdMechHz = (u32)pr[VELCMD]*2/pr[PM_POLES];
		uwSpdMechCf = 500*1024/uwSpdMechHz;
		
		Speed_vInit();
	}
	uwSpdRefRadio = pr[PM_POLES] * 250;


  if(RunCmd)
  {
		if(IFtoFunSign == 1)
		{	
			uwSpdFdb = (u32)sal_stLpfOutSpdDis.swLpfOut * 1039 >> 10;//102%
      uwDisSpdFdb = ((u32)uwSpdFdb * pr[FB])>>15;
		}	
    else   //1400W   5000--
		{	
      uwDisSpdFdb = (ulabs(srg_slVelRef>>16))*pr[FB]>>15;
		}
		uwIqFdb = (s32)sal_stLpfOutIqFdb.swLpfOut * 725 >> 10;//725=1024/1.414
		uwDisIRef = (u32)uwIqFdb * cf_uwIBAp >> 15;
		
		uwMotPowWattCal = (s32)sal_stLpfOutPowerCal.swLpfOut * cf_uwPbWa >> 10;//Watt
		uwMotPowWattDis = (s32)sal_stLpfOutPowerDis.swLpfOut * cf_uwPbWa >> 10;//Watt
		if(TEMP_uACVol != 0)
		  adAverageCurrent = (u32)uwMotPowWattDis * 12 / TEMP_uACVol;//12=100/59*10/1.414
		else 
			adAverageCurrent = 0;
  }
  else
  {	
    uwDisSpdFdb = 0;
    uwDisIRef = 0;
		adAverageCurrent = 0;
		uwMotPowWattCal = 0;
		uwMotPowWattDis = 0;
  }  
	
  if(PowerFst == 1)
  { 
    swIFFocSpdPu = (s32)pr[IFFOCSPD] * 32768  / pr[FB];
		cf_uwUBVt = 10000;
    cf_uwIBAp = 165; 	 //1.65V/10(放大倍数)/0.01Ω = 165
    cf_uwPolePairs = (pr[PM_POLES]) >> 1; 
    cf_uwRbOm = (u32)cf_uwUBVt*100/cf_uwIBAp;	    				
    cf_uwLbHu = (u32)cf_uwRbOm*15915/pr[FB];	   				
    cf_uwWebRad = (u32)pr[FB]*1287>>8; 
    cf_uwtbSec = 0x20000000/cf_uwWebRad; 
    cf_uwPbWa = (UQWORD)cf_uwUBVt*cf_uwIBAp*251658>>29; //2^5Watt
    cf_ulTbNm = ((UQWORD)cf_uwPbWa*cf_uwtbSec*cf_uwPolePairs)>>2;    				
    cf_ulJbKgm = (UQWORD)cf_ulTbNm*cf_uwtbSec*cf_uwtbSec*cf_uwPolePairs>>32; // Base Inertia, 2^(-39) kg.m^2   				
    cf_uwTBCLast = (u32)pr[FB]*20589/GLB_TBC_FREQ_CST_HZ;	    				
    cf_uwThetaTLast = (u32)cf_uwTBCLast * 1024 / 3217;	    
    cf_uwTBSLast = ((u32)cf_uwTBCLast * GLB_TBS_CST_CNT)>>5;  
    cf_uwPMRs = (u32)pr[RsPM]*3277/cf_uwRbOm;
    cf_uwPMLd = (u32)pr[PMLD]*5120/cf_uwLbHu;
    cf_uwPMLq = (u32)pr[PMLQ]*5120/cf_uwLbHu;

		
		vol_VdcCf (&vol_stVdcCf);
    vlr_Cf(&vlr_stCf);	

    sal_stLpfCfIn.uwFc = 50*10; 
    sal_stLpfCfIn.uwTctrl = cf_uwTBSLast;
    sal_LPFCf (&sal_stLpfCfIn, &sal_uwLpfCfPowerCal);
		
		sal_stLpfCfIn.uwFc = 50*10; 
    sal_stLpfCfIn.uwTctrl = cf_uwTBSLast;
    sal_LPFCf (&sal_stLpfCfIn, &sal_uwLpfCfPowerDis);
		
	  sal_stLpfCfIn.uwFc = 50*10; 
	  sal_stLpfCfIn.uwTctrl = cf_uwTBSLast;
    sal_LPFCf (&sal_stLpfCfIn, &sal_uwLpfCfSpdFun);
	  sal_stLpfCfIn.uwFc = 5*10; 
	  sal_stLpfCfIn.uwTctrl = cf_uwTBSLast;
    sal_LPFCf (&sal_stLpfCfIn, &sal_uwLpfCfIqFdb);
	  sal_stLpfCfIn.uwFc = 25*10; 
	  sal_stLpfCfIn.uwTctrl = cf_uwTBSLast;
    sal_LPFCf (&sal_stLpfCfIn, &sal_uwLpfCfVdc);
	  sal_stLpfCfIn.uwFc = 15*10; 
	  sal_stLpfCfIn.uwTctrl = cf_uwTBSLast;
    sal_LPFCf (&sal_stLpfCfIn, &sal_uwLpfCfSpdDis);
  }
	
	pr[EEROM_EN] = 1;
	
}


