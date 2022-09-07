

#include "ctrlmode.h"
#include "function.h"	
#include "SciToOprt.h"
#include "application.h"
#include "ntc.h"
#include "spi.h"
#include "core_cm4.h"



void SysTick_vInit(void)
{

  SysTick_Config(CKTIM/GLB_TBS_FREQ_CST_HZ);  //1mS

}


extern void USART1_Recv_Monitor(void);
void SysTick_Handler(void)//1ms  
{
	
	u16 uwTTmp,VolCom;
	s32 slTemp;

		//GPIO_SetBits(GPIOC, GPIO_Pin_13); 
	if(DelayTime!=0) DelayTime--;			//ÉÏµçÑÓÊ±
	
	
  ctm_TBSIsr();
	
	wendu_adc_value_save =wendu_adc_value_save*7+ g_lTemper*8;
	wendu_adc_value_save=(wendu_adc_value_save>>3);
	wendu_adc_value=(wendu_adc_value_save>>3);
	uwTemper = wendu_dianzu_count_wendu(wendu_advalue_count_dianzu(wendu_adc_value));
	
	if(RunCmd)
	{	
	  slTemp = ((s32)uwAcrUQlim * uwAcrUQlim) - ((s32)clr_stIdLoopTBCOut.swUdRef * clr_stIdLoopTBCOut.swUdRef);
    if (slTemp < 0)
    {
      slTemp = 0;
    }
    slTemp = mth_uwSqrt(slTemp);
    ulAcrUQlim = slTemp<<15;
  }
	else
	{	
		ulAcrUQlim = (u32)uwAcrUQlim<<15;
	}	
	
	if(RunCmd)
	  sal_swLpfIn = uwabs(clr_stIqLoopTBCIn.swIqFdb);
	else
		sal_swLpfIn = 0;
	
  sal_LPF (&sal_swLpfIn, &sal_uwLpfCfIqFdb, &sal_stLpfOutIqFdb);


	
	if(RunCmd)  VolCom = 1126;
	 else VolCom = 1024;   //100%

	uwVolBusCom = ((u32)g_uVdcDis * VolCom >> 10) + 71;//AC 5V
	uwTTmp = (u32)uwVolBusCom*123>>10;//123=1024*100/59/1.414/10	
	sal_swLpfIn = uwabs(uwTTmp);
  sal_LPF (&sal_swLpfIn, &sal_uwLpfCfVdc, &sal_stLpfOutVdc);
	TEMP_uACVol = sal_stLpfOutVdc.swLpfOut;
	
	if((RunCmd == 1) && (uwRunSpdDec == 1) && ((ulabs(srg_slVelRef)) < slIFFocSpd))
	{	
		RunCmd = 0;	
  }	
	
	if(RunCmd)
	{	
	  sal_swLpfIn = pll_stActFlxTBCOut.slFreq>>16;
    sal_LPF (&sal_swLpfIn, &sal_uwLpfCfSpdFun, &sal_stLpfOutSpdFun);
	}
	if(RunCmd)
    sal_swLpfIn = ulabs(kw_slFunVelEst>>16);
	else
	  sal_swLpfIn = 0;	
	
  sal_LPF (&sal_swLpfIn, &sal_uwLpfCfSpdDis, &sal_stLpfOutSpdDis);

	
	if(RunCmd)
	  sal_swLpfIn = sal_stLpfOutPowerCal.swLpfOut;
	else
	  sal_swLpfIn = 0;
	
  sal_LPF (&sal_swLpfIn, &sal_uwLpfCfPowerDis, &sal_stLpfOutPowerDis);//Q15
	
	Ext_Led();
	
	if(DelayTime==0)
	{   	  
    Fault_Management();
		
//		Motor_Ctr();
	}
	
  ADC_SoftwareStartConvCtrl(ADC1, ENABLE);	
}




