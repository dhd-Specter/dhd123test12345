

#include "at32f4xx.h"
#include "performance.h"
#include "SEGGER_RTT.h"
#include "application.h"
#include "ctrlmode.h"
#include "spi.h"



void ADC_DMA_vInit(void)
{
  DMA_InitType     DMA_InitStructure; //1234  //test1
	ADC_InitType     ADC_InitStructure;
	GPIO_InitType    GPIO_InitStructure;	//5678
	
  RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1 , ENABLE);		
	RCC_AHBPeriphClockCmd( RCC_AHBPERIPH_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_APB2CLK_Div6);	
	
  //PA0--IDU   PA1--IDV    PA2--IDW    PA3--TD    PA4--VDC    PA5--IDC      
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pins = GPIO_Pins_0|GPIO_Pins_1|GPIO_Pins_2|GPIO_Pins_3|GPIO_Pins_4|GPIO_Pins_5|GPIO_Pins_6|GPIO_Pins_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	
		/* ADC1 configuration ------------------------------------------------------*/
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode              = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanMode          = ENABLE;
	ADC_InitStructure.ADC_ContinuousMode    = DISABLE;
	ADC_InitStructure.ADC_ExternalTrig      = ADC_ExternalTrig_None;
	ADC_InitStructure.ADC_DataAlign         = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NumOfChannel      = 4;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channels configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_28_5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_28_5); 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_28_5); 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_28_5);
	
	ADC_InjectedSequencerLengthConfig(ADC1,4);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7_5);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_7_5);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_7_5);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_7_5);
	
	ADC_ExternalTrigInjectedConvConfig(ADC1,ADC_ExternalTrigInjec_TMR1_TRGO);
	
	ADC_ExternalTrigInjectedConvCtrl(ADC1, ENABLE);

  ADC_AutoInjectedConvCtrl(ADC1, DISABLE); 		

	
	/* Enable ADC1 DMA */
	ADC_DMACtrl(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Ctrl(ADC1, ENABLE);

	/* Enable ADC1 reset calibration register */   
	ADC_RstCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));

	
	ADC_INTConfig(ADC1,ADC_INT_JEC,ENABLE);
	
	
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_Reset(DMA1_Channel1);
	DMA_DefaultInitParaConfig(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)&ADC1->RDOR;
	DMA_InitStructure.DMA_MemoryBaseAddr        = (uint32_t)RegularConvData_Tab;
	DMA_InitStructure.DMA_Direction             = DMA_DIR_PERIPHERALSRC;
	DMA_InitStructure.DMA_BufferSize            = 4;
	DMA_InitStructure.DMA_PeripheralInc         = DMA_PERIPHERALINC_DISABLE;
	DMA_InitStructure.DMA_MemoryInc             = DMA_MEMORYINC_ENABLE;
	DMA_InitStructure.DMA_PeripheralDataWidth   = DMA_PERIPHERALDATAWIDTH_HALFWORD;
	DMA_InitStructure.DMA_MemoryDataWidth       = DMA_MEMORYDATAWIDTH_HALFWORD;
	DMA_InitStructure.DMA_Mode                  = DMA_MODE_CIRCULAR;
	DMA_InitStructure.DMA_Priority              = DMA_PRIORITY_HIGH;
	DMA_InitStructure.DMA_MTOM                  = DMA_MEMTOMEM_DISABLE;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* Enable DMA1 channel1 */
	DMA_ChannelEnable(DMA1_Channel1, ENABLE);
  

  DMA_INTConfig(DMA1_Channel1,DMA_INT_TC,ENABLE);  

}


void ADC_COMP_IRQHandler(void)
{
   if(ADC_GetFlagStatus(ADC1,ADC_FLAG_JEC) != RESET)  //50uS
	 {

		  ADC_ClearFlag(ADC1,ADC_FLAG_JEC);

		   
		  InjectedConvData_Tab[0] =  ADC1->JDOR1;
		  InjectedConvData_Tab[1] =  ADC1->JDOR2;	
		  InjectedConvData_Tab[2] =  ADC1->JDOR3;		 
		  InjectedConvData_Tab[3] =  ADC1->JDOR4;		 

		  h_ADCBusvolt = (InjectedConvData_Tab[3]&0x0FFF);    

						//母线电压计算
			g_lBusVol = h_ADCBusvolt;//Q12  
//			g_lDisBusVol = (((s32)g_lBusVol*124)>>10) + (((s32)g_lDisBusVol*900)>>10);//
      g_lDisBusVol = g_lBusVol;
			g_uVdcDis = ((s32)g_lDisBusVol*8833)>>12;	  //8833=803*3.3/3

			vol_VdcTBC (&vol_stVdcCf, &vol_stVdcTBCOut);
			uwUlimt = (u32)vol_stVdcTBCOut.uwVdc*37838>>15;// 37838 = 32768*2/sqrt(3)

			uwAcrUDlim = (u32)uwUlimt * 29491 >> 15;		   //90%   
			uwAcrUQlim = (u32)uwUlimt * 29491 >> 15;		   //90%


			slVdcPWMCf = (s32)vlr_stCf.uwKUdcCf1 * 1024 / vol_stVdcTBCOut.uwVdcPct;
			
			if(sampleVddDivideTwo_bit == 0)
			{
				SVPWM_3ShuntCurrentReadingCalibration();
			}

			if(sampleVddDivideTwo_bit == 1)
			{
				if(MotorDir)
				{
					UCur = -(((s32)InjectedConvData_Tab[0] << 4) - hPhaseAOffset);
					WCur = -(((s32)InjectedConvData_Tab[1] << 4) - hPhaseBOffset);
					VCur = -(((s32)InjectedConvData_Tab[2] << 4) - hPhaseCOffset);
				}
				else
				{
					UCur = -(((s32)InjectedConvData_Tab[0] << 4) - hPhaseAOffset);
					VCur = -(((s32)InjectedConvData_Tab[1] << 4) - hPhaseBOffset);
					WCur = -(((s32)InjectedConvData_Tab[2] << 4) - hPhaseCOffset);
				}
				
				if(UCur>32767)  UCur = 32767;
				else if(UCur < -32767) UCur = -32767;
				
				if(VCur>32767)  VCur = 32767;
				else if(VCur < -32767) VCur = -32767;

				if(WCur>32767)  WCur = 32767;
				else if(WCur < -32767) WCur = -32767;			
			}

			ctm_TBC();

			uwTBCCnt++;
			if(uwTBCCnt >= GLB_TBS_CST_CNT)
			{
				uwTBCCnt = 0;
			}

			acValBuffer.Sine1 = UCur;//uwKeySpeedRefFil;//UCur;//fun_stMLpfTBCOut.slStaFlxAlfa>>15;
			acValBuffer.Sine2 = VCur;//clr_stIdLoopTBCIn.swIdFdb;//VCur;//TIM1->CCR3;//cdt_stInvParkIn.swq;//srg_slVelRef>>15;//uwKeySpeedRef>>0;
			acValBuffer.Sine3 = WCur;//kw_ulFunDegreeEst>>15;//g_uVdcDis;//clr_stIqLoopTBCIn.swIqFdb;//WCur;//TIM1->CCR2;//cdt_stInvParkIn.swd>>0;//srg_stVelRefIn.slVelRef>>15;//tempTest;//kw_ulFunDegreeEst>>15;
			acValBuffer.Sine4 = g_uVdcDis;//cdt_stInvParkIn.uldegree>>15;//h_ADCBusvolt;//TIM1->CCR1;//cdt_stInvParkIn.uldegree>>15;;//fcmd.uw.hi>>0;//pll_stActFlxTBCOut.slFreq>>15;
			if(sampleDelayCnt==0)
			{
					SEGGER_RTT_Write(JS_RTT_Channel, &acValBuffer, sizeof(acValBuffer));
			}
			else
			{
					sampleDelayCnt--;
			}

     
//			GPIO_ResetBits(GPIOB,GPIO_Pins_6);
	 }
}





void DMA1_Channel1_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC1) != RESET)
	{  		
		DMA_ClearFlag(DMA1_FLAG_TC1);
		
		ADC_SoftwareStartConvCtrl(ADC1, DISABLE); //在SYSTICK中 开启转换

		g_lTemper = (RegularConvData_Tab[0]&0x0FFF)>>2;
		
		g_lBusCur= RegularConvData_Tab[1]&0x0FFF;
		
		AVI1 = (RegularConvData_Tab[2]&0x0FFF)>>2;
		
		AVI2 = (RegularConvData_Tab[3]&0x0FFF)>>2;
	}
}


         











