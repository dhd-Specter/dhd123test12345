

#include "at32f4xx.h"
#include "application.h"
#include "ctrlmode.h"
#include "function.h"	


void TIM1_vInit(void)
{

  TMR_TimerBaseInitType  TIM1_TimeBaseStructure;
  TMR_OCInitType         TIM1_OCInitStructure;
  TMR_BRKDTInitType      TIM1_BDTRInitStructure;
  GPIO_InitType          GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOB, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_TMR1, ENABLE);	
	
 
  TMR_TimeBaseStructInit(&TIM1_TimeBaseStructure);
  TIM1_TimeBaseStructure.TMR_DIV = PWM_PRSC;
  TIM1_TimeBaseStructure.TMR_CounterMode = TMR_CounterDIR_CenterAligned1;
  TIM1_TimeBaseStructure.TMR_Period = PWM_PERIOD;
  TIM1_TimeBaseStructure.TMR_ClockDivision = TMR_CKD_DIV2; 					          //死区和数字滤波用
  TIM1_TimeBaseStructure.TMR_RepetitionCounter = 0;
  TMR_TimeBaseInit(TMR1, &TIM1_TimeBaseStructure);

  TMR_OCStructInit(&TIM1_OCInitStructure);
  TIM1_OCInitStructure.TMR_OCMode = TMR_OCMode_PWM1; 
  TIM1_OCInitStructure.TMR_OutputState = TMR_OutputState_Enable; 
  TIM1_OCInitStructure.TMR_OutputNState = TMR_OutputNState_Enable;                  
  TIM1_OCInitStructure.TMR_Pulse = 0x00;                                      //dummy value
  TIM1_OCInitStructure.TMR_OCPolarity = TMR_OCPolarity_Low;                    //上桥低电平有效
  TIM1_OCInitStructure.TMR_OCNPolarity = TMR_OCNPolarity_Low;         				//下桥低电平有效
  TIM1_OCInitStructure.TMR_OCIdleState = TMR_OCIdleState_Set;
  TIM1_OCInitStructure.TMR_OCNIdleState = TMR_OCNIdleState_Set;          
  
  TMR_OC1Init(TMR1, &TIM1_OCInitStructure); 
  TMR_OC2Init(TMR1, &TIM1_OCInitStructure);
  TMR_OC3Init(TMR1, &TIM1_OCInitStructure);
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pins = GPIO_Pins_8 | GPIO_Pins_9 | GPIO_Pins_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
  GPIO_InitStructure.GPIO_Pull = GPIO_Pull_PU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinsSource8,  GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinsSource9,  GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinsSource10, GPIO_AF_2);
	
  GPIO_InitStructure.GPIO_Pins = GPIO_Pins_13 | GPIO_Pins_14 | GPIO_Pins_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
  GPIO_InitStructure.GPIO_Pull = GPIO_Pull_PU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinsSource13,  GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinsSource14,  GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinsSource15,  GPIO_AF_2);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pins = GPIO_Pins_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinsSource12, GPIO_AF_2);  
  
  TMR_OCStructInit(&TIM1_OCInitStructure);
  TIM1_OCInitStructure.TMR_OCMode = TMR_OCMode_PWM2;  
  TIM1_OCInitStructure.TMR_OutputState = TMR_OutputState_Enable; 
  TIM1_OCInitStructure.TMR_OutputNState = TMR_OutputNState_Enable;                  
  TIM1_OCInitStructure.TMR_Pulse = PWM_PERIOD - 1; 
  TIM1_OCInitStructure.TMR_OCPolarity = TMR_OCPolarity_High;                   	   
  TIM1_OCInitStructure.TMR_OCNPolarity = TMR_OCNPolarity_High;         				
  TIM1_OCInitStructure.TMR_OCIdleState = TMR_OCIdleState_Reset;
  TIM1_OCInitStructure.TMR_OCNIdleState = TMR_OCNIdleState_Reset;            
  TMR_OC4Init(TMR1, &TIM1_OCInitStructure);
  
  TMR_OC1PreloadConfig(TMR1, TMR_OCPreload_Enable);
  TMR_OC2PreloadConfig(TMR1, TMR_OCPreload_Enable);
  TMR_OC3PreloadConfig(TMR1, TMR_OCPreload_Enable);
  TMR_OC4PreloadConfig(TMR1, TMR_OCPreload_Enable);

  TIM1_BDTRInitStructure.TMR_OSIMRState = TMR_OSIMRState_Enable;
  TIM1_BDTRInitStructure.TMR_OSIMIState = TMR_OSIMIState_Enable;
  TIM1_BDTRInitStructure.TMR_LOCKgrade = TMR_LOCKgrade_1; 
  TIM1_BDTRInitStructure.TMR_DeadTime = DEADTIME;
  TIM1_BDTRInitStructure.TMR_Break = TMR_Break_Enable;
  TIM1_BDTRInitStructure.TMR_BreakPolarity = TMR_BreakPolarity_Low;			  //OC低有效
  TIM1_BDTRInitStructure.TMR_AutomaticOutput = TMR_AutomaticOutput_Enable;
  TMR_BRKDTConfig(TMR1, &TIM1_BDTRInitStructure);

  TMR_SelectOutputTrigger(TMR1, TMR_TRGOSource_OC4Ref);
	
	
  TMR_ClearITPendingBit(TMR1, TMR_INT_Break);
  TMR_INTConfig(TMR1, TMR_INT_Break,ENABLE);

  TMR_ClearFlag(TMR1, TMR_FLAG_Update);
  TMR_INTConfig(TMR1, TMR_FLAG_Update, ENABLE);
	
  TMR_Cmd(TMR1, ENABLE);
}


void TMR1_BRK_UP_TRG_COM_IRQHandler(void)
{
//	static u8  OnOff_bit;

	if(TMR_GetFlagStatus(TMR1,TMR_FLAG_Break) != RESET)
	{
		TMR_ClearFlag(TMR1, TMR_FLAG_Break); 
		if(DelayTime==0)
		{		 
		  if(!uwErrClear)	
      {				
			 g_uErrorCode = C_GUOLIU;
			 RunCmd = 0;
			 PWM_DIS_DUTY(); 
			 TMR_CtrlPWMOutputs(TMR1, ENABLE);
		  }
	   }
	}
	if(TMR_GetFlagStatus(TMR1,TMR_FLAG_Update) != RESET)
	{
		TMR_ClearFlag(TMR1, TMR_FLAG_Update); 
    
    
	}
}



