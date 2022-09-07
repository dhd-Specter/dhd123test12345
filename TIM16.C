
#include "at32f4xx.h"
#include "application.h"
#include "ctrlmode.h"
#include "function.h"	

void TIM16_vInit(void)
{

  TMR_TimerBaseInitType  TIM1_TimeBaseStructure;
	TMR_ICInitType         TIM_ICInitStructure; 
  GPIO_InitType            GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOB, ENABLE);	  
  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_TMR16, ENABLE);	
	
	
	GPIO_InitStructure.GPIO_Pins = GPIO_Pins_8;				 //PA1 输入 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 		//输入 
  GPIO_InitStructure.GPIO_Pull = GPIO_Pull_PU ;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
 	GPIO_PinAFConfig(GPIOB, GPIO_PinsSource8,  GPIO_AF_2);
	
 	TIM1_TimeBaseStructure.TMR_Period = 20000; //设定计数器自动重装值 最大10ms溢出  
	TIM1_TimeBaseStructure.TMR_DIV =(60-1); 	//预分频器,2M的计数频率,0.5us加1.	   
	TIM1_TimeBaseStructure.TMR_ClockDivision = TMR_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM1_TimeBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;  //TIM向上计数模式

	TMR_TimeBaseInit(TMR16, &TIM1_TimeBaseStructure); //根据指定的参数初始化TIMx

  TIM_ICInitStructure.TMR_Channel = TMR_Channel_1;  // 选择输入端 IC1映射到TI16上
  TIM_ICInitStructure.TMR_ICPolarity = TMR_ICPolarity_Falling;	//下降沿捕获
  TIM_ICInitStructure.TMR_ICSelection = TMR_ICSelection_DirectTI;
  TIM_ICInitStructure.TMR_ICDIV = TMR_ICDIV_DIV1;	 //配置输入分频,不分频 
  TIM_ICInitStructure.TMR_ICFilter = 0;//0x03;//IC4F=0011 配置输入滤波器 8个定时器时钟周期滤波
  TMR_ICInit(TMR16, &TIM_ICInitStructure);//初始化定时器输入捕获通道

  TMR_Cmd(TMR16,ENABLE ); 	//使能定时器16
 
 						
 	TMR_INTConfig( TMR16,TMR_FLAG_Update|TMR_FLAG_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断
	

}

unsigned int HiTime = 0;
unsigned int LoTime = 0;
unsigned short PWM_Duty_Percent = 0;
unsigned long PWM_Duty_PercentTemp = 0;
unsigned int PWM_HZ = 0;
unsigned int PWM_OverTime = 500;
void  TMR16_GLOBAL_IRQHandler(void)
{
//	unsigned long Temp;
 if(TMR_GetFlagStatus(TMR16,TMR_FLAG_Update) != RESET)
 {

	 if(PWM_OverTime < 1000) PWM_OverTime += 10;
	  else
		{
			 if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pins_8))  
			 {
				 if(PWM_Duty_PercentTemp) PWM_Duty_Percent = 950; //如果上电检测到捕获信号，则此时是占空比调节到最大了
				   else PWM_Duty_Percent = 0;                    //上电没检测到捕获信号，PWM源没有接入
			 }
			 else  //最小占空比
			 {
				 PWM_Duty_Percent = 50;
			 }
		}

    TMR_ClearFlag(TMR16, TMR_FLAG_Update); 
 }
 
 if(TMR_GetFlagStatus(TMR16,TMR_FLAG_CC1) != RESET)
 {
 
//	 if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pins_8))//上升沿捕获
//	 {     
//		 HiTime = TMR_GetCapture1(TMR16);
//		 TMR_OC1PolarityConfig(TMR16,TMR_ICPolarity_Falling);		//设置为下降沿捕获		
////     TMR_SetCounter(TMR16,0);	   	//清空定时器值
//		 TMR16->CNT = 0;
//		 TMR16->CC1 = 0;
//	 }
//	 else //下降沿沿捕获
//	 {
//		 LoTime = TMR_GetCapture1(TMR16);
//		 TMR_OC1PolarityConfig(TMR16,TMR_ICPolarity_Rising);		//设置为上升沿捕获		
////     TMR_SetCounter(TMR16,0);	   	//清空定时器值
//		 TMR16->CNT = 0;
//		 TMR16->CC1 = 0;
//	 }
	 		  TMR16->CNT = 0;
	 if((TMR16->CCE&0x02)==0)//上升沿捕获
	 {     
		  TMR16->CCE |= 0x02;  //下降沿捕获
		 if((TMR16->STS & 0x200) == 0) HiTime = TMR16->CC1;
		  else TMR16->STS &= 0xfdff;

//		 HiTime = TMR_GetCapture1(TMR16);
//		 TMR16->CC1 = 0;
//		 TMR_OC1PolarityConfig(TMR16,TMR_ICPolarity_Falling);		//设置为下降沿捕获		
////     TMR_SetCounter(TMR16,0);	   	//清空定时器值
//		 TMR16->CNT = 0;
		 
	 }
	 else //下降沿沿捕获
	 {
		 	TMR16->CCE &= 0xFD;  //下降沿捕获
		  if((TMR16->STS & 0x200) == 0) LoTime = TMR16->CC1;
		    else TMR16->STS &= 0xfdff;
		  
//		  TMR16->CNT = 0;
//		 LoTime = TMR_GetCapture1(TMR16);
////		 TMR16->CC1 = 0;
////		 TMR_OC1PolarityConfig(TMR16,TMR_ICPolarity_Rising);		//设置为上升沿捕获		
////     TMR_SetCounter(TMR16,0);	   	//清空定时器值
//		 TMR16->CNT = 0;
		 
	 }
	 
//	 if(PWM_OverTime) PWM_OverTime--;
//	  else 
//		{			
//			//if((HiTime*100/PWM_HZ)>95)
//				
//			PWM_HZ = 1000000/((HiTime + LoTime)/2);
//			

//			Temp = (unsigned int)LoTime*1000/(HiTime + LoTime);
//			PWM_Duty_PercentTemp = ((PWM_Duty_PercentTemp*900 + Temp*124)>>10);
//	 
//	    if(PWM_Duty_PercentTemp < 50)  PWM_Duty_PercentTemp = 50;
//	    else if(PWM_Duty_PercentTemp > 950) PWM_Duty_PercentTemp = 950;
//			PWM_Duty_Percent = PWM_Duty_PercentTemp;	 
//	  }

//	 TMR_ClearFlag(TMR16, TMR_FLAG_CC1); 
 }

}




