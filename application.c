
#include "ctrlmode.h"
#include "function.h"	
#include "SEGGER_RTT.h"
#include "SciToOprt.h"
#include "application.h"
#include "ntc.h"
#include "spi.h"





LED_FLASH_STRU  g_hLedFlash;

volatile s16 DelayTime;

u16  g_uErrorCode;
u32  ulIrate150Cnt = 0;
u16  uwVolBusCom = 3100;
u16  uwTemper;
u16  g_lTemper;
u16  g_lBusCur = 0;
u16  h_ADCBusvolt;
u16  AVI1;
u16  AVI2;
u16 SpeedSourceTemp;

#define MaxSpeedRpm   8000
#define MinSpeedRpm   1000




void Ext_Led(void)
{
    //运行灯 
	if(g_uErrorCode)
  {  
    g_hLedFlash.state = FAULT1;      
  }  
  else
  {  
    g_hLedFlash.state = NORMAL1;
  }   
  if( g_hLedFlash.FlashInterTime==0 )
  {
    if(g_hLedFlash.state==NORMAL1)
    {
			GPIOC->OPTDT ^= GPIO_Pins_13;
      g_hLedFlash.FlashInterTime = 200;
    }
    else
    {
      if( g_hLedFlash.FlashStopInterTime==0 )
      {
        if( g_hLedFlash.FlashCnt==0 )//
        {
          g_hLedFlash.FlashStopInterTime = 3000;
          if(g_uErrorCode != 0)
						g_hLedFlash.FlashCnt = 2*g_uErrorCode;
					else
						g_hLedFlash.FlashCnt = 2*g_uErrorCode;
        }
        else
        {
          g_hLedFlash.FlashCnt--;
					GPIOC->OPTDT ^= GPIO_Pins_13;
          g_hLedFlash.FlashInterTime = 500;
        }
      }
      else
      {         
        g_hLedFlash.FlashInterTime = 0;
        g_hLedFlash.FlashStopInterTime--;
				GPIO_SetBits(GPIOC, GPIO_Pins_13);
      }
    }
	}
  else
  {
    g_hLedFlash.FlashInterTime--;
  }

}





void OverLoad_Protection(void)
{	
  if((sal_stLpfOutIqFdb.swLpfOut > uwIrateOver) && (RunCmd))
	{
		if (ulIrate150Cnt < 60000) ulIrate150Cnt++;
		else  g_uErrorCode = C_GUOZAI;
	}	
	else ulIrate150Cnt = 0;
} 



void Fault_Management(void)
{
	
	OverLoad_Protection();
	
	if(g_cOverCtrlFlag == 1)
	{	
		uwKeySpeedRef = (u32)uwKeySpeedRefFil * 24576 >> 15;//75%
	}	  
	else
	{	    	

	 uwKeySpeedRef = (u32)uwKeySpeedRefFil;
	}	
	  
	if(g_uVdcDis > DC_OVLimt)
	{
		if(swOVCnt < 20) swOVCnt++;
		 else g_uErrorCode = C_GUOYA;	
	}
	else if(g_uVdcDis < DC_LVLimt)
	{
		if(swLVCnt < 20) swLVCnt++;
		 else g_uErrorCode = C_QIANYA;
	}
	else 
	{
	 swOVCnt = 0;
	 swLVCnt = 0;
	}
	
	
	if(kw_uwPlugSpdHz > pr[SPDMAX])
	{	
		if(slHighSpdCnt < 3000)  slHighSpdCnt++;
		else
		{
			if(g_uErrorCode == C_NO_ERR)
	    {    
				g_uErrorCode = C_OVERSPD_GZ;
		  }
    }	
  }					
	else slHighSpdCnt = 0;
	
	
	if(g_uErrorCode)
	{
		RunCmd = 0; 
		PWM_DIS_DUTY(); 
	}	
}

unsigned int Efre_max;
unsigned int Efre_min;
void Speed_vInit(void)
{
  Efre_max = (u32)MaxSpeedRpm*pr[PM_POLES]/12;
	Efre_min = (u32)MinSpeedRpm*pr[PM_POLES]/12;
}



void Motor_Ctr(void)
{

	unsigned int SpeedSource;
	unsigned int SpeedValue;

	
  SpeedSource = (u32)AVI1*1000/1023;   //10bit模拟量  转换成0--1000
	
	SpeedSourceTemp = (((u32)SpeedSourceTemp*800 + SpeedSource*224)>>10);  //滤波
	
	SpeedValue = SpeedSourceTemp;
	
	if(SpeedValue < 50)   SpeedValue = 50;     //5%
	 else if(SpeedValue > 950)  SpeedValue = 950;  //95%
  

	SpeedValue = (Efre_max - Efre_min)*(SpeedValue-50)/900;
	
	uwKeySpeedRefFil = Efre_min + SpeedValue;

}















