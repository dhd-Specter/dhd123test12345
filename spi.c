#include "at32f4xx.h"
#include "ctrlmode.h"
#include "function.h"	
#include "SEGGER_RTT.h"
#include "SciToOprt.h"
#include "application.h"
#include "ntc.h"
#include "spi.h"


spi_synch   spi_synch_blx;
txflag      TXStr_BLX;
DYLimt_str  DYLimt_blx;  
BChange_str BChange_blx;
ComBUF_str  ComBuf_blx;


u16  uwKeySpeedUartOrg = 0,uwKeySpeedUartOrg1 = 0,uwKeySpeedUartOrg2 = 0,uwKeySpeedRefRec = 0,uwKeySpeedRefFil = 0;
u32  ulKeySpeedUart = 0;
u32  ulKeySpeedRefFil = 0; 
u8   uwMotorNumSPINum = c_MotoType_init;
u16  TEMP_uACVol,TEMP_uwDisSpdFdb,TEMP_uwIqFdb;


volatile u16  txgz_delay_time;
volatile u8   f_txgz;







void spi_synch_init(void)
{
	unsigned char i;

	spi_synch_blx.bit_state=bit_idle;
	spi_synch_blx.bit_num=0;
	spi_synch_blx.state=txrx_idle;
	spi_synch_blx.tx_index=0;
	spi_synch_blx.rx_index=0;
	spi_synch_blx.tx_data=C_DATA_IDLE;
	spi_synch_blx.rx_data=C_DATA_IDLE;
	spi_synch_blx.tx_data_jg_time=0;
	for(i=0;i<c_max_num;i++)
	{
		spi_synch_blx.tx_buf[i]=C_DATA_IDLE;
		spi_synch_blx.rx_buf[i]=C_DATA_IDLE;
	}

	TXStr_BLX.value=0;
  BChange_blx.value=0;
  DYLimt_blx.QYSD_Limt=c_QYsd_Init;
  DYLimt_blx.GYSD_Limt=c_GYsd_Init;    

}





void PIN_MISO_O(u8 temp)
{
	if(temp==SPIhigh)
	{
		GPIO_SetBits(GPIOB, GPIO_Pins_4); 
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pins_4); 
	}
}



void spi_ctr(void)
{
    spi_synch_blx.tx_data_jg_time=0;
	  if(spi_synch_blx.state==txrx_endok)
	  {
	    PIN_MISO_O(SPIhigh);
	  }
	  else if(spi_synch_blx.state==txrx_idle)
	  {
	    if(spi_synch_blx.bit_state==bit_idle)
	    {
		    if((R_PIN_MOSI_CLK())==0)
		    {
		      spi_synch_blx.bit_num=0;
		      spi_synch_blx.bit_state=bit_rxing;
		      spi_synch_blx.tx_index=0;
		      spi_synch_blx.rx_index=0;						//????
		      spi_synch_blx.state=txrx_rxing;
		      spi_synch_blx.tx_data=spi_synch_blx.tx_buf[spi_synch_blx.tx_index];
		      if(spi_synch_blx.tx_data&0x80)
		      {
			      PIN_MISO_O(SPIhigh);
		      }
		      else
		      {
			      PIN_MISO_O(SPIlow);
		      }
		      spi_synch_blx.tx_data<<=1;

		      if(R_PIN_MOSI())
		      {
			      spi_synch_blx.rx_data|=0x01;
		      }
		      else
		      {
			      spi_synch_blx.rx_data&=0xfe;
		      }
		      spi_synch_blx.rx_data<<=1;           //????
		    }
	    }
	  }
	  else if(spi_synch_blx.state==txrx_rxing)
	  {
	    if(spi_synch_blx.bit_state==bit_idle)
	    {
		    spi_synch_blx.bit_num=0;
		    spi_synch_blx.bit_state=bit_rxing;
		    spi_synch_blx.tx_data=spi_synch_blx.tx_buf[spi_synch_blx.tx_index];
		    if(spi_synch_blx.tx_data&0x80)
		    {
		      PIN_MISO_O(SPIhigh);
		    }
		    else
		    {
		      PIN_MISO_O(SPIlow);
		    }
		    spi_synch_blx.tx_data<<=1;

		    if(R_PIN_MOSI())
		    {
		      spi_synch_blx.rx_data|=0x01;
		    }
		    else
		    {
		      spi_synch_blx.rx_data&=0xfe;
		    }
		    spi_synch_blx.rx_data<<=1;           //????
	    }
	    else if(spi_synch_blx.bit_state==bit_rxing)
	    {
		    if((R_PIN_MOSI_CLK())==0)
		    {
		      spi_synch_blx.bit_num++;
		      if(spi_synch_blx.bit_num<7)
		      {
			      if(spi_synch_blx.tx_data&0x80)
			      {
			        PIN_MISO_O(SPIhigh);
			      }
			      else
			      {
			        PIN_MISO_O(SPIlow);
			      }
			      spi_synch_blx.tx_data<<=1;

			      if(R_PIN_MOSI())
			      {
			        spi_synch_blx.rx_data|=0x01;
			      }
			      else
			      {
			        spi_synch_blx.rx_data&=0xfe;
			      }
			      spi_synch_blx.rx_data<<=1;
		      }
		      else if(spi_synch_blx.bit_num<8)
		      {
			      if(spi_synch_blx.tx_data&0x80)
			      {
			        PIN_MISO_O(SPIhigh);
			      }
			      else
			      {
			        PIN_MISO_O(SPIlow);
			      }
			      if(R_PIN_MOSI())
			      {
			        spi_synch_blx.rx_data|=0x01;
			      }
			      else
			      {
			        spi_synch_blx.rx_data&=0xfe;
			      }
			      spi_synch_blx.bit_state=bit_endok;
		      }
		    }
	    }
	    else if(spi_synch_blx.bit_state==bit_endok)
	    {
		    spi_synch_blx.rx_buf[spi_synch_blx.rx_index]=spi_synch_blx.rx_data&0xFF;
		    spi_synch_blx.rx_index++;
		    spi_synch_blx.bit_state=bit_idle;
		    spi_synch_blx.bit_num=0;
		    if(spi_synch_blx.rx_index>=c_max_num)
		    {
		      spi_synch_blx.state=txrx_endok;
		      spi_synch_blx.rx_index=0;
		      spi_synch_blx.tx_index=0;
		      PIN_MISO_O(SPIhigh);

		    }
		    else
		    {
		      spi_synch_blx.tx_index++;
	        if(spi_synch_blx.rx_index>=c_max_num)
	        {

	        }
		      else spi_synch_blx.tx_data=spi_synch_blx.tx_buf[spi_synch_blx.tx_index];
		    }
	    }	
    }

}


/****************************************************************************
* 名称：calcrc16()
* 功能：CRC 校验  多项试ccitt X16+X12+X5+1（0X1021） 16位
* 入口参数；输入数据的首地址 数据长度 
* 出口参数：16位CRC
****************************************************************************/
u16 crc16_update (u16 crc, u8 date)
{
        u8 i;
        crc = crc ^(u16)date; //((u16)date << 8);
        for (i=0; i<8; i++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xa001;//0x1021;
            else
                crc >>= 1;
        }
        return crc;
}

//计算CRC16
u16 calcrc16(u8 *ptr, u8 count)
{
    u16 crc = 0xffff;
    while (count--)
    {
        crc =crc16_update(crc,*ptr++);
    }
    crc   =   ((crc&0x00ff)<<8)|((crc&0xff00)>>8); //
    return crc;
}


void txgz_check(void)
{
	if(f_txgz==0)
	{
		if(txgz_delay_time>=c_txgz_stopmoto_maxtime)
		{
			f_txgz=1;
			
	    if(RunCmd==1)
			{
				RunCmd = 0;
			}
		}
		else 
			txgz_delay_time++;
	}
}

void spi_synch_tongxun_check(void)
{
//  unsigned char i;
  u16  crc_word,crc_rxd; 
  if(spi_synch_blx.state==txrx_endok)
  {

	  crc_word = calcrc16(spi_synch_blx.rx_buf, c_max_num-2);
		crc_rxd=spi_synch_blx.rx_buf[c_max_num-2];
		crc_rxd<<=8;
		crc_rxd+=spi_synch_blx.rx_buf[c_max_num-1];
		if(crc_word == crc_rxd)
	  {
	    synch_receive_proc();
	  }
	  synch_send_proc();
  }
}


u16 FreTemp;
void synch_receive_proc(void)
{
	u16 temp;
  u32 ulTmp;
	

  if(g_uErrorCode==C_TONGXUN_GZ)
  {
  	g_uErrorCode=C_NO_ERR;
  }

	f_txgz=0;
	txgz_delay_time=0;

  if((spi_synch_blx.rx_buf[4]==c_normal)||(spi_synch_blx.rx_buf[4]==c_SW_com))
  {          
   if(spi_synch_blx.rx_buf[3] < pr[ACCTIME]) 
		 uwAccTime = pr[ACCTIME]; 
   else 
		 uwAccTime = spi_synch_blx.rx_buf[3];   
/******************************************************************************/
//by jason in 20160926      
		if(guowen_bl!=spi_synch_blx.rx_buf[6])
    {
      guowen_bl=spi_synch_blx.rx_buf[6];
      guowen_zh_adcvalue();
    }


    if(spi_synch_blx.rx_buf[4]==c_SW_com)
    {
    	TXStr_BLX.bits.f_SW_ack=1;
    } 	       	
  }   
  else if(spi_synch_blx.rx_buf[4]==c_dir_com)                          //V3.0??????
  {          
   	if(spi_synch_blx.rx_buf[3]==CCW)
   	{
      if(pr[VELDIR]==CW)
      {
        dir_change_time=0; 
				/*
			  if(RunCmd==1)
        {
	        RunCmd = 0;
          TXStr_BLX.bits.f_dir_pro=1;
        }
				*/
      }
   	  //pr[VELDIR]=CCW;
   	}
	  else
	  {
	    if(pr[VELDIR]==CCW)
      {
        dir_change_time=0;
			  /*
			  if(RunCmd==1)
        {
	        RunCmd = 0;
          TXStr_BLX.bits.f_dir_pro=1;
        }
				*/
      }
	    //pr[VELDIR]=CW;
	  }
	  if(guowen_bl!=spi_synch_blx.rx_buf[6])
    {
      guowen_bl=spi_synch_blx.rx_buf[6];
      guowen_zh_adcvalue();
    }
	  TXStr_BLX.bits.f_dir_ack=1;                               //?????
  }

   else if(spi_synch_blx.rx_buf[4]==c_BitChange_com)                          //V4.0以上新的命令
   {
   	 temp=spi_synch_blx.rx_buf[5];
   	 temp=temp<<8;
     BChange_blx.value=temp+spi_synch_blx.rx_buf[3];
   	 TXStr_BLX.bits.f_BChange_ack=1;
		 if(spi_synch_blx.rx_buf[3]&0x02)  LU_Disable = 1;  //欠压保护取消
		   else  LU_Disable = 0;                            //欠压保护使能

		 if(spi_synch_blx.rx_buf[3]&0x04)  OU_Disable = 1;  //过压保护取消
		   else  OU_Disable = 0;                            //过压保护使能

		 if(spi_synch_blx.rx_buf[3]&0x08)  OT_Disable = 1;  //过温保护取消
		   else  OT_Disable = 0;                            //过温保护使能		 
		 
   	 if(guowen_bl!=spi_synch_blx.rx_buf[6])
     {
        guowen_bl=spi_synch_blx.rx_buf[6];
        guowen_zh_adcvalue();
     }
     if(BChange_blx.bits.f_INS==1)
      {

  		}
  		else
  		{
 
  		}
  		if(BChange_blx.bits.f_CM==1)
  		{
  			pr[POWLIM_EN]=1;
  		}
  		else
  		{
  			pr[POWLIM_EN]=0;
  		}
   }
   else if(spi_synch_blx.rx_buf[4]==c_DYLimtLZChange_com)       //力矩提升在永磁定义里，暂不处理                  
   {
   	    ComBuf_blx.D2BUF=spi_synch_blx.rx_buf[6];
   	    temp=TXRX_ZH_DYValue(spi_synch_blx.rx_buf[3]);
   	 	  if(temp<c_QYsd_Min) temp=c_QYsd_Min;
   	 	  else if(temp>c_QYsd_Max) temp=c_QYsd_Max;
   	 	  DYLimt_blx.QYSD_Limt=temp;	
   	 	  temp=TXRX_ZH_DYValue(spi_synch_blx.rx_buf[5]);
   	 	  if(temp<c_GYsd_Min) temp=c_GYsd_Min;
   	 	  else if(temp>c_GYsd_Max) temp=c_GYsd_Max;
   	 	  DYLimt_blx.GYSD_Limt=temp;	
   	 	  TXStr_BLX.bits.f_DYLimtLZChange_ack=1;
   }
   else if(spi_synch_blx.rx_buf[4]==c_ParmReg1Change_com)       //机型选择，其它保留               
   {
   	    if((spi_synch_blx.rx_buf[3]<=c_MotoType_Max)&&(spi_synch_blx.rx_buf[3]>=c_MotoType_Min))
   	    {
   	      uwMotorNumSPINum=spi_synch_blx.rx_buf[3];
					f_poweron_onlycheck_50_60_ok=0;                     //置上最大频率发送最大标志，清发送时间
	        sysInitCnt=0;				
   	    }
   	    ComBuf_blx.D1BUF=spi_synch_blx.rx_buf[5];
   	    ComBuf_blx.D2BUF=spi_synch_blx.rx_buf[6];
   	 	  TXStr_BLX.bits.f_REG1Change_ack=1;
   }
    


  if((spi_synch_blx.rx_buf[2]&0x01) == 0x01)
  {
    if((g_uErrorCode == C_NO_ERR) && (g_uErrorIni == C_NO_ERR) && (slUnRunCnt > 3000)  && (DelayTime==0))//3S
    {
			if((RunCmd==0)&&(TXStr_BLX.bits.f_dir_pro==0))
	    {
					if(f_moto_idle==0)	
		      {					
				    RunCmd = 1;
					  uwRunSpdDec = 0;
		        moto_idel_timecnt=c_moto_idle_maxtime;
		        f_moto_idle=1;
		      }
			}
    }
  }
  else if(((spi_synch_blx.rx_buf[2]&0x01) == 0x00)||(TXStr_BLX.bits.f_dir_pro==1))
  {    		
		
		if(RunCmd == 1)
		{	
//		  if((pr[CTRLM] == 7) && (IFtoFunSign == 1))
//			  uwRunSpdDec = 1;
//			else
//			{
//				RunCmd = 0;
//			
//			}
			RunCmd = 0;
    }
  
    else
    {   	

    }        
  }
  
  temp = (u16)spi_synch_blx.rx_buf[0]<<8;
  temp += spi_synch_blx.rx_buf[1];
  FreTemp = temp;
	if(temp >= 1000) temp = 1000;
	if(temp < 100) temp = 100;
	
	temp=((u32)temp * 205) >> 10;	                 //  /5

	uwKeySpeedUartOrg1 = spi_synch_blx.rx_buf[0];
	uwKeySpeedUartOrg2 = spi_synch_blx.rx_buf[1];
	
  if(pr[POWLIM_EN] == 0)	
	{	
    uwKeySpeedUartOrg = temp;
    ulKeySpeedUart = (u32)temp * 328;     //328/32768
    uwKeySpeedRefRec = ulKeySpeedUart * uwSpdRefRadio >> 15;//pr[VELCMD]>>15;
	
      ulKeySpeedRefFil = ((u32)uwKeySpeedRefRec*512)+((u32)uwKeySpeedRefFil*512);
      uwKeySpeedRefFil = ulKeySpeedRefFil >> 10;
		
//		uwKeySpeedRefFil = FreTemp*pr[PM_POLES]/2;
     
		if(uwKeySpeedRefFil > pr[VELCMD])
		  uwKeySpeedRefFil = pr[VELCMD];
	
	  if(uwKeySpeedRefFil < (pr[VELCMD]>>2)) 
		  uwKeySpeedRefFil = (pr[VELCMD]>>2);
  }
	else
	{	
    uwKeySpeedUartOrg = (u32)temp * uwSpdMechCf>>10;
    ulKeySpeedUart = (u32)temp*1044>>10;     //1.02
		ulTmp = ulKeySpeedUart*ulKeySpeedUart*ulKeySpeedUart;//0~100 -->> 0~1000000
    uwKeySpeedRefRec = ulTmp * pr[POW_LIM] >> 20;//pr[VELCMD]
	
      ulKeySpeedRefFil = ((u32)uwKeySpeedRefRec*512)+((u32)uwKeySpeedRefFil*512);
      uwKeySpeedRefFil = ulKeySpeedRefFil >> 10;
	
		if(uwKeySpeedRefFil > pr[POW_LIM])
		  uwKeySpeedRefFil = pr[POW_LIM];
	
	  if(uwKeySpeedRefFil < (pr[POW_LIM]>>4)) 
		  uwKeySpeedRefFil = (pr[POW_LIM]>>4);
	}	

}



void synch_send_proc(void)
{

		u16 crc_word;
  if(uwDisSpdFdb==0) 
	{
		TEMP_uwDisSpdFdb=0;
	}	
  else
  {
		TEMP_uwDisSpdFdb=(u32)uwDisSpdFdb*2/pr[PM_POLES];     //
  }

  TEMP_uwIqFdb =uwDisIRef;                //0.1A RMS

/******************************************************************************/
/*******************************************************************************************************************/      
     
  if(TXStr_BLX.bits.f_dir_ack==1)
  {
  	TXStr_BLX.bits.f_dir_ack=0;
  	spi_synch_blx.tx_buf[6] = uwTemper;//wendu_dianzu_count_wendu(wendu_advalue_count_dianzu(wendu_adc_value));
  	TEMP_uACVol&=0x7ff;
  	spi_synch_blx.tx_buf[3] = ((TEMP_uACVol>>8)|(c_dir_com<<3));
    spi_synch_blx.tx_buf[2] = TEMP_uACVol & 0x00ff;
    spi_synch_blx.tx_buf[1] = pr[VELDIR];                 //????
  }
  else if(TXStr_BLX.bits.f_SW_ack==1)
  {
  	TXStr_BLX.bits.f_SW_ack=0;
  	spi_synch_blx.tx_buf[6] = (c_MB_Version_Num>>8);//wendu_dianzu_count_wendu(wendu_advalue_count_dianzu(wendu_adc_value));
  	TEMP_uACVol&=0x7ff;
  	spi_synch_blx.tx_buf[3] = ((TEMP_uACVol>>8)|(c_SW_com<<3));
    spi_synch_blx.tx_buf[2] = TEMP_uACVol & 0x00ff;
    spi_synch_blx.tx_buf[1] = c_MB_Version_Num;                 //????
  }

  else if(TXStr_BLX.bits.f_BChange_ack==1)
  {
    TXStr_BLX.bits.f_BChange_ack=0;
    spi_synch_blx.tx_buf[6] = (BChange_blx.value>>8);
    TEMP_uACVol&=0x7ff;
    spi_synch_blx.tx_buf[3] = ((TEMP_uACVol>>8)|(c_BitChange_com<<3));
    spi_synch_blx.tx_buf[2] = TEMP_uACVol & 0x00ff;
    spi_synch_blx.tx_buf[1] = (BChange_blx.value&0x0ff);                 
  } 
  else if(TXStr_BLX.bits.f_DYLimtLZChange_ack==1)
  {
    TXStr_BLX.bits.f_DYLimtLZChange_ack=0;       	
    spi_synch_blx.tx_buf[6] =ComBuf_blx.D2BUF;
    TEMP_uACVol&=0x7ff;
    spi_synch_blx.tx_buf[3] = ((TEMP_uACVol>>8)|(c_DYLimtLZChange_com<<3));
    spi_synch_blx.tx_buf[2] = DY_ZH_TXRXValue(DYLimt_blx.GYSD_Limt);
    spi_synch_blx.tx_buf[1] = DY_ZH_TXRXValue(DYLimt_blx.QYSD_Limt);
  } 
  else if(TXStr_BLX.bits.f_REG1Change_ack==1)
  {
    TXStr_BLX.bits.f_REG1Change_ack=0;       	
    spi_synch_blx.tx_buf[6] =ComBuf_blx.D2BUF;
    TEMP_uACVol&=0x7ff;
    spi_synch_blx.tx_buf[3] = ((TEMP_uACVol>>8)|(c_ParmReg1Change_com<<3));
    spi_synch_blx.tx_buf[2] =ComBuf_blx.D1BUF;
    spi_synch_blx.tx_buf[1] = uwMotorNumSPINum;
  }            
  else
  {
/*******************************************************************************************************************/
//by jason in 20160926            
    spi_synch_blx.tx_buf[6] = uwTemper;//wendu_dianzu_count_wendu(wendu_advalue_count_dianzu(wendu_adc_value));
/****************************************************************************/   	
  	spi_synch_blx.tx_buf[3] = TEMP_uACVol>>8;
    spi_synch_blx.tx_buf[2] = TEMP_uACVol & 0x00ff;
    spi_synch_blx.tx_buf[1] = adAverageCurrent & 0x00ff;//TEMP_uwIqFdb & 0x00ff;
  }

  if(g_uErrorCode==C_NO_ERR)
  {
  	if(f_txgz==1)
  	{
  		g_uErrorCode = C_TONGXUN_GZ;
  	}
  }
	if(f_poweron_onlycheck_50_60_ok == 0)
	{	
    spi_synch_blx.tx_buf[0] = (g_uErrorCode & 0x007f)|0x0080;
	  spi_synch_blx.tx_buf[5] = uwSpdMechHz>>8;
    spi_synch_blx.tx_buf[4] = uwSpdMechHz & 0x00ff;
	}	
	else
	{	
		spi_synch_blx.tx_buf[0] = (g_uErrorCode & 0x007f);
	  spi_synch_blx.tx_buf[5] = TEMP_uwDisSpdFdb>>8;
    spi_synch_blx.tx_buf[4] = TEMP_uwDisSpdFdb & 0x00ff;
	}	

  	
		crc_word=calcrc16(spi_synch_blx.tx_buf, c_max_num-2);
		spi_synch_blx.tx_buf[c_max_num-2]=crc_word/0x100;                                   	  
    spi_synch_blx.tx_buf[c_max_num-1]=crc_word%0x100;

    spi_synch_blx.state=txrx_idle;
}









