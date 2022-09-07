


#include "ctrlmode.h"
#include "function.h"	
#include "SEGGER_RTT.h"
#include "SciToOprt.h"
#include "application.h"
#include "ntc.h"
#include "spi.h"


const unsigned int wendu_biao[c_wendu_max_index] =     {c_wendu_f30_dianzuvalue,
	                                                      c_wendu_f25_dianzuvalue , c_wendu_f20_dianzuvalue , c_wendu_f15_dianzuvalue,c_wendu_f10_dianzuvalue,c_wendu_f5_dianzuvalue ,
	                                                      c_wendu_0_dianzuvalue   , c_wendu_5_dianzuvalue   , c_wendu_10_dianzuvalue ,c_wendu_15_dianzuvalue ,c_wendu_20_dianzuvalue ,
																										 		c_wendu_25_dianzuvalue  , c_wendu_30_dianzuvalue  , c_wendu_35_dianzuvalue ,c_wendu_40_dianzuvalue ,c_wendu_45_dianzuvalue ,
																										 		c_wendu_50_dianzuvalue  , c_wendu_55_dianzuvalue  , c_wendu_60_dianzuvalue ,c_wendu_65_dianzuvalue ,c_wendu_70_dianzuvalue , 
																										 		c_wendu_75_dianzuvalue  , c_wendu_80_dianzuvalue  , c_wendu_85_dianzuvalue ,c_wendu_90_dianzuvalue ,c_wendu_95_dianzuvalue ,
																											 	c_wendu_100_dianzuvalue , c_wendu_105_dianzuvalue , c_wendu_110_dianzuvalue,c_wendu_115_dianzuvalue,c_wendu_120_dianzuvalue,
																											 	c_wendu_125_dianzuvalue 
																										 		};
u16  wendu_adc_value=0, wendu_adc_value_save=0;
	
u8 g_cOverCtrlFlag;


u16 wendu_count_dianzu_value(u16 wendu)
{
	u8 i,j;
	u16 temp;
	if(wendu>=120)
	{
		return(c_wendu_120_dianzuvalue);
	}
	else if(wendu==0)
	{
		return(c_wendu_0_dianzuvalue);
	}
	else 
	{
		i=wendu/5; //
		j=wendu%5;
		temp=wendu_biao[i+6]-(wendu_biao[i+6]-wendu_biao[i+7])/5*j;
		return(temp);
	}
}


unsigned long int wendu_advalue_count_dianzu(u16 wendu_advalue)
{ 
	if(wendu_advalue==0) return(c_wendu_max_value);
	return((unsigned long int)((unsigned long int)((unsigned long int)(wendu_advalue)*2000)/(1024-wendu_advalue)));
}



u8 wendu_dianzu_count_wendu(unsigned long int dianzhu_value)
{ 
	u8 i;
  u8 temp;
	
	if(dianzhu_value>=wendu_biao[1]) return(0x80+25);               //-25
	else if(dianzhu_value>c_wendu_0_dianzuvalue)
	{		 
		 for(i=0;i<5;i++)
		 {
		 	 if(dianzhu_value<wendu_biao[5-i])
		 	 {		 	 	 
		 	 	 temp=5-((unsigned int)((unsigned int)(wendu_biao[5-i]-dianzhu_value))*5)/((unsigned int)((wendu_biao[5-i]-wendu_biao[6-i])));
		 	 	 temp=temp+5*i+0x80;
		 	 	 return(temp);
		 	 }
		 }
	}
	else if(dianzhu_value>c_wendu_125_dianzuvalue)
	{
		 for(i=7;i<c_wendu_max_index;i++)
		 {
		 	 if(dianzhu_value>wendu_biao[i])
		 	 {		 
		 	 	 temp=(unsigned char)((unsigned int)((unsigned int)(wendu_biao[i-1]-dianzhu_value)*5)/((unsigned int)(wendu_biao[i-1]-wendu_biao[i])));
		 	 	 temp=temp+5*(i-7);
         temp=(u8)((u16)((u16)((u16)(temp)*c_wendu_wuchabuchang_fz)/c_wendu_wuchabuchang_fm));
         if(temp>125) temp=125;
 	 	 
		 	 	 return(temp);
		 	 }
		 }
	}
	else return(125);
	
	return 0;
}																												
																												


