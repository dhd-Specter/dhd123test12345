
#include "at32f4xx.h"



//NTC 2K
#define c_wendu_f40_dianzuvalue						  105705 
#define c_wendu_f35_dianzuvalue						  79126 
#define c_wendu_f30_dianzuvalue						  59794 
#define c_wendu_f25_dianzuvalue						  45630 
#define c_wendu_f20_dianzuvalue						  35144 
#define c_wendu_f15_dianzuvalue						  27303 
#define c_wendu_f10_dianzuvalue						  21377 
#define c_wendu_f5_dianzuvalue						  16869 
#define c_wendu_0_dianzuvalue						    13411 
#define c_wendu_5_dianzuvalue						    10735 
#define c_wendu_10_dianzuvalue						  8653 
#define c_wendu_15_dianzuvalue						  7018 
#define c_wendu_20_dianzuvalue						  5726 
#define c_wendu_25_dianzuvalue						  4700 
#define c_wendu_30_dianzuvalue						  3879 
#define c_wendu_35_dianzuvalue						  3219 
#define c_wendu_40_dianzuvalue						  2685 
#define c_wendu_45_dianzuvalue						  2250 
#define c_wendu_50_dianzuvalue						  1895 
#define c_wendu_55_dianzuvalue						  1604 
#define c_wendu_60_dianzuvalue						  1363
#define c_wendu_65_dianzuvalue						  1163 
#define c_wendu_70_dianzuvalue						  996 
#define c_wendu_75_dianzuvalue						  857 
#define c_wendu_80_dianzuvalue						  740 
#define c_wendu_85_dianzuvalue						  641 
#define c_wendu_90_dianzuvalue						  558 
#define c_wendu_95_dianzuvalue						  487 
#define c_wendu_100_dianzuvalue						  426 
#define c_wendu_105_dianzuvalue						  375
#define c_wendu_110_dianzuvalue						  330 
#define c_wendu_115_dianzuvalue						  292 
#define c_wendu_120_dianzuvalue						  259 
#define c_wendu_125_dianzuvalue						  230 

#define c_wendu_max_index    							34             //
#define c_wendu_min_zhihuan_value					20               //
#define c_wendu_100du_dianzu_value				c_wendu_100_dianzuvalue
#define c_wendu_25du_dianzu_value				  c_wendu_25_dianzuvalue

#define c_wendu_max_value				        105


#define c_wendu_wuchabuchang_fz				  25    //25   27:25==108:100         
#define c_wendu_wuchabuchang_fm				  25    //25   27:25==108:100 



extern u16  wendu_adc_value,wendu_adc_value_save;
extern u8 g_cOverCtrlFlag;

extern u16 wendu_count_dianzu_value(u16 wendu);
extern unsigned long int wendu_advalue_count_dianzu(u16 wendu_advalue);
extern u8 wendu_dianzu_count_wendu(unsigned long int dianzhu_value);



