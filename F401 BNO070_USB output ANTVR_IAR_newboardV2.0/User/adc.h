#ifndef __ADC_H
#define __ADC_H	

void Adc_Init(void);
int16_t  Get_Adc2(void); 
int16_t  Get_Adc(int8_t ch); 
int16_t Get_Adc_Average(int8_t ch,int8_t times); 
 
#endif 
