#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
	   
ADC_HandleTypeDef hadc1;    

//��ʼ��ADC
//�������ǽ��Թ���ͨ��Ϊ��
//����Ĭ�Ͻ�����ͨ��0~3																	   
void  Adc_Init(void)
{ 	
 
  ADC_ChannelConfTypeDef sConfig;
    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  HAL_ADC_DeInit(&hadc1);

  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = DISABLE; //EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);
  
    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}	

int16_t Get_Adc2(void)   
{
  int16_t ADC_Value;
    /*##-3- Start the conversion process #######################################*/
 HAL_ADC_Start(&hadc1);
   /*##-4- Wait for the end of conversion #####################################*/
  /*  Before starting a new conversion, you need to check the current state of
       the peripheral; if it�s busy you need to wait for the end of current
       conversion before starting a new one.
       For simplicity reasons, this example is just waiting till the end of the
       conversion, but application may perform other tasks while conversion
       operation is ongoing. */
  HAL_ADC_PollForConversion(&hadc1, 10);  
  /* Check if the continuous conversion of regular channel is finished */
 // if ((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
  if (HAL_ADC_GetState(&hadc1))
  {
    /*##-5- Get the converted value of regular channel  ########################*/
    ADC_Value = HAL_ADC_GetValue(&hadc1);
  }
  return ADC_Value;
}

//���ADCֵ
//ch:ͨ��ֵ 0~3
int16_t Get_Adc(int8_t ch)   
{
#if 0
  //����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
#endif     
}

int16_t Get_Adc_Average(int8_t ch,int8_t times)
{
#if 0
	int32_t temp_val=0;
	int8_t t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
                osDelay(5);
	}
	return temp_val/times;
#endif        
} 	 



























