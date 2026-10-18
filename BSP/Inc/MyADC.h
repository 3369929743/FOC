#ifndef __MYADC_H__
#define __MYADC_H__

#include "main.h"

#define ADC_LIST \
    X(1, ADC1)   \
    X(2, ADC2)   


typedef enum{
    #define X(Index, Instance) ADC_##Index = Index,
    ADC_LIST
    #undef X
} ADC_Num_e;

typedef struct{
    ADC_HandleTypeDef *hadc;

    float voltage;
}ADC_t;

uint8_t My_ADC_Init(ADC_t* adc, ADC_Num_e adc_num);
void My_ADC_RegularStart(ADC_t* adc);
uint32_t My_ADC_Get_InjectedValue(ADC_t* adc, uint32_t InjectedRank);

#endif