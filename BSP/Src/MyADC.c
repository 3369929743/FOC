#include "MyADC.h"

#define ADC_RESOLUTION 4095
#define ADC_VOLTAGE_REF 3.3f

#define X(Index, Instance) extern __weak ADC_HandleTypeDef hadc##Index;
ADC_LIST
#undef X

static ADC_HandleTypeDef* const ADC_Map[] = {
    #define X(Index, Instance) [Index] = &hadc##Index,
    ADC_LIST
    #undef X
};

#define ADC_MAX_NUM (sizeof(ADC_Map) / sizeof(ADC_Map[0]))

static ADC_t *ADC_Instance[ADC_MAX_NUM] = {NULL};

static inline ADC_t* ADC_GetInstance(ADC_HandleTypeDef* hadc)
{
    switch ((uint32_t)hadc->Instance) {
    #define X(Index, Instance) case (uint32_t)(Instance): return ADC_Instance[Index];
    ADC_LIST
    #undef X
    default:
        return NULL;
    }
}

uint8_t My_ADC_Init(ADC_t* adc, ADC_Num_e adc_num){
    if(adc_num >= ADC_MAX_NUM || adc_num == 0 || ADC_Map[adc_num] == NULL){
        return 1; 
    }
    ADC_Instance[adc_num] = adc;
    adc->hadc = ADC_Map[adc_num];
    return 0;
}

void My_ADC_RegularStart(ADC_t* adc){
    HAL_ADC_Start(adc->hadc);
    HAL_ADC_StartSampling(adc->hadc);
}

uint32_t My_ADC_Get_InjectedValue(ADC_t* adc, uint32_t InjectedRank){
    HAL_ADCEx_InjectedStart(adc->hadc);
    uint32_t adc_value = HAL_ADCEx_InjectedGetValue(adc->hadc, InjectedRank);
    adc->voltage = adc_value * ADC_VOLTAGE_REF / ADC_RESOLUTION;
    return adc_value;
}