#ifndef __MYADC_H__
#define __MYADC_H__

#include "main.h"

#define ADC_LIST \
    X(1, ADC1)   \
    X(2, ADC2)   

typedef struct ADC_Struct ADC_t;

typedef void (*ADC_ConvCpltCallback_t)(struct ADC_Struct* adc);
typedef void (*ADC_InjectedConvCpltCallback_t)(struct ADC_Struct* adc);

typedef enum{
    #define X(Index, Instance) ADC_##Index = Index,
    ADC_LIST
    #undef X
} ADC_Num_e;

typedef struct ADC_Struct{
    ADC_HandleTypeDef *hadc;

    ADC_ConvCpltCallback_t ConvCpltCallback;
    ADC_InjectedConvCpltCallback_t InjectedConvCpltCallback;
}ADC_t;

typedef struct{
    ADC_Num_e adc_num;

    uint32_t value;
    float voltage;
    float current;
}ADC_Channel_t;

typedef struct{
    float current_gain;      //运放增益
    float V_bias;            //偏置电压
    float R_Shunt;           //采样电阻

    float Voltage_scale;     //电压缩放因子，用于将采样值转换为电压
    float Current_scale;     //电流缩放因子，用于将采样值转换为电流
}ADC_BSP_Param_t;

/* ===================ADC实例初始化=================== */
uint8_t My_ADC_Init(ADC_t* adc, ADC_Num_e adc_num);
void My_ADC_Channel_Init(ADC_Channel_t* adc, ADC_Num_e adc_num);
void My_ADC_BSP_Param_Init(ADC_BSP_Param_t* adc, float current_gain, float V_bias, float R_Shunt);

/* ===================ADC回调函数注册=================== */
void My_ADC_ConvCpltCallback_Register(ADC_t* adc, ADC_ConvCpltCallback_t ConvCpltCallback);
void My_ADC_InjectedConvCpltCallback_Register(ADC_t* adc, ADC_InjectedConvCpltCallback_t InjectedConvCpltCallback);

/* ===================ADC开启转换=================== */
void My_ADC_RegularStart(ADC_t* adc);
void My_ADC_InjectedStart(ADC_t* adc);
void My_ADC_RegularStart_IT(ADC_t* adc);
void My_ADC_InjectedStart_IT(ADC_t* adc);

/* ===================ADC获取转换值=================== */
void My_ADC_Get_RegularValue(ADC_Channel_t* adc);
void My_ADC_Get_InjectedValue(ADC_Channel_t* adc, uint32_t InjectedRank);

/* ===================ADC电压电流计算=================== */
void My_ADC_Calculate_Voltage(ADC_Channel_t* adc, ADC_BSP_Param_t* param);
void My_ADC_Calculate_Current(ADC_Channel_t* adc, ADC_BSP_Param_t* param);

#endif