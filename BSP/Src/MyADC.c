#include "MyADC.h"

#define ADC_RESOLUTION 4096        //ADC分辨率
#define ADC_VOLTAGE_REF 3.3f        //ADC参考电压，单位：V

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

/**
 * @brief  根据ADC HAL句柄获取对应的软件ADC实例
 * @param  hadc: ADC_HandleTypeDef指针，指向触发中断的ADC句柄
 * @retval ADC_t*: 返回对应的ADC实例指针，未找到则返回NULL
 * @note   通过硬件外设地址(Instance)反查实例数组，使用X-Macro(ADC_LIST)自动生成case分支
 */
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

/**
 * @brief  初始化ADC实例，建立软件ADC与硬件ADC句柄的映射关系
 * @param  adc: 指向ADC实例结构体的指针
 * @param  adc_num: ADC编号枚举值，指定要使用的硬件ADC（ADC_1/ADC_2）
 * @retval uint8_t: 返回0表示初始化成功，返回1表示参数无效（编号越界或为ADC_0）
 * @note   该函数将ADC实例注册到全局数组中，便于中断中通过硬件句柄反查实例
 */
uint8_t My_ADC_Init(ADC_t* adc, ADC_Num_e adc_num){
    if(adc_num >= ADC_MAX_NUM || adc_num == 0 || ADC_Map[adc_num] == NULL){
        return 1; 
    }
    ADC_Instance[adc_num] = adc;
    adc->hadc = ADC_Map[adc_num];
    return 0;
}

/**
 * @brief  初始化ADC板级参数并预计算缩放因子
 * @param  adc: 指向ADC_BSP_Param_t结构体的指针，用于存储参数
 * @param  current_gain: 运放增益（V/V）
 * @param  V_bias: 零电流时的ADC偏置计数值（原始ADC值），例如12bit中点=2048
 * @param  R_Shunt: 采样电阻阻值，单位：Ω
 * @note   预计算Voltage_scale = V_ref / 4096，Current_scale = V_ref / (4096 × gain × R_shunt)，
 *         从而在中断中仅需 (value - V_bias) × scale 即可得到结果，避免重复浮点除法。
 */
void My_ADC_BSP_Param_Init(ADC_BSP_Param_t* adc, float current_gain, float V_bias, float R_Shunt){
    adc->current_gain = current_gain;
    adc->V_bias = V_bias;
    adc->R_Shunt = R_Shunt;

    adc->Voltage_scale = ADC_VOLTAGE_REF / ADC_RESOLUTION;
    adc->Current_scale = ADC_VOLTAGE_REF / (current_gain * R_Shunt * ADC_RESOLUTION);
   }

/**
 * @brief  启动ADC常规通道转换及采样
 * @param  adc: 指向ADC实例结构体的指针
 * @note   使能ADC适用于常规（非注入）通道连续采集场景
 */
void My_ADC_RegularStart(ADC_t* adc){
    HAL_ADC_Start(adc->hadc);
}

/**
 * @brief  启动ADC注入通道转换
 * @param  adc: 指向ADC实例结构体的指针
 * @note   注入组无连续转换模式，每次触发仅执行一次注入序列后自动停止，
 *         因此每次需要新数据时需重新调用此函数启动。
 */
void My_ADC_InjectedStart(ADC_t* adc){
    HAL_ADCEx_InjectedStart(adc->hadc);
}

/**
 * @brief  获取ADC常规通道的采样值
 * @param  adc: 指向ADC实例结构体的指针
 * @note   读取当前ADC常规通道的转换结果并存入 adc->value。
 *         调用前需确保已通过 My_ADC_RegularStart() 启动转换。
 */
void My_ADC_Get_RegularValue(ADC_t* adc){
    adc->value = HAL_ADC_GetValue(adc->hadc);
}

/**
 * @brief  获取ADC注入通道的采样值并转换为电压
 * @param  adc: 指向ADC实例结构体的指针
 * @param  InjectedRank: 注入通道的组内编号（ADC_INJECTED_RANK_1 ~ 4）
 * @retval uint32_t: 返回原始ADC计数值（0~4095）
 * @note   注入通道通常由定时器触发，优先级高于常规通道，适合FOC电流采样等需要精确时刻的场景。
 *         同时将计数值换算为实际电压存入 adc->voltage（单位V）
 */
void My_ADC_Get_InjectedValue(ADC_t* adc, uint32_t InjectedRank){
    HAL_ADCEx_InjectedStart(adc->hadc);
    adc->value = HAL_ADCEx_InjectedGetValue(adc->hadc, InjectedRank);
}

/**
 * @brief  将ADC原始值转换为差分电压（相对偏置点）
 * @param  adc: 指向ADC实例结构体的指针，读取 adc->value 并写入 adc->voltage
 * @param  param: 指向ADC板级参数的指针，提供偏置计数值 V_bias 和电压缩放因子 Voltage_scale
 * @note   结果 = (value - V_bias) × Voltage_scale，即采样电阻两端的差分电压，单位：V。
 *         零电流时结果为0，正/负分别对应正/反向电流。
 */
void My_ADC_Calculate_Voltage(ADC_t* adc, ADC_BSP_Param_t* param){
    adc->voltage = (float)(adc->value - param->V_bias) * param->Voltage_scale;
}

/**
 * @brief  将ADC原始值转换为相电流
 * @param  adc: 指向ADC实例结构体的指针，读取 adc->value 并写入 adc->current
 * @param  param: 指向ADC板级参数的指针，提供偏置计数值 V_bias 和电流缩放因子 Current_scale
 * @note   结果 = (value - V_bias) × Current_scale，单位：A。
 *         Current_scale 已在 My_ADC_BSP_Param_Init 中预计算，此处仅需一次减法+乘法。
 */
void My_ADC_Calculate_Current(ADC_t* adc, ADC_BSP_Param_t* param){
    adc->current = (float)(adc->value - param->V_bias) * param->Current_scale;
}