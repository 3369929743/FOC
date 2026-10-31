#include "MyADC.h"
#include <stdint.h>

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
 * @brief  初始化ADC通道实例，记录所属的硬件ADC编号
 * @param  adc: 指向ADC_Channel_t结构体的指针
 * @param  adc_num: 该通道所属的ADC编号（ADC_1/ADC_2），用于后续通过ADC_Map反查硬件句柄
 * @note   一个硬件ADC（如ADC1）可以有多个注入通道（如Rank1/2/3对应U/V/W三相），
 *         每个通道通过此函数独立记录其硬件归属，读取时通过ADC_Map[adc_num]获取句柄。
 */
void My_ADC_Channel_Init(ADC_Channel_t* adc, ADC_Num_e adc_num){
    adc->adc_num = adc_num;
}

/**
 * @brief  初始化ADC板级参数并预计算缩放因子
 * @param  adc: 指向ADC_BSP_Param_t结构体的指针，用于存储参数
 * @param  current_gain: 运放增益（V/V）
 * @param  R_Shunt: 采样电阻阻值，单位：Ω
 * @note   预计算Voltage_scale = V_ref / 4096，Current_scale = V_ref / (4096 × gain × R_shunt)，
 *         从而在中断中仅需 (value - V_bias) × scale 即可得到结果，避免重复浮点除法。
 */
void My_ADC_BSP_Param_Init(ADC_BSP_Param_t* adc, float current_gain, float R_Shunt){
    adc->current_gain = current_gain;
    adc->R_Shunt = R_Shunt;

    adc->Voltage_scale = ADC_VOLTAGE_REF / ADC_RESOLUTION;
    adc->Current_scale = ADC_VOLTAGE_REF / (current_gain * R_Shunt * ADC_RESOLUTION);
   }


/**
 * @brief  启动ADC自校准
 * @param  adc: 指向ADC实例结构体的指针
 * @param  SingleDiff: 校准模式选择，ADC_SINGLE_ENDED（单端）或 ADC_DIFFERENTIAL_ENDED（差分）
 * @retval 无
 * @note   STM32G4系列ADC内置自校准功能，通过校准内部电容阵列来补偿失调误差。
 *         应在 ADC 初始化完成后、启动转换前调用，否则转换结果可能存在固定偏置。
 *         校准期间ADC自动完成采样和误差计算，无需外部输入。
 */
void My_ADC_Calibration_Start(ADC_t* adc, uint32_t SingleDiff){
    HAL_ADCEx_Calibration_Start(adc->hadc, SingleDiff);
}

uint8_t My_ADC_Bias_Calibreation_Start(ADC_Channel_t* self){
    self->Calibration_Sum += self->Value;
    self->Calibration_Count++;
    if(self->Calibration_Count >= 1024){
        self->Value_Bias = (float)self->Calibration_Sum / (float)self->Calibration_Count;
        self->Calibration_Count = 0;
        self->Calibration_Sum = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief  注册ADC规则通道转换完成用户回调
 * @param  adc: 指向ADC实例结构体的指针
 * @param  ConvCpltCallback: 用户自定义回调函数指针，规则组转换完成后由HAL中断回调转发调用
 */
void My_ADC_ConvCpltCallback_Register(ADC_t* adc, ADC_ConvCpltCallback_t ConvCpltCallback){
    adc->ConvCpltCallback = ConvCpltCallback;
}

/**
 * @brief  注册ADC注入通道转换完成用户回调
 * @param  adc: 指向ADC实例结构体的指针
 * @param  InjectedConvCpltCallback: 用户自定义回调函数指针，注入组转换完成后由HAL中断回调转发调用
 */
void My_ADC_InjectedConvCpltCallback_Register(ADC_t* adc, ADC_InjectedConvCpltCallback_t InjectedConvCpltCallback){
    adc->InjectedConvCpltCallback = InjectedConvCpltCallback;
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
 * @brief  启动ADC常规通道转换及采样（中断模式）
 * @param  adc: 指向ADC实例结构体的指针
 * @note   与 My_ADC_RegularStart 不同，此函数使能中断模式。
 *         常规通道转换完成后触发 ADC 中断，HAL 调用 HAL_ADC_ConvCpltCallback，
 *         适用于需要在转换完成后立即在中断中处理数据的场景。
 */
void My_ADC_RegularStart_IT(ADC_t* adc){
    HAL_ADC_Start_IT(adc->hadc);
}

/**
 * @brief  启动ADC注入通道转换（中断模式）
 * @param  adc: 指向ADC实例结构体的指针
 * @note   与 My_ADC_InjectedStart 不同，此函数使能中断模式。
 *         注入组转换完成后触发 JEOC 中断，HAL 调用 HAL_ADCEx_InjectedConvCpltCallback，
 *         适用于 FOC 等需要在每次注入序列完成后立即处理数据的场景。
 *         注意：注入组无连续转换模式，需在回调中重新调用此函数以实现连续触发。
 */
void My_ADC_InjectedStart_IT(ADC_t* adc){
    HAL_ADCEx_InjectedStart_IT(adc->hadc);
}

/**
 * @brief  获取ADC常规通道的采样值
 * @param  adc: 指向ADC_Channel_t结构体的指针
 * @note   读取当前ADC常规通道的转换结果并存入 adc->value。
 *         调用前需确保已通过 My_ADC_RegularStart() 启动转换。
 */
void My_ADC_Get_RegularValue(ADC_Channel_t* adc){
    adc->Value = HAL_ADC_GetValue(ADC_Map[adc->adc_num]);
}

/**
 * @brief  获取ADC注入通道的采样值
 * @param  adc: 指向ADC_Channel_t结构体的指针
 * @param  InjectedRank: 注入通道的组内编号（ADC_INJECTED_RANK_1 ~ 4）
 * @note   注入通道通常由定时器触发，优先级高于常规通道，适合FOC电流采样等需要精确时刻的场景。
 *         读取的原始ADC计数值存入 adc->value，后续通过 My_ADC_Calculate_Current 换算为实际电流。
 */
void My_ADC_Get_InjectedValue(ADC_Channel_t* adc, uint32_t InjectedRank){
    adc->Value = HAL_ADCEx_InjectedGetValue(ADC_Map[adc->adc_num], InjectedRank);
}

/**
 * @brief  将ADC原始值转换为差分电压（相对偏置点）
 * @param  adc: 指向ADC_Channel_t结构体的指针，读取 adc->value 并写入 adc->voltage
 * @param  param: 指向ADC_BSP_Param_t的指针，提供偏置计数值 V_bias 和电压缩放因子 Voltage_scale
 * @note   结果 = (value - V_bias) × Voltage_scale，即采样电阻两端的差分电压，单位：V。
 *         零电流时结果为0，正/负分别对应正/反向电流。
 */
void My_ADC_Calculate_Voltage(ADC_Channel_t* adc, ADC_BSP_Param_t* param){
    adc->Voltage = ((float)adc->Value - adc->Value_Bias) * param->Voltage_scale;
}

/**
 * @brief  将ADC原始值转换为相电流
 * @param  adc: 指向ADC_Channel_t结构体的指针，读取 adc->Value 并写入 adc->current
 * @param  param: 指向ADC_BSP_Param_t的指针，提供偏置计数值 V_bias 和电流缩放因子 Current_scale
 * @note   结果 = (value - V_bias) × Current_scale，单位：A。
 *         Current_scale 已在 My_ADC_BSP_Param_Init 中预计算，此处仅需一次减法+乘法。
 */
void My_ADC_Calculate_Current(ADC_Channel_t* adc, ADC_BSP_Param_t* param){
    adc->Current = ((float)adc->Value - adc->Value_Bias) * param->Current_scale;
}

/**
 * @brief  ADC注入通道转换完成HAL回调函数
 * @param  hadc: ADC_HandleTypeDef指针，指向触发中断的ADC句柄
 * @note   该函数由HAL库在注入组转换完成中断中自动调用。
 *         通过硬件句柄反查软件ADC实例，若实例已注册用户回调，
 *         则转发调用，实现硬件中断到应用层的解耦。
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
    ADC_t *adc = ADC_GetInstance(hadc);
    if(adc != NULL){
        if(adc->InjectedConvCpltCallback != NULL){
            adc->InjectedConvCpltCallback(adc);
        }
    }
}

/**
 * @brief  ADC规则通道转换完成HAL回调函数
 * @param  hadc: ADC_HandleTypeDef指针，指向触发中断的ADC句柄
 * @note   该函数由HAL库在规则组转换完成中断中自动调用。
 *         通过硬件句柄反查软件ADC实例，若实例已注册用户回调，
 *         则转发调用，实现硬件中断到应用层的解耦。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    ADC_t *adc = ADC_GetInstance(hadc);
    if(adc != NULL){
        if(adc->ConvCpltCallback != NULL){
            adc->ConvCpltCallback(adc);
        }
    }
}