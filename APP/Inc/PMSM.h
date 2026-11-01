#ifndef __PMSM_H__
#define __PMSM_H__

/* =============== 算法头文件 ================== */
#include "FOC_math.h"
#include "PID.h"
#include "Encoder.h"
#include "MyADC.h"
#include "Timer.h"

/* =============== 电机硬件配置头文件 ================== */
#include "Motor_FOC_Config.h"

typedef enum{
    PMSM_OK = 0x0U,
    PMSM_ERROR = 0x1U,
}PMSM_Status_e;

typedef enum{
    STATE_IDLE = 0,
    STATE_CALIBRATION,
    STATE_FOC_RUNNING,
} PMSM_Mode_e;

typedef struct {
    /* 编码器对象 */
    Encoder_t Encoder;

    /* ADC对象 */
    ADC_t ADC;
    /* ADC通道 */
    ADC_Channel_t ADC_IU_Ch, ADC_IV_Ch, ADC_IW_Ch;
    /* ADC配置参数 */
    ADC_BSP_Param_t ADC_BSP_Param;

    /* 定时器对象 */
    Timer_t Timer_FOC;
    /* 定时器通道 */
    uint32_t Timer_Channel_Vu, Timer_Channel_Vv, Timer_Channel_Vw, Timer_Channel_ADC;

    /* FOC对象 */
    FOC_t Foc;

    /* PID对象 */
    PID_t PID_Iq, PID_Id;

    /* 电机状态 */
    PMSM_Mode_e Mode;
} PMSM_t;

typedef struct {
    /* ========== 函数指针 ============== */
    Encoder_Read_Data_t Read_Data;
    PID_Confg_t *PID_Id_Config;
    PID_Confg_t *PID_Iq_Config;

    /* ========== 硬件配置 ============== */
    ADC_Num_e ADC_Num;                                        //ADC编号，ADC_1或ADC_2
    Timer_Num_e Timer_Num;                                        //定时器编号，Timer_1或Timer_2
    
    /* ========== 其他配置参数 ============== */
    /* 电机参数 */
    float Ubus;                                             //母线电压
    float TARR;                                             //自动重装值
    /* 编码器参数 */
    uint32_t Encoder_Res;                                   //编码器分辨率
    uint32_t Motor_Pn;                                      //电机极对数
    uint32_t Encoder_Dir;                                   //编码器方向
    uint32_t Encoder_Error;                                 //编码器误差补偿
    /* ADC参数 */
    float ADC_Current_Gain;                                 //运放增益
    float ADC_Shunt_Res;                                    //采样电阻
    uint32_t SingleDiff;                                    //ADC采样模式选择，ADC_SINGLE_ENDED（单端）或 ADC_DIFFERENTIAL_ENDED（差分）
    /* 定时器参数 */
    uint32_t Timer_Cmp_Trigger_ADC;                            //定时器比较值，用于触发ADC采样
    uint32_t Timer_Channel_Vu;                              //定时器通道，TIM_CHANNEL_1、TIM_CHANNEL_2或TIM_CHANNEL_4（PWM通道4）
    uint32_t Timer_Channel_Vv;                              //定时器通道，TIM_CHANNEL_1、TIM_CHANNEL_2或TIM_CHANNEL_4（PWM通道4）
    uint32_t Timer_Channel_Vw;                              //定时器通道，TIM_CHANNEL_1、TIM_CHANNEL_2或TIM_CHANNEL_4（PWM通道4）
    uint32_t Timer_Channel_ADC;                             //定时器通道，TIM_CHANNEL_1、TIM_CHANNEL_2或TIM_CHANNEL_4（PWM通道4）
} PMSM_Config_t;

/* 初始化PMSM电机 */
PMSM_Status_e PMSM_Init(PMSM_t *self, PMSM_Config_t *config);
/* 运行PMSM电机 */
void PMSM_Run(PMSM_t *self);

/* 切换PMSM电机模式 */
void PMSM_IDLE_Mode(PMSM_t *self);
void PMSM_CALIBRATION_Mode(PMSM_t *self);
void PMSM_FOC_RUNNING_Mode(PMSM_t *self);             //校准后会自动运行，一般不用该API


#endif