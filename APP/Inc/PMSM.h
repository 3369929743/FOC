#ifndef __PMSM_H__
#define __PMSM_H__

/* =============== 算法头文件 ================== */
#include "FOC_math.h"
#include "PID.h"
#include "Encoder.h"
#include "MyADC.h"

/* =============== 电机硬件配置头文件 ================== */
#include "Motor_FOC_Config.h"

typedef void (*PMSM_PWM_Write_t)(uint32_t u, uint32_t v, uint32_t w);

typedef enum{
    STATE_IDLE = 0,
    STATE_CALIBRATION,
    STATE_FOC_RUNNING,
} PMSM_State_e;

typedef struct {
    /* 编码器对象 */
    Encoder_t Encoder;

    /* ADC对象 */
    ADC_t ADC;
    /* ADC通道 */
    ADC_Channel_t ADC_IU, ADC_IV, ADC_IW;
    /* ADC配置参数 */
    ADC_BSP_Param_t ADC_BSP_Param;

    /* FOC对象 */
    FOC_t Foc;

    /* PID对象 */
    PID_t PID_Iq, PID_Id;

    /* 电机状态 */
    PMSM_State_e State;
    
    /* PWM写入函数 */
    PMSM_PWM_Write_t PWM_Write;
} PMSM_t;

/* 初始化PMSM电机 */
void PMSM_Init(PMSM_t *self, PMSM_PWM_Write_t PWM_Write, Encoder_Read_Data_t Read_Data, PID_Confg_t *PID_Id_Config, PID_Confg_t *PID_Iq_Config);
/* 运行PMSM电机 */
void PMSM_Run(PMSM_t *self);

/* 切换PMSM电机模式 */
void PMSM_IDLE_Mode(PMSM_t *self);
void PMSM_CALIBRATION_Mode(PMSM_t *self);
void PMSM_FOC_RUNNING_Mode(PMSM_t *self);             //校准后会自动运行，一般不用该API


#endif