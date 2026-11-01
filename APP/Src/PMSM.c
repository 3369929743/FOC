#include "PMSM.h"

void PMSM_Init(PMSM_t *self, PMSM_PWM_Write_t PWM_Write, Encoder_Read_Data_t Read_Data, PID_Confg_t *PID_Id_Config, PID_Confg_t *PID_Iq_Config){
    /* 初始化PWM写入函数 */
    self->PWM_Write = PWM_Write;

    /* 初始化FOC配置参数 */
    FOC_Param_Init(&self->Foc, U_BUS, ARR);

    /* 初始化编码器参数 */
    Encoder_Param_Init(&self->Encoder, ENCODER_RES_CALCULATE, MOTOR_PN, ENCODER_DIR, ENCODER_ERROR);
    Encoder_Read_Data_Register(&self->Encoder, Read_Data);

    /* 初始化PID控制器 */
    PID_Init(&self->PID_Iq, PID_Iq_Config);
    PID_Init(&self->PID_Id, PID_Id_Config);

    /* 设置电流目标值 */
    PID_Set_Target(&self->PID_Iq, 0.0f);
    PID_Set_Target(&self->PID_Id, 0.0f);

    /* 初始化ADC参数 */
    My_ADC_BSP_Param_Init(&self->ADC_BSP_Param, ADC_CURRENT_GAIN, ADC_SHUNT_RES);
    My_ADC_Init(&self->ADC, ADC_1);
    My_ADC_Channel_Init(&self->ADC_IU, ADC_1);
    My_ADC_Channel_Init(&self->ADC_IV, ADC_1);
    My_ADC_Channel_Init(&self->ADC_IW, ADC_1);

    /* ADC自校准，补偿内部失调误差，必须在HAL_ADC_Init之后、启动转换前调用 */
    My_ADC_Calibration_Start(&self->ADC, ADC_SINGLE_ENDED);

    /* 初始化ADC注入模式 */
    My_ADC_InjectedStart_IT(&self->ADC);
}

static inline void PMSM_Calibration_Run(PMSM_t *self){
    uint8_t Calib_u, Calib_v, Calib_w;
    Calib_u = My_ADC_Bias_Calibreation_Start(&self->ADC_IU);
    Calib_v = My_ADC_Bias_Calibreation_Start(&self->ADC_IV);
    Calib_w = My_ADC_Bias_Calibreation_Start(&self->ADC_IW);
    if(Calib_u && Calib_v && Calib_w){
        self->State = STATE_FOC_RUNNING;
    }else{
        self->State = STATE_CALIBRATION;
    }
}

static inline void PMSM_FOC_Run(PMSM_t *self){
    /* 获取编码器角度 */
    Encoder_Update(&self->Encoder, &self->Foc.Theta);

    /* 计算cos_Theta和sin_Theta */
    FOC_Get_Cos_Sin(&self->Foc);

    /* 计算电流 */
    self->Foc.Iu = My_ADC_Calculate_Current(&self->ADC_IU, &self->ADC_BSP_Param);
    self->Foc.Iv = My_ADC_Calculate_Current(&self->ADC_IV, &self->ADC_BSP_Param);
    self->Foc.Iw = My_ADC_Calculate_Current(&self->ADC_IW, &self->ADC_BSP_Param);

    /* 计算Id和Iq,正向变换 */
    FOC_Conv_Update(&self->Foc);

    /* 更新Vq和Vd,电流PI闭环 */  
    self->Foc.Vq =  PID_Calculate(&self->PID_Iq, self->Foc.Iq);
    self->Foc.Vd = PID_Calculate(&self->PID_Id, self->Foc.Id);

    /* 反Park变换 */
    FOC_Rev_Conv_Update(&self->Foc);

    /* 更新PWM占空比 */
    self->PWM_Write(self->Foc.Tcmp1, self->Foc.Tcmp2, self->Foc.Tcmp3);
}

void PMSM_Run(PMSM_t *self){
    switch(self->State){
        case STATE_IDLE:
            break;
        case STATE_CALIBRATION:
            PMSM_Calibration_Run(self);
            break;  
        case STATE_FOC_RUNNING:
            PMSM_FOC_Run(self);
            break;
        }
}

void PMSM_IDLE_Mode(PMSM_t *self){
    self->State = STATE_IDLE;
}

void PMSM_CALIBRATION_Mode(PMSM_t *self){
    self->State = STATE_CALIBRATION;
}

void PMSM_FOC_RUNNING_Mode(PMSM_t *self){
    self->State = STATE_FOC_RUNNING;
}
