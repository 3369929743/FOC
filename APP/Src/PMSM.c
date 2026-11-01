#include "PMSM.h"
#include "Timer.h"

PMSM_Status_e PMSM_Init(PMSM_t *self, PMSM_Config_t *config){
    if(self == NULL || config == NULL){
        return PMSM_ERROR;
    }

    if(config->Ubus == 0.0f || config->TARR == 0.0f){
        return PMSM_ERROR;
    }
    /* 初始化FOC配置参数 */
    FOC_Param_Init(&self->Foc, config->Ubus, config->TARR);

    if(config->Read_Data == NULL){
        return PMSM_ERROR;
    }
    else if(config->Encoder_Res == 0 || config->Motor_Pn == 0){
        return PMSM_ERROR;
    }
    /* 初始化编码器参数 */
    Encoder_Param_Init(&self->Encoder, config->Encoder_Res, config->Motor_Pn, config->Encoder_Dir, config->Encoder_Error);
    Encoder_Read_Data_Register(&self->Encoder, config->Read_Data);

    if(config->PID_Iq_Config == NULL || config->PID_Id_Config == NULL){
        return PMSM_ERROR;
    }
    else if(config->PID_Iq_Config->OutMax == 0.0f || config->PID_Id_Config->OutMax == 0.0f){
        return PMSM_ERROR;
    }
    /* 初始化PID控制器 */
    PID_Init(&self->PID_Iq, config->PID_Iq_Config);
    PID_Init(&self->PID_Id, config->PID_Id_Config);

    /* 设置电流目标值 */
    PID_Set_Target(&self->PID_Iq, 0.0f);
    PID_Set_Target(&self->PID_Id, 0.0f);

    if(config->ADC_Num == ADC_NUM_INVALID){
        return PMSM_ERROR;
    }
    else if(config->ADC_Current_Gain == 0.0f || config->ADC_Shunt_Res == 0.0f){
        return PMSM_ERROR;
    }
    /* 初始化ADC参数 */
    My_ADC_BSP_Param_Init(&self->ADC_BSP_Param, config->ADC_Current_Gain, config->ADC_Shunt_Res);
    if(My_ADC_Init(&self->ADC, config->ADC_Num) != PMSM_OK){
        return PMSM_ERROR;
    }
    if(config->Timer_Num == TIMER_NUM_INVALID){
        return PMSM_ERROR;
    }
    /* 初始化定时器 */
    if(Timer_Init(&self->Timer_FOC, config->Timer_Num) != TIMER_OK){
        return PMSM_ERROR;
    }
    My_ADC_Channel_Init(&self->ADC_IU_Ch, config->ADC_Num);
    My_ADC_Channel_Init(&self->ADC_IV_Ch, config->ADC_Num);
    My_ADC_Channel_Init(&self->ADC_IW_Ch, config->ADC_Num);

    /* ADC自校准，补偿内部失调误差，必须在HAL_ADC_Init之后、启动转换前调用 */
    My_ADC_Calibration_Start(&self->ADC, config->SingleDiff);

    /* 初始化ADC注入模式 */
    My_ADC_InjectedStart_IT(&self->ADC);
    
    
    /* 设置定时器比较值 */
    self->Timer_Channel_ADC = config->Timer_Channel_ADC;
    self->Timer_Channel_Vu = config->Timer_Channel_Vu;
    self->Timer_Channel_Vv = config->Timer_Channel_Vv;
    self->Timer_Channel_Vw = config->Timer_Channel_Vw;
    Timer_Set_CompareValue(&self->Timer_FOC, config->Timer_Cmp_Trigger_ADC, config->Timer_Channel_ADC);
    Timer_Comple_PWM_Start(&self->Timer_FOC, self->Timer_Channel_Vu);
    Timer_Comple_PWM_Start(&self->Timer_FOC, self->Timer_Channel_Vv);
    Timer_Comple_PWM_Start(&self->Timer_FOC, self->Timer_Channel_Vw);
    Timer_PWM_Start(&self->Timer_FOC, self->Timer_Channel_ADC);

    self->Mode = STATE_IDLE;

    return PMSM_OK;
}

static inline void PMSM_Calibration_Run(PMSM_t *self){
    uint8_t Calib_u, Calib_v, Calib_w;
    Calib_u = My_ADC_Bias_Calibreation_Start(&self->ADC_IU_Ch);
    Calib_v = My_ADC_Bias_Calibreation_Start(&self->ADC_IV_Ch);
    Calib_w = My_ADC_Bias_Calibreation_Start(&self->ADC_IW_Ch);
    if(Calib_u && Calib_v && Calib_w){
        self->Mode = STATE_FOC_RUNNING;
    }else{
        self->Mode = STATE_CALIBRATION;
    }
}

static inline void PMSM_FOC_Run(PMSM_t *self){
    /* 获取编码器角度 */
    Encoder_Update(&self->Encoder, &self->Foc.Theta);

    /* 计算cos_Theta和sin_Theta */
    FOC_Get_Cos_Sin(&self->Foc);

    /* 计算电流 */
    self->Foc.Iu = My_ADC_Calculate_Current(&self->ADC_IU_Ch, &self->ADC_BSP_Param);
    self->Foc.Iv = My_ADC_Calculate_Current(&self->ADC_IV_Ch, &self->ADC_BSP_Param);
    self->Foc.Iw = My_ADC_Calculate_Current(&self->ADC_IW_Ch, &self->ADC_BSP_Param);

    /* 计算Id和Iq,正向变换 */
    FOC_Conv_Update(&self->Foc);

    /* 更新Vq和Vd,电流PI闭环 */  
    self->Foc.Vq =  PID_Calculate(&self->PID_Iq, self->Foc.Iq);
    self->Foc.Vd = PID_Calculate(&self->PID_Id, self->Foc.Id);

    /* 反Park变换 */
    FOC_Rev_Conv_Update(&self->Foc);

    /* 更新PWM占空比 */
    Timer_Set_CompareValue(&self->Timer_FOC, self->Foc.Tcmp1, self->Timer_Channel_Vu);
    Timer_Set_CompareValue(&self->Timer_FOC, self->Foc.Tcmp2, self->Timer_Channel_Vv);
    Timer_Set_CompareValue(&self->Timer_FOC, self->Foc.Tcmp3, self->Timer_Channel_Vw);
}

void PMSM_Run(PMSM_t *self){
    switch(self->Mode){
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
    self->Mode = STATE_IDLE;
}

void PMSM_CALIBRATION_Mode(PMSM_t *self){
    self->Mode = STATE_CALIBRATION;
}

void PMSM_FOC_RUNNING_Mode(PMSM_t *self){
    self->Mode = STATE_FOC_RUNNING;
}
