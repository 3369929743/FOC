#include "PMSM.h"
#include "Timer.h"

/**
 * @brief   PMSM 电机初始化
 * @param   self   PMSM 对象指针，用于存储初始化后的外设句柄和状态
 * @param   config PMSM 配置参数指针，包含电机硬件参数和外设配置
 * @retval  PMSM_OK    初始化成功，可进入校准或运行模式
 * @retval  PMSM_ERROR 初始化失败，参数无效或外设初始化异常
 * @note    初始化顺序严格按依赖关系排列：
 *          FOC参数 -> 编码器 -> PID -> ADC -> 定时器 -> PWM输出
 *          每个模块初始化前都会进行参数有效性检查。
 *          初始化完成后电机处于 STATE_IDLE 空闲状态，需调用模式切换函数启动。
 */
PMSM_Status_e PMSM_Init(PMSM_t *self, PMSM_Config_t *config){
    /* 空指针检查，防止后续操作访问非法内存 */
    if(self == NULL || config == NULL){
        return PMSM_ERROR;
    }

    /* 检查FOC核心参数：母线电压决定电压利用率上限，TARR决定PWM分辨率 */
    if(config->Ubus == 0.0f || config->TARR == 0.0f){
        return PMSM_ERROR;
    }
    /* 初始化FOC配置参数：计算电压基准值、SVPWM相关常量等 */
    FOC_Param_Init(&self->Foc, config->Ubus, config->TARR);

    /* 检查编码器参数：Read_Data为数据读取回调，极对数和分辨率用于角度解算 */
    if(config->Read_Data == NULL){
        return PMSM_ERROR;
    }
    else if(config->Encoder_Res == 0 || config->Motor_Pn == 0){
        return PMSM_ERROR;
    }
    /* 初始化编码器：设置分辨率、极对数、方向和误差补偿，注册数据读取函数 */
    Encoder_Param_Init(&self->Encoder, config->Encoder_Res, config->Motor_Pn, config->Encoder_Dir, config->Encoder_Error);
    Encoder_Read_Data_Register(&self->Encoder, config->Read_Data);

    /* 检查PID参数：配置指针不得为空，输出限幅OutMax为0会导致PI无输出 */
    if(config->PID_Iq_Config == NULL || config->PID_Id_Config == NULL){
        return PMSM_ERROR;
    }
    else if(config->PID_Iq_Config->OutMax == 0.0f || config->PID_Id_Config->OutMax == 0.0f){
        return PMSM_ERROR;
    }
    /* 初始化PID控制器：Id环和Iq环各一个PI控制器 */
    PID_Init(&self->PID_Iq, config->PID_Iq_Config);
    PID_Init(&self->PID_Id, config->PID_Id_Config);

    /* 设置电流目标值为0，初始状态下不输出力矩，安全启动 */
    PID_Set_Target(&self->PID_Iq, 0.0f);
    PID_Set_Target(&self->PID_Id, 0.0f);

    /* 检查ADC参数：编号不能为无效值，运放增益和采样电阻用于电流换算 */
    if(config->ADC_Num == ADC_NUM_INVALID){
        return PMSM_ERROR;
    }
    else if(config->ADC_Current_Gain == 0.0f || config->ADC_Shunt_Res == 0.0f){
        return PMSM_ERROR;
    }
    /* 初始化ADC BSP参数：保存运放增益和采样电阻，用于后续电流值计算 */
    My_ADC_BSP_Param_Init(&self->ADC_BSP_Param, config->ADC_Current_Gain, config->ADC_Shunt_Res);
    /* 初始化ADC外设：配置ADC时钟、分辨率等底层硬件 */
    if(My_ADC_Init(&self->ADC, config->ADC_Num) != PMSM_OK){
        return PMSM_ERROR;
    }

    /* 检查定时器参数：编号不能为无效值 */
    if(config->Timer_Num == TIMER_NUM_INVALID){
        return PMSM_ERROR;
    }
    /* 初始化定时器：配置定时器时钟、计数模式，用于生成PWM和触发ADC */
    if(Timer_Init(&self->Timer_FOC, config->Timer_Num) != TIMER_OK){
        return PMSM_ERROR;
    }

    /* 初始化三相电流ADC通道（U、V、W），用于电机相电流采样 */
    My_ADC_Channel_Init(&self->ADC_IU_Ch, config->ADC_Num);
    My_ADC_Channel_Init(&self->ADC_IV_Ch, config->ADC_Num);
    My_ADC_Channel_Init(&self->ADC_IW_Ch, config->ADC_Num);

    /* ADC自校准：补偿内部失调误差，消除ADC零点漂移，必须在HAL_ADC_Init之后、启动转换前调用 */
    My_ADC_Calibration_Start(&self->ADC, config->SingleDiff);

    /* 启动ADC注入组中断转换：ADC由定时器触发，在下桥臂导通中点采样相电流 */
    My_ADC_InjectedStart_IT(&self->ADC);
    
    /* 保存定时器通道编号到对象，供后续PWM更新时使用 */
    self->Timer_Channel_ADC = config->Timer_Channel_ADC;
    self->Timer_Channel_Vu = config->Timer_Channel_Vu;
    self->Timer_Channel_Vv = config->Timer_Channel_Vv;
    self->Timer_Channel_Vw = config->Timer_Channel_Vw;
    /* 设置ADC触发比较值：控制在PWM周期中哪个时刻触发ADC采样（通常在下桥臂中点） */
    Timer_Set_CompareValue(&self->Timer_FOC, config->Timer_Cmp_Trigger_ADC, config->Timer_Channel_ADC);
    /* 启动三相互补PWM输出（U、V、W）：上下桥臂互补带死区，驱动三相逆变器 */
    Timer_Comple_PWM_Start(&self->Timer_FOC, self->Timer_Channel_Vu);
    Timer_Comple_PWM_Start(&self->Timer_FOC, self->Timer_Channel_Vv);
    Timer_Comple_PWM_Start(&self->Timer_FOC, self->Timer_Channel_Vw);
    /* 启动ADC触发用PWM通道：该通道不驱动功率管，仅用于产生ADC采样触发信号 */
    Timer_PWM_Start(&self->Timer_FOC, self->Timer_Channel_ADC);

    /* 初始化为空闲状态，等待用户调用模式切换API启动电机 */
    self->Mode = STATE_IDLE;

    return PMSM_OK;
}

/**
 * @brief   电流 ADC 偏置校准状态机（静态内联）
 * @param   self PMSM 对象指针
 * @note    对 U、V、W 三相电流 ADC 通道分别执行偏置校准。
 *          每次调用尝试对所有通道采样并计算偏置值，耗时较短可多次迭代。
 *          三相全部校准完成后自动切换到 STATE_FOC_RUNNING，
 *          否则保持在 STATE_CALIBRATION 状态等待下次调用继续校准。
 *          该函数由 PMSM_Run() 在 STATE_CALIBRATION 状态下循环调用。
 */
static inline void PMSM_Calibration_Run(PMSM_t *self){
    uint8_t Calib_u, Calib_v, Calib_w;
    /* 分别对 U、V、W 三相 ADC 通道进行偏置校准，返回非0表示该相校准完成 */
    Calib_u = My_ADC_Bias_Calibreation_Start(&self->ADC_IU_Ch);
    Calib_v = My_ADC_Bias_Calibreation_Start(&self->ADC_IV_Ch);
    Calib_w = My_ADC_Bias_Calibreation_Start(&self->ADC_IW_Ch);
    /* 三相全部校准完成后，自动切换到 FOC 运行状态 */
    if(Calib_u && Calib_v && Calib_w){
        self->Mode = STATE_FOC_RUNNING;
    }else{
        /* 仍有通道未完成校准，继续保持校准状态 */
        self->Mode = STATE_CALIBRATION;
    }
}

/**
 * @brief   FOC 主控制循环（静态内联）
 * @param   self PMSM 对象指针
 * @note    FOC 完整控制流程（6步）：
 *          1. 读取编码器角度 Theta
 *          2. 计算 cos_Theta / sin_Theta（后续 Park 变换用）
 *          3. 读取三相电流 Iu/Iv/Iw（下桥臂采样，需按实际硬件方向取反）
 *          4. Clarke + Park 正向变换，将三相电流转换为 Id、Iq
 *          5. 反 Park 变换（内部通过回调完成 PI 闭环计算 Vd/Vq，再生成三相比较值 Tcmp1/2/3）
 *          6. 将 Tcmp 写入定时器比较寄存器，更新 PWM 占空比输出
 *          该函数由 PMSM_Run() 在 STATE_FOC_RUNNING 状态下循环调用，
 *          通常放在定时器中断或主循环中，调用频率等于 PWM 频率。
 */
static inline void PMSM_FOC_Run(PMSM_t *self){
    /* 获取编码器角度 Theta，用于后续坐标变换 */
    Encoder_Update(&self->Encoder, &self->Foc.Theta);

    /* 计算 cos_Theta 和 sin_Theta，供 Park/反Park 变换使用 */
    FOC_Get_Cos_Sin(&self->Foc);

    /* 读取三相电流，将 ADC 原始值换算为实际电流值（安培）
     * 注意：若下桥臂采样方向与 FOC 定义不一致，需要在此取反 */
    self->Foc.Iu = My_ADC_Calculate_Current(&self->ADC_IU_Ch, &self->ADC_BSP_Param);
    self->Foc.Iv = My_ADC_Calculate_Current(&self->ADC_IV_Ch, &self->ADC_BSP_Param);
    self->Foc.Iw = My_ADC_Calculate_Current(&self->ADC_IW_Ch, &self->ADC_BSP_Param);

    /* Clarke + Park 正向变换，将三相静止坐标系电流变换为旋转坐标系 Id、Iq */
    FOC_Conv_Update(&self->Foc);

    // /* 更新Vq和Vd,电流PI闭环 */  
    // self->Foc.Vq =  PID_Calculate(&self->PID_Iq, self->Foc.Iq);
    // self->Foc.Vd = PID_Calculate(&self->PID_Id, self->Foc.Id);

    FOC_Rev_Conv_Update(&self->Foc);

    /*将三相比较值写入定时器 CCR 寄存器，更新 PWM 占空比 */
    Timer_Set_CompareValue(&self->Timer_FOC, self->Foc.Tcmp1, self->Timer_Channel_Vu);
    Timer_Set_CompareValue(&self->Timer_FOC, self->Foc.Tcmp2, self->Timer_Channel_Vv);
    Timer_Set_CompareValue(&self->Timer_FOC, self->Foc.Tcmp3, self->Timer_Channel_Vw);
}

/**
 * @brief   PMSM 电机主运行函数（状态分发器）
 * @param   self PMSM 对象指针
 * @note    根据当前状态 Mode 分发到对应的处理函数：
 *          - STATE_IDLE:          空闲，不执行任何操作（PWM 保持但不再更新）
 *          - STATE_CALIBRATION:   调用 PMSM_Calibration_Run() 执行 ADC 偏置校准
 *          - STATE_FOC_RUNNING:   调用 PMSM_FOC_Run() 执行 FOC 主控制循环
 *          该函数应被周期性调用（放于定时器中断回调或主循环中），
 *          调用频率需等于 PWM 频率以保证 FOC 实时性。
 */
void PMSM_Run(PMSM_t *self){
    switch(self->Mode){
        case STATE_IDLE:
            /* 空闲：不执行任何控制操作 */
            break;
        case STATE_CALIBRATION:
            /* 校准：逐次完成三相 ADC 偏置校准，全部完成后自动切换到 FOC_RUNNING */
            PMSM_Calibration_Run(self);
            break;  
        case STATE_FOC_RUNNING:
            /* FOC 运行：执行完整的电流环控制（采样 -> 变换 -> PI闭环 -> SVPWM输出） */
            PMSM_FOC_Run(self);
            break;
        }
}

/**
 * @brief   将电机切换到空闲模式
 * @param   self PMSM 对象指针
 * @note    Mode 设为 STATE_IDLE 后，PMSM_Run() 不再执行任何控制逻辑，
 *          PWM 持续输出但占空比不再更新。
 */
void PMSM_IDLE_Mode(PMSM_t *self){
    self->Mode = STATE_IDLE;
}

/**
 * @brief   将电机切换到校准模式
 * @param   self PMSM 对象指针
 * @note    进入校准模式后，PMSM_Run() 会循环调用 ADC 偏置校准，
 *          三相全部校准完成后自动切换到 FOC_RUNNING 模式。
 */
void PMSM_CALIBRATION_Mode(PMSM_t *self){
    self->Mode = STATE_CALIBRATION;
}

/**
 * @brief   将电机切换到 FOC 运行模式
 * @param   self PMSM 对象指针
 * @note    通常无需直接调用此 API，校准完成后会自动切换到该模式。
 *          若电机已校准过（如热启动），可直接调用跳过校准步骤。
 */
void PMSM_FOC_RUNNING_Mode(PMSM_t *self){
    self->Mode = STATE_FOC_RUNNING;
}