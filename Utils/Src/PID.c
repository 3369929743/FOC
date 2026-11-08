/**
 * @file PID.c
 * @brief 位置式 PID，支持浮点计算和整数接口的定点计算。
 *
 * 使用约定：
 * 1. 在 PID.h 中设置 PID_USE_FLOAT：1 为浮点模式，0 为定点模式。
 *    PID_Confg_t 始终用 float 配置；Ts 的单位为秒，调用周期应与 Ts 一致。
 *    配置填写连续形式的 Kp、Ki、Kd，内部自动换算 Ki*Ts 和 Kd/Ts。
 *    PID_Init 成功后才能使用，调用方须检查返回值是否为 PID_OK。
 *
 * 2. 浮点模式：目标、反馈、输出直接使用 float，死区与误差使用相同单位。
 *    定点模式：目标、反馈、输出使用 int32_t，不在接口内自动缩放信号。
 *    调用方自行约定整数单位；若 255 代表实际值 25.5，则反馈也按该单位传入。
 *    积分限幅和输出限幅填写对应输出整数单位的值，定点模式下小数会截断。
 *    PID_SIGNAL_SCALE 当前不参与计算，仅定义该宏不会自动转换任何输入输出。
 *    Kp、Ki、Kd 须按所选输入/输出单位配置，输入输出换用不同倍率时须换算增益。
 *
 * 3. 定点系数和 Alpha 在内部放大 PID_COEFF_SCALE 倍，调用方不要重复放大。
 *    Integral_Zoom 保留该倍率的积分累计值，合成输出后才还原系数倍率。
 *    定点模式读取积分应换算 Integral_Zoom，Integral 成员仅在浮点模式更新。
 *    非零 Ki 若在初始化时量化为零，PID_Init 返回 PID_ERROR；Ki=0 允许关闭积分。
 *    选择系数小数位时需兼顾精度和范围，并保证输入、转换及中间运算不溢出。
 *
 * 4. Error_Deadband 可以配置小数，单位与传入的整数误差相同。
 *    定点模式内部同时放大死区和比较用的误差，以保存小数阈值。
 *    例如死区 1.5：整数误差 +/-1 归零，+/-2 正常计算。
 *    若整数 1 代表实际值 0.1，则死区 1.5 代表实际值 0.15。
 *    判断使用严格不等式，边界仍参与计算；死区为 0 表示关闭，负值初始化报错。
 *    死区内 P 项为零、积分保持已有值，输出不一定为零；D 项仍可能有过渡响应。
 *
 * 5. Alpha=0 按 1 处理，表示关闭微分低通滤波；有效滤波系数应在 (0, 1] 内。
 *    IntMax/OutMax=0 使用对应类型的最大值，IntMin/OutMin=0 使用对称负限幅。
 *    PID_Clear 和 PID_Change_Param 清除误差、积分、输出及滤波状态，保留目标和反馈。
 *
 * 定点模式示例（输入输出均使用调用方约定的整数单位）：
 *     PID_t pid = {0};
 *     PID_Confg_t config = {
 *         .Ts = 0.01f, .Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0f,
 *         .OutMax = 100.0f, .Error_Deadband = 1.5f
 *     };
 *     if (PID_Init(&pid, &config) != PID_OK) {
 *         Error_Handler();  // 由调用方提供错误处理，失败后不继续使用该实例
 *     }
 *     PID_Set_Target(&pid, 100);
 *     PID_val out1 = PID_Calculate(&pid, 99);  // 误差 1，在死区内，输出 0
 *     PID_val out2 = PID_Calculate(&pid, 98);  // 误差 2，在死区外，输出 2
 */
#include "PID.h"
#include <limits.h>
#include <float.h>

/**
  * @brief  PID控制器初始化函数
  * @param  PID: PID控制器结构体指针，用于存储PID参数和状态
  * @param  Config: PID配置参数结构体指针，包含Kp/Ki/Kd增益系数、积分/输出限幅值及滤波系数
  * @retval 无
  * @note   - 根据PID_USE_FLOAT宏选择浮点型或定点型初始化方式
  *         - 当IntMin/OutMin为0时，自动设置为对应Max的负值（对称限幅）
  *         - 当Alpha为0时，默认设置为1.0（不进行滤波）
  *         - 初始化完成后自动调用PID_Clear清除所有状态变量
  * @warning 调用此函数会清除PID的历史状态（误差、积分、输出等）
  */
/**
  * @brief  PID控制器初始化
  * @param  PID: PID控制器结构体指针
  * @param  Config: PID配置参数(Kp/Ki/Kd/限幅/滤波系数)
  * @retval 无
  * @note   初始化后自动清零状态
  */
PID_Status_t PID_Init(PID_t *PID, PID_Confg_t *Config)
{
    /* 滤波系数修正 */
    if(Config->Alpha == 0)
    {
        Config->Alpha = 1.0f;
    }
    if(Config->Ts <= 0)
    {
        return PID_ERROR;  /* 采样时间不能为0，避免除以零错误 */
    }
    PID->Ts = Config->Ts;
    
#if PID_USE_FLOAT
    /* 浮点模式 */
    PID->Kp = Config->Kp;
    PID->Ki = Config->Ki * Config->Ts;
    PID->Kd = Config->Kd / Config->Ts;
    PID->IntMax = (Config->IntMax == 0) ? FLT_MAX : Config->IntMax;
    PID->IntMin = (Config->IntMin == 0) ? -PID->IntMax : Config->IntMin;
    PID->OutMax = (Config->OutMax == 0) ? FLT_MAX : Config->OutMax;
    PID->OutMin = (Config->OutMin == 0) ? -PID->OutMax : Config->OutMin;
    PID->Alpha = Config->Alpha;
    if(Config->Error_Deadband < 0) return PID_ERROR;
    PID->Error_Deadband = Config->Error_Deadband;
#else
    /* 定点模式: 放大转整型 */
    PID->Kp = (PID_val)(Config->Kp * PID_COEFF_SCALE);
    PID->Ki = (PID_val)((Config->Ki * Config->Ts) * PID_COEFF_SCALE);
    /* 积分系数量化为零：需调整定点精度，并重新检查系数范围和乘法溢出风险 */
    if(Config->Ki != 0 && PID->Ki == 0) return PID_ERROR;
    PID->Kd = (PID_val)((Config->Kd / Config->Ts) * PID_COEFF_SCALE);
    PID->IntMax = (Config->IntMax == 0) ? INT32_MAX : Config->IntMax;
    PID->IntMin = (Config->IntMin == 0) ? -PID->IntMax : Config->IntMin;
    PID->OutMax = (Config->OutMax == 0) ? INT32_MAX : Config->OutMax;
    PID->OutMin = (Config->OutMin == 0) ? -PID->OutMax : Config->OutMin;
    PID->Alpha = (PID_val)(Config->Alpha * PID_COEFF_SCALE);
    if(Config->Error_Deadband < 0) return PID_ERROR;
    PID->Error_Deadband = (PID_val)(Config->Error_Deadband * PID_COEFF_SCALE);
#endif
    
    /* 清零状态 */
    PID_Clear(PID);

    return PID_OK;  /* 初始化成功 */
}

/**
  * @brief  清除PID控制器状态
  * @param  PID: PID控制器结构体指针
  * @retval 无
  * @note   - 将所有误差、积分、输出及目标/实际值清零
  *         - 用于初始化、复位PID控制器或切换控制模式后清除旧状态
  * @warning 调用此函数会丢失所有历史状态信息，建议在修改PID参数后调用
  */
void PID_Clear(PID_t *PID)
{
    PID->Integral = 0;           /* 累积误差（积分项）清零：消除历史误差累积 */
    PID->Pre_Error = 0;          /* 上一次误差清零：清除微分项的历史参考值 */
    PID->Cur_Error = 0;          /* 当前误差清零：确保下次计算从零开始 */
    PID->Output = 0;             /* PID输出值清零：复位控制量输出 */
    PID->Error_Rate_Filter = 0;  /* 误差变化率滤波值清零：复位微分项低通滤波器状态 */

#if !PID_USE_FLOAT
    PID->Integral_Zoom = 0;  /* 累积误差（积分项）清零：消除历史误差累积 */
#endif
}

#if !PID_USE_FLOAT
/**
  * @brief  定点数还原函数（将放大后的定点数转换回原始值）
  * @param  x: 放大后的定点数值（int64_t类型防止中间计算溢出）
  * @retval 还原后的整型数值
  * @note   - 采用四舍五入方式右移还原，保留原始符号
  *         - 四舍五入原理：加上缩放因子的一半(PID_COEFF_SCALE/2)后再右移，实现四舍五入效果
  *         - 使用int64_t防止乘法运算时溢出
  * @warning 仅在定点型模式(!PID_USE_FLOAT)下使用
  */
static inline PID_val PID_Restore(int64_t x)
{
    /* 取输入值的绝对值：用于统一处理正负数的四舍五入运算 */
    int64_t abs_x = x > 0 ? x : -x;
    
    /* 四舍五入还原：
     * 1. abs_x + PID_COEFF_SCALE/2：加上缩放因子的一半，实现四舍五入
     * 2. >> MULTIPLE_COEFF：右移还原（等价于除以PID_COEFF_SCALE）
     * 3. 根据原符号返回正负值：保持原始数值的符号
     */
    return (x > 0) ? ((abs_x + PID_COEFF_SCALE / 2) >> MULTIPLE_COEFF) : -((abs_x + PID_COEFF_SCALE / 2) >> MULTIPLE_COEFF);
}
#endif

/**
  * @brief  设置PID目标值
  * @param  PID: PID控制器结构体指针
  * @param  Target: 目标设定值（期望系统达到的值）
  * @retval 无
  * @note   目标值是PID控制的期望值，PID会根据目标值与实际值的差值进行调节
  */
void PID_Set_Target(PID_t *PID, PID_val Target)
{
    PID->Target = Target;  /* 更新目标设定值：作为PID控制的期望参考值 */
}

/**
  * @brief  设置PID实际值（反馈值）
  * @param  PID: PID控制器结构体指针
  * @param  Actual: 实际测量值（传感器采集的系统当前状态值）
  * @retval 无
  * @note   实际值是系统的当前反馈值，PID会根据目标值与实际值的差值（误差）进行调节
  */
void PID_Set_Actual(PID_t *PID, PID_val Actual)
{
    PID->Actual = Actual;  /* 更新实际反馈值：来自传感器或编码器的测量数据 */
}

/**
  * @brief  PID控制器核心更新函数（执行PID运算）
  * @param  PID: PID控制器结构体指针
  * @retval 无
  * @note   - 包含误差计算、积分限幅、微分项低通滤波、PID公式运算及输出限幅
  *         - 微分项采用一阶低通滤波，抑制高频噪声
  *         - 积分项和输出项均有限幅保护，防止积分饱和和输出超限
  * @warning 此函数为静态内部函数，需配合PID_Set_Target/PID_Set_Actual使用
  */
static void PID_Update(PID_t *PID)
{
#if PID_USE_FLOAT

    PID->Pre_Error = PID->Cur_Error;                                    /* 保存上一次误差：用于计算误差变化率（微分项） */
    PID->Cur_Error = PID->Target - PID->Actual;                         /* 计算当前误差：目标值 - 实际值，PID控制的核心输入 */
    if(PID->Cur_Error < PID->Error_Deadband && PID->Cur_Error > -PID->Error_Deadband){
        PID->Cur_Error = 0;
    }

    PID->Integral += PID->Ki * PID->Cur_Error;                                    /* 累积误差（积分项）：累加历史误差，用于消除稳态误差 */
    /* ========== 第二步：积分限幅（防止积分饱和） ========== */
    /* 积分饱和现象：当系统长时间存在误差时，积分项会不断累积导致输出过大
     * 限幅作用：限制积分项的最大/最小值，避免系统响应过冲或失控 */
    if (PID->Integral > PID->IntMax)
    {
        PID->Integral = PID->IntMax;                                    /* 积分项超过上限：钳位到最大值 */
    }
    else if (PID->Integral < PID->IntMin)
    {
        PID->Integral = PID->IntMin;                                    /* 积分项低于下限：钳位到最小值 */
    }
#else

    PID->Pre_Error = PID->Cur_Error;                                    /* 保存上一次误差：用于计算误差变化率（微分项） */
    PID->Cur_Error = PID->Target - PID->Actual;                         /* 计算当前误差：目标值 - 实际值，PID控制的核心输入 */
    if(((int64_t)PID->Cur_Error * PID_COEFF_SCALE) < PID->Error_Deadband && ((int64_t)PID->Cur_Error * PID_COEFF_SCALE) > -PID->Error_Deadband){
        PID->Cur_Error = 0;
    }

    PID->Integral_Zoom += (int64_t)PID->Ki * PID->Cur_Error;
    if(PID->Integral_Zoom > (int64_t)PID->IntMax * PID_COEFF_SCALE)
    {
        PID->Integral_Zoom = (int64_t)PID->IntMax * PID_COEFF_SCALE;
    }
    else if (PID->Integral_Zoom < (int64_t)PID->IntMin * PID_COEFF_SCALE)
    {
        PID->Integral_Zoom = (int64_t)PID->IntMin * PID_COEFF_SCALE;
    }
#endif
    
#if PID_USE_FLOAT
    /* ========== 第三步：浮点模式 - 微分项滤波 + PID计算 ========== */
    /* 微分项一阶低通滤波：
     * - 误差变化率 = 当前误差 - 上次误差
     * - 滤波公式：y(n) = α*x(n) + (1-α)*y(n-1)
     * - Alpha越大，滤波效果越弱，响应越快；Alpha越小，滤波效果越强，响应越平滑 */
    PID->Error_Rate_Filter = (PID->Cur_Error - PID->Pre_Error) * PID->Alpha + PID->Error_Rate_Filter * (1.0f - PID->Alpha);
    
    /* PID核心公式（位置式）：
     * Output = Kp*Error + Ki*∫Error + Kd*d(Error)/dt
     * - 比例项(P)：响应当前误差，Kp越大响应越快但可能超调
     * - 积分项(I)：消除稳态误差，Ki过大可能导致振荡
     * - 微分项(D)：预测误差趋势，抑制超调，Kd过大可能放大噪声 */
    PID->Output = PID->Kp * PID->Cur_Error + PID->Integral + PID->Kd * PID->Error_Rate_Filter;
        
    /* ========== 第四步：输出限幅（确保输出在安全范围内） ========== */
    /* 输出限幅作用：
     * - 保护执行机构（如电机、舵机）不受过大控制量损坏
     * - 确保控制量在PWM/DAC等输出设备的量程范围内 */
    if (PID->Output > PID->OutMax)
    {
        PID->Output = PID->OutMax;                                      /* 输出超过上限：钳位到最大值 */
    }
    else if (PID->Output < PID->OutMin)
    {
        PID->Output = PID->OutMin;                                      /* 输出低于下限：钳位到最小值 */
    }
#else
    /* ========== 第三步：定点模式 - 微分项滤波 + PID计算 ========== */
    /* 定点化一阶低通滤波：
     * - 使用PID_Restore还原历史滤波项，避免定点数乘法溢出
     * - (PID_COEFF_SCALE - PID->Alpha) 等价于浮点模式的 (1 - Alpha) 
     * - 这是D项   */
    PID->Error_Rate_Filter = PID_Restore((int64_t)(PID->Cur_Error - PID->Pre_Error) * PID->Alpha + (int64_t)PID->Error_Rate_Filter * (PID_COEFF_SCALE - PID->Alpha));
    
    /* 定点化PID核心公式：
     * - 使用int64_t中间变量防止乘法运算溢出
     * - Kd项的滤波结果需要单独还原（PID_Restore）
     * - 最终输出需要整体还原（PID_Restore） */
    int64_t Temp = (int64_t)PID->Kp * PID->Cur_Error + (int64_t)PID->Integral_Zoom + (int64_t)PID->Kd * PID->Error_Rate_Filter;
    if(Temp > (int64_t)PID->OutMax * PID_COEFF_SCALE)
    {
        Temp = (int64_t)PID->OutMax * PID_COEFF_SCALE;
    }
    else if (Temp < (int64_t)PID->OutMin * PID_COEFF_SCALE)
    {
        Temp = (int64_t)PID->OutMin * PID_COEFF_SCALE;
    }
    PID->Output = PID_Restore(Temp);                                    /* 最终输出还原：将定点数转换回原始数值范围 */
#endif
}

/**
  * @brief  获取PID输出值
  * @param  PID: PID控制器结构体指针
  * @retval PID控制器的当前输出值（经过限幅处理后的最终控制量）
  * @note   该输出值可直接用于驱动执行机构（如电机PWM、舵机角度等）
  */
PID_val PID_Get_Output(PID_t *PID)
{
    return PID->Output;  /* 返回当前PID输出值：已包含比例、积分、微分项的综合作用 */
}

/**
  * @brief  PID计算函数（设置实际值并执行PID运算）
  * @param  PID: PID控制器结构体指针
  * @param  Actual: 当前实际测量值（传感器采集的系统当前状态值）
  * @retval PID计算后的输出值（经过限幅处理后的最终控制量）
  * @note   - 该函数为一步到位的调用接口，内部自动更新实际值、执行PID运算并返回输出
  *         - 适用于需要简洁调用的场景，等价于依次调用PID_Set_Actual() + PID_Update() + PID_Get_Output()
  *         - 推荐在定时中断或周期性任务中调用，保证PID控制周期稳定
  * @warning 调用前需确保已通过PID_Set_Target()设置目标值，否则目标值为0
  */
PID_val PID_Calculate(PID_t *PID, PID_val Actual)
{
    /* 步骤1：更新实际反馈值 - 将传感器/编码器采集的当前系统状态写入PID控制器 */
    PID->Actual = Actual;
    
    /* 步骤2：执行PID核心运算 - 内部完成误差计算、积分限幅、微分滤波、PID公式运算、输出限幅 */
    PID_Update(PID);
    
    /* 步骤3：返回计算结果 - 返回经过PID运算和限幅处理后的最终控制量，可直接用于驱动执行机构 */
    return PID->Output;
}

/**
  * @brief  动态修改PID参数
  * @param  PID: PID控制器结构体指针
  * @param  Kp: 比例系数
  * @param  Ki: 积分系数
  * @param  Kd: 微分系数
  * @param  Alpha: 滤波系数(0表示不滤波)
  * @retval 无
  * @note   修改后自动清零状态，防止新旧参数不匹配
  */
void PID_Change_Param(PID_t *PID, float Kp, float Ki, float Kd, float Alpha)
{
    /* 滤波系数修正 */
    if(Alpha == 0)
    {
        Alpha = 1.0f;
    }
    
#if PID_USE_FLOAT
    /* 浮点模式: 直接赋值 */
    PID->Alpha = Alpha;
    PID->Kp = Kp ;
    PID->Ki = Ki * PID->Ts;
    PID->Kd = Kd / PID->Ts;
#else
    /* 定点模式: 放大转整型 */
    PID->Alpha = (PID_val)(Alpha * PID_COEFF_SCALE);
    PID->Kp = (PID_val)(Kp * PID_COEFF_SCALE);
    PID->Ki = (PID_val)((Ki * PID->Ts) * PID_COEFF_SCALE);
    PID->Kd = (PID_val)((Kd / PID->Ts) * PID_COEFF_SCALE);
#endif
    
    /* 清零状态 */
    PID_Clear(PID);
}
