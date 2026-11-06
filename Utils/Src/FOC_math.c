#include "FOC_math.h"
#include <math.h>
#include <stdint.h>

#define SQRT3_DIV_2 0.8660254f          //二分之根号3
#define __2PI 6.2831853f                //2π
#define ONE_DIV_SQRT3 0.57735027f         //根号3的倒数

/**
 * @brief 求三个浮点数的最大值
 * @param a 第一个比较值
 * @param b 第二个比较值
 * @param c 第三个比较值
 * @return 三者中的最大值，若任一参数为NaN则返回NAN
 * @note  使用__builtin_isnan检测非法浮点数，防止NaN污染后续计算
 */
static inline float Max(float a, float b, float c){
    if(__builtin_isnan(a) || __builtin_isnan(b) || __builtin_isnan(c)){
        return NAN;
    }
    return a > b ? (a > c ? a : c) : (b > c ? b : c);
}

/**
 * @brief 求三个浮点数的最小值
 * @param a 第一个比较值
 * @param b 第二个比较值
 * @param c 第三个比较值
 * @return 三者中的最小值，若任一参数为NaN则返回NAN
 * @note  使用__builtin_isnan检测非法浮点数，防止NaN污染后续计算
 */
static inline float Min(float a, float b, float c){
    if(__builtin_isnan(a) || __builtin_isnan(b) || __builtin_isnan(c)){
        return NAN;
    }
    return a < b ? (a < c ? a : c) : (b < c ? b : c);
}

/**
 * @brief FOC基础参数初始化：设置母线电压和定时器自动重装值
 * @param self FOC对象指针
 * @param Ubus 母线电压(V)
 * @param TARR 定时器自动重装值(ARR)，决定PWM分辨率
 */
void FOC_Param_Init(FOC_t *self, float Ubus, float TARR){
    self->Ubus = Ubus;
    self->TARR = TARR;
}

/**
 * @brief 根据电角度Theta计算cos和sin值，用于Park变换和反Park变换
 * @param self FOC对象指针，输入Theta，输出cos_Theta/sin_Theta
 */
void FOC_Get_Cos_Sin(FOC_t *self){
    self->cos_Theta = cosf(self->Theta);
    self->sin_Theta = sinf(self->Theta);
}

/**
 * @brief 反Park变换：将dq旋转坐标系电压转换为αβ静止坐标系电压
 * @param self FOC对象指针，包含Vd/Vq/Theta输入，输出Valpha/Vbeta
 * @note  Vd为直轴电压(励磁分量)，Vq为交轴电压(转矩分量)，Theta为电角度
 */
static inline void FOC_Rev_Park(FOC_t *self)
{
    self->Valpha = self->Vd * self->cos_Theta - self->Vq * self->sin_Theta;
    self->Vbeta = self->Vd * self->sin_Theta + self->Vq * self->cos_Theta;
}

/**
 * @brief 反Clarke变换：将αβ静止坐标系电压转换为三相UVW电压
 * @param self FOC对象指针，输入Valpha/Vbeta，输出Vu/Vv/Vw
 */
static inline void FOC_Rev_Clarke(FOC_t *self)
{
    self->Vu = self->Valpha;
    self->Vv = -0.5f * self->Valpha + SQRT3_DIV_2 * self->Vbeta;
    self->Vw = -0.5f * self->Valpha - SQRT3_DIV_2 * self->Vbeta;
}

/**
 * @brief 零序注入法SVPWM：注入三次谐波将三相电压居中，提高直流母线电压利用率约15%
 * @param self FOC对象指针，输入Vu/Vv/Vw，输出Vu_mod/Vv_mod/Vw_mod和比较值Tcmp1/2/3
 * @note  采用min-max法计算零序分量Voffset = -(Vmax+Vmin)/2，注入后相电压峰值不超过母线一半
 */
static inline void FOC_Zero_Injection(FOC_t *self)
{
    float Vmax = Max(self->Vu, self->Vv, self->Vw);
    float Vmin = Min(self->Vu, self->Vv, self->Vw);
    if(__builtin_isnan(Vmax) || __builtin_isnan(Vmin)){
        self->Vu_mod = 0.0f;
        self->Vv_mod = 0.0f;
        self->Vw_mod = 0.0f;
        return;
    }
    float Voffset = - 0.5f * (Vmax + Vmin);

    self->Vu_mod = self->Vu + Voffset;
    self->Vv_mod = self->Vv + Voffset;
    self->Vw_mod = self->Vw + Voffset;

    self->Tcmp1 = ((self->Vu_mod) / self->Ubus + 0.5f) * self->TARR;
    self->Tcmp2 = ((self->Vv_mod) / self->Ubus + 0.5f) * self->TARR;
    self->Tcmp3 = ((self->Vw_mod) / self->Ubus + 0.5f) * self->TARR;

    self->Tcmp1 = (uint32_t)(self->Tcmp1 + 0.5f);
    self->Tcmp2 = (uint32_t)(self->Tcmp2 + 0.5f);
    self->Tcmp3 = (uint32_t)(self->Tcmp3 + 0.5f);
}

/**
 * @brief FOC电压更新总函数：依次执行反Park变换、反Clarke变换、零序注入SVPWM
 * @param self FOC对象指针
 * @note  输入Vd/Vq/Theta，最终输出三相定时器比较值Tcmp1/2/3用于PWM生成
 */
void FOC_Rev_Conv_Update(FOC_t *self){
    FOC_Rev_Park(self);
    FOC_Rev_Clarke(self);
    FOC_Zero_Injection(self);
}

/**
 * @brief V/f控制参数初始化：设置加速度、目标速度、死区阈值及FOC时间步长
 * @param self V/f控制结构体指针
 * @param Acc 加速度
 * @param Target_Speed 目标速度
 * @param Speed_Dead 死区阈值，用于抑制稳态抖振
 * @param FOC_Ts FOC控制周期(s)
 */
void FOC_Vf_Control_Param_Init(Vf_Control_t *self, float Acc, float Target_Speed, float Speed_Dead, float FOC_Ts){
    self->Acc = Acc;
    self->Target_Speed = Target_Speed;
    self->Speed_Dead = Speed_Dead;
    self->FOC_Ts = FOC_Ts;
}

/**
 * @brief 设置V/f控制的加速度
 * @param self V/f控制结构体指针
 * @param Acc 加速度
 */
void FOC_Vf_Control_SetAcc(Vf_Control_t *self, float Acc){
    self->Acc = Acc;
}

/**
 * @brief 设置V/f控制的目标速度
 * @param self V/f控制结构体指针
 * @param Target_Speed 目标速度
 */
void FOC_Vf_Control_SetTarget_Speed(Vf_Control_t *self, float Target_Speed){
    self->Target_Speed = Target_Speed;
}

/**
 * @brief 设置V/f控制的速度死区阈值
 * @param self V/f控制结构体指针
 * @param Speed_Dead 死区阈值，用于抑制稳态抖振
 */
void FOC_Vf_Control_SetSpeed_Dead(Vf_Control_t *self, float Speed_Dead){
    self->Speed_Dead = Speed_Dead;
}

/**
 * @brief 设置V/f控制的FOC时间步长
 * @param self V/f控制结构体指针
 * @param FOC_Ts FOC控制周期(s)
 */
void FOC_Vf_Control_SetFOC_Ts(Vf_Control_t *self, float FOC_Ts){
    self->FOC_Ts = FOC_Ts;
}

/**
 * @brief V/f控制更新：执行带死区的速度积分和角度积分
 * @param self V/f控制结构体指针
 * @note  计算流程：原始误差 → 死区处理 → 加速度方向判断 → 速度积分 → 角度积分 → 角度归一化到[0, 2π)
 */
static inline void FOC_Vf_Control_Update(Vf_Control_t *self){

    /*  计算原始不带死区的误差 */
    self->Raw_Error = self->Target_Speed - self->Speed;

    /*  计算经过死区的误差 */
    if(fabsf(self->Raw_Error) < self->Speed_Dead){
        self->Error = 0.0f;
    }else{
        self->Error = self->Raw_Error;
    }

    /*  计算加速度的方向 */
    if(self->Error > 0){
        self->Acc_Dir = 1;
    }
    else if(self->Error < 0){
        self->Acc_Dir = -1;
    }
    else{
        self->Acc_Dir = 0;
    }

    /*  计算速度积分 */
    self->Speed += self->Acc_Dir * self->Acc * self->FOC_Ts;

    /*  计算角度积分 */
    self->Theta += self->Speed * self->FOC_Ts;
        
    /*  角度积分项归一化 */
    while(self->Theta > __2PI) self->Theta -= __2PI;
    while(self->Theta < 0) self->Theta += __2PI;    
}

/**
 * @brief FOC总更新函数：依次执行V/f速度控制更新和SVPWM电压更新
 * @param self FOC对象指针
 * @note  先更新V/f控制得到电角度Theta，再执行反Park→反Clarke→SVPWM生成PWM比较值
 */
void FOC_Vf_Update(FOC_t *self){
    FOC_Vf_Control_Update(&self->Vf_Param);
    self->Theta = self->Vf_Param.Theta;
    FOC_Get_Cos_Sin(self);
    FOC_Rev_Conv_Update(self);
}

/**
 * @brief Clarke变换：将三相UVW电流转换为αβ静止坐标系电流
 * @param self FOC对象指针，输入Iu/Iv/Iw，输出Ialpha/Ibeta
 * @note  Iu + Iv + Iw = 0(基尔霍夫电流定律)，因此只需两相电流即可计算
 */
static inline void FOC_Clarke(FOC_t *self){
    self->Ialpha = self->Iu;
    self->Ibeta =  (self->Iv - self->Iw) * ONE_DIV_SQRT3;
}

/**
 * @brief Park变换：将αβ静止坐标系电流转换为dq旋转坐标系电流
 * @param self FOC对象指针，输入Ialpha/Ibeta/Theta，输出Id/Iq
 * @note  Id为直轴电流(励磁分量)，Iq为交轴电流(转矩分量)，用于电流闭环控制
 */
static inline void FOC_Park(FOC_t *self){
    self->Id = self->Ialpha * self->cos_Theta + self->Ibeta * self->sin_Theta;
    self->Iq = -self->Ialpha * self->sin_Theta + self->Ibeta * self->cos_Theta;
}

/**
 * @brief 电流采样变换总函数：依次执行Clarke变换和Park变换
 * @param self FOC对象指针
 * @note  将三相电流Iu/Iv/Iw转换为dq旋转坐标系电流Id/Iq，用于电流闭环反馈
 */
void FOC_Conv_Update(FOC_t *self){
    FOC_Clarke(self);
    FOC_Park(self);
}