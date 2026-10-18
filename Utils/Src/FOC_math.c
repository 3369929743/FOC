#include "FOC_math.h"
#include <math.h>

#define SQRT3_DIV_2 0.8660254f          //二分之根号3
#define __2PI 6.2831853f                //2π

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

void FOC_Param_Init(FOC_t *foc, float Ubus, float TARR){
    foc->Ubus = Ubus;
    foc->TARR = TARR;
}

/**
 * @brief 反Park变换：将dq旋转坐标系电压转换为αβ静止坐标系电压
 * @param foc FOC对象指针，包含Vd/Vq/Theta输入，输出Valpha/Vbeta
 * @note  Vd为直轴电压(励磁分量)，Vq为交轴电压(转矩分量)，Theta为电角度
 */
void FOC_Rev_Park(FOC_t *foc)
{
    float sin_theta = sinf(foc->Theta);
    float cos_theta = cosf(foc->Theta);

    foc->Valpha = foc->Vd * cos_theta - foc->Vq * sin_theta;
    foc->Vbeta = foc->Vd * sin_theta + foc->Vq * cos_theta;
}

/**
 * @brief 反Clarke变换：将αβ静止坐标系电压转换为三相UVW电压
 * @param foc FOC对象指针，输入Valpha/Vbeta，输出Vu/Vv/Vw
 */
void FOC_Rev_Clarke(FOC_t *foc)
{
    foc->Vu = foc->Valpha;
    foc->Vv = -0.5f * foc->Valpha + SQRT3_DIV_2 * foc->Vbeta;
    foc->Vw = -0.5f * foc->Valpha - SQRT3_DIV_2 * foc->Vbeta;
}

/**
 * @brief 零序注入法SVPWM：注入三次谐波将三相电压居中，提高直流母线电压利用率约15%
 * @param foc FOC对象指针，输入Vu/Vv/Vw，输出Vu_mod/Vv_mod/Vw_mod和比较值Tcmp1/2/3
 * @note  采用min-max法计算零序分量Voffset = -(Vmax+Vmin)/2，注入后相电压峰值不超过母线一半
 */
void FOC_Zero_Injection(FOC_t *foc)
{
    float Vmax = Max(foc->Vu, foc->Vv, foc->Vw);
    float Vmin = Min(foc->Vu, foc->Vv, foc->Vw);
    if(__builtin_isnan(Vmax) || __builtin_isnan(Vmin)){
        foc->Vu_mod = 0.0f;
        foc->Vv_mod = 0.0f;
        foc->Vw_mod = 0.0f;
        return;
    }
    float Voffset = - 0.5f * (Vmax + Vmin);

    foc->Vu_mod = foc->Vu + Voffset;
    foc->Vv_mod = foc->Vv + Voffset;
    foc->Vw_mod = foc->Vw + Voffset;

    foc->Tcmp1 = ((foc->Vu_mod) / foc->Ubus + 0.5f) * foc->TARR;
    foc->Tcmp2 = ((foc->Vv_mod) / foc->Ubus + 0.5f) * foc->TARR;
    foc->Tcmp3 = ((foc->Vw_mod) / foc->Ubus + 0.5f) * foc->TARR;
}

/**
 * @brief FOC电压更新总函数：依次执行反Park变换、反Clarke变换、零序注入SVPWM
 * @param foc FOC对象指针
 * @note  输入Vd/Vq/Theta，最终输出三相定时器比较值Tcmp1/2/3用于PWM生成
 */
void FOC_Voltage_Update(FOC_t *foc){
    FOC_Rev_Park(foc);
    FOC_Rev_Clarke(foc);
    FOC_Zero_Injection(foc);
}

void FOC_Vf_Control_Param_Init(Vf_Control_t *vf, float Acc, float Target_Speed, float Speed_Dead, float FOC_Ts){
    vf->Acc = Acc;
    vf->Target_Speed = Target_Speed;
    vf->Speed_Dead = Speed_Dead;
    vf->FOC_Ts = FOC_Ts;
}

void Foc_Vf_Control_SetAcc(Vf_Control_t *vf, float Acc){
    vf->Acc = Acc;
}

void Foc_Vf_Control_SetTarget_Speed(Vf_Control_t *vf, float Target_Speed){
    vf->Target_Speed = Target_Speed;
}

void Foc_Vf_Control_SetSpeed_Dead(Vf_Control_t *vf, float Speed_Dead){
    vf->Speed_Dead = Speed_Dead;
}

void Foc_Vf_Control_SetFOC_Ts(Vf_Control_t *vf, float FOC_Ts){
    vf->FOC_Ts = FOC_Ts;
}

void Foc_Vf_Control_Update(Vf_Control_t *vf){

    /*  计算原始不带死区的误差 */
    vf->Raw_Error = vf->Target_Speed - vf->Speed;

    /*  计算经过死区的误差 */
    if(fabsf(vf->Raw_Error) < vf->Speed_Dead){
        vf->Error = 0.0f;
    }else{
        vf->Error = vf->Raw_Error;
    }

    /*  计算加速度的方向 */
    if(vf->Error > 0){
        vf->Acc_Dir = 1;
    }
    else if(vf->Error < 0){
        vf->Acc_Dir = -1;
    }
    else{
        vf->Acc_Dir = 0;
    }

    /*  计算速度积分 */
    vf->Speed += vf->Acc_Dir * vf->Acc * vf->FOC_Ts;

    /*  计算角度积分 */
    vf->Theta += vf->Speed * vf->FOC_Ts;
        
    /*  角度积分项归一化 */
    if(vf->Theta > __2PI) vf->Theta -= __2PI;
    else if(vf->Theta < 0) vf->Theta += __2PI;
}

void FOC_Update(FOC_t *foc){
    Foc_Vf_Control_Update(&foc->Vf_Param);
    foc->Theta = foc->Vf_Param.Theta;
    FOC_Voltage_Update(foc);
}