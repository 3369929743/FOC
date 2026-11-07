#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

#define PID_USE_FLOAT 1
// #define PID_USE_FLOAT 0

#if PID_USE_FLOAT
    typedef struct{
    float Ts;                           //采样时间

    float Kp, Ki, Kd;                 //PID参数，这个是用于计算的参数，实际使用时需要根据采样时间进行换算；使用定点数运算时进行定点化处理   

    float Pre_Error, Cur_Error;       // 上一次和当前的误差
    float Integral, IntMax, IntMin;   // 积分项和积分项的上下限

    float Actual, Target;

    float OutMax, OutMin;

    float Output;

    float Alpha, Error_Rate_Filter;
    } PID_t;
#else
    #define MULTIPLE 10
    #define PID_ZOOM (1 << MULTIPLE)
    typedef struct{
    int32_t Ts;                           //采样时间

    int32_t Kp, Ki, Kd;                 //PID参数，这个是用于计算的参数，实际使用时需要根据采样时间进行换算；使用定点数运算时进行定点化处理   

    int32_t Pre_Error, Cur_Error;       // 上一次和当前的误差
    int32_t Integral, IntMax, IntMin;   // 积分项和积分项的上下限

    int32_t Actual, Target;

    int32_t OutMax, OutMin;

    int32_t Output;

    int32_t Alpha, Error_Rate_Filter;
} PID_t;
#endif

typedef enum{
    PID_OK = 0x0U,
    PID_ERROR = 0x1U
}PID_Status_t;

/* PID配置结构体,该结构体由用户填写，填浮点数，如果要用定点数已近内部处理了 */
typedef struct {
    float Ts;                           //采样时间

    float Kp, Ki, Kd;                   //PID参数   

    float IntMax, IntMin;               //积分项的上下限

    float OutMax, OutMin;               //输出的上下限

    float Alpha;                        //平滑系数，这是d项的一阶低通滤波
}PID_Confg_t;

PID_Status_t PID_Init(PID_t *PID, PID_Confg_t *Config);
void PID_Clear(PID_t *PID);
void PID_Set_Target(PID_t *PID, PID_val Target);
void PID_Set_Actual(PID_t *PID, PID_val Actual);
PID_val PID_Get_Output(PID_t *PID);                     //获取当前输出，只返回当前输出
PID_val PID_Calculate(PID_t *PID, PID_val Actual);      //计算当前输出
void PID_Change_Param(PID_t *PID, float Kp, float Ki, float Kd, float Alpha);

#endif