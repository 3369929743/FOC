#ifndef __FOC_MATH_H__
#define __FOC_MATH_H__

typedef struct Vf_Control_struct{
    /*  输入参数 */
    float Acc;              //加速度
    float Target_Speed;     //目标速度
    float Speed_Dead;       //死区阈值

    /*  中间参数 */
    float Raw_Error;       //原始误差
    float Error;           //经过死区的误差
    float Acc_Dir;         //加速度方向
    float FOC_Ts;          //FOC时间步长

    /*  输出参数 */
    float Speed;           //输出速度
    float Theta;           //输出角度
}Vf_Control_t;

typedef struct
{
    float Ubus;                                   //母线电压
    float TARR;                                   //自动重装值
    float Vd, Vq;                                 //d轴电压，d轴为直轴，控制磁场和厉磁分量；q轴电压，q轴为交轴，控制转矩分量
    float Theta;                                  //转子在磁场中的电角度，转子和d轴同轴

    float cos_Theta, sin_Theta;                   //cos(Theta)，sin(Theta)，用于Park变换和反Park变换

    float Valpha, Vbeta;                          //alpha轴电压，alpha轴为直角坐标系的x轴；beta轴电压，beta轴为直角坐标系的y轴

    float Vu, Vv, Vw;                             //u相电压，v相电压，w相电压

    float Vu_mod, Vv_mod, Vw_mod;                 //u相电压，经过FOC变换后的电压；v相电压，经过FOC变换后的电压；w相电压，经过FOC变换后的电压

    volatile float Tcmp1, Tcmp2, Tcmp3;           // u相比较值；v相比较值；w相比较值

    float Iu, Iv, Iw;                             //u相电流，v相电流，w相电流

    float Ialpha, Ibeta;                          //alpha轴电流，beta轴电流

    float Id, Iq;                                 //d轴电流，q轴电流，用于电流闭环控制的电流

    Vf_Control_t Vf_Param;                        //v/f控制参数，用于v/f控制模式
} FOC_t;

/* ===================FOC参数初始化=================== */
void FOC_Param_Init(FOC_t *self, float Ubus, float TARR);

/* ===================FOC控制参数初始化（v/f）=================== */
void FOC_Vf_Control_Param_Init(Vf_Control_t *self, float Acc, float Target_Speed, float Speed_Dead, float FOC_Ts);
void FOC_Vf_Control_SetAcc(Vf_Control_t *self, float Acc);
void FOC_Vf_Control_SetTarget_Speed(Vf_Control_t *self, float Target_Speed);
void FOC_Vf_Control_SetSpeed_Dead(Vf_Control_t *self, float Speed_Dead);

/* ===================FOC参数更新=================== */
/* 用于Park变换和反Park变换的三角函数 */
void FOC_Get_Cos_Sin(FOC_t *self);
/* 正向Park变换和Clarke变换 */
void FOC_Conv_Update(FOC_t *self);
/* 反Park变换和反Clarke变换 */
void FOC_Rev_Conv_Update(FOC_t *self);

/* ===================FOC模式=================== */
/* 用于v/f控制 */
void FOC_Vf_Update(FOC_t *self);                

#endif