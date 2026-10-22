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
    float Ubus;             //母线电压
    float TARR;             //自动重装值
    float Vd;               //d轴电压，d轴为直轴，控制磁场和厉磁分量
    float Vq;               //q轴电压，q轴为交轴，控制转矩分量
    float Theta;            //转子在磁场中的电角度，转子和d轴同轴

    float Valpha;           //alpha轴电压，alpha轴为直角坐标系的x轴
    float Vbeta;            //beta轴电压，beta轴为直角坐标系的y轴

    float Vu;               //u相电压
    float Vv;               //v相电压
    float Vw;               //w相电压

    float Vu_mod;           //u相电压，经过FOC变换后的电压
    float Vv_mod;           //v相电压，经过FOC变换后的电压
    float Vw_mod;           //w相电压，经过FOC变换后的电压

    volatile float Tcmp1;    // u相比较值
    volatile float Tcmp2;    // v相比较值
    volatile float Tcmp3;    // w相比较值

    Vf_Control_t Vf_Param;
} FOC_t;

void FOC_Param_Init(FOC_t *foc, float Ubus, float TARR);
void FOC_Rev_Park(FOC_t *foc);
void FOC_Rev_Clarke(FOC_t *foc);
void FOC_Zero_Injection(FOC_t *foc);
void FOC_Voltage_Update(FOC_t *foc);

void FOC_Vf_Control_Param_Init(Vf_Control_t *vf, float Acc, float Target_Speed, float Speed_Dead, float FOC_Ts);
void Foc_Vf_Control_SetAcc(Vf_Control_t *vf, float Acc);
void Foc_Vf_Control_SetTarget_Speed(Vf_Control_t *vf, float Target_Speed);
void Foc_Vf_Control_SetSpeed_Dead(Vf_Control_t *vf, float Speed_Dead);
void Foc_Vf_Control_Update(Vf_Control_t *vf);

void FOC_Eocoder_Update(FOC_t *foc, float Theta_e);

void FOC_Update(FOC_t *foc);
void FOC_Eocoder_Update(FOC_t *foc, float Theta_e);

#endif
