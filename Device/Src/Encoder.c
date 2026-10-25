#include "Encoder.h"
#include "math.h"

#define __2PI 6.2831853f                //2π

/**
 * @brief 编码器参数初始化：设置分辨率、电机极对数和编码器方向
 * @param encoder 编码器结构体指针
 * @param Encoder_Res 编码器分辨率(每圈脉冲数)
 * @param Motor_Pn 电机极对数
 * @param Encoder_Dir 编码器方向(0=正向, 非0=反向)
 */
void Encoder_Param_Init(Encoder_t *encoder, uint32_t Encoder_Res, uint32_t Motor_Pn, uint32_t Encoder_Dir, uint32_t Encoder_Error){
    encoder->Encoder_Res = Encoder_Res;
    encoder->Motor_Pn = Motor_Pn;
    encoder->Encoder_Dir = Encoder_Dir;
    encoder->Encoder_Error = Encoder_Error;
}

/**
 * @brief 注册编码器数据读取函数指针
 * @param encoder 编码器结构体指针
 * @param Read_Data 编码器数据读取回调函数，返回原始脉冲计数值
 */
void Encoder_Read_Data_Register(Encoder_t *encoder, Encoder_Read_Data_t Read_Data){
    encoder->Read_Data = Read_Data;
}
                                                                                                                                                                                                                                                                                                                                                    
/**
 * @brief 编码器数据更新：读取原始脉冲并转换为机械角度和电角度
 * @param encoder 编码器结构体指针
 * @note  方向为正向时直接使用原始脉冲；方向为反向时取补码(Encoder_Res - Temp)
 *        机械角度 Theta = Pulse_Count / Encoder_Res * 2π
 *        电角度 Theta_e = Theta * Motor_Pn
 */
float Encoder_Update(Encoder_t *encoder){
    uint32_t Temp = encoder->Read_Data();
    if(encoder->Encoder_Dir == 0){                               // 编码器方向为正
        encoder->Pulse_Count = Temp;
    }
    else{                                                        // 编码器方向为负
        encoder->Pulse_Count = (Temp == 0) ? 0 : encoder->Encoder_Res - Temp;
    }
    float Temp_Theta = (encoder->Pulse_Count - encoder->Encoder_Error + encoder->Encoder_Res) % encoder->Encoder_Res;
    encoder->Theta = (float)Temp_Theta / (float)encoder->Encoder_Res * __2PI;
    encoder->Theta_e = encoder->Theta * (float)encoder->Motor_Pn;
    encoder->Theta_e = fmodf(encoder->Theta_e, __2PI);
    if(encoder->Theta_e < 0){
        encoder->Theta_e += __2PI;
    }
    return encoder->Theta_e;
}

uint32_t Encoder_Get_Count(Encoder_t *encoder){
    return encoder->Pulse_Count;
}