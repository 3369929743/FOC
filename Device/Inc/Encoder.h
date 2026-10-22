#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>

typedef uint32_t (* Encoder_Read_Data_t)(void);

typedef struct
{
    uint32_t Encoder_Res;               // 编码器分辨率
    uint32_t Motor_Pn;                  // 电机极数
    uint32_t Encoder_Dir;               // 编码器方向
    uint32_t Encoder_Error;             // 编码器零点误差

    uint32_t Pulse_Count;               // 编码器脉冲计数
    float Theta;                        // 角度
    float Theta_e;                      // 电角度

    Encoder_Read_Data_t Read_Data;      // 编码器读取数据函数指针
} Encoder_t;

void Encoder_Param_Init(Encoder_t *encoder, uint32_t Encoder_Res, uint32_t Motor_Pn, uint32_t Encoder_Dir, uint32_t Encoder_Error);
void Encoder_Read_Data_Register(Encoder_t *encoder, Encoder_Read_Data_t Read_Data);
float Encoder_Update(Encoder_t *encoder);

#endif