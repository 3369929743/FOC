#include "Encoder.h"

#define __2PI 6.2831853f                //2π
void Encoder_Param_Init(Encoder_t *encoder, uint32_t Encoder_Res, uint32_t Motor_Pn, uint32_t Encoder_Dir){
    encoder->Encoder_Res = Encoder_Res;
    encoder->Motor_Pn = Motor_Pn;
    encoder->Encoder_Dir = Encoder_Dir;
}

void Encoder_Read_Data_Register(Encoder_t *encoder, Encoder_Read_Data_t Read_Data){
    encoder->Read_Data = Read_Data;
}

void Encoder_Data_Update(Encoder_t *encoder){
    uint32_t Temp = encoder->Read_Data();
    if(encoder->Encoder_Dir == 0){                               // 编码器方向为正
        encoder->Pulse_Count = Temp;
    }
    else{                                                        // 编码器方向为负
        encoder->Pulse_Count = (encoder->Pulse_Count == 0) ? 0 : encoder->Encoder_Res - Temp;
    }
    encoder->Theta = (float)encoder->Pulse_Count / (float)encoder->Encoder_Res * __2PI;
    encoder->Theta_e = encoder->Theta * (float)encoder->Motor_Pn;
}
