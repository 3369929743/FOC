#include "My_Cordic.h"
#include "cordic.h"
#include "stm32g4xx_hal_cordic.h"

#define __DIV_PI (0.31830989f)
#define Q31_ZOOM (1U << 31)
#define Q31_SCALE (1.0f / Q31_ZOOM)

static CORDIC_ConfigTypeDef Cordic_Sin_Config = {
    .Function = CORDIC_FUNCTION_SINE,
    .InSize = CORDIC_INSIZE_32BITS,
    .OutSize = CORDIC_OUTSIZE_32BITS,
    .Scale = CORDIC_SCALE_0,
    .NbRead = CORDIC_NBREAD_2,
    .NbWrite = CORDIC_NBWRITE_1,
    .Precision = CORDIC_PRECISION_6CYCLES,   
};

void My_Cordic_Init(void){
    HAL_CORDIC_Configure(&hcordic, &Cordic_Sin_Config);
}

void My_Cordic_Get_Sin_Cos(float Theta_e, float *Sin, float *Cos){
    Theta_e *= __DIV_PI;
    if(Theta_e >= 1.0f){
        Theta_e -= 2.0f;
    }

    int32_t Angle_Q31 = (int32_t)(Theta_e * (float)Q31_ZOOM);
    CORDIC->WDATA = (uint32_t)Angle_Q31;

    int32_t Sin_Q31 = (int32_t)CORDIC->RDATA;
    int32_t Cos_Q31 = (int32_t)CORDIC->RDATA;

    *Sin = (float)Sin_Q31 * Q31_SCALE;
    *Cos = (float)Cos_Q31 * Q31_SCALE;
}