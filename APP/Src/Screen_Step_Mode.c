#include "Screen_Step_Mode.h"
#include "Serial.h"
#include <stdint.h>
#include "Motor_FOC_Config.h"
#include "Encoder.h"

#define SCREEN_STEP 100                    //屏幕步进值
#define SCREEN_STEP_SCALE (SCREEN_STEP / ENCODER_RES)     //屏幕步进值缩放因子                    

typedef struct{
    uint32_t Send_FLAG;           //发送标志位

    uint32_t Screen_Step_Bias;   //刚上电的时候编码器的读数，之后都减去，为了使刚上电的时候屏幕步进值为0
} Screen_Step_Mode_t;

static Screen_Step_Mode_t Screen_Step_Mode;

void Screen_Step_Mode_Init(Encoder_t *encoder){
    Screen_Step_Mode.Screen_Step_Bias = Encoder_Get_Count(encoder);      //获取刚上电的时候的编码器计数值
}

void Screen_Step_Mode_Send_Start(void){
    Screen_Step_Mode.Send_FLAG = 1;
}

void Screen_Step_Mode_Send(Serial_t *Serial, Encoder_t *encoder){
    if(!Screen_Step_Mode.Send_FLAG) return;
    if(Serial == NULL) return;
    uint32_t Count = Encoder_Get_Count(encoder);                                        //获取编码器计数值
    uint8_t pData[3] = {0};                                                             //数据缓冲区
    pData[0] = 0xAA;                                                                    //包头1 
    pData[1] = 0x55;                                                                    //包头2 
    int32_t Count_Temp = (Count - Screen_Step_Mode.Screen_Step_Bias);
    if(Count_Temp >= ENCODER_RES_CALCULATE) Count_Temp -= ENCODER_RES_CALCULATE;        //处理编码器计数值超出范围的情况
    if(Count_Temp < 0) Count_Temp += ENCODER_RES_CALCULATE;                             //处理编码器计数值超出范围的情况
    uint8_t Screen_Step = Count_Temp * SCREEN_STEP_SCALE;                               //计算屏幕步进值
    pData[2] = Screen_Step;                                                             //屏幕步进值
    Serial_IT_SendArray(Serial, pData, 3);                                  //发送数据
    Screen_Step_Mode.Send_FLAG = 0;
}
