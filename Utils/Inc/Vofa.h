#ifndef __VOFA_H__
#define __VOFA_H__

#include <stdint.h>

#define VOFA_CH_BYTE 4   //一个通道是float，4字节大小

typedef struct Vofa_Ch_struct{
    float *Datap;
}Vofa_Ch_t;

void Vofa_Init(uint8_t (*Vofa_SendData)(uint8_t *Data, uint16_t Len));
void Vofa_Ch_Register(uint8_t Ch_num, ...);
void Vofa_Data_Register(uint8_t Data_num, ...);
void Vofa_Ch_Data_Register(Vofa_Ch_t *ch, float *Datap);
uint8_t Vofa_Send(void);

#endif