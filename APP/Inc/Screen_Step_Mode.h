#ifndef __SCREEN_STEP_MODE_H__
#define __SCREEN_STEP_MODE_H__

#include <stdint.h>
typedef struct Serial_Struct Serial_t;
typedef struct Encoder_Struct Encoder_t;

void Screen_Step_Mode_Init(Encoder_t *encoder);
void Screen_Step_Mode_Send_Start(void);
void Screen_Step_Mode_Send(Serial_t *Serial, Encoder_t *encoder);

#endif