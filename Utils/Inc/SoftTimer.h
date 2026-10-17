#ifndef __SOFTTIMER_H__
#define __SOFTTIMER_H__

#include <stdint.h>

typedef enum{
    SOFTTIMER_MODE_SINGLE = 0,
    SOFTTIMER_MODE_PERIODIC = 1,
}SoftTimer_Mode_e;

typedef struct SoftTimer{
    uint32_t TimeCount;
    uint32_t ReloadCount;

    SoftTimer_Mode_e Mode;

    uint8_t isRunning : 1;
    uint8_t isElapsed : 1;
}SoftTimer_t;

void SoftTimer_Init(SoftTimer_t *Timer, SoftTimer_Mode_e Mode, uint32_t TimeCount);
void SoftTimer_Reset(SoftTimer_t *Timer);
void SoftTimer_Start(SoftTimer_t *Timer);
void SoftTimer_Stop(SoftTimer_t *Timer);
uint8_t SoftTimer_Trigger(SoftTimer_t *Timer);
void SoftTimer_Update(SoftTimer_t *Timer);      

#endif