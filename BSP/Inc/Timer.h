#ifndef __TIMER_H__
#define __TIMER_H__

#include "main.h"

#define TIMER_LIST \
    X(1, TIM1)     \
    X(2, TIM2)     \
    X(3, TIM3)     \
    X(4, TIM4)     \
    X(6, TIM6)     \
    X(7, TIM7)     \
    X(8, TIM8)     \
    X(15, TIM15)   \
    X(16, TIM16)   \
    X(17, TIM17)   

typedef enum{
    #define X(Index, Instance) Timer_##Index = Index,
    TIMER_LIST
    #undef X
} Timer_Num_e;

typedef struct Timer_Struct{
    TIM_HandleTypeDef *htim;
    uint32_t Channel;
    
    void (*PeriodElapsedCallback)(struct Timer_Struct *timer);
} Timer_t;

uint8_t Timer_Init(Timer_t* timer, Timer_Num_e TIM_Num);
void Timer_Comple_PWM_Start(Timer_t *timer, uint32_t Channel);
void Timer_Start_IT(Timer_t* timer, void(*Callback)(Timer_t *timer));

/**
 * @brief  设置定时器的比较值，用于控制PWM输出的占空比
 * @param  timer: 指向定时器结构体的指针
 * @param  CompareValue: 要设置的比较值，范围为0-ARR
 * @param  Channel: 要设置的PWM通道，TIM_CHANNEL_1、TIM_CHANNEL_2或TIM_CHANNEL_ALL（全部通道）
 * @retval 无
 * @note   该函数会将用户指定的比较值写入定时器的比较寄存器中，
 *         以实现PWM输出占空比的控制
 */
static inline void Timer_Set_CompareValue(Timer_t* timer, uint32_t CompareValue, uint32_t Channel){
    if(timer == NULL) return;
    __HAL_TIM_SET_COMPARE(timer->htim, Channel, CompareValue);
}

#endif