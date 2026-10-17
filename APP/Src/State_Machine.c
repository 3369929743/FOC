#include "State_Machine.h"
#include "Timer.h"
#include "FOC_math.h"
#include "Vofa.h"
#include "SoftTimer.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include "Motor_FOC_Config.h"

#define __2PI 6.2831853f

/* ====================== 全局变量 ====================== */


/*-------------------- 定时器实例 -------------------- */
static Timer_t TIM_Global;
static Timer_t TIM_FOC;

/*-------------------- 软定时器实例 -------------------- */
static SoftTimer_t Soft_Timer_FOC;

/*-------------------- Vofa通道实例 -------------------- */
static Vofa_Ch_t FOC_Acc_Ch;
static Vofa_Ch_t FOC_Speed_Ch;
static Vofa_Ch_t FOC_Theta_Ch;

/*-------------------- FOC电机实例 -------------------- */
static FOC_t Foc_Motor = {
    .Vd = 0.0f,
    .Vq = 0.0f,
    .Theta = 0.0f,
};

/* ====================== 定时器回调函数 ====================== */
static void Timer_Global_PeriodElapsedCallback(Timer_t *timer)
{
    if(timer == &TIM_Global){
        SoftTimer_Update(&Soft_Timer_FOC);
    }
}

static void Timer_Foc_CaculateCallback(Timer_t *timer){
    if(timer == &TIM_FOC){
        FOC_Update(&Foc_Motor);
        Timer_Set_CompareValue(&TIM_FOC, (uint32_t)Foc_Motor.Tcmp1, TIM_CHANNEL_1);
        Timer_Set_CompareValue(&TIM_FOC, (uint32_t)Foc_Motor.Tcmp2, TIM_CHANNEL_2);
        Timer_Set_CompareValue(&TIM_FOC, (uint32_t)Foc_Motor.Tcmp3, TIM_CHANNEL_3);
    }
}

/* ====================== 主状态机函数 ====================== */
void State_Machine_Init(void){
    Timer_Init(&TIM_Global, Timer_6);
    Timer_Init(&TIM_FOC, Timer_1);
    Timer_Start_IT(&TIM_Global, Timer_Global_PeriodElapsedCallback);
    Timer_Start_IT(&TIM_FOC, Timer_Foc_CaculateCallback);
    Timer_Comple_PWM_Start(&TIM_FOC, TIM_CHANNEL_1);
    Timer_Comple_PWM_Start(&TIM_FOC, TIM_CHANNEL_2);
    Timer_Comple_PWM_Start(&TIM_FOC, TIM_CHANNEL_3);

    SoftTimer_Init(&Soft_Timer_FOC, SOFTTIMER_MODE_PERIODIC, 1);
    SoftTimer_Start(&Soft_Timer_FOC);

    FOC_Param_Init(&Foc_Motor, U_BUS, ARR);
    FOC_Vf_Control_Param_Init(&Foc_Motor.Vf_Param, 30.0f, 0.0f, 0.001f, FOC_CLK_PERIOD);

    Vofa_Init(CDC_Transmit_FS);
    Vofa_Ch_Register(3, &FOC_Acc_Ch, &FOC_Speed_Ch, &FOC_Theta_Ch);
    Vofa_Data_Register(3, &Foc_Motor.Vf_Param.Acc, &Foc_Motor.Vf_Param.Speed, &Foc_Motor.Vf_Param.Theta);

    Foc_Vf_Control_SetTarget_Speed(&Foc_Motor.Vf_Param, 100);
}

void State_Machine_Loop(void){
    if(SoftTimer_Trigger(&Soft_Timer_FOC)){
        Vofa_Send();
    }
}