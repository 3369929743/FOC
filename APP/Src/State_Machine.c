#include "State_Machine.h"
#include "Timer.h"
#include "FOC_math.h"
#include "Vofa.h"
#include "SoftTimer.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include "Motor_FOC_Config.h"
#include "Encoder.h"
#include "MyADC.h"

#include "spi.h"

#define __2PI 6.2831853f

/* ====================== MT6701接收 ====================== */
uint32_t MT6701_ReadData(void)
{
    uint8_t rx[3] = {0};
    uint8_t tx[3] = {0};
    
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, 0xffffff);    	
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);  
    return ((rx[0] << 6) | (rx[1] >> 2)) & 0x3FFF;
}

/* ====================== 全局变量 ====================== */


/*-------------------- 定时器实例 -------------------- */
static Timer_t TIM_Global;
static Timer_t TIM_FOC;

/*-------------------- 软定时器实例 -------------------- */
static SoftTimer_t Soft_Timer_FOC;

/*-------------------- Vofa通道实例 -------------------- */
static Vofa_Ch_t ADC_IU_Ch;
static Vofa_Ch_t ADC_IV_Ch;
static Vofa_Ch_t ADC_IW_Ch;

/*-------------------- 编码器实例 -------------------- */
static Encoder_t Encoder_1;

/*-------------------- FOC电机实例 -------------------- */
static FOC_t Foc_Motor = {
    .Vd = 0.0f,
    .Vq = 0.0f,
    .Theta = 0.0f,
};

/*-------------------- ADC实例 -------------------- */
static ADC_t ADC_Current;

static ADC_Channel_t ADC_IU;
static ADC_Channel_t ADC_IV;
static ADC_Channel_t ADC_IW;

static ADC_BSP_Param_t ADC_BSP_Param;


/* ====================== 定时器回调函数 ====================== */
static void Timer_Global_PeriodElapsedCallback(Timer_t *timer)
{
    if(timer == &TIM_Global){
        SoftTimer_Update(&Soft_Timer_FOC);
    }
}

static void Timer_Foc_CaculateCallback(Timer_t *timer){
    if(timer == &TIM_FOC){

    }
}

/* ====================== ADC回调函数 ====================== */
void My_ADC_InjectedConvCpltCallback(ADC_t* adc){
    if(adc == &ADC_Current){
        /* 读取ADC值 */
        My_ADC_Get_InjectedValue(&ADC_IU, ADC_INJECTED_RANK_1);
        My_ADC_Get_InjectedValue(&ADC_IV, ADC_INJECTED_RANK_2);
        My_ADC_Get_InjectedValue(&ADC_IW, ADC_INJECTED_RANK_3);

        /* 计算电流 */
        My_ADC_Calculate_Current(&ADC_IU, &ADC_BSP_Param);
        My_ADC_Calculate_Current(&ADC_IV, &ADC_BSP_Param);
        My_ADC_Calculate_Current(&ADC_IW, &ADC_BSP_Param);

        /* 进行FOC计算 */
        FOC_Eocoder_Update(&Foc_Motor, Encoder_Update(&Encoder_1));
        Timer_Set_CompareValue(&TIM_FOC, (uint32_t)Foc_Motor.Tcmp1, TIM_CHANNEL_1);
        Timer_Set_CompareValue(&TIM_FOC, (uint32_t)Foc_Motor.Tcmp2, TIM_CHANNEL_2);
        Timer_Set_CompareValue(&TIM_FOC, (uint32_t)Foc_Motor.Tcmp3, TIM_CHANNEL_3);

        /* 启动下一次转换 */
        My_ADC_InjectedStart_IT(&ADC_Current);
    }
}

/* ====================== 主状态机函数 ====================== */
void State_Machine_Init(void){
    /* 初始化FOC电机参数 */
    FOC_Param_Init(&Foc_Motor, U_BUS, ARR);
    FOC_Vf_Control_Param_Init(&Foc_Motor.Vf_Param, 30.0f, 0.0f, 0.001f, FOC_CLK_PERIOD);

    /* 初始化编码器参数 */
    Encoder_Param_Init(&Encoder_1, ENCODER_RES, MOTOR_PN, ENCODER_DIR, ENCODER_ERROR);
    Encoder_Read_Data_Register(&Encoder_1, MT6701_ReadData);

    /* 初始化ADC参数 */
    My_ADC_BSP_Param_Init(&ADC_BSP_Param, ADC_CURRENT_GAIN, ADC_BIAS, ADC_SHUNT_RES);
    My_ADC_Init(&ADC_Current, ADC_1);
    My_ADC_Channel_Init(&ADC_IU, ADC_1);
    My_ADC_Channel_Init(&ADC_IV, ADC_1);
    My_ADC_Channel_Init(&ADC_IW, ADC_1);

    /* 初始化ADC转换完成回调函数 */
    My_ADC_InjectedConvCpltCallback_Register(&ADC_Current, My_ADC_InjectedConvCpltCallback);

    /* 初始化ADC注入模式 */
    My_ADC_InjectedStart_IT(&ADC_Current);

    /* 初始化定时器 */
    Timer_Init(&TIM_Global, Timer_6);
    Timer_Init(&TIM_FOC, Timer_1);
    Timer_Set_CompareValue(&TIM_FOC, (uint32_t)FOC_PWM_TRIGGER_CCR, TIM_CHANNEL_4);
    Timer_Comple_PWM_Start(&TIM_FOC, TIM_CHANNEL_1);
    Timer_Comple_PWM_Start(&TIM_FOC, TIM_CHANNEL_2);
    Timer_Comple_PWM_Start(&TIM_FOC, TIM_CHANNEL_3);
    Timer_Start_Base_IT(&TIM_Global, Timer_Global_PeriodElapsedCallback);
    Timer_Start_PulseFinished_IT(&TIM_FOC, TIM_CHANNEL_4, Timer_Foc_CaculateCallback);

    /* 初始化软定时器 */
    SoftTimer_Init(&Soft_Timer_FOC, SOFTTIMER_MODE_PERIODIC, 1);
    SoftTimer_Start(&Soft_Timer_FOC);

    /* 初始化Vofa通道 */
    Vofa_Init(CDC_Transmit_FS);

    Vofa_Ch_Register(3,&ADC_IU_Ch, &ADC_IV_Ch, &ADC_IW_Ch);
    Vofa_Data_Register(3, &ADC_IU.current, &ADC_IV.current, &ADC_IW.current);
}

void State_Machine_Loop(void){
    if(SoftTimer_Trigger(&Soft_Timer_FOC)){
        Vofa_Send();
    }
}