/* ====================== FOC需要的头文件 ====================== */
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
#include "Serial.h"
#include "Key.h"

/* ====================== 实现功能需要的头文件 ====================== */
#include "Screen_Step_Mode.h"

/* ====================== 临时头文件 ====================== */
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

/* ====================== 电机状态 ====================== */
typedef enum{
    STATE_IDLE = 0,
    STATE_CALIBRATION,
    STATE_FOC_RUNNING,
} Motor_State_e;

/* ====================== 全局变量 ====================== */


/*-------------------- 定时器实例 -------------------- */
static Timer_t TIM_Global;
static Timer_t TIM_FOC;

/*-------------------- 软定时器实例 -------------------- */
static SoftTimer_t Soft_Timer_Vofa;
static SoftTimer_t Soft_Timer_Encoder_Count_Mode;

/*-------------------- Vofa通道实例 -------------------- */
static Vofa_Ch_t ADC_IU_Ch;
static Vofa_Ch_t ADC_IV_Ch;
static Vofa_Ch_t ADC_IW_Ch;

/*-------------------- 编码器实例 -------------------- */
static Encoder_t Encoder_Motor;

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

/*-------------------- 串口实例 -------------------- */
static Serial_t Serial_Screen;

/* ====================== 电机状态 ====================== */
static volatile Motor_State_e Motor_State = STATE_IDLE;

/*-------------------- 按键实例 -------------------- */
static Key_t Key_1;

/* ====================== 定时器回调函数 ====================== */
static void Timer_Global_PeriodElapsedCallback(Timer_t *timer)
{
    if(timer == &TIM_Global){
        SoftTimer_Update(&Soft_Timer_Vofa);
        SoftTimer_Update(&Soft_Timer_Encoder_Count_Mode);
        Key_Tick(&Key_1, HAL_GPIO_ReadPin(Key_1_GPIO_Port, Key_1_Pin));
    }
}

/* ====================== ADC回调函数 ====================== */
void My_ADC_InjectedConvCpltCallback(ADC_t* adc){
    if(adc == &ADC_Current){
        switch(Motor_State){
            case STATE_IDLE:
                break;
            case STATE_CALIBRATION:
                /* 读取ADC值 */
                My_ADC_Get_InjectedValue(&ADC_IU, ADC_INJECTED_RANK_1);
                My_ADC_Get_InjectedValue(&ADC_IV, ADC_INJECTED_RANK_2);
                My_ADC_Get_InjectedValue(&ADC_IW, ADC_INJECTED_RANK_3);

                uint8_t Calib_u, Calib_v, Calib_w;
                Calib_u = My_ADC_Bias_Calibreation_Start(&ADC_IU);
                Calib_v = My_ADC_Bias_Calibreation_Start(&ADC_IV);
                Calib_w = My_ADC_Bias_Calibreation_Start(&ADC_IW);
                if(Calib_u && Calib_v && Calib_w){
                    Motor_State = STATE_FOC_RUNNING;
                }else{
                    Motor_State = STATE_CALIBRATION;
                }
                break;
            case STATE_FOC_RUNNING:
                /* 读取ADC值 */
                My_ADC_Get_InjectedValue(&ADC_IU, ADC_INJECTED_RANK_1);
                My_ADC_Get_InjectedValue(&ADC_IV, ADC_INJECTED_RANK_2);
                My_ADC_Get_InjectedValue(&ADC_IW, ADC_INJECTED_RANK_3);

                /* 计算电流 */
                My_ADC_Calculate_Current(&ADC_IU, &ADC_BSP_Param);
                My_ADC_Calculate_Current(&ADC_IV, &ADC_BSP_Param);
                My_ADC_Calculate_Current(&ADC_IW, &ADC_BSP_Param);

                // /* 进行FOC计算 */
                // FOC_Eocoder_Update(&Foc_Motor, Encoder_Update(&Encoder_Motor));

                // /* 发送编码器计数值 */
                // Screen_Step_Mode_Send_Start();

                /* 更新PWM占空比 */
                // Timer_Set_CompareValue(&TIM_FOC, (uint32_t)Foc_Motor.Tcmp1, TIM_CHANNEL_1);
                // Timer_Set_CompareValue(&TIM_FOC, (uint32_t)Foc_Motor.Tcmp2, TIM_CHANNEL_2);
                // Timer_Set_CompareValue(&TIM_FOC, (uint32_t)Foc_Motor.Tcmp3, TIM_CHANNEL_3);
                break;
        }
    }
}

/*-------------------- 按键事件函数 -------------------- */
void Key_Global_Event_Callback(Key_t *Key, Key_Event_e Event){
    if(Key == NULL) return;
    if(Key == &Key_1){
        switch (Event) {
            case KEY_EVENT_ONCE_PRESS:
                Motor_State = STATE_CALIBRATION;
                break;
            case KEY_EVENT_DOUBLE_PRESS:
                break;
            case KEY_EVENT_LONG_PRESS:
                break;
            default:
                break;
        }
    }
}

/* ====================== 主状态机函数 ====================== */
void State_Machine_Init(void){
    /* 初始化FOC电机参数 */
    FOC_Param_Init(&Foc_Motor, U_BUS, ARR);
    FOC_Vf_Control_Param_Init(&Foc_Motor.Vf_Param, 30.0f, 0.0f, 0.001f, FOC_CLK_PERIOD);

    /* 初始化编码器参数 */
    Encoder_Param_Init(&Encoder_Motor, ENCODER_RES_CALCULATE, MOTOR_PN, ENCODER_DIR, ENCODER_ERROR);
    Encoder_Read_Data_Register(&Encoder_Motor, MT6701_ReadData);

    /* 获取校准值 */
    Encoder_Update(&Encoder_Motor);
    Screen_Step_Mode_Init(&Encoder_Motor);

    /* 初始化ADC参数 */
    My_ADC_BSP_Param_Init(&ADC_BSP_Param, ADC_CURRENT_GAIN, ADC_SHUNT_RES);
    My_ADC_Init(&ADC_Current, ADC_1);
    My_ADC_Channel_Init(&ADC_IU, ADC_1);
    My_ADC_Channel_Init(&ADC_IV, ADC_1);
    My_ADC_Channel_Init(&ADC_IW, ADC_1);

    /* ADC自校准，补偿内部失调误差，必须在HAL_ADC_Init之后、启动转换前调用 */
    My_ADC_Calibration_Start(&ADC_Current, ADC_SINGLE_ENDED);

    /* 初始化ADC转换完成回调函数 */
    My_ADC_InjectedConvCpltCallback_Register(&ADC_Current, My_ADC_InjectedConvCpltCallback);

    /* 初始化ADC注入模式 */
    My_ADC_InjectedStart_IT(&ADC_Current);

    /* 初始化定时器 */
    Timer_Init(&TIM_Global, Timer_6);
    Timer_Init(&TIM_FOC, Timer_1);
    Timer_Set_CompareValue(&TIM_FOC, (uint32_t)FOC_PWM_TRIGGER_CCR, TIM_CHANNEL_4);
    // Timer_Comple_PWM_Start(&TIM_FOC, TIM_CHANNEL_1);
    // Timer_Comple_PWM_Start(&TIM_FOC, TIM_CHANNEL_2);
    // Timer_Comple_PWM_Start(&TIM_FOC, TIM_CHANNEL_3);
    Timer_PWM_Start(&TIM_FOC, TIM_CHANNEL_4);
    Timer_Start_Base_IT(&TIM_Global, Timer_Global_PeriodElapsedCallback);

    /* 初始化软定时器 */
    SoftTimer_Init(&Soft_Timer_Vofa, SOFTTIMER_MODE_PERIODIC, 1);
    SoftTimer_Start(&Soft_Timer_Vofa);
    SoftTimer_Init(&Soft_Timer_Encoder_Count_Mode, SOFTTIMER_MODE_PERIODIC, 20);
    SoftTimer_Start(&Soft_Timer_Encoder_Count_Mode);

    /* 初始化按键 */
    Key_Init(&Key_1, KEY_LEVEL_LOW);
    Key_SetClickTimeout(&Key_1, 0);

    /* 注册按键事件回调函数 */
    Key_SetEventCallback(&Key_1, Key_Global_Event_Callback);

    /* 初始化Vofa通道 */
    Vofa_Init(CDC_Transmit_FS);

    Vofa_Ch_Register(3,&ADC_IU_Ch, &ADC_IV_Ch, &ADC_IW_Ch);
    Vofa_Data_Register(3, &ADC_IU.Current, &ADC_IV.Current, &ADC_IW.Current);

    /* 初始化串口 */
    Serial_Init(&Serial_Screen, Serial_3);
}

void State_Machine_Loop(void){
    /* 管理Vofa的发送 */
    if(SoftTimer_Trigger(&Soft_Timer_Vofa)){
        Vofa_Send();                            
    }

    /* 管理编码器计数值的发送 */
    if(SoftTimer_Trigger(&Soft_Timer_Encoder_Count_Mode)){
        Screen_Step_Mode_Send(&Serial_Screen, &Encoder_Motor);
    }

    /* 管理按键事件 */
    Key_Trigger_Event(&Key_1);
}