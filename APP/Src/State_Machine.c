/* ====================== 状态机头文件 ====================== */
#include "State_Machine.h"
#include "Motor_FOC_Config.h"
#include "MyADC.h"
#include "PID.h"
#include "Timer.h"
#include "Vofa.h"
#include "SoftTimer.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include "Serial.h"
#include "Key.h"

/* ====================== PMSM控制头文件 ====================== */
#include "PMSM.h"

/* ====================== 实现功能头文件 ====================== */
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

/* ====================== 全局变量 ====================== */


/*-------------------- PMSM电机实例 -------------------- */
static PMSM_t PMSM;

/*-------------------- PID配置参数 -------------------- */
// static PID_Confg_t PID_Id_Config = {6.28f, .IntMax = 8.0f, .OutMax = 12.0f};
// static PID_Confg_t PID_Iq_Config = {6.28f, .IntMax = 8.0f, .OutMax = 12.0f};

static PID_Confg_t PID_Id_Config = {.Kp = 0.01f, .IntMax = 0.0f, .OutMax = 1.2f};
static PID_Confg_t PID_Iq_Config = {.Kp = 0.01f, .IntMax = 0.0f, .OutMax = 1.2f};

/*-------------------- PMSM电机配置参数 -------------------- */
static PMSM_Config_t PMSM_Config = {
    /* 编码器读取函数 */
    .Read_Data = MT6701_ReadData,
    /* PID配置参数 */
    .PID_Id_Config = &PID_Id_Config,
    .PID_Iq_Config = &PID_Iq_Config,
    /* 定时器通道配置 */
    .Timer_Channel_Vu = TIM_CHANNEL_1,
    .Timer_Channel_Vv = TIM_CHANNEL_2,
    .Timer_Channel_Vw = TIM_CHANNEL_3,
    .Timer_Channel_ADC = TIM_CHANNEL_4,
    .Timer_Cmp_Trigger_ADC = FOC_PWM_TRIGGER_CCR,
    .Timer_Num = Timer_1,
    /* ADC参数配置 */
    .ADC_Num = ADC_1,
    .ADC_Current_Gain = ADC_CURRENT_GAIN,
    .ADC_Shunt_Res = ADC_SHUNT_RES,
    .SingleDiff = ADC_SINGLE_ENDED,
    /* 编码器参数配置 */
    .Encoder_Dir = ENCODER_DIR,
    .Encoder_Res = ENCODER_RES_CALCULATE,
    .Encoder_Error = ENCODER_ERROR,
    /* 电机参数配置 */
    .Motor_Pn = MOTOR_PN,
    .Ubus = U_BUS,
    .TARR = ARR_CALCULATE,
};

/*-------------------- 定时器实例 -------------------- */
static Timer_t TIM_Global;

/*-------------------- 软定时器实例 -------------------- */
static SoftTimer_t Soft_Timer_Vofa;

/*-------------------- Vofa通道实例 -------------------- */
Vofa_Ch_t ADC_Id_Ch, ADC_Iq_Ch;
Vofa_Ch_t ADC_Vd_Ch, ADC_Vq_Ch;
Vofa_Ch_t FOC_Theta_Ch;

/*-------------------- 串口实例 -------------------- */
static Serial_t Serial_Screen;

/*-------------------- 按键实例 -------------------- */
static Key_t Key_1;
static Key_t Key_2;

/* ====================== 定时器回调函数 ====================== */
static void Timer_Global_PeriodElapsedCallback(Timer_t *timer)
{
    if(timer == &TIM_Global){
        SoftTimer_Update(&Soft_Timer_Vofa);
        Key_Tick(&Key_1, HAL_GPIO_ReadPin(Key_1_GPIO_Port, Key_1_Pin));
        Key_Tick(&Key_2, HAL_GPIO_ReadPin(Key_2_GPIO_Port, Key_2_Pin));
    }
}

/* ====================== ADC回调函数 ====================== */
void My_ADC_InjectedConvCpltCallback(ADC_t* adc){
    if(adc == &PMSM.ADC){
        /* 读取ADC值 */
        My_ADC_Get_InjectedValue(&PMSM.ADC_IU_Ch, ADC_INJECTED_RANK_3);
        My_ADC_Get_InjectedValue(&PMSM.ADC_IV_Ch, ADC_INJECTED_RANK_2);
        My_ADC_Get_InjectedValue(&PMSM.ADC_IW_Ch, ADC_INJECTED_RANK_1);

        /* 运行PMSM电机 */
        PMSM_Run(&PMSM);
    }
}

/*-------------------- 按键事件函数 -------------------- */
void Key_Global_Event_Callback(Key_t *Key, Key_Event_e Event){
    if(Key == NULL) return;
    if(Key == &Key_1){
        switch (Event) {
            case KEY_EVENT_ONCE_PRESS:
                PMSM_CALIBRATION_Mode(&PMSM);
                break;
            case KEY_EVENT_DOUBLE_PRESS:
                break;
            case KEY_EVENT_LONG_PRESS:
                break;
            default:
                break;
        }
    }
    if(Key == &Key_2){
        switch (Event) {
            case KEY_EVENT_ONCE_PRESS:
                PMSM.PID_Iq.Target += 0.05f;
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
    /* 初始化ADC转换完成回调函数 */
    My_ADC_InjectedConvCpltCallback_Register(&PMSM.ADC, My_ADC_InjectedConvCpltCallback);

    /* 初始化PMSM电机 */
    if(PMSM_Init(&PMSM, &PMSM_Config) != PMSM_OK){
        Error_Handler();
    }
    PMSM_IDLE_Mode(&PMSM);

    /* 初始化按键 */
    Key_Init(&Key_1, KEY_LEVEL_LOW);
    Key_Init(&Key_2, KEY_LEVEL_LOW);
    Key_SetClickTimeout(&Key_1, 0);
    Key_SetClickTimeout(&Key_2, 0);

    /* 注册按键事件回调函数 */
    Key_SetEventCallback(&Key_1, Key_Global_Event_Callback);
    Key_SetEventCallback(&Key_2, Key_Global_Event_Callback);

    /* 初始化Vofa通道 */
    Vofa_Init(CDC_Transmit_FS);
    /* 注册Vofa通道 */
    Vofa_Ch_Register(4, &ADC_Id_Ch, &ADC_Iq_Ch, &ADC_Vd_Ch, &ADC_Vq_Ch);
    Vofa_Ch_Register(1, &FOC_Theta_Ch);
    /* 设置Vofa通道数据 */
    Vofa_Data_Register(4, &PMSM.Foc.Id, &PMSM.Foc.Iq, &PMSM.Foc.Vd, &PMSM.Foc.Vq);
    Vofa_Data_Register(1, &PMSM.Foc.Theta);

    /* 初始化串口 */1
    Serial_Init(&Serial_Screen, Serial_3);

    /* 初始化定时器 */
    Timer_Init(&TIM_Global, Timer_6);
    Timer_Start_Base_IT(&TIM_Global, Timer_Global_PeriodElapsedCallback);

    /* 初始化软定时器 */
    SoftTimer_Init(&Soft_Timer_Vofa, SOFTTIMER_MODE_PERIODIC, 1);
    SoftTimer_Start(&Soft_Timer_Vofa);

    // PID_Set_Target(&PMSM.PID_Iq, 0.1f);
}

void State_Machine_Loop(void){
    /* 管理Vofa的发送 */
    if(SoftTimer_Trigger(&Soft_Timer_Vofa)){
        Vofa_Send();                            
    }

    /* 管理按键事件 */
    Key_Trigger_Event(&Key_1);
}