#ifndef __MOTOR_FOC_CONFIG_H__
#define __MOTOR_FOC_CONFIG_H__

/*===================== 电机参数 ====================== */
#define U_BUS 24.0f                                     //母线电压
#define ARR 4249.0f                                     //自动重装值
#define ARR_CALCULATE (ARR + 1)                         //用于计算的自动重装值，为了不丢掉0的精度，加一

#define MOTOR_PN 10                                     //电机极对数

/*===================== FOC参数 ====================== */
#define FOC_PWM_TRIGGER_CCR (ARR - 1)                   //FOC PWM触发比较值
#define FOC_CLK 20000.0f                                //FOC时钟频率，单位：Hz
#define FOC_CLK_PERIOD (1.0f / FOC_CLK)                 //FOC时钟周期，单位：s

/*===================== 编码器参数 ====================== */
#define ENCODER_RES 16383.0f                            //编码器分辨率，单位：计数/转
#define ENCODER_RES_CALCULATE (ENCODER_RES + 1)         //用于FOC计算的编码器分辨率，单位：计数/转
#define ENCODER_DIR 0.0f                                //编码器方向
#define ENCODER_ERROR 12498                             //编码器零点误差

/*===================== ADC参数 ====================== */
#define ADC_CURRENT_GAIN 20                             //运放增益
#define ADC_SHUNT_RES 0.001f                            // 采样电阻，单位：Ω

#endif
