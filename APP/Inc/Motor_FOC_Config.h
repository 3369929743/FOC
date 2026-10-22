#ifndef __MOTOR_FOC_CONFIG_H__
#define __MOTOR_FOC_CONFIG_H__

/*===================== 电机参数 ====================== */
#define U_BUS 24.0f                     //母线电压
#define ARR 4250.0f                     //自动重装值
#define MOTOR_PN 10                  //电机极对数

/*===================== FOC参数 ====================== */
#define FOC_PWM_TRIGGER_CCR 4249.0f    //FOC PWM触发比较值
#define FOC_CLK 20000.0f                //FOC时钟频率，单位：Hz
#define FOC_CLK_PERIOD (1.0f / FOC_CLK) //FOC时钟周期，单位：s

/*===================== 编码器参数 ====================== */
#define ENCODER_RES 16384.0f             //编码器分辨率，单位：计数/转
#define ENCODER_DIR 0.0f                //编码器方向
#define ENCODER_ERROR 12498              //编码器零点误差

/*===================== ADC参数 ====================== */
#define ADC_BIAS 2048                  //ADC偏置电压，单位：V
#define ADC_CURRENT_GAIN 20            //运放增益
#define ADC_SHUNT_RES 0.001f           // 采样电阻，单位：Ω

#endif
