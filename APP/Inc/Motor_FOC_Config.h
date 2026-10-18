#ifndef __MOTOR_FOC_CONFIG_H__
#define __MOTOR_FOC_CONFIG_H__

/*===================== 电机参数 ====================== */
#define U_BUS 24.0f                     //母线电压
#define ARR 4250.0f                     //自动重装值

/*===================== FOC参数 ====================== */
#define FOC_CLK 20000.0f                //FOC时钟频率，单位：Hz
#define FOC_CLK_PERIOD (1.0f / FOC_CLK) //FOC时钟周期，单位：s

#endif