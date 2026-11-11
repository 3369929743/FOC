#ifndef __MY_MATH_H__
#define __MY_MATH_H__

/* =========================宏定义开关========================= */
// #define USE_AUTO_TABLE 1
 #define USE_FLOAT_TABLE 1
/* ========================================================== */

#if USE_AUTO_TABLE
void My_math_Table_Create(void);
#endif

float My_math_Sin(float Theta_e);
float My_math_Cos(float Theta_e);
void My_math_Get_Sin_Cos(float Theta_e, float *sin, float *cos);

#endif