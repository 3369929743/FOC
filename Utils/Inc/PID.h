#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

#define PID_USE_FLOAT 1
// #define PID_USE_FLOAT 0

#if PID_USE_FLOAT
    typedef float PID_val;
#else
    typedef int32_t PID_val;
    #define MULTIPLE 10
    #define PID_ZOOM (1 << MULTIPLE)
#endif

typedef struct
{
    PID_val Kp, Ki, Kd;

    PID_val Pre_Error, Cur_Error, ErrorInt, IntMax, IntMin;

    PID_val Actual, Target;

    PID_val OutMax, OutMin;

    PID_val Output;

    PID_val Alpha, Error_Rate_Filter;
} PID_t;

typedef struct {
    float Kp, Ki, Kd;

    float IntMax, IntMin;

    float OutMax, OutMin;

    float Alpha;
}PID_Confg_t;

void PID_Init(PID_t *PID, PID_Confg_t *Config);
void PID_Clear(PID_t *PID);
void PID_Set_Target(PID_t *PID, PID_val Target);
void PID_Set_Actual(PID_t *PID, PID_val Actual);
PID_val PID_Get_Output(PID_t *PID);
PID_val PID_Calculate(PID_t *PID, PID_val Actual);
void PID_Change_Param(PID_t *PID, float Kp, float Ki, float Kd, float Alpha);

#endif