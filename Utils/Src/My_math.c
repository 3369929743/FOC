#include "My_math.h"
#include <stdint.h>

#if USE_AUTO_TABLE
#include <math.h>

#define __2PI 6.2831853f
#define __PI_DIV_2 1.5707963f
#define SIN_TABLE_SIZE 1024                    //正弦表的大小
#define SIN_VAL_SCALE 32767                    //正弦值的放大倍数,刚好是int16_t的范围
#define ANGLE_SCALE (65536 / __2PI)            //角度的映射关系，将角度映射到0-65535的范围，（2PI / 65536）为单位角度，角度除以单位角度为映射后的角度
#define SIN_INT_SCALE (1 / SIN_VAL_SCALE)      //最后把计算出来的正弦值还原回去，乘倒数比浮点数快
#define INDEX_SCALE (SIN_TABLE_SIZE / __2PI)   //索引的映射关系，将角度映射到0-1023的范围，（2PI / 1024）为单位角度，角度除以单位角度为映射后的索引
#define TABLE_MASK (SIN_TABLE_SIZE - 1)        //正弦表的掩码，用于将索引限制在0-1023的范围内

float sin_table[SIN_TABLE_SIZE];
void My_math_Table_Create(void){
    //创建正弦表
    for(int i = 0; i < SIN_TABLE_SIZE; i++){
        sin_table[i] = sinf(i * __2PI / SIN_TABLE_SIZE);
    }
}
#else
#include "sin_table.h"
#endif

#if USE_FLOAT_TABLE
static inline float Calculate_Sin(float Theta_e){
    float Index_float = (Theta_e * INDEX_SCALE);            //将角度映射到0-1023的范围
    uint32_t Index_int = (uint32_t)Index_float ;            // 取整，得到索引
    float fraction = Index_float - Index_int;               // 计算小数部分,在归一化之前计算出小数，不然会有问题
    uint32_t Next_Index = (Index_int + 1) & TABLE_MASK;     // 计算下一个索引
    Index_int &= TABLE_MASK;                                // 索引限制在0-1023的范围内

    float y0 = sin_table[Index_int];                        // 获取当前索引的正弦值
    float y1 = sin_table[Next_Index];                       // 获取下一个索引的正弦值

    return y0 + (y1 - y0) * fraction;                       // 计算结果  
}

float My_math_Sin(float Theta_e){
    return Calculate_Sin(Theta_e);
}

float My_math_Cos(float Theta_e){
    Theta_e += __PI_DIV_2;
    if(Theta_e >= __2PI) Theta_e -= __2PI;
    return Calculate_Sin(Theta_e);
}

void My_math_Get_Sin_Cos(float Theta_e, float *sin, float *cos){
    *sin = Calculate_Sin(Theta_e);

    Theta_e += __PI_DIV_2;
    if(Theta_e >= __2PI) Theta_e -= __2PI;
    *cos = Calculate_Sin(Theta_e);
}
#else
static inline float Calculate_Sin_Int(float Theta_e){
    uint32_t Index_Raw = (uint32_t)(Theta_e * ANGLE_SCALE);     //进行角度映射，如果传进来2pi就会算到65536，回绕到0
    uint16_t Index = Index_Raw & 0xFFFFu;                    // 按位与回绕一下，这里需要保留16位

    /*将角度映射后的结果右移6位，剩下10位作为索引，因为正弦表的大小为1024，然后再用右移的64计算偏移量
    相当于先除以64，拿Flash类比的话，就是分成1024页，每页64字节*/
    uint16_t Index_Int = Index >> 6;     
    uint16_t Next_Index = (Index_Int + 1) & TABLE_MASK;

    int16_t y0 = sin_table_int[Index_Int];
    int16_t y1 = sin_table_int[Next_Index];
    
    /*获取低6位，值为0-63的整数，如果值是63，63/64相当于98.4%，后面直接进位，0%和100%一样
      但是这边不是百分制，相当于扩大64倍*/
    uint16_t fraction = Index & 0x3F; 
    int32_t offset = (y1 - y0) * fraction ;    //计算偏移量吗，乘上x/64的时候，会有小数部分，这边直接取整会有精度损失
    int16_t result = y0 + ((offset + 32) >> 6);
    return result * SIN_INT_SCALE;
}

float My_math_Sin(float Theta_e){
    return Calculate_Sin_Int(Theta_e);
}

float My_math_Cos(float Theta_e){
    Theta_e += __PI_DIV_2;
    if(Theta_e >= __2PI) Theta_e -= __2PI;
    return Calculate_Sin_Int(Theta_e);
}

void My_math_Get_Sin_Cos(float Theta_e, float *sin, float *cos){
    *sin = Calculate_Sin_Int(Theta_e);

    Theta_e += __PI_DIV_2;
    if(Theta_e >= __2PI) Theta_e -= __2PI;
    *cos = Calculate_Sin_Int(Theta_e);
}
#endif
