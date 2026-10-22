#include "Vofa.h"
#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#define VOFA_CH_MAX 8    //最大通道数
#define VOFA_TAIL_BYTE 4 //尾部字节数

#define IS_LITTLE_ENDIAN 1
//#define IS_LITTLE_ENDIAN 0

typedef uint8_t (*Vofa_FuncSend_t)(uint8_t *Data, uint16_t Len);

typedef struct{
    uint8_t Ch_Count;
    uint8_t Data_Register_Index;

    Vofa_Ch_t *Ch[VOFA_CH_MAX];
    uint8_t Tx_Buffer[VOFA_CH_MAX * VOFA_CH_BYTE + VOFA_TAIL_BYTE];
}Vofa_Manager_t;

static Vofa_Manager_t Vofa_Manager = {0};
static uint8_t Vofa_Tail[VOFA_TAIL_BYTE] = {0x00, 0x00, 0x80, 0x7f};
static Vofa_FuncSend_t Vofa_SendData = NULL;

/**
 * @brief  初始化 Vofa 模块，注册底层数据发送回调
 * @param  Vofa_FuncSend  发送函数指针，原型 void(uint8_t *Data, uint16_t Len)
 */
void Vofa_Init(Vofa_FuncSend_t Vofa_FuncSend){
    Vofa_SendData = Vofa_FuncSend;
}

/**
 * @brief  注册Vofa通道，将通道结构体依次加入发送队列
 * @param  Ch_num: 本次注册的通道数量
 * @param  ...: 可变参数，依次传入 Vofa_Ch_t* 指针，数量须与 Ch_num 一致
 * @note   通道按注册顺序排列，后续通过 Vofa_Data_Register 按相同顺序绑定数据源。
 *         超出 VOFA_CH_MAX 的通道将被忽略，传入 NULL 也会被跳过。
 */
void Vofa_Ch_Register(uint8_t Ch_num, ...){
    va_list args;
    va_start(args, Ch_num);
    for(uint8_t i = 0; i < Ch_num; i++){
        Vofa_Ch_t *ch = va_arg(args, Vofa_Ch_t*);
        if(ch != NULL && Vofa_Manager.Ch_Count < VOFA_CH_MAX){
            Vofa_Manager.Ch[Vofa_Manager.Ch_Count++] = ch;
        }
    }
    va_end(args);
}

/**
 * @brief  绑定Vofa通道的数据源，按注册顺序依次关联 float 变量指针
 * @param  Data_num: 本次绑定的数据源数量
 * @param  ...: 可变参数，依次传入 float* 指针，数量须与 Data_num 一致
 * @note   绑定的顺序必须与 Vofa_Ch_Register 注册通道的顺序一致。
 *         内部使用 Data_Register_Index 记录当前位置，可多次调用分批绑定。
 *         超出已注册通道数的参数将被忽略。
 */
void Vofa_Data_Register(uint8_t Data_num, ...){
    va_list args;
    va_start(args, Data_num);
    for (uint8_t i = 0;
         i < Data_num && Vofa_Manager.Data_Register_Index < Vofa_Manager.Ch_Count;
         i++) {
        Vofa_Manager.Ch[Vofa_Manager.Data_Register_Index++]->Datap = va_arg(args, float *);
    }
    va_end(args);
}

/**
 * @brief  单独为某个已注册的通道绑定 float 数据源
 * @param  ch: 指向目标通道结构体的指针
 * @param  Datap: 指向 float 变量的指针，作为该通道的数据源
 * @note   与 Vofa_Data_Register 不同，此函数可指定任意已注册通道，无需按顺序批量绑定。
 */
void Vofa_Ch_Data_Register(Vofa_Ch_t *ch, float *Datap){
    if(ch != NULL){
        ch->Datap = Datap;
    }
}

/**
 * @brief  发送所有已注册通道的 float 数据到上位机（Vofa/FireWater 协议）
 * @retval uint8_t: 返回1表示发送成功，返回0表示发送失败（未注册发送函数或无通道）
 * @note   数据格式为小端 float 数组 + 4字节尾部（0x00 0x00 0x80 0x7f）。
 *         未绑定数据源的通道默认发送 NaN。
 *         发送完成后不自动复位 Data_Register_Index，允许重复发送同一组数据。
 */
uint8_t Vofa_Send(void)
{
    if (Vofa_SendData == NULL) return 0;
    uint8_t  Ch_Count = Vofa_Manager.Ch_Count;
    if (Ch_Count == 0) return 0;

    uint8_t   *p  = Vofa_Manager.Tx_Buffer;
    Vofa_Ch_t **ch = Vofa_Manager.Ch;
    uint16_t   total_len = Ch_Count * VOFA_CH_BYTE + VOFA_TAIL_BYTE;

    static float Default_Dataf = NAN;

#if IS_LITTLE_ENDIAN
    for(uint8_t i = 0; i < Ch_Count; i++){
            float *Datap = ch[i]->Datap != NULL ? ch[i]->Datap : &Default_Dataf;
            memcpy(p, Datap, sizeof(float));
            p += VOFA_CH_BYTE;
        }
#else
    for(uint8_t i = 0; i < Ch_Count; i++){
        uint32_t Temp;
        float *Datap = ch[i]->Datap != NULL ? ch[i]->Datap : &Default_Dataf;
        memcpy(&Temp, Datap, sizeof(Temp));
        p[0] = (uint8_t)(Temp >> 0);
        p[1] = (uint8_t)(Temp >> 8);
        p[2] = (uint8_t)(Temp >> 16);
        p[3] = (uint8_t)(Temp >> 24);
        p += VOFA_CH_BYTE;
    }
#endif
    memcpy(p, Vofa_Tail, VOFA_TAIL_BYTE);
    Vofa_SendData(Vofa_Manager.Tx_Buffer, total_len);
    return 1;
}