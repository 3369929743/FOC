#include "vofa.h"
#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#define VOFA_CH_MAX 8    //最大通道数
#define VOFA_TAIL_BYTE 4 //尾部字节数

static const union {
    uint32_t i;
    uint8_t  b[4];
} u = {0x01000000};

#define IS_LITTLE_ENDIAN  (u.b[0] == 0x00)

typedef uint8_t (*Vofa_FuncSend_t)(uint8_t *Data, uint16_t Len);

typedef struct{
    uint8_t Ch_Count;
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

void Vofa_Data_Register(uint8_t Data_num, ...){
    va_list args;
    va_start(args, Data_num);
    for (uint8_t i = 0; i < Data_num && i < Vofa_Manager.Ch_Count; i++) {
        Vofa_Manager.Ch[i]->Datap = va_arg(args, float *);
    }
    va_end(args);
}

void Vofa_Ch_Data_Register(Vofa_Ch_t *ch, float *Datap){
    if(ch != NULL){
        ch->Datap = Datap;
    }
}

uint8_t Vofa_Send(void)
{
    if (Vofa_SendData == NULL) return 0;
    uint8_t  Ch_Count = Vofa_Manager.Ch_Count;
    if (Ch_Count == 0) return 0;

    uint8_t   *p  = Vofa_Manager.Tx_Buffer;
    Vofa_Ch_t **ch = Vofa_Manager.Ch;
    uint16_t   total_len = Ch_Count * VOFA_CH_BYTE + VOFA_TAIL_BYTE;

    static float Default_Dataf = NAN;

    if (IS_LITTLE_ENDIAN) {
        for (uint8_t i = 0; i < Ch_Count; i++) {
            float *Datap = ch[i]->Datap != NULL ? ch[i]->Datap : &Default_Dataf;
            memcpy(p, Datap, sizeof(float));
            p += VOFA_CH_BYTE;
        }
    } else {
        for (uint8_t i = 0; i < Ch_Count; i++) {
            uint32_t Temp;
            float *Datap = ch[i]->Datap != NULL ? ch[i]->Datap : &Default_Dataf;
            memcpy(&Temp, Datap, sizeof(Temp));
            p[0] = (uint8_t)(Temp >> 0);
            p[1] = (uint8_t)(Temp >> 8);
            p[2] = (uint8_t)(Temp >> 16);
            p[3] = (uint8_t)(Temp >> 24);
            p += VOFA_CH_BYTE;
        }
    }

    memcpy(p, Vofa_Tail, VOFA_TAIL_BYTE);
    Vofa_SendData(Vofa_Manager.Tx_Buffer, total_len);
    return 1;
}