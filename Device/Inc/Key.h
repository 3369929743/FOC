#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>

#define KEY_LEVEL_LOW 0
#define KEY_LEVEL_HIGH 1

typedef enum{
    KEY_STATE_IDLE = 0,               // 空闲状态
    KEY_STATE_HOLD = 1,               // 按住状态
    KEY_STATE_WAIT_CLICK = 2,         // 等待连击
    KEY_STATE_LONG_PRESS = 3,         // 长按状态
} Key_State_e;

typedef enum{
    KEY_EVENT_NONE = 0,               // 无事件
    KEY_EVENT_ONCE_PRESS = 1,         // 单击事件
    KEY_EVENT_DOUBLE_PRESS = 2,       // 双击事件
    KEY_EVENT_LONG_PRESS = 3,         // 长按事件
    KEY_EVENT_LONG_PRESS_REPEAT = 4,  // 长按重复事件
} Key_Event_e;

typedef struct Key_Struct{
    uint8_t Press_Level;                    // 按键按下时的电平

    uint8_t Filter_Count;                   // 消抖计数器
    uint8_t Filter_Press;                   // 消抖后的按键是否按下

    Key_State_e State;                      // 按键状态

    uint16_t Time_Count;                    // 按键按下时间计数器
    uint8_t Click_Count;                    // 连击计数器
    uint16_t Click_Timeout;                 // 连击超时时间计数器
    uint16_t Long_Press_Timeout;            // 长按超时时间计数器
    uint16_t Long_Press_Repeat_Timeout;     // 长按重复事件超时时间计数器

    volatile Key_Event_e Event; // 按键事件

    void (*Key_EventCallback)(struct Key_Struct *Key, Key_Event_e Event); // 按键事件回调函数指针
} Key_t;

void Key_Init(Key_t *Key, uint8_t Press_Level);
void Key_SetClickTimeout(Key_t *Key, uint16_t Timeout);
void Key_SetLongPressTimeout(Key_t *Key, uint16_t Timeout);
void Key_SetLongPressRepeatTimeout(Key_t *Key, uint16_t Timeout);
void Key_SetEventCallback(Key_t *Key, void (*Callback)(Key_t *Key, Key_Event_e Event));
Key_Event_e Key_GetEvent(Key_t *Key);

void Key_Tick(Key_t *Key, uint8_t Cur_Level);
void Key_Trigger_Event(Key_t *Key);

#endif