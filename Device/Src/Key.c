/**
 ******************************************************************************
 * @file           : Key.c
 * @brief          : 按键状态机模块，支持消抖、单击、双击、长按及长按重复事件
 ******************************************************************************
 * @使用方法
 * 
 * 1. 定义按键变量和回调函数
 *    Key_t MyKey;
 *    void MyKey_Callback(Key_t *Key, Key_Event_e Event) {
 *        switch(Event) {
 *            case KEY_EVENT_ONCE_PRESS:    // 单击处理
 *                break;
 *            case KEY_EVENT_DOUBLE_PRESS:  // 双击处理
 *                break;
 *            case KEY_EVENT_LONG_PRESS:    // 长按处理
 *                break;
 *            case KEY_EVENT_LONG_PRESS_REPEAT:  // 长按重复处理
 *                break;
 *            default:
 *                break;
 *        }
 *    }
 * 
 * 2. 初始化按键对象
 *    Key_Init(&MyKey, KEY_LEVEL_LOW);  // KEY_LEVEL_LOW表示按下为低电平
 * 
 * 3. 注册事件回调函数
 *    Key_SetEventCallback(&MyKey, MyKey_Callback);
 * 
 * 4. 在定时器中断中周期性调用Key_Tick进行状态更新
 *    void Timer_Callback(Timer_t *Timer) {
 *        Key_Tick(&MyKey, HAL_GPIO_ReadPin(GPIOx, GPIO_PIN_x));
 *    }
 * 
 * 5. 在while(1)主循环中调用Key_Trigger_Event触发事件回调
 *    while(1) {
 *        Key_Trigger_Event(&MyKey);
 *    }
 * 
 * 完整示例:
 *    // 定义按键实例
 *    Key_t Key_B_13;
 *    
 *    // 定义按键事件回调函数
 *    void Key_B_13_Press_Callback(Key_t *Key, Key_Event_e Event) {
 *        switch(Event) {
 *            case KEY_EVENT_ONCE_PRESS:
 *                LED_ON(&User_LED);
 *                break;
 *            case KEY_EVENT_DOUBLE_PRESS:
 *                LED_OFF(&User_LED);
 *                break;
 *            case KEY_EVENT_LONG_PRESS:
 *                LED_BLINK(&User_LED, 100, 100, 5);
 *                break;
 *            default:
 *                break;
 *        }
 *    }
 *    
 *    // 在初始化代码中
 *    Key_Init(&Key_B_13, KEY_LEVEL_LOW);
 *    Key_SetEventCallback(&Key_B_13, Key_B_13_Press_Callback);
 *    
 *    // 在定时器回调中更新按键状态（非阻塞）
 *    void Timer_TickCallback(Timer_t *Timer) {
 *        Key_Tick(&Key_B_13, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));
 *    }
 *    
 *    // 在while(1)主循环中触发事件
 *    while(1) {
 *        Key_Trigger_Event(&Key_B_13);
 *    }
 * 
 * @note   - Key_Tick()需在定时器中断中周期性调用，默认周期为1ms
 *         - 消抖时间默认10ms，可通过修改SHAKE_TIME宏调整
 *         - 长按时间默认1000ms，可通过Key_SetLongPressTimeout()修改
 *         - 连击超时默认200ms，可通过Key_SetClickTimeout()修改
 *         - 长按重复触发间隔默认100ms，可通过Key_SetLongPressRepeatTimeout()修改
 *         - 按键事件回调在while(1)中触发，避免在中断中执行耗时操作
 ******************************************************************************
 */
#include "Key.h"
#include <stddef.h>

#define TICK_TIME 1                                 // 每次调用的间隔时间（定时器中断周期）
#define SHAKE_TIME (10 / TICK_TIME)                 // 消抖时间
#define CLICK_TIMEOUT (200 / TICK_TIME)             // 默认连击超时时间
#define LONG_PRESS_TIME (1000 / TICK_TIME)          // 默认长按时间
#define LONG_PRESS_REPEAT_TIME (100 / TICK_TIME)    // 长按重复事件触发间隔

/**
 * @brief 保存按键事件到按键对象
 * @param Key 按键对象指针
 * @param Event 要保存的按键事件类型
 * @note 该函数为内联函数，用于将事件写入按键对象的事件缓冲区
 */
static inline void Key_Event_Save(Key_t *Key, Key_Event_e Event){
    Key->Event = Event;
}

/**
 * @brief 初始化按键对象
 * @param Key 按键对象指针
 * @param Press_Level 按键按下时的电平状态
 * @note 将按键状态机复位到空闲状态，并设置默认的超时时间
 */
void Key_Init(Key_t *Key, uint8_t Press_Level){
    if(Key == NULL) return;
    Key->State = KEY_STATE_IDLE;
    Key->Press_Level = Press_Level;
    Key->Time_Count = 0;
    Key->Filter_Count = 0;
    Key->Filter_Press = 0;
    Key->Click_Count = 0;
    Key->Event = KEY_EVENT_NONE;
    Key->Click_Timeout = CLICK_TIMEOUT;
    Key->Long_Press_Timeout = LONG_PRESS_TIME;
    Key->Long_Press_Repeat_Timeout = LONG_PRESS_REPEAT_TIME;
}

/**
 * @brief 设置按键连击超时时间
 * @param Key 按键对象指针
 * @param Timeout 连击超时时间（单位：Tick次数）
 * @note 用于配置两次按键之间的最大间隔时间，超过该时间则判定为新的按键序列
         将这个时间配置为0表示禁用连击检测
 */
void Key_SetClickTimeout(Key_t *Key, uint16_t Timeout){
    if(Key == NULL) return;
    Key->Click_Timeout = Timeout;
}

/**
 * @brief 设置按键长按超时时间
 * @param Key 按键对象指针
 * @param Timeout 长按超时时间（单位：Tick次数）
 * @note 用于配置判定长按事件的时间阈值，超过该时间则触发长按事件
 */
void Key_SetLongPressTimeout(Key_t *Key, uint16_t Timeout){
    if(Key == NULL) return;
    Key->Long_Press_Timeout = Timeout;
}

/**
 * @brief 设置按键长按重复触发间隔时间
 * @param Key 按键对象指针
 * @param Timeout 长按重复触发间隔时间（单位：Tick次数）
 * @note 用于配置长按状态下重复触发事件的间隔时间，超过该时间则再次触发长按事件
         将这个时间配置为0表示禁用长按重复触发
 */
void Key_SetLongPressRepeatTimeout(Key_t *Key, uint16_t Timeout){
    if(Key == NULL) return;
    Key->Long_Press_Repeat_Timeout = Timeout;
}

/**
 * @brief 设置按键事件回调函数
 * @param Key 按键对象指针
 * @param Callback 按键事件回调函数指针，当按键事件触发时会被调用
 * @note 用于注册自定义的按键事件处理函数，实现按键事件的异步处理
 */
void Key_SetEventCallback(Key_t *Key, void (*Callback)(Key_t *Key, Key_Event_e Event)){
    if(Key == NULL) return;
    Key->Key_EventCallback = Callback;
}

/**
 * @brief 按键状态机处理函数（需在定时器中断中周期性调用）
 * @param Key 按键对象指针
 * @param Cur_Level 当前检测到的按键电平状态
 * @note 该函数实现按键消抖、状态机流转及事件触发，包含以下状态：
 *       - KEY_STATE_IDLE: 空闲状态，等待按键按下
 *       - KEY_STATE_HOLD: 保持状态，判断是长按还是短按
 *       - KEY_STATE_WAIT_CLICK: 等待连击状态，统计连击次数
 *       - KEY_STATE_LONG_PRESS: 长按状态，检测长按重复事件
 */
void Key_Tick(Key_t *Key, uint8_t Cur_Level){
    if(Key == NULL) return;
    uint8_t is_press = (Cur_Level == Key->Press_Level) ? 1 : 0;

    // 按键消抖处理：连续检测到相同状态超过消抖时间才确认状态变化
    if(is_press != Key->Filter_Press){
        Key->Filter_Count ++;
        if(Key->Filter_Count >= SHAKE_TIME){
            Key->Filter_Count = 0;
            Key->Filter_Press = is_press;
        }
    }
    else {
        Key->Filter_Count = 0;
    }

    // 按键状态机处理
    switch(Key->State){
        case KEY_STATE_IDLE:  // 空闲状态
            if(Key->Filter_Press){
                Key->State = KEY_STATE_HOLD;
                Key->Time_Count = 0;
            }
            break;
        case KEY_STATE_HOLD:  // 保持状态，判断长按或短按
            if(Key->Filter_Press){
                Key->Time_Count ++;
                if(Key->Time_Count >= Key->Long_Press_Timeout){
                    Key_Event_Save(Key, KEY_EVENT_LONG_PRESS);
                    Key->State = KEY_STATE_LONG_PRESS;
                    Key->Time_Count = 0;
                }
            }
            else{
                Key->Click_Count ++;
                Key->State = KEY_STATE_WAIT_CLICK;
                Key->Time_Count = 0;
            }
            break;
        case KEY_STATE_WAIT_CLICK:  // 等待连击状态
            if(Key->Filter_Press){
                Key->State = KEY_STATE_HOLD;
                Key->Time_Count = 0;
            }
            else{
                Key->Time_Count ++;
                if(Key->Time_Count >= Key->Click_Timeout){
                    switch(Key->Click_Count){
                        case 1:
                            Key_Event_Save(Key, KEY_EVENT_ONCE_PRESS);
                            break;
                        case 2:
                            Key_Event_Save(Key, KEY_EVENT_DOUBLE_PRESS);
                            break;
                        default:
                            break;
                    }
                    Key->State = KEY_STATE_IDLE;
                    Key->Time_Count = 0;
                    Key->Click_Count = 0;
                }
            }
            break;
        case KEY_STATE_LONG_PRESS:  // 等待长按状态（长按后）
            if(Key->Filter_Press){
                Key->Time_Count ++;
                if(Key->Time_Count >= Key->Long_Press_Repeat_Timeout){
                    Key_Event_Save(Key, KEY_EVENT_LONG_PRESS_REPEAT);
                    Key->Time_Count = 0;
                }
            }
            else{
                Key->State = KEY_STATE_IDLE;
                Key->Time_Count = 0;
                Key->Click_Count = 0;
            }
            break;
        default:
            Key->State = KEY_STATE_IDLE;
            Key->Time_Count = 0;
            Key->Click_Count = 0;
            break;
    }
}

/**
 * @brief 获取按键事件并清除事件标志
 * @param Key 按键对象指针
 * @return 按键事件类型，如果没有事件则返回KEY_EVENT_NONE
 * @note 该函数会读取当前按键事件并自动清除事件标志，确保事件不会被重复读取
 */
Key_Event_e Key_GetEvent(Key_t *Key){
    if(Key == NULL) return KEY_EVENT_NONE;
    Key_Event_e Event = Key->Event;
    if(Event == KEY_EVENT_NONE) return KEY_EVENT_NONE;
    Key->Event = KEY_EVENT_NONE;  // 获取事件后清除事件标志
    return Event;
}

/**
 * @brief 触发按键事件回调函数
 * @param Key 按键对象指针
 * @note 该函数会获取当前按键事件，如果事件有效且已注册回调函数，则调用回调函数处理事件
 */
void Key_Trigger_Event(Key_t *Key){
    if(Key == NULL) return;
    if(Key->Event == KEY_EVENT_NONE) return;
    if(Key->Key_EventCallback == NULL) return;
    Key_Event_e Event = Key->Event;
    Key->Event = KEY_EVENT_NONE;
    Key->Key_EventCallback(Key, Event);
}