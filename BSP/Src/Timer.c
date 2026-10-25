/**
 ******************************************************************************
 * @file           : Timer.c
 * @brief          : 软件定时器模块，基于STM32 HAL库TIM中断实现
 ******************************************************************************
 * @使用方法
 * 
 * 1. 定义定时器变量
 *    Timer_t MyTimer;
 * 
 * 2. 定义定时器回调函数
 *    void MyTimer_Callback(Timer_t *Timer) {
 *        // 定时器周期结束时执行的代码
 *    }
 * 
 * 3. 初始化定时器（在MX_TIMx_Init()之后调用）
 *    Timer_Init(&MyTimer, Timer_2);  // Timer_2对应TIM2，根据实际使用的定时器修改
 * 
 * 4. 启动定时器中断
 *    Timer_Start_IT(&MyTimer, MyTimer_Callback);
 * 
 * 完整示例:
 *    // 定义定时器实例
 *    Timer_t Timer_Tick;
 *    
 *    // 定义回调函数
 *    void Timer_TickCallback(Timer_t *Timer) {
 *        // 此处编写定时器中断回调逻辑
 *        // 注意：中断中应避免耗时操作
 *    }
 *    
 *    // 在初始化代码中（USER CODE BEGIN 2区域）
 *    Timer_Init(&Timer_Tick, Timer_2);
 *    Timer_Start_IT(&Timer_Tick, Timer_TickCallback);
 * 
 * @note   - 硬件定时器（TIM2/TIM3等）需在CubeMX中配置并调用MX_TIMx_Init()
 *         - Timer_2/Timer_3等枚举值对应TIMER_LIST中定义的定时器编号
 *         - 回调函数在中断上下文中执行，应避免调用阻塞函数
 *         - 多个软件定时器可复用同一个硬件TIM，但需合理配置周期
 ******************************************************************************
 */

#include "Timer.h"

#ifdef TIMER_MODULE_ENABLED

#define X(Index, Instance) extern __weak TIM_HandleTypeDef htim##Index;
TIMER_LIST
#undef X

static TIM_HandleTypeDef* const TIM_Map[] = {
    #define X(Index, Instance) [Index] = &htim##Index,
    TIMER_LIST
    #undef X
};

#define TIM_MAX_NUM (sizeof(TIM_Map) / sizeof(TIM_Map[0]))

static Timer_t *Timer_Instance[TIM_MAX_NUM] = {NULL};

/**
 * @brief  根据TIM句柄获取对应的定时器实例
 * @param  htim: TIM_HandleTypeDef指针，指向需要查询的定时器句柄
 * @retval Timer_t*: 返回对应的定时器实例指针，如果未找到则返回NULL
 * @note   该函数通过switch-case语句将硬件定时器地址映射到软件定时器实例，
 *         使用X-Macro技术(TIMER_LIST)自动生成case分支，避免手动编写重复代码
 */
static inline Timer_t* Timer_GetInstance(TIM_HandleTypeDef* htim)
{
    switch ((uint32_t)htim->Instance) {
    #define X(Index, Instance) case (uint32_t)(Instance): return Timer_Instance[Index];
    TIMER_LIST
    #undef X
    default:
        return NULL;
    }
}

/**
 * @brief  初始化定时器实例，建立软件定时器与硬件定时器的映射关系
 * @param  timer: 指向待初始化的定时器结构体指针
 * @param  TIM_Num: 定时器编号枚举值，指定要使用的硬件定时器
 * @retval uint8_t: 返回0表示初始化成功，返回1表示参数错误（编号越界或为0）
 * @note   该函数会将定时器实例注册到全局数组中，以便中断回调时快速查找
 */
uint8_t Timer_Init(Timer_t* timer, Timer_Num_e TIM_Num)
{
    /* 检查定时器编号是否合法：不能为0（保留给NULL），不能超出最大数量 */
    if(TIM_Num >= TIM_MAX_NUM || TIM_Num == 0 || TIM_Map[TIM_Num] == NULL)
    {
        return 1;
    }
    /* 将当前定时器实例注册到全局映射数组中 */
    Timer_Instance[TIM_Num] = timer;
    /* 将硬件定时器句柄关联到软件定时器结构体 */
    timer->htim = TIM_Map[TIM_Num];
    /* 初始化回调函数指针为空，等待用户通过Timer_Start_IT设置 */
    timer->PeriodElapsedCallback = NULL;
    timer->TriggerCallback = NULL;
    return 0;
}

void Timer_PWM_Start(Timer_t *timer, uint32_t Channel){
    if(timer == NULL || timer->htim == NULL) return;
    timer->Channel = Channel;
    HAL_TIM_PWM_Start(timer->htim, timer->Channel);
}

/**
 * @brief  启动定时器的完整PWM模式，包括正向和互补输出
 * @param  timer: 指向定时器结构体的指针
 * @param  Channel: 要启动的PWM通道，TIM_CHANNEL_1/TIM_CHANNEL_2/TIM_CHANNEL_3
 * @retval 无
 * @note   该函数会调用HAL库函数启动指定通道的PWM输出，
 *         并使能对应的PWMN输出通道（如果存在）
 */
void Timer_Comple_PWM_Start(Timer_t *timer, uint32_t Channel){
    if(timer == NULL || timer->htim == NULL) return;
    timer->Channel = Channel;
    HAL_TIM_PWM_Start(timer->htim, timer->Channel);
    HAL_TIMEx_PWMN_Start(timer->htim, timer->Channel);
}

/**
 * @brief  启动定时器中断，并设置周期结束时的回调函数
 * @param  timer: 指向定时器结构体的指针
 * @param  Callback: 定时器周期结束时的回调函数指针，函数签名为 void(*)(Timer_t*)
 * @retval 无
 * @note   该函数会保存用户回调函数指针，然后调用HAL库启动定时器中断
 */
void Timer_Start_Base_IT(Timer_t* timer, void(*Callback)(Timer_t *timer))
{
    /* 空指针检查 */
    if(timer == NULL) return;
    /* 保存用户定义的回调函数，供中断发生时调用 */
    timer->PeriodElapsedCallback = Callback;
    /* 调用STM32 HAL库函数，启动定时器并使能更新中断 */
    HAL_TIM_Base_Start_IT(timer->htim);
}

/**
 * @brief  启动PWM输出并注册触发中断回调
 * @param  timer: 指向定时器结构体的指针
 * @param  Channel: PWM输出通道（TIM_CHANNEL_1 ~ TIM_CHANNEL_4）
 * @param  Callback: 触发中断时的回调函数指针，函数签名为 void(*)(Timer_t*)
 * @retval 无
 * @note   该函数会保存用户回调函数指针和通道号，
 *         然后启动对应通道的PWM输出并开启触发中断。
 */
void Timer_Start_PWM_IT(Timer_t* timer, uint32_t Channel, void(*Callback)(Timer_t *timer))
{
    if(timer == NULL || timer->htim == NULL) return;
    timer->Channel = Channel;
    timer->TriggerCallback = Callback;
    HAL_TIM_PWM_Start_IT(timer->htim, timer->Channel);
}

/**
 * @brief  启动PWM输出并注册脉冲完成中断回调
 * @param  timer: 指向定时器结构体的指针
 * @param  Channel: PWM输出通道（TIM_CHANNEL_1 ~ TIM_CHANNEL_4）
 * @param  Callback: 脉冲完成时的回调函数指针，函数签名为 void(*)(Timer_t*)
 * @retval 无
 * @note   该函数会保存用户回调函数指针和通道号，
 *         然后启动对应通道的PWM输出及其中断。
 *         适用于FOC中PWM触发ADC注入采样后，在脉冲完成中断里读取电流。
 */
void Timer_Start_PulseFinished_IT(Timer_t *timer, uint32_t Channel, void(*Callback)(Timer_t *timer)){
    if(timer == NULL || timer->htim == NULL) return;
    timer->Channel = Channel;
    timer->PulseFinishedCallback = Callback;
    HAL_TIM_PWM_Start_IT(timer->htim, timer->Channel);
}

/**
 * @brief  定时器周期结束中断回调函数，用于处理用户定义的回调函数
 * @param  htim: TIM_HandleTypeDef指针，指向触发中断的定时器句柄
 * @retval 无
 * @note   该函数会根据定时器句柄查找对应的定时器实例，
 *         并调用用户定义的周期结束回调函数（如果存在）
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    Timer_t* timer = Timer_GetInstance(htim);
    if(timer != NULL)
    {
        if(timer->PeriodElapsedCallback != NULL)
        {
            timer->PeriodElapsedCallback(timer);
        }
    }
}

/**
 * @brief  PWM触发中断回调函数，用于处理PWM触发事件
 * @param  htim: TIM_HandleTypeDef指针，指向触发中断的定时器句柄
 * @retval 无
 * @note   该函数会根据定时器句柄查找对应的定时器实例，
 *         并调用用户定义的Trigger回调函数（如果存在）
 */
void HAL_TIM_PWM_TriggerCallback(TIM_HandleTypeDef *htim)
{
    Timer_t* timer = Timer_GetInstance(htim);
    if(timer != NULL)
    {
        if(timer->TriggerCallback != NULL)
        {
            timer->TriggerCallback(timer);
        }
    }
}

/**
 * @brief  PWM脉冲完成中断回调函数，用于处理PWM脉冲结束事件
 * @param  htim: TIM_HandleTypeDef指针，指向触发中断的定时器句柄
 * @retval 无
 * @note   该函数会根据定时器句柄查找对应的定时器实例，
 *         并调用用户定义的PulseFinished回调函数（如果存在）。
 *         常用于FOC中在PWM中心点触发ADC注入采样后，在此回调中读取并处理电流数据。
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    Timer_t* timer = Timer_GetInstance(htim);
    if(timer != NULL)
    {
        if(timer->PulseFinishedCallback != NULL)
        {
            timer->PulseFinishedCallback(timer);
        }
    }
}

#endif