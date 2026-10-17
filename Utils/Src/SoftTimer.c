#include "SoftTimer.h"
#include <stddef.h>

#define TIMER_TICK_MS 1       // 定时器中断周期为1ms

/**
  * @brief  初始化软件定时器
  * @param  Timer: 指向软件定时器结构体的指针
  * @param  Mode: 定时器工作模式（单次模式或周期模式）
  * @param  TimeCount: 定时时间计数值（单位：ms）
  * @retval 无
  */
void SoftTimer_Init(SoftTimer_t *Timer, SoftTimer_Mode_e Mode, uint32_t TimeCount){
    if(Timer == NULL) return;                              // 空指针检查
    if(TimeCount == 0) TimeCount = 1;                      // 定时值不能为0，最小设为1
    Timer->isRunning = 0;                                  // 初始化定时器为停止状态
    Timer->isElapsed = 0;                                  // 清除超时标志
    Timer->TimeCount = TimeCount / TIMER_TICK_MS;             // 根据定时器节拍计算计数值
    Timer->ReloadCount = TimeCount / TIMER_TICK_MS;           // 保存重载值，用于周期模式或复位
    Timer->Mode = Mode;                                    // 设置定时器工作模式
}

/**
  * @brief  启动软件定时器
  * @param  Timer: 指向软件定时器结构体的指针
  * @retval 无
  */
void SoftTimer_Start(SoftTimer_t *Timer){
    if(Timer == NULL) return;                              // 空指针检查
    Timer->isRunning = 1;                                  // 设置定时器运行标志
    Timer->isElapsed = 0;                                  // 清除超时标志
}

/**
  * @brief  停止软件定时器
  * @param  Timer: 指向软件定时器结构体的指针
  * @retval 无
  */
void SoftTimer_Stop(SoftTimer_t *Timer){
    if(Timer == NULL) return;                              // 空指针检查
    Timer->isRunning = 0;                                  // 清除定时器运行标志
    Timer->isElapsed = 0;                                  // 清除超时标志
}

/**
  * @brief  更新软件定时器计数值（需在定时器中断或周期任务中调用）
  * @param  Timer: 指向软件定时器结构体的指针
  * @retval 无
  */
void SoftTimer_Update(SoftTimer_t *Timer){
    if(Timer == NULL || Timer->isElapsed || !Timer->isRunning) return;  // 空指针检查或定时器已超时/停止则返回
    if(Timer->TimeCount > 0){
        Timer->TimeCount--;                                  // 计数值递减
    }
    if(Timer->TimeCount == 0){
        Timer->isElapsed = 1;                                // 计数值为0，设置超时标志
    }
}

/**
  * @brief  复位软件定时器（重新加载计数值并清除超时标志）
  * @param  Timer: 指向软件定时器结构体的指针
  * @retval 无
  */
void SoftTimer_Reset(SoftTimer_t *Timer){
    if(Timer == NULL) return;                              // 空指针检查
    Timer->TimeCount = Timer->ReloadCount;                 // 重新加载计数值
    Timer->isElapsed = 0;                                  // 清除超时标志
}

/**
  * @brief  触发软件定时器（检查超时标志并根据模式处理）
  * @param  Timer: 指向软件定时器结构体的指针
  * @retval 1: 定时器已超时并成功触发; 0: 定时器未超时
  */
uint8_t SoftTimer_Trigger(SoftTimer_t *Timer){
    if(Timer == NULL || Timer->isElapsed == 0) return 0;   // 空指针检查或定时器未超时则返回0
    Timer->isElapsed = 0;                                  // 清除超时标志
    if(Timer->Mode == SOFTTIMER_MODE_PERIODIC){
        Timer->TimeCount = Timer->ReloadCount;             // 周期模式：重新加载计数值
    }
    else{
        Timer->isRunning = 0;                              // 单次模式：停止定时器
    }
    return 1;                                              // 返回1表示成功触发
}