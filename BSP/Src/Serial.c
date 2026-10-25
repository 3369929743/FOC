/**
  ******************************************************************************
  * @file    Serial.c
  * @brief   串口驱动模块（基于HAL库中断方式）
  * @note    使用方法示例：
  * 
  * ============================================================================
  * 方式一：配合串口流（Serial_Stream）使用 - 推荐用于不定长数据接收
  * ============================================================================
  * @code
  * // 1. 定义串口、FIFO和串口流对象及相关缓冲区
  * Serial_t Serial_Blue;                          // 串口对象
  * FIFO_t FIFO_Blue;                              // FIFO对象
  * Serial_Stream_t Serial_Stream_Blue;            // 串口流对象
  * uint8_t Blue_FIFO_Buff[128];                   // FIFO缓冲区
  * uint8_t Blue_RxTemp_Buff[64];                  // 接收临时缓冲区
  * uint8_t Blue_User_Buff[64];                    // 用户数据缓冲区
  * 
  * // 2. 配置串口流参数
  * Serial_Stream_Confg_t Serial_Stream_Blue_Confg = {
  *   .Serial = &Serial_Blue,                      // 绑定串口对象
  *   .FIFO = &FIFO_Blue,                          // 绑定FIFO对象
  *   .FIFO_Buff = Blue_FIFO_Buff,                 // FIFO缓冲区指针
  *   .Rx_Temp_Buff = Blue_RxTemp_Buff,            // 接收临时缓冲区指针
  *   .Serial_Num = Serial_2,                      // 串口号（如Serial_1/Serial_2/Serial_3）
  *   .FIFO_Size = sizeof(Blue_FIFO_Buff),         // FIFO缓冲区大小
  *   .Rx_Temp_Buff_Size = sizeof(Blue_RxTemp_Buff), // 接收临时缓冲区大小
  * };
  * 
  * // 3. 初始化串口流（在main函数的初始化阶段调用）
  * // 注意：Serial_Stream_Init内部会自动调用Serial_Init、设置接收缓冲区和回调函数
  * Serial_Stream_Init(&Serial_Stream_Blue, &Serial_Stream_Blue_Confg);
  * 
  * // 4. 在主循环中读取数据
  * while(1) {
  *   // 方式一：读取单个字节
  *   uint8_t Byte;
  *   if(Serial_Stream_ReadByte(&Serial_Stream_Blue, &Byte)) {
  *     // 处理读取到的字节数据
  *   }
  *   
  *   // 方式二：读取多个字节（推荐）
  *   if(Serial_Stream_ReadArray(&Serial_Stream_Blue, Blue_User_Buff, sizeof(Blue_User_Buff))) {
  *     // 处理读取到的数组数据，Blue_User_Buff中即为接收到的数据
  *   }
  *   
  *   // 发送数据
  *   Serial_SendByte(Serial_Stream_Blue.Serial, 0x55);
  *   Serial_SendArray(Serial_Stream_Blue.Serial, Blue_User_Buff, 10);
  *   Serial_SendString(Serial_Stream_Blue.Serial, "Hello");
  *   Serial_Printf(Serial_Stream_Blue.Serial, "Value: %d", i);
  * }
  * @endcode
  * 
  * ============================================================================
  * 方式二：不使用串口流，直接使用Serial模块
  * ============================================================================
  * @code
  * // 1. 定义串口对象和接收缓冲区
  * Serial_t Serial_Debug;                         // 串口对象
  * uint8_t Debug_RxBuffer[64];                    // 接收缓冲区
  * uint8_t Debug_User_Buff[64];                   // 用户数据缓冲区
  * uint8_t Debug_Flag = 0;                        // 数据接收标志
  * 
  * // 2. 自定义接收回调函数
  * void Debug_RxCallback(Serial_t *Serial, uint16_t Size) {
  *   // 将接收到的数据复制到用户缓冲区
  *   memcpy(Debug_User_Buff, Serial->pRxBuff, Size);
  *   Debug_Flag = 1;                              // 设置数据就绪标志
  *   
  *   // 重新启动空闲中断接收
  *   Serial_ReceiveToIdle_IT(Serial);
  * }
  * 
  * // 3. 初始化串口（在main函数的初始化阶段调用）
  * Serial_Init(&Serial_Debug, Serial_1);          // 初始化串口对象
  * Serial_SetRxBuffer(&Serial_Debug, Debug_RxBuffer, sizeof(Debug_RxBuffer)); // 设置接收缓冲区
  * Serial_SetRxCallback(&Serial_Debug, Debug_RxCallback); // 注册接收回调函数
  * Serial_ReceiveToIdle_IT(&Serial_Debug);        // 启动空闲中断接收
  * 
  * // 4. 在主循环中处理数据
  * while(1) {
  *   if(Debug_Flag) {
  *     Debug_Flag = 0;                            // 清除标志
  *     // 处理Debug_User_Buff中的数据
  *   }
  *   
  *   // 发送数据
  *   Serial_SendByte(&Serial_Debug, 0x55);
  *   Serial_SendString(&Serial_Debug, "Hello");
  *   Serial_Printf(&Serial_Debug, "Count: %d", count);
  * }
  * @endcode
  * 
  * ============================================================================
  * 蓝牙显示模式（配合蓝牙串口使用）
  * ============================================================================
  * @code
  * // 假设已初始化蓝牙串口流：Serial_Stream_Blue
  * 
  * // 波形显示模式 - 将数据发送到蓝牙APP进行波形绘制
  * Serial_Printf(Serial_Stream_Blue.Serial, "[plot,%d]", i);
  * 
  * // 虚拟OLED显示模式 - 在蓝牙APP上模拟OLED显示效果
  * Serial_Printf(Serial_Stream_Blue.Serial, "[display,0,0,Hello World]");
  * @endcode
  * 
  * @note  注意事项：
  *        1. 发送函数（Serial_SendByte/Array/String/Printf）会等待isBusy标志为0才发送
  *        2. 接收功能必须先调用Serial_SetRxBuffer设置接收缓冲区
  *        3. 空闲中断接收模式适用于不定长数据帧，按字节数接收模式适用于定长数据
  *        4. 使用串口流时，无需手动设置接收缓冲区和回调，Serial_Stream_Init会自动处理
  *        5. TxBuffer_Size默认为64字节，发送数据超出此长度会被截断
  ******************************************************************************
  */
#include "Serial.h"
#include "string.h"
#include <stdint.h>
#include "stdarg.h"
#include "stdio.h"

#ifdef SERIAL_MODULE_ENABLED

#define X(Index, Instance) extern __weak UART_HandleTypeDef huart##Index;
SERIAL_LIST
#undef X

static UART_HandleTypeDef* const Serial_Map[] = {
    NULL,
    #define X(Index, Instance) [Index] = &huart##Index,
    SERIAL_LIST
    #undef X 
};

#define SERIAL_MAX_NUM (sizeof(Serial_Map) / sizeof(Serial_Map[0]))

static Serial_t *Serial_Instance[SERIAL_MAX_NUM] = {NULL};    

/**
 * @brief 根据UART句柄获取对应的串口对象实例
 * @param huart UART句柄指针
 * @return Serial_t* 返回对应的串口对象指针，如果未找到则返回NULL
 * @note 该函数通过比较UART外设实例地址来查找已注册的串口对象，
 *       支持USART1、USART2、USART3三个串口实例
 */
static inline Serial_t *Serial_GetInstance(UART_HandleTypeDef *huart)
{
    /* 将UART外设实例寄存器基地址转换为整数，用于switch-case匹配 */
    switch((uint32_t)huart->Instance)
    {
        /* 使用X-macro展开为多个case分支：
         * case (uint32_t)USART1: return Serial_Instance[1];
         * case (uint32_t)USART2: return Serial_Instance[2];
         * case (uint32_t)USART3: return Serial_Instance[3];
         */
        #define X(Index, Instance) case (uint32_t)(Instance): return Serial_Instance[Index];
        SERIAL_LIST
        #undef X
        default:
            return NULL;
    }
}

/**
 * @brief 初始化串口对象
 * @param Serial 串口对象指针
 * @param Serial_Num 串口编号
 * @return uint8_t 返回0表示初始化成功，返回1表示参数错误（编号超出范围或为0）
 * @note 该函数将指定编号的UART句柄映射到串口对象，初始化状态标志和回调函数，
 *       并将串口对象注册到实例数组中
 */
uint8_t Serial_Init(Serial_t *Serial, Serial_Num_e Serial_Num)
{
    if(Serial_Num >= SERIAL_MAX_NUM || Serial_Num == 0)
    {
        return 1;
    }
    Serial->huart = Serial_Map[Serial_Num];
    Serial->isBusy = 0;
    Serial_Instance[Serial_Num] = Serial;
    Serial->RxCallback = NULL;
    Serial->TxCallback = NULL;
    Serial->ErrorCallback = NULL;
    return 0;
}

/**
 * @brief 启动串口发送操作（内部辅助函数）
 * @param Serial 串口对象指针
 * @param Size 要发送的数据长度（字节数）
 * @note 该函数将isBusy标志置1，并调用HAL库中断发送函数启动数据传输
 */
static inline void Serial_Start_IT_Send(Serial_t *Serial, uint16_t Size)
{
    Serial->isBusy = 1;
    HAL_UART_Transmit_IT(Serial->huart, Serial->TxBuffer, Size);
}

void Serial_SendBytes(Serial_t *Serial, uint8_t *pData, uint16_t Size, uint32_t Timeout){
    HAL_UART_Transmit(Serial->huart, pData, Size, Timeout);
}

/**
 * @brief 通过串口发送单个字节
 * @param Serial 串口对象指针
 * @param Byte 待发送的字节数据
 * @note 发送前会等待串口空闲，然后将字节复制到TxBuffer并通过中断方式发送
 */
void Serial_IT_SendByte(Serial_t *Serial, uint8_t Byte)
{
    if(Serial == NULL) return;
    uint32_t Timeout = 10000;
    while(Serial->isBusy && Timeout--);
    memcpy(Serial->TxBuffer, &Byte, 1);
    Serial_Start_IT_Send(Serial, 1);
}

/**
 * @brief 通过串口发送字节数组
 * @param Serial 串口对象指针
 * @param Array 待发送的数据数组指针
 * @param Size 要发送的数据长度（字节数）
 * @return uint8_t 返回0表示发送成功，返回1表示数据长度超出缓冲区大小限制
 * @note 数据会先复制到内部TxBuffer，然后通过中断方式发送；发送时会将isBusy标志置1
 */
uint8_t Serial_IT_SendArray(Serial_t *Serial, uint8_t *Array, uint16_t Size)
{
    if(Serial == NULL) return 1;
    if(Size > TxBuffer_Size)
    {
        Size = TxBuffer_Size;
        return 1;
    }
    uint32_t Timeout = 10000;
    while(Serial->isBusy && Timeout--);
    memcpy(Serial->TxBuffer, Array, Size);
    Serial_Start_IT_Send(Serial, Size);
    return 0;
}

/**
 * @brief 通过串口发送字符串
 * @param Serial 串口对象指针
 * @param String 待发送的字符串指针
 * @return uint8_t 返回0表示发送成功，返回1表示字符串长度超出缓冲区大小限制
 * @note 数据会先复制到内部TxBuffer，然后通过中断方式发送；发送时会将isBusy标志置1
 */
uint8_t Serial_IT_SendString(Serial_t *Serial, char *String)
{
    if(Serial == NULL) return 1;
    uint16_t Size = strlen(String);
    if(Size > TxBuffer_Size)
    {
        return 1;
    }
    uint32_t Timeout = 10000;
    while(Serial->isBusy && Timeout--);
    memcpy(Serial->TxBuffer, String, Size);
    Serial_Start_IT_Send(Serial, Size);
    return 0;
}

/**
 * @brief 通过串口发送格式化字符串（类似printf）
 * @param Serial 串口对象指针
 * @param Format 格式化字符串指针，支持printf风格的格式说明符
 * @param ... 可变参数列表，对应格式化字符串中的占位符
 * @note 发送前会等待串口空闲，使用vsnprintf将格式化数据写入TxBuffer，
 *       若格式化后的数据长度超出缓冲区大小则截断，然后通过中断方式发送
 */
void Serial_IT_Printf(Serial_t *Serial, char *Format, ...)
{
    if(Serial == NULL) return;
    uint32_t Timeout = 10000;
    while(Serial->isBusy && Timeout--);
    va_list Args;
    va_start(Args, Format);
    uint16_t Size = vsnprintf((char *)Serial->TxBuffer, (int)TxBuffer_Size, Format, Args);
    if(Size >= TxBuffer_Size)
    {
        Size = TxBuffer_Size - 1;
    }
    Serial_Start_IT_Send(Serial, Size);
    va_end(Args);
}

/**
 * @brief  设置串口接收完成回调函数
 * @param  Serial: 指向串口对象结构体的指针
 * @param  RxCallback: 接收完成回调函数指针，函数签名为 void(*)(Serial_t *Serial, uint16_t Size)
 * @retval 无
 * @note   该回调函数会在串口空闲中断触发（表示一帧数据接收完成）时被调用，
 *         用户可在回调中解析接收到的数据
 */
void Serial_SetRxCallback(Serial_t *Serial, void (*RxCallback)(struct Serial_Struct *Serial, uint16_t Size))
{
    if(Serial == NULL) return;
    Serial->RxCallback = RxCallback;
}


/**
 * @brief  设置串口发送完成回调函数
 * @param  Serial: 指向串口对象结构体的指针
 * @param  TxCallback: 发送完成回调函数指针，函数签名为 void(*)(Serial_t *Serial)
 * @retval 无
 * @note   该回调函数会在串口发送完成中断触发时被调用，
 *         用户可在回调中处理发送完成后的逻辑（如释放缓冲区、触发下一次发送等）
 */
void Serial_SetTxCallback(Serial_t *Serial, void (*TxCallback)(struct Serial_Struct *Serial))
{
    if(Serial == NULL) return;
    Serial->TxCallback = TxCallback;
}

/**
 * @brief  设置接收缓冲区
 * @param  Serial: 指向串口对象结构体的指针
 * @param  Buffer: 接收缓冲区指针
 * @param  Size: 接收缓冲区大小（字节数）
 * @retval 无
 * @note   使用接收功能前必须先调用此函数设置接收缓冲区，通常配合FIFO使用
 */
void Serial_SetRxBuffer(Serial_t *Serial, uint8_t *Buffer, uint16_t Size)
{
    if(Serial == NULL || Buffer == NULL || Size == 0) return;
    Serial->pRxBuff = Buffer;
    Serial->RxSize = Size;
}

/**
 * @brief  启动串口空闲中断接收模式
 * @param  Serial: 指向串口对象结构体的指针
 * @retval 无
 * @note   该函数配置串口以空闲中断方式接收数据，参数为接收缓冲区总大小（Serial->RxSize）；
 *         HAL库会在缓冲区中持续接收数据，触发空闲中断的条件：
 *         1. 总线空闲（一段时间无新数据，表示一帧数据接收完成）
 *         2. 缓冲区满（接收字节数达到RxSize）
 *         中断触发后调用HAL_UARTEx_RxEventCallback，传入实际接收到的字节数；
 *         适用于不定长数据帧接收场景；使用前必须先调用Serial_SetRxBuffer设置接收缓冲区
 */
void Serial_ReceiveToIdle_IT(Serial_t *Serial)
{
    if(Serial == NULL || Serial->pRxBuff == NULL || Serial->RxSize == 0) return;
    HAL_UARTEx_ReceiveToIdle_IT(Serial->huart, Serial->pRxBuff, Serial->RxSize);
}

/**
 * @brief  启动串口中断接收模式（按指定字节数触发中断）
 * @param  Serial: 指向串口对象结构体的指针
 * @param  Size: 每次触发接收完成中断的字节数（每接收到Size字节进一次中断）
 * @retval 无
 * @note   该函数配置串口以中断方式接收数据，HAL库会在接收缓冲区中累计接收数据，
 *         当接收到的字节数达到Size时，触发接收完成中断（HAL_UART_RxCpltCallback）；
 *         例如：Size=5，则每收到5字节触发一次中断；
 *         使用前必须先调用Serial_SetRxBuffer设置接收缓冲区，且Size不能超出缓冲区大小
 */
void Serial_Receive_IT(Serial_t *Serial, uint16_t Size)
{
    if(Serial == NULL || Serial->pRxBuff == NULL || Serial->RxSize == 0) return;
    if(Size > Serial->RxSize)
    {
        Size = Serial->RxSize;
    }
    HAL_UART_Receive_IT(Serial->huart, Serial->pRxBuff, Size);
}

/**
 * @brief 清除串口错误标志并重新启动接收
 * @param Serial 串口对象指针
 * @note 该函数通过读取状态寄存器(SR)和数据寄存器(DR)来清除串口错误标志；
 *       清除错误后，自动重新启动空闲中断接收
 */
static void Serial_Error_Clear(Serial_t *Serial)
{
    if(Serial == NULL) return;
    if(Serial->pRxBuff == NULL || Serial->RxSize == 0) return;
    
    __HAL_UART_CLEAR_FEFLAG(Serial->huart);
    __HAL_UART_CLEAR_PEFLAG(Serial->huart);
    __HAL_UART_CLEAR_NEFLAG(Serial->huart);
    __HAL_UART_CLEAR_OREFLAG(Serial->huart);

    HAL_UARTEx_ReceiveToIdle_IT(Serial->huart, Serial->pRxBuff, Serial->RxSize);
}

/**
 * @brief UART接收完成回调函数（HAL库中断回调）
 * @param huart UART句柄指针
 * @note 该函数由HAL库在UART接收完成时自动调用；
 *       函数会获取对应的串口对象实例，如果注册了接收回调函数，则调用用户回调并传入参数1表示接收完成
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  Serial_t *Serial = Serial_GetInstance(huart);
  if(Serial && Serial->RxCallback)
  {
    Serial->RxCallback(Serial, 1);
  }
}

/**s
 * @brief UART接收事件回调函数（HAL库空闲中断回调）
 * @param huart UART句柄指针
 * @param Size 实际接收到的数据长度（字节数）
 * @note 该函数由HAL库在UART空闲中断或接收事件发生时自动调用；
 *       函数会获取对应的串口对象实例，如果注册了接收回调函数，则调用用户回调并传入实际接收的数据长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  Serial_t *Serial = Serial_GetInstance(huart);
  if(Serial && Serial->RxCallback)
  {
    Serial->RxCallback(Serial, Size);
  }
}

/**
 * @brief UART发送完成回调函数（HAL库中断回调）
 * @param huart UART句柄指针
 * @note 该函数由HAL库在UART发送完成时自动调用；
 *       函数会将isBusy标志清零表示串口空闲，如果注册了发送回调函数，则调用用户回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  Serial_t *Serial = Serial_GetInstance(huart);
  if(Serial)
  {
    Serial->isBusy = 0;
    if(Serial->TxCallback)
    {
      Serial->TxCallback(Serial);
    }
  }
}

/**
 * @brief UART错误回调函数（HAL库中断回调）
 * @param huart UART句柄指针
 * @note 该函数由HAL库在UART发生错误时自动调用；
 *       函数会先清除错误标志并重新启动接收，如果注册了错误回调函数，则调用用户回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  Serial_t *Serial = Serial_GetInstance(huart);
  if(Serial)
  {
    Serial_Error_Clear(Serial);
    if(Serial->ErrorCallback)
    {
        Serial->ErrorCallback(Serial);
    }
  }
}
#endif