#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

// --- 全局共享变量声明 (Extern) ---

/**
  * @brief  用于HAL库UART单字节接收的全局缓冲区.
  * @note   此变量由 protocol.c 定义, 并由 stm32f4xx_it.c 中的中断回调函数使用.
  */
extern uint8_t g_uart3_rx_byte;


// --- 公共API函数原型 ---

/**
  * @brief  初始化串口协议模块.
  * @note   此函数会启动USART3的接收中断, 准备接收数据.
  * @param  None
  * @retval None
  */
void Protocol_Init(void);

/**
  * @brief  处理已接收到的完整指令.
  * @note   此函数应在主循环(例如RTOS任务)中被周期性调用.
  * 它会检查是否有新指令需要解析和执行.
  * @param  None
  * @retval None
  */
void Protocol_Process(void);

/**
  * @brief  向上位机发送轮速报告.
  * @note   格式: $R <spd0>,<spd1>,<spd2>*<checksum>\r\n
  * @param  None
  * @retval None
  */
void Protocol_Report_Speeds(void);

/**
  * @brief  【供中断调用】处理UART接收到的单个字节.
  * @note   此函数应在UART的接收完成回调函数 (HAL_UART_RxCpltCallback) 中被调用.
  * @param  byte: 接收到的字节
  * @retval None
  */
void Protocol_Rx_Byte_Handler(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* INC_PROTOCOL_H_ */
