#include "protocol.h"
#include "usart.h"         // for huart3 handle
#include "motor_control.h"  // for motor control functions
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// --- 宏定义 ---
#define RX_BUFFER_SIZE 128 // 接收缓冲区大小

// --- 模块内变量定义 ---

// 【修正】在此文件中定义单字节缓冲区实体
uint8_t g_uart3_rx_byte = 0;

// 存储一整行指令的缓冲区
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_index = 0;
// 一行数据接收完成的标志位 (0:未完成, 1:已完成)
static volatile uint8_t rx_line_ready = 0;

// --- 函数指针类型定义 (用于消息路由) ---
typedef void (*CommandHandler_t)(const char* params);

// --- 命令处理函数原型 ---
static void handle_set_single_speed(const char* params);
static void handle_set_all_speeds(const char* params);
static void handle_query_speeds(const char* params);

// --- 命令路由表 ---
typedef struct {
    char command_char;
    CommandHandler_t handler;
} Command_t;

static const Command_t command_table[] = {
    {'S', handle_set_single_speed},
    {'A', handle_set_all_speeds},
    {'Q', handle_query_speeds},
};
#define COMMAND_TABLE_SIZE (sizeof(command_table) / sizeof(Command_t))

// --- 校验和计算 ---
static uint8_t calculate_checksum(const uint8_t* data, uint16_t len)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// --- API函数实现 ---

void Protocol_Init(void)
{
    // 【修正】启动USART3接收中断，让它将接收到的第一个字节放入我们定义的全局缓冲区中
    HAL_UART_Receive_IT(&huart3, &g_uart3_rx_byte, 1);
}

void Protocol_Process(void)
{
    if (rx_line_ready)
    {
        // 1. 验证协议帧基本结构
        if (rx_buffer[0] == '$' && strchr((char*)rx_buffer, '*'))
        {
            // 2. 提取命令、数据和校验和
            char cmd_char = rx_buffer[1];
            char* params_start = (char*)&rx_buffer[3];
            char* checksum_ptr = strchr((char*)rx_buffer, '*');
            
            // 将 '*' 替换为字符串结束符，从而分离出数据部分
            *checksum_ptr = '\0';
            
            char* checksum_str = checksum_ptr + 1;
            uint8_t received_checksum = (uint8_t)strtol(checksum_str, NULL, 16);

            // 3. 计算并校验Checksum (从命令字符到数据结束)
            uint8_t calculated_checksum = calculate_checksum(&rx_buffer[1], strlen((char*)&rx_buffer[1]));

            if (received_checksum == calculated_checksum)
            {
                // 4. 校验成功，进行消息路由
                for (int i = 0; i < COMMAND_TABLE_SIZE; i++)
                {
                    if (command_table[i].command_char == cmd_char)
                    {
                        command_table[i].handler(params_start); // 调用对应的处理函数
                        break;
                    }
                }
            }
        }
        
        // 5. 清理缓冲区，准备下一次接收
        memset(rx_buffer, 0, RX_BUFFER_SIZE);
        rx_index = 0;
        rx_line_ready = 0;
    }
}

void Protocol_Report_Speeds(void)
{
    char tx_buffer[100];
    char data_payload[80];

    // 1. 格式化数据负载
    sprintf(data_payload, "R %.2f,%.2f,%.2f",
            Motor_Get_Speed(0),
            Motor_Get_Speed(1),
            Motor_Get_Speed(2));
            
    // 2. 计算校验和
    uint8_t checksum = calculate_checksum((uint8_t*)data_payload, strlen(data_payload));

    // 3. 格式化完整的发送帧
    sprintf(tx_buffer, "$%s*%02X\r\n", data_payload, checksum);

    // 4. 发送
    HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
}

void Protocol_Rx_Byte_Handler(uint8_t byte)
{
    if (rx_line_ready) return; // 如果上一行还没处理，则丢弃新数据

    if (byte == '\n' || byte == '\r')
    {
        if (rx_index > 0)
        {
            rx_buffer[rx_index] = '\0'; // 添加字符串结束符
            rx_line_ready = 1;          // 设置标志位
        }
    }
    else
    {
        if (rx_index < (RX_BUFFER_SIZE - 1))
        {
            rx_buffer[rx_index++] = byte;
        }
        // 如果缓冲区满了但没收到换行符，则丢弃数据，防止溢出
        else {
            rx_index = 0;
            memset(rx_buffer, 0, RX_BUFFER_SIZE);
        }
    }
}

// --- 静态命令处理函数 ---

static void handle_set_single_speed(const char* params)
{
    int id;
    float speed;
    if (sscanf(params, "%d,%f", &id, &speed) == 2)
    {
        Motor_Set_Target_Speed(id, speed);
    }
}

static void handle_set_all_speeds(const char* params)
{
    float s0, s1, s2;
    if (sscanf(params, "%f,%f,%f", &s0, &s1, &s2) == 3)
    {
        Motor_Set_Target_Speed(0, s0);
        Motor_Set_Target_Speed(1, s1);
        Motor_Set_Target_Speed(2, s2);
    }
}

static void handle_query_speeds(const char* params)
{
    // 当收到查询指令时，立即上报一次轮速
    Protocol_Report_Speeds();
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart: UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        // 调用我们协议模块中的字节处理器
        Protocol_Rx_Byte_Handler(huart->Instance->DR);
        
        // 重新使能下一次单字节接收中断
        HAL_UART_Receive_IT(&huart3, (uint8_t*)huart->Instance->DR, 1);
    }
}

