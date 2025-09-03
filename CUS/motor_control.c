/* motor_control.c */

#include "motor_control.h"
#include "tim.h" // 引入CubeMX生成的定时器句柄

Motor_t g_motors[MOTOR_COUNT];

// --- API函数实现 ---

void Motor_Init(void)
{
    // --- 配置电机1 ---
    g_motors[0].id = 1;
    g_motors[0].pwm_timer = &htim1;
    g_motors[0].pwm_ch_forward = TIM_CHANNEL_1;
    g_motors[0].pwm_ch_reverse = TIM_CHANNEL_2;
    g_motors[0].pwm_max_pulse = 999; // ARR的值
    g_motors[0].encoder_timer = &htim3; // 假设电机1用TIM3编码器
    g_motors[0].encoder_ppr = 500.0f;
    g_motors[0].last_encoder_count = 0;
    
    // --- 配置电机2 ---
    g_motors[1].id = 2;
    g_motors[1].pwm_timer = &htim1;
    g_motors[1].pwm_ch_forward = TIM_CHANNEL_3;
    g_motors[1].pwm_ch_reverse = TIM_CHANNEL_4;
    g_motors[1].pwm_max_pulse = 999;
    g_motors[1].encoder_timer = &htim4; // 假设电机2用TIM4编码器
    g_motors[1].encoder_ppr = 500.0f;
    g_motors[1].last_encoder_count = 0;
    
    // 如果有更多电机，继续在这里配置 g_motors[2], g_motors[3]...
    g_motors[2].id = 3;
    g_motors[2].pwm_timer = &htim2; 
    g_motors[2].pwm_ch_forward = TIM_CHANNEL_1;
    g_motors[2].pwm_ch_reverse = TIM_CHANNEL_2;
    g_motors[2].pwm_max_pulse = 999; // ARR的值与TIM1一致
    g_motors[2].encoder_timer = &htim5; // 使用新的TIM5
    g_motors[2].encoder_ppr = 500.0f; // 假设编码器参数相同
    g_motors[2].last_encoder_count = 0;
}

void Motor_Start(void)
{
    // 启动所有PWM通道
    HAL_TIM_PWM_Start(g_motors[0].pwm_timer, g_motors[0].pwm_ch_forward);
    HAL_TIM_PWM_Start(g_motors[0].pwm_timer, g_motors[0].pwm_ch_reverse);
    HAL_TIM_PWM_Start(g_motors[1].pwm_timer, g_motors[1].pwm_ch_forward);
    HAL_TIM_PWM_Start(g_motors[1].pwm_timer, g_motors[1].pwm_ch_reverse);
    
    // --- 【新增】启动电机3的硬件 ---
    HAL_TIM_PWM_Start(g_motors[2].pwm_timer, g_motors[2].pwm_ch_forward);
    HAL_TIM_PWM_Start(g_motors[2].pwm_timer, g_motors[2].pwm_ch_reverse);
    
    // 启动所有编码器
    HAL_TIM_Encoder_Start(g_motors[0].encoder_timer, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(g_motors[1].encoder_timer, TIM_CHANNEL_ALL);
    
    HAL_TIM_Encoder_Start(g_motors[2].encoder_timer, TIM_CHANNEL_ALL);
    
    // 启动用于测速的基准定时器
    HAL_TIM_Base_Start_IT(&htim9);
}

void Motor_Set_Speed(uint8_t motor_id, int16_t speed_cmd)
{
    // 查找对应ID的电机
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (g_motors[i].id == motor_id)
        {
            g_motors[i].target_speed_cmd = speed_cmd; // 保存指令

            if (speed_cmd > 100) speed_cmd = 100;
            if (speed_cmd < -100) speed_cmd = -100;

            uint8_t absolute_speed = (speed_cmd < 0) ? -speed_cmd : speed_cmd;
            uint32_t pulse = (uint32_t)(absolute_speed / 100.0f * g_motors[i].pwm_max_pulse);

            if (speed_cmd > 0) // 正转
            {
                __HAL_TIM_SET_COMPARE(g_motors[i].pwm_timer, g_motors[i].pwm_ch_forward, pulse);
                __HAL_TIM_SET_COMPARE(g_motors[i].pwm_timer, g_motors[i].pwm_ch_reverse, 0);
            }
            else if (speed_cmd < 0) // 反转
            {
                __HAL_TIM_SET_COMPARE(g_motors[i].pwm_timer, g_motors[i].pwm_ch_forward, 0);
                __HAL_TIM_SET_COMPARE(g_motors[i].pwm_timer, g_motors[i].pwm_ch_reverse, pulse);
            }
            else // 停止
            {
                __HAL_TIM_SET_COMPARE(g_motors[i].pwm_timer, g_motors[i].pwm_ch_forward, 0);
                __HAL_TIM_SET_COMPARE(g_motors[i].pwm_timer, g_motors[i].pwm_ch_reverse, 0);
            }
            
            return; // 找到并设置后即可退出
        }
    }
}

float Motor_Get_Speed(uint8_t motor_id)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (g_motors[i].id == motor_id)
        {
            return g_motors[i].speed_rpm;
        }
    }
    return 0.0f;
}

void Motor_Encoder_Update_Callback(void)
{
    // 定义常量
    const float ENCODER_MODE_MULT = 4.0f;
    const float SAMPLING_PERIOD_S = 0.01f; // 假设TIM2中断周期为10ms

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        // 1. 读取当前计数值
        int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(g_motors[i].encoder_timer);
        
        // 2. 计算差值
        int16_t delta_count = current_count - g_motors[i].last_encoder_count;
        
        // 3. 更新状态
        g_motors[i].last_encoder_count = current_count;
        
        // 4. 计算RPM
        float conversion_factor = 60.0f / (g_motors[i].encoder_ppr * ENCODER_MODE_MULT * SAMPLING_PERIOD_S);
        g_motors[i].speed_rpm = delta_count * conversion_factor;
    }
}
