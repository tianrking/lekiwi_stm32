/**
  ******************************************************************************
  * @file           : motor_control.c
  * @brief          : Implementation for motor control module.
  ******************************************************************************
  */

#include "motor_control.h"
#include "tim.h" // 引入CubeMX生成的定时器句柄

// --- 私有(模块内)全局变量 ---
// 电机对象数组
Motor_t g_motors[MOTOR_COUNT];

// --- API函数实现 ---

void Motor_Init(void)
{
    // --- 配置电机1 --- E9 E11 控制电机 编码器 PA6 PA7
    g_motors[0].id = 0;
    g_motors[0].pwm_timer = &htim1;
    g_motors[0].pwm_ch_forward = TIM_CHANNEL_1;
    g_motors[0].pwm_ch_reverse = TIM_CHANNEL_2;
    g_motors[0].pwm_max_pulse = 999;
    g_motors[0].encoder_timer = &htim3;
    g_motors[0].encoder_ppr = 110.0f; // 11(基础PPR) * 10(减速比)
    g_motors[0].last_encoder_count = 0;
    // 初始化PID参数
    g_motors[0].pid_kp = 0.6f;
    g_motors[0].pid_ki = 0.01f;  
    g_motors[0].pid_kd = 0.0f; 
    g_motors[0].pid_error_last = 0.0f;
    g_motors[0].pid_error_integral = 0.0f;
    g_motors[0].pid_integral_limit = 2000.0f;
    
    // --- 配置电机2 ---  E13 E14控制电机 编码器 D12 D13
    g_motors[1].id = 1;
    g_motors[1].pwm_timer = &htim1;
    g_motors[1].pwm_ch_forward = TIM_CHANNEL_3;
    g_motors[1].pwm_ch_reverse = TIM_CHANNEL_4;
    g_motors[1].pwm_max_pulse = 999;
    g_motors[1].encoder_timer = &htim4;
    g_motors[1].encoder_ppr = 110.0f;
    g_motors[1].last_encoder_count = 0;
    // 初始化PID参数
    g_motors[1].pid_kp = 0.6f;
    g_motors[1].pid_ki = 0.01f;
    g_motors[1].pid_kd = 0.0f;
    g_motors[1].pid_error_last = 0.0f;
    g_motors[1].pid_error_integral = 0.0f;
    g_motors[1].pid_integral_limit = 2000.0f;

// --- 配置电机3 --- 电机 A5 B3 编码器 A0 A1
    g_motors[2].id = 2;
    g_motors[2].pwm_timer = &htim2; // TIM2用于电机3的PWM
    g_motors[2].pwm_ch_forward = TIM_CHANNEL_1;
    g_motors[2].pwm_ch_reverse = TIM_CHANNEL_2;
    g_motors[2].pwm_max_pulse = 999;
    g_motors[2].encoder_timer = &htim5;
    g_motors[2].encoder_ppr = 110.0f;
    g_motors[2].last_encoder_count = 0;
    // 初始化PID参数
    g_motors[2].pid_kp = 0.6f;
    g_motors[2].pid_ki = 0.01f;
    g_motors[2].pid_kd = 0.0f;
    g_motors[2].pid_error_last = 0.0f;
    g_motors[2].pid_error_integral = 0.0f;
    g_motors[2].pid_integral_limit = 2000.0f;
}

void Motor_Start(void)
{
    // 启动所有PWM通道
    HAL_TIM_PWM_Start(g_motors[0].pwm_timer, g_motors[0].pwm_ch_forward);
    HAL_TIM_PWM_Start(g_motors[0].pwm_timer, g_motors[0].pwm_ch_reverse);
    HAL_TIM_PWM_Start(g_motors[1].pwm_timer, g_motors[1].pwm_ch_forward);
    HAL_TIM_PWM_Start(g_motors[1].pwm_timer, g_motors[1].pwm_ch_reverse);
    HAL_TIM_PWM_Start(g_motors[2].pwm_timer, g_motors[2].pwm_ch_forward);
    HAL_TIM_PWM_Start(g_motors[2].pwm_timer, g_motors[2].pwm_ch_reverse);
    
    // 启动所有编码器
    HAL_TIM_Encoder_Start(g_motors[0].encoder_timer, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(g_motors[1].encoder_timer, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(g_motors[2].encoder_timer, TIM_CHANNEL_ALL);
    
    // 启动用于测速的基准定时器 (TIM9)
    HAL_TIM_Base_Start_IT(&htim9);
}

void Motor_Set_Target_Speed(uint8_t motor_id, float target_rpm)
{
    // 增加一个边界检查，防止数组越界
    if (motor_id < MOTOR_COUNT)
    {
        g_motors[motor_id].speed_rpm_target = target_rpm;
    }
}

void Motor_Run_PID_Control(void)
{
    const float SAMPLING_PERIOD_S = 0.01f; // 采样周期 (与TIM9中断周期一致)

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        // 1. 计算误差
        float error = g_motors[i].speed_rpm_target - g_motors[i].speed_rpm_actual;

        // 2. 计算积分项 I (包含抗积分饱和)
        g_motors[i].pid_error_integral += error * SAMPLING_PERIOD_S;
        if (g_motors[i].pid_error_integral > g_motors[i].pid_integral_limit) {
            g_motors[i].pid_error_integral = g_motors[i].pid_integral_limit;
        } else if (g_motors[i].pid_error_integral < -g_motors[i].pid_integral_limit) {
            g_motors[i].pid_error_integral = -g_motors[i].pid_integral_limit;
        }
        
        // 3. 计算微分项 D
        float derivative = (error - g_motors[i].pid_error_last) / SAMPLING_PERIOD_S;
        
        // 4. PID总输出计算 (P + I + D)
        float output = (g_motors[i].pid_kp * error) + 
                       (g_motors[i].pid_ki * g_motors[i].pid_error_integral) + 
                       (g_motors[i].pid_kd * derivative);

        // 5. 更新历史误差值
        g_motors[i].pid_error_last = error;

        // 6. 输出限幅 (例如，限制在-100%到+100%的占空比指令)
        if (output > 100.0f) output = 100.0f;
        if (output < -100.0f) output = -100.0f;

        // 7. 将PID输出转换为PWM信号并应用到电机
        int16_t speed_cmd = (int16_t)output;
        uint8_t absolute_speed = (speed_cmd < 0) ? -speed_cmd : speed_cmd;
        uint32_t pulse = (uint32_t)(absolute_speed / 100.0f * g_motors[i].pwm_max_pulse);

        if (speed_cmd >= 0) { // 正转或停止
            __HAL_TIM_SET_COMPARE(g_motors[i].pwm_timer, g_motors[i].pwm_ch_forward, pulse);
            __HAL_TIM_SET_COMPARE(g_motors[i].pwm_timer, g_motors[i].pwm_ch_reverse, 0);
        } else { // 反转
            __HAL_TIM_SET_COMPARE(g_motors[i].pwm_timer, g_motors[i].pwm_ch_forward, 0);
            __HAL_TIM_SET_COMPARE(g_motors[i].pwm_timer, g_motors[i].pwm_ch_reverse, pulse);
        }
    }
}

float Motor_Get_Speed(uint8_t motor_id)
{
    if (motor_id < MOTOR_COUNT)
    {
        return g_motors[motor_id].speed_rpm_actual;
    }
    return 0.0f;
}

void Motor_Encoder_Update_Callback(void)
{
    const float ENCODER_MODE_MULT = 4.0f;    // 4倍频
    const float SAMPLING_PERIOD_S = 0.01f;   // 10ms采样周期 (与TIM9中断周期一致)

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(g_motors[i].encoder_timer);
        int16_t delta_count = current_count - g_motors[i].last_encoder_count;
        g_motors[i].last_encoder_count = current_count;
        
        float conversion_factor = 60.0f / (g_motors[i].encoder_ppr * ENCODER_MODE_MULT * SAMPLING_PERIOD_S);
        g_motors[i].speed_rpm_actual = delta_count * conversion_factor;
    }
}

void Motor_Set_PWM_Duty(uint8_t motor_id, int16_t duty_cycle)
{
    if (motor_id < MOTOR_COUNT)
    {
        // 1. 限幅
        if (duty_cycle > 100) duty_cycle = 100;
        if (duty_cycle < -100) duty_cycle = -100;

        // 2. 计算绝对值和脉冲值
        uint8_t absolute_speed = (duty_cycle < 0) ? -duty_cycle : duty_cycle;
        uint32_t pulse = (uint32_t)(absolute_speed / 100.0f * g_motors[motor_id].pwm_max_pulse);

        // 3. 应用到PWM通道
        if (duty_cycle >= 0) { // 正转或停止
            __HAL_TIM_SET_COMPARE(g_motors[motor_id].pwm_timer, g_motors[motor_id].pwm_ch_forward, pulse);
            __HAL_TIM_SET_COMPARE(g_motors[motor_id].pwm_timer, g_motors[motor_id].pwm_ch_reverse, 0);
        } else { // 反转
            __HAL_TIM_SET_COMPARE(g_motors[motor_id].pwm_timer, g_motors[motor_id].pwm_ch_forward, 0);
            __HAL_TIM_SET_COMPARE(g_motors[motor_id].pwm_timer, g_motors[motor_id].pwm_ch_reverse, pulse);
        }
    }
}
