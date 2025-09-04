/**
  ******************************************************************************
  * @file           : motor_control.h
  * @brief          : Header for motor_control.c file.
  * This module encapsulates all motor control logic,
  * including PWM driving, encoder feedback, and PID speed control.
  ******************************************************************************
  */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stm32f4xx_hal.h" // 包含此文件以识别TIM_HandleTypeDef

// --- 定义项目中控制的电机数量 ---
#define MOTOR_COUNT 3

// --- 电机"树状"完整结构体定义 ---
typedef struct {
    /* --- 静态配置参数 (在初始化时设置) --- */
    uint8_t             id;                 // 电机ID (1, 2, 3...)
    TIM_HandleTypeDef* pwm_timer;          // PWM定时器句柄
    uint32_t            pwm_ch_forward;     // 正转PWM通道
    uint32_t            pwm_ch_reverse;     // 反转PWM通道
    uint32_t            pwm_max_pulse;      // PWM最大脉冲值 (ARR的值)
    
    TIM_HandleTypeDef* encoder_timer;      // 编码器定时器句柄
    float               encoder_ppr;        // 输出轴每转一圈的脉冲数 (基础PPR * 减速比)
    
    /* --- 动态运动参数 (运行时更新) --- */
    volatile float      speed_rpm_actual;   // 【反馈】编码器测得的实时转速 (RPM)
    volatile float      speed_rpm_target;   // 【输入】PID的目标转速 (RPM)
    
    /* --- PID 控制器参数 (增益) --- */
    float               pid_kp;             // P: 比例 Proportional
    float               pid_ki;             // I: 积分 Integral
    float               pid_kd;             // D: 微分 Derivative
    
    /* --- PID 内部状态变量 ("记忆") --- */
    float               pid_error_last;     // 上一次的误差 (用于D项)
    float               pid_error_integral; // 误差累积 (用于I项)
    float               pid_integral_limit; // 积分限幅，防止积分饱和
    
    /* --- 内部状态变量 --- */
    int16_t             last_encoder_count; // 用于计算速度差值

} Motor_t;


/* --- 公共API函数原型 --- */

/**
  * @brief  初始化电机模块，配置所有电机的硬件参数和PID初始值.
  * @param  None
  * @retval None
  */
void Motor_Init(void);

/**
  * @brief  启动所有与电机相关的硬件定时器 (PWM, Encoder, Base Interrupt).
  * @param  None
  * @retval None
  */
void Motor_Start(void);

/**
  * @brief  设置单个电机的期望目标转速 (单位: RPM).
  * @param  motor_id: 电机ID (0, 1, 2, 3...)
  * @param  target_rpm: 目标转速，可正可负
  * @retval None
  */
void Motor_Set_Target_Speed(uint8_t motor_id, float target_rpm);

/**
  * @brief  执行所有电机的PID闭环控制计算，并更新PWM输出.
  * @note   此函数应在主控制循环中以固定频率被周期性调用 (例如在FreeRTOS任务中每20ms调用一次).
  * @param  None
  * @retval None
  */
void Motor_Run_PID_Control(void);

/**
  * @brief  获取单个电机的实时测量转速.
  * @param  motor_id: 电机ID (0, 1, 2, 3...)
  * @retval 电机当前的实时转速 (RPM)
  */
float Motor_Get_Speed(uint8_t motor_id);

/**
  * @brief  编码器数据更新回调函数.
  * @note   此函数应在基准定时器 (TIM9) 的中断服务程序中被调用，用以计算所有电机的瞬时速度.
  * @param  None
  * @retval None
  */
void Motor_Encoder_Update_Callback(void);

/**
  * @brief  【新增】以开环方式直接设定电机的PWM占空比.
  * @note   此函数将绕过PID控制器，直接控制PWM输出.
  * @param  motor_id: 电机ID (0,1, 2, 3...)
  * @param  duty_cycle: 占空比指令 (-100 to +100). 负数反转, 正数正转.
  * @retval None
  */
void Motor_Set_PWM_Duty(uint8_t motor_id, int16_t duty_cycle);

#endif /* INC_MOTOR_CONTROL_H_ */
