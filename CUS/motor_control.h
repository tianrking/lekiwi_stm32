/* motor_control.h */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stm32f4xx_hal.h" // 包含此文件以识别TIM_HandleTypeDef

// --- 定义电机数量 ---
#define MOTOR_COUNT 3 // 我们现在控制2个电机

// --- 电机"树状"结构体定义 ---
typedef struct {
    /* --- 静态配置参数 (在初始化时设置) --- */
    uint8_t             id;                 // 电机ID
    TIM_HandleTypeDef* pwm_timer;          // PWM定时器句柄
    uint32_t            pwm_ch_forward;     // 正转PWM通道
    uint32_t            pwm_ch_reverse;     // 反转PWM通道
    uint32_t            pwm_max_pulse;      // PWM最大脉冲值 (ARR)
    
    TIM_HandleTypeDef* encoder_timer;      // 编码器定时器句柄
    float               encoder_ppr;        // 编码器每转脉冲数
    
    /* --- 动态运动参数 (运行时更新) --- */
    volatile float      speed_rpm;          // [输出] 编码器测得的实时转速
    int16_t             target_speed_cmd;   // [输入] 目标速度指令 (-100 to 100)
    
    /* --- 内部状态变量 --- */
    int16_t             last_encoder_count; // 用于计算速度差值

} Motor_t;


/* --- 公共API函数原型 --- */

// 1. 初始化电机模块 (配置所有电机的参数)
void Motor_Init(void);

// 2. 启动所有与电机相关的硬件 (PWM和编码器)
void Motor_Start(void);

// 3. 设置单个电机的速度指令
void Motor_Set_Speed(uint8_t motor_id, int16_t speed_cmd);

// 4. 获取单个电机的实时转速
float Motor_Get_Speed(uint8_t motor_id);

// 5. [内部调用] 在定时器中断中被调用的编码器数据更新函数
void Motor_Encoder_Update_Callback(void);


#endif /* INC_MOTOR_CONTROL_H_ */
