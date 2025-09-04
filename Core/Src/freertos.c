/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "motor_control.h"
#include "ppm.h"
#include "protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
//    Motor_Set_Speed(1, 25);  // 电机1以25%速度正转
//    Motor_Set_Speed(2, -50); // 电机2以50%速度反转
extern Motor_t g_motors[MOTOR_COUNT];
int16_t spp[MOTOR_COUNT];


//////////////  for rc
#define RC_DEADBAND 10
// 定义最大RPM，用于将遥控器指令映射到转速
#define MAX_RPM 1000.0f
///////////////////
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    
  Motor_Init();  // 初始化我们的电机模块
  Motor_Start(); // 启动所有电机相关的硬件
    
  Protocol_Init();
    
  HAL_TIM_IC_Start_IT(&htim10, TIM_CHANNEL_1);

  Motor_Set_Target_Speed(0, 0.0f);   // 命令电机0 (原电机1) 以 500 RPM 旋转
  Motor_Set_Target_Speed(1, 0.0f);  // 命令电机1 (原电机2) 以 300 RPM 反向旋转
  Motor_Set_Target_Speed(2, 0.0f);     // 命令电机2 (原电机3) 保持静止
    
    /* Infinite loop */
  for(;;)
  {
    Protocol_Process();
      
    // 检查是否有新的一帧PPM遥控数据
    if (ppm_frame_ready)
    {
        // 1. 处理原始数据，填充 filtered_channel_values 数组
        process_channel_values();
        
        // 2. 清除标志位，等待下一帧
        ppm_frame_ready = 0;

        // 3. 【核心控制逻辑】
        // a. 获取前进/后退 和 旋转的遥控指令 (-100 to 100)
        int16_t forward_cmd = filtered_channel_values[2];
        int16_t rotation_cmd = filtered_channel_values[0];
        
        // b. 应用死区
        if (forward_cmd > -RC_DEADBAND && forward_cmd < RC_DEADBAND) {
            forward_cmd = 0;
        }
        if (rotation_cmd > -RC_DEADBAND && rotation_cmd < RC_DEADBAND) {
            rotation_cmd = 0;
        }

        // c. 【运动混合】计算每个轮子的最终指令
        //    最终指令 = 前进/后退分量 + 旋转分量
        int16_t motor_cmd_0 = forward_cmd + rotation_cmd;
        int16_t motor_cmd_1 = -forward_cmd + rotation_cmd;
        int16_t motor_cmd_2 = 0           + rotation_cmd; // 电机2只参与旋转

        // d. 【指令限幅】防止叠加后的值超出 -100 ~ 100 的范围
        if (motor_cmd_0 > 100) motor_cmd_0 = 100;
        if (motor_cmd_0 < -100) motor_cmd_0 = -100;
        
        if (motor_cmd_1 > 100) motor_cmd_1 = 100;
        if (motor_cmd_1 < -100) motor_cmd_1 = -100;
        
        if (motor_cmd_2 > 100) motor_cmd_2 = 100;
        if (motor_cmd_2 < -100) motor_cmd_2 = -100;
        
        // e. 将最终的控制指令 (-100 to 100) 映射为PID的目标转速 (RPM)
        float target_rpm_0 = (motor_cmd_0 / 100.0f) * MAX_RPM;
        float target_rpm_1 = (motor_cmd_1 / 100.0f) * MAX_RPM;
        float target_rpm_2 = (motor_cmd_2 / 100.0f) * MAX_RPM;
        
        // f. 将计算出的目标速度下发给PID控制器
        Motor_Set_Target_Speed(0, target_rpm_0);
        Motor_Set_Target_Speed(1, target_rpm_1);
        Motor_Set_Target_Speed(2, target_rpm_2);
    }

      
//        for(int i = 0 ;i <3 ; i++)
//      {
//          Motor_Set_PWM_Duty(i,spp[i]);
//      }
    //Motor_Run_PID_Control();
    //osDelay(1);
      Motor_Run_PID_Control();
      osDelay(20); 
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

