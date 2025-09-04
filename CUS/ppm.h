#ifndef __PPM_H__
#define __PPM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h" // 包含标准整数类型定义

#define PPM_CHANNELS 8 // 定义PPM通道数量

// --- 全局变量声明 (Extern) ---
// 其他文件可以通过包含本头文件来访问这些变量

// 存储由中断写入的原始脉冲宽度 (单位: µs)
extern uint16_t PPM_Databuf[PPM_CHANNELS];

// 存储处理后的最终控制值 (-100 to 100)
extern volatile int filtered_channel_values[PPM_CHANNELS];

// 存储一帧数据是否接收完成的标志位 (0:未完成, 1:已完成)
extern volatile uint8_t ppm_frame_ready;

/**
  * @brief  处理原始PPM数据, 进行映射和滤波.
  * @note   此函数读取 PPM_Databuf, 并将最终结果存入 filtered_channel_values 数组.
  * @param  None
  * @retval None
  */
void process_channel_values(void);


#ifdef __cplusplus
}
#endif

#endif /* __PPM_H__ */
