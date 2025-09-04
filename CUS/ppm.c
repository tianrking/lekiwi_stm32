#include "ppm.h"

// --- 常量定义 ---
#define O_MIN 1000 // 遥控器原始输出最小值 (1000µs = 1ms)
#define O_MAX 2000 // 遥控器原始输出最大值 (2000µs = 2ms)
#define FILTER_ALPHA 0.8f  // 滤波系数 (0.0 to 1.0)

// --- 全局变量定义 (实体) ---
// 在此文件中定义变量实体，其他文件通过 extern 来引用它们
uint16_t   PPM_Databuf[PPM_CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
volatile int filtered_channel_values[PPM_CHANNELS] = {0};
volatile uint8_t ppm_frame_ready = 0;

/**
  * @brief  将一个值从输入范围映射到输出范围.
  */
static int map_value(int value, int in_min, int in_max, int out_min, int out_max) {
    if (value < in_min) value = in_min;
    if (value > in_max) value = in_max;
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
  * @brief  对新值进行一阶低通滤波.
  */
static int apply_filter(int new_value, int old_value) {
    return old_value + FILTER_ALPHA * (new_value - old_value);
}

/**
  * @brief  处理PPM原始数据，进行映射和滤波.
  */
void process_channel_values() {
    for (int i = 0; i < PPM_CHANNELS; i++) {
        // 1. 将PPM_Databuf中的原始脉宽 (1000-2000µs) 映射到 -100 到 100
        int mapped_value = map_value(PPM_Databuf[i],
                                     O_MIN,
                                     O_MAX,
                                     -100, 100);

        // 2. 对映射后的值进行滤波，使其更平滑
        filtered_channel_values[i] = apply_filter(mapped_value, filtered_channel_values[i]);
    }
}
