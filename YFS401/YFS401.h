#ifndef __YFS401_H__
#define __YFS401_H__

#include <stdint.h>
#include "stdio.h"

// 定义结构体来保存每个流量传感器的数据
typedef struct
{
    uint8_t receive_flag;   // 1秒标志位
    uint16_t pluse_1s;      // 1秒内脉冲计数
    uint32_t accumulated_pulse_count;
    uint16_t saved_pulse_count;    // 用于校准的脉冲计数保存

    float instant;          // 瞬时流量
    float acculat;          // 累计流量
} GOLBAL_FLOW;

// 定义一个数组来保存三个传感器的数据
extern GOLBAL_FLOW golbal_flow[3];
// 定义 pulses_per_liter 数组，用于存储校准后的脉冲数
extern uint16_t pulses_per_liter[3];
// 函数声明，用于读取每个流量传感器的数据
void Flow_Read(GOLBAL_FLOW* flow_sensor, uint8_t sensor_id);

#endif /* __YFS401_H__ */
