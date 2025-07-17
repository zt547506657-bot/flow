#include "YFS401.h"
#include <stdint.h>

GOLBAL_FLOW golbal_flow[3];

// 定义流量传感器的相关参数（例如，校准系数等）
float flowK[3] = {5.0f, 5.5f, 8.0f};                  // 流量系数 K
// 初始化 pulses_per_liter 数组，存储每个流量计的校准脉冲数
uint16_t pulses_per_liter[3] = {300, 330, 480};  // 默认值，可以在初始化时赋值

//==============================================================================
// @函数名: Flow_Read
// @功能: 读取单个流量传感器的数据
// @参数: flow_sensor 指向单个流量传感器数据的指针
// @参数: sensor_id 传感器的 ID，确定使用哪个传感器的 K 值和脉冲计数
//==============================================================================
void Flow_Read(GOLBAL_FLOW* flow_sensor, uint8_t sensor_id)
{
    float pulseCntValue = pulses_per_liter[sensor_id];  // 使用校准后的脉冲计数值

    if (flow_sensor->pluse_1s > 0)
    {
        flow_sensor->acculat += (flow_sensor->pluse_1s / pulseCntValue);   // 单位：L
        if (flow_sensor->acculat >= 1000.0f)  // 累计流量大于或等于1000L
        {
            flow_sensor->acculat = 0.0f;  // 重置累计流量
        }
        flow_sensor->pluse_1s = 0;
    }
    else
    {
        flow_sensor->instant = 0.0f;  // 如果没有脉冲，瞬时流量为0
    }
    flow_sensor->receive_flag = 0;   // 重置标志位
}


