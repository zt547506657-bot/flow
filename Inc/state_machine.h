#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>
#include "relay_control.h"

// 定义状态机的不同状态
typedef enum {
    STATE_IDLE,             // 空闲状态
    STATE_CALIBRATING,      // 校准状态
    STATE_EXCHANGING,       // 换水状态
    STATE_MANUAL_CONTROL,   // 手动控制状态
    STATE_FINISHED          // 完成状态
} State;

// 定义状态机结构体，包含当前状态及相关变量
typedef struct StateMachine {
    State currentState;              // 当前状态
    uint8_t currentStep;             // 当前换水步骤
    uint8_t totalSteps;              // 换水的总步骤数
    float totalExchangeVolume;       // 总换水量
    float ROWaterVolume;             // 当前步骤的RO水量
    float mixedWaterVolume;          // 当前步骤的混水量
    float accumulatedROVolume;       // 已累积的RO水量
    float accumulatedMixedVolume;    // 已累积的混水量
    float accumulatedInflow;         // 已累积的进水量
    float accumulatedOutflow;        // 已累积的废水量
    uint8_t waterMode;               // 换水模式（0: 同进同出, 1: 先出后进）
    bool isWaterChangeActive;        // 是否处于换水状态
    uint8_t calibrationTimer[3];     // 三个流量计的校准倒计时
    bool isCalibrating[3];           // 三个流量计的校准状态标志
    bool relayStatus[3];             // 三个继电器的状态
    float setpoint[3];               // 每个流量计的设定值
    float mix_ratio;                 // 用于存储混水比例的变量
    uint16_t exchange_steps;         // 换水的步骤数
    float manual_setpoint[3];        // 用于手动流量控制的设定值
} StateMachine;

// 状态机相关函数的声明
void initStateMachine(StateMachine *sm);               // 初始化状态机
void updateStateMachine(StateMachine *sm);             // 更新状态机状态
void startCalibration(StateMachine *sm, uint8_t sensorID);  // 开始校准流程
void stopCalibration(StateMachine *sm, uint8_t sensorID, bool isForced);
void startExchange(StateMachine *sm);                  // 开始换水流程
void controlWaterFlow(StateMachine *sm);               // 控制水流的过程
void finishExchange(StateMachine *sm);                 // 完成换水流程
void updateDisplay(StateMachine *sm);                  // 更新显示（可选）
void UpdateWaterFlowDisplay(float ro_water_flow, float mixed_water_flow, float waste_water_flow); // 更新水流量显示
void ManualFlowControl(uint8_t control_id);  // 手动控制继电器

#endif // STATE_MACHINE_H
