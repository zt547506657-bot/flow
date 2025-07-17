#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

#include "main.h"
#include <stdbool.h>

// 前向声明 StateMachine 结构体
struct StateMachine;

// 继电器索引定义
#define RO_INDEX     0  // RO 继电器
#define MIXED_INDEX  1  // MIXED 继电器
#define WASTE_INDEX  2  // WASTE 继电器

// 初始化所有继电器，将它们设置为关闭状态
void Relay_Init(struct StateMachine *sm);

// 打开指定继电器，传入继电器ID
void Relay_On(struct StateMachine *sm, uint8_t relayID);

// 关闭指定继电器，传入继电器ID
void Relay_Off(struct StateMachine *sm, uint8_t relayID);

// 获取指定继电器的状态，传入继电器ID，返回0表示关闭，1表示打开
uint8_t Relay_GetStatus(struct StateMachine *sm, uint8_t relayID);

// 控制继电器开关，传入继电器ID和状态（true: 打开, false: 关闭）
void relayControl(struct StateMachine *sm, uint8_t relayID, bool state);

#endif // RELAY_CONTROL_H
