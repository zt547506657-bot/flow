#include "relay_control.h"
#include "state_machine.h"

// 初始化所有继电器，将它们设置为关闭状态
void Relay_Init(struct StateMachine *sm) {
    Relay_Off(sm, RO_INDEX);
    Relay_Off(sm, MIXED_INDEX);
    Relay_Off(sm, WASTE_INDEX);
}

// 打开指定继电器，传入继电器ID
void Relay_On(struct StateMachine *sm, uint8_t relayID) {
    relayControl(sm, relayID, true);
}

// 关闭指定继电器，传入继电器ID
void Relay_Off(struct StateMachine *sm, uint8_t relayID) {
    relayControl(sm, relayID, false);
}

// 控制继电器的开关函数，根据传入的状态设置相应的GPIO引脚电平
void relayControl(struct StateMachine *sm, uint8_t relayID, bool state) {
    GPIO_PinState pinState = state ? GPIO_PIN_SET : GPIO_PIN_RESET;

    switch (relayID) {
        case RO_INDEX:
            HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, pinState);
            sm->relayStatus[RO_INDEX] = state;
            break;
        case MIXED_INDEX:
            HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, pinState);
            sm->relayStatus[MIXED_INDEX] = state;
            break;
        case WASTE_INDEX:
            HAL_GPIO_WritePin(RELAY3_GPIO_Port, RELAY3_Pin, pinState);
            sm->relayStatus[WASTE_INDEX] = state;
            break;
        default:
            // 无效继电器ID，错误处理
            return;
    }
}
