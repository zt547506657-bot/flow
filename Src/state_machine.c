#include "state_machine.h"
#include "relay_control.h"
#include "tjc_usart_hmi.h"
#include "../../Drivers/BSP/YFS401/YFS401.h"
#include <stdio.h>
#include <string.h>

// 初始化状态机
void initStateMachine(StateMachine *sm) {
    sm->currentState = STATE_IDLE;
    sm->currentStep = 0;
    sm->totalSteps = 0;
    sm->totalExchangeVolume = 0.0f;
    sm->ROWaterVolume = 0.0f;
    sm->mixedWaterVolume = 0.0f;
    sm->accumulatedROVolume = 0.0f;
    sm->accumulatedMixedVolume = 0.0f;
    sm->accumulatedInflow = 0.0f;
    sm->accumulatedOutflow = 0.0f;
    sm->waterMode = 0;
    sm->isWaterChangeActive = false;

    for (int i = 0; i < 3; i++) {
        sm->calibrationTimer[i] = 0;
        sm->isCalibrating[i] = false;
        sm->relayStatus[i] = false;
    }

    Relay_Init(sm);  // 初始化继电器状态
}

// 更新状态机状态
void updateStateMachine(StateMachine *sm) {
    switch (sm->currentState) {
        case STATE_IDLE:
            // 空闲状态下的处理
            break;

        case STATE_CALIBRATING:
            // 校准状态下的处理
            for (int i = 0; i < 3; i++) {
                if (sm->isCalibrating[i] && sm->calibrationTimer[i] > 0) {
                    sm->calibrationTimer[i]--;  // 倒计时减1秒

                    // 更新校准倒计时的显示
                    TJCPrintf("t7.txt=\"流量计%d校准中，剩余时间: %d秒\"\xff\xff\xff", i + 1, sm->calibrationTimer[i]);

                    if (sm->calibrationTimer[i] == 0) {
                        stopCalibration(sm, i,false);  // 倒计时结束，停止校准
                    }
                }
            }
            break;

        case STATE_EXCHANGING:
            controlWaterFlow(sm);  // 控制水流过程

            break;

        case STATE_MANUAL_CONTROL:
            // 手动控制状态
            ManualFlowControl(sm->currentStep);  // 处理当前继电器的手动控制
            break;

        case STATE_FINISHED:
            // 完成状态的处理
            break;
    }
}

// 开始校准流程
void startCalibration(StateMachine *sm, uint8_t sensorID) {
    // 重置流量计相关变量
    golbal_flow[sensorID].accumulated_pulse_count = 0;
    golbal_flow[sensorID].acculat = 0.0f;
    sm->isCalibrating[sensorID] = true;
    sm->calibrationTimer[sensorID] = 30;  // 设置30秒倒计时
    sm->currentState = STATE_CALIBRATING;  // 切换到校准状态
    Relay_On(sm, sensorID);  // 打开对应的继电器

    // 显示校准开始的提示
    TJCPrintf("t7.txt=\"流量计%d校准开始\"\xff\xff\xff", sensorID + 1);
}
// 停止校准流程
void stopCalibration(StateMachine *sm, uint8_t sensorID, bool isForced) {
    // 停止校准过程，关闭校准标志位和计时器
    sm->isCalibrating[sensorID] = false;
    sm->calibrationTimer[sensorID] = 0;

    // 强制关闭对应的继电器，确保继电器状态一致
    Relay_Off(sm, sensorID);

    if (isForced) {
        // 非正常中断校准，重置脉冲数和流量计数据
        golbal_flow[sensorID].accumulated_pulse_count = 0;
        golbal_flow[sensorID].acculat = 0.0f;

        // 重置 x6, x7, x8 为 0
        TJCPrintf("main.x%d.val=0", sensorID + 6);  // 重置 x6, x7, x8 为 0

        // 显示中断校准的提示信息
        TJCPrintf("t7.txt=\"流量计%d校准被中断\"\xff\xff\xff", sensorID + 1);
    } else {
        // 正常结束校准，读取并记录校准期间的脉冲数
        uint16_t calibration_pulse_count = golbal_flow[sensorID].accumulated_pulse_count;

        // 显示校准结束的提示和脉冲数
        TJCPrintf("t7.txt=\"校准脉冲数: %d\"\xff\xff\xff", calibration_pulse_count);

        // 将校准得到的脉冲数应用到 pulses_per_liter 数组
        pulses_per_liter[sensorID] = calibration_pulse_count;

        // 保存校准结果
        golbal_flow[sensorID].saved_pulse_count = calibration_pulse_count;

        // 提示用户输入液体体积
        TJCPrintf("t7.txt=\"请输入流量计%d校准液体容积 (L)\"\xff\xff\xff", sensorID + 1);

        // 将校准脉冲数发送到 n0, n1, n2 控件，更新显示
        TJCPrintf("main.n%d.val=%d", sensorID, calibration_pulse_count);

        // 在校准计算完成后，重置 x6, x7, x8 为 0
        TJCPrintf("main.x%d.val=0", sensorID + 6);  // 重置 x6, x7, x8 为 0
    }

    // 重置按钮状态
    TJCPrintf("bt%d.val=0\xff\xff\xff", sensorID + 7);

    // 切换状态机到空闲状态
    sm->currentState = STATE_IDLE;
}


// 初始化脉冲数，并显示在串口屏的 n0, n1, n2 控件上
void InitializePulsesPerLiterDisplay(void) {
    // 将 pulses_per_liter 的值赋给 n0, n1, n2 并显示在屏幕上
    TJCPrintf("main.n0.val=%d", pulses_per_liter[0]);
    TJCPrintf("main.n1.val=%d", pulses_per_liter[1]);
    TJCPrintf("main.n2.val=%d", pulses_per_liter[2]);

    // 在 t7 控件上显示确认信息
    TJCPrintf("t7.txt=\"脉冲数已初始化\"\xff\xff\xff");
}

// 开始换水流程
void startExchange(StateMachine *sm) {
    if (!sm->isWaterChangeActive) {
        sm->currentStep = 0;
        sm->totalSteps = sm->exchange_steps;
        sm->ROWaterVolume = sm->totalExchangeVolume * (sm->mix_ratio / (sm->mix_ratio + 1));  // 计算RO水量
        sm->mixedWaterVolume = sm->totalExchangeVolume - sm->ROWaterVolume;  // 计算混水量

        sm->accumulatedROVolume = 0.0f;
        sm->accumulatedMixedVolume = 0.0f;
        sm->accumulatedInflow = 0.0f;
        sm->accumulatedOutflow = 0.0f;

        // 重置显示控件的值
        TJCPrintf("x11.val=0");
        TJCPrintf("x12.val=0");
        TJCPrintf("x13.val=0");
        TJCPrintf("x14.val=0");

        TJCPrintf("t7.txt=\"开始换水: RO=%.2f, 混水=%.2f, 废水=%.2f\"\xff\xff\xff", sm->ROWaterVolume, sm->mixedWaterVolume, sm->totalExchangeVolume);

        if (sm->waterMode == 1) {  // 先出后进模式
            sm->relayStatus[WASTE_INDEX] = true;  // 打开废水继电器
            relayControl(sm, WASTE_INDEX, true);
        } else {  // 同进同出模式
            sm->relayStatus[RO_INDEX] = true;
            sm->relayStatus[MIXED_INDEX] = true;
            sm->relayStatus[WASTE_INDEX] = true;

            relayControl(sm, RO_INDEX, true);
            relayControl(sm, MIXED_INDEX, true);
            relayControl(sm, WASTE_INDEX, true);
        }

        sm->isWaterChangeActive = true;
        sm->currentState = STATE_EXCHANGING;
    }
}


// 控制水流的过程
void controlWaterFlow(StateMachine *sm) {
	 TJCPrintf("t7.txt=\"Step %d, RO=%.2f, Mixed=%.2f, Outflow=%.2f\"\xff\xff\xff",
	              sm->currentStep, sm->accumulatedROVolume, sm->accumulatedMixedVolume, sm->accumulatedOutflow);
	UpdateWaterFlowDisplay(sm->accumulatedROVolume, sm->accumulatedMixedVolume, sm->accumulatedOutflow);
    if (sm->waterMode == 1) {  // 先出后进模式
        if (sm->relayStatus[WASTE_INDEX] && sm->accumulatedOutflow >= sm->totalExchangeVolume * (sm->currentStep + 1)) {
            sm->relayStatus[WASTE_INDEX] = false;
            relayControl(sm, WASTE_INDEX, false);

            sm->relayStatus[RO_INDEX] = true;
            sm->relayStatus[MIXED_INDEX] = true;
            relayControl(sm, RO_INDEX, true);
            relayControl(sm, MIXED_INDEX, true);
        } else if (!sm->relayStatus[WASTE_INDEX]) {
            if (sm->accumulatedROVolume >= sm->ROWaterVolume * (sm->currentStep + 1)) {
                sm->relayStatus[RO_INDEX] = false;
                relayControl(sm, RO_INDEX, false);
            }
            if (sm->accumulatedMixedVolume >= sm->mixedWaterVolume * (sm->currentStep + 1)) {
                sm->relayStatus[MIXED_INDEX] = false;
                relayControl(sm, MIXED_INDEX, false);
            }
            if (!sm->relayStatus[RO_INDEX] && !sm->relayStatus[MIXED_INDEX]) {
                TJCPrintf("t7.txt=\"本次换水步骤完成\"\xff\xff\xff");
                if (++sm->currentStep < sm->totalSteps) {
                    startExchange(sm);  // 继续下一步
                } else {
                    finishExchange(sm);  // 全部步骤完成
                }
            }
        }
    } else {  // 同进同出模式
        if (sm->accumulatedROVolume >= sm->ROWaterVolume * (sm->currentStep + 1)) {
            sm->relayStatus[RO_INDEX] = false;
            relayControl(sm, RO_INDEX, false);
        }
        if (sm->accumulatedMixedVolume >= sm->mixedWaterVolume * (sm->currentStep + 1)) {
            sm->relayStatus[MIXED_INDEX] = false;
            relayControl(sm, MIXED_INDEX, false);
        }
        if (sm->accumulatedOutflow >= sm->totalExchangeVolume * (sm->currentStep + 1)) {
            sm->relayStatus[WASTE_INDEX] = false;
            relayControl(sm, WASTE_INDEX, false);
        }
        if (!sm->relayStatus[RO_INDEX] && !sm->relayStatus[MIXED_INDEX] && !sm->relayStatus[WASTE_INDEX]) {
            TJCPrintf("t7.txt=\"本次换水步骤完成\"\xff\xff\xff");
            if (++sm->currentStep < sm->totalSteps) {
                startExchange(sm);  // 继续下一步
            } else {
                finishExchange(sm);  // 全部步骤完成
            }
        }
    }
}


// 完成换水流程
void finishExchange(StateMachine *sm) {
    // 关闭所有继电器
    for (int i = 0; i < 3; i++) {
        sm->relayStatus[i] = false;
        relayControl(sm, i, false);
    }

    // 停止换水状态
    sm->isWaterChangeActive = false;
    sm->currentState = STATE_FINISHED;

    // 发送换水完成信息到 t7 控件
    TJCPrintf("t7.txt=\"换水完成\"\xff\xff\xff");

    // 重置 x11, x12, x13, x14 控件的值
		TJCPrintf("x11.val=0");
		TJCPrintf("x12.val=0");
		TJCPrintf("x13.val=0");
		TJCPrintf("x14.val=0");

    // 显示总的换水量
    TJCPrintf("t7.txt=\"总计RO水: %.2f L, 混水: %.2f L, 排水: %.2f L\"\xff\xff\xff",
              sm->accumulatedROVolume, sm->accumulatedMixedVolume, sm->accumulatedOutflow);
}

// 更新显示
void updateDisplay(StateMachine *sm) {
    // 这里实现OLED或串口屏显示的更新逻辑
}
