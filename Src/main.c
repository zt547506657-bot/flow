/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Drivers/BSP/YFS401/YFS401.h"
#include "stdio.h"
#include <string.h>
#include <stdbool.h>
#include "tjc_usart_hmi.h"
#include "state_machine.h"
#include "relay_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EXCHANGE_FLOW_COMPLETE (0xFFFF) // 换水完成标志
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
StateMachine stateMachine;
float bt6_setpoint = 0.0f;  // 专用于bt6的设定值缓存

float last_acculat[3] = {0.0f, 0.0f, 0.0f}; // 存储上一次的累计流量值
float setpoint[3] = {0.0f, 0.0f, 0.0f}; // 虚拟浮点数控件设定的数值
bool relay_status[3] = {false, false, false}; // 继电器状态
bool is_calibrating[3] = {false, false, false};  // 增加校准状态标志位
volatile bool update_display_flag[3] = {false, false, false};  // 标志位数组
float manual_setpoint[3] = {0.0f, 0.0f, 0.0f};  // 分别用于 bt4, bt5, bt6 的手动控制设定值



// 一键换水相关变量
//float total_exchange_volume = 0.0f; // 总换水量
//float current_volume = 0.0f; // 当前换水量
//uint8_t exchange_steps = 0; // 换水步骤数
//uint8_t current_step = 0; // 当前换水步骤
//bool is_exchanging = false; // 是否正在换水
//uint8_t water_mode = 0; // 0: 同进同出, 1: 先出后进
//float RO_water_volume = 0.0f;  // 当前步骤中计划使用的RO水体积（以升为单位）
//float mixed_water_volume = 0.0f;  // 当前步骤中计划使用的混水体积（以升为单位）
//float accumulated_RO_volume = 0.0f;  // 在整个一键换水过程中，已累积的RO水体积（以升为单位）
//float accumulated_mixed_volume = 0.0f;  // 在整个一键换水过程中，已累积的混水体积（以升为单位）
//float accumulated_inflow = 0.0f;  // 在整个一键换水过程中，RO水和混水的总累计进水量（以升为单位）
//float accumulated_outflow = 0.0f;  // 在整个一键换水过程中，废水的总累计排水量（以升为单位）
//bool is_water_change_active = false;  // 标志位，指示一键换水过程是否处于激活状态。如果为 true，表示正在进行换水操作。
//float mix_ratio = 0.0f; // 定义一个用于混水比例的变量
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void processData(void);    // 处理接收到的数据帧，根据帧头调用相应的处理函数
void processFrame55(void); // 处理 0x55 帧头的数据帧，处理 b0, b1, b2 控件的清零命令
void processFrame56(void); // 处理 0x56 帧头的数据帧，处理流量控制逻辑和流量输入的通讯
void processFrame57(void); // 处理 0x57 帧头的数据帧，处理 bt4, bt5, bt6 控件的流量控制逻辑
void processFrame58(void); // 处理 0x58 帧头的数据帧，处理 bt7, bt8, bt9 控件的流量校准逻辑
void processFrame59(void); // 处理 0x59 帧头的数据帧，处理流量校准计数的输入通讯
void processFrame60(void); // 处理 0x60 帧头的数据帧，处理 n0, n1, n2 控件的脉冲输入通讯
void processFrame61(void); // 处理 0x61 帧头的数据帧，处理换水过程通讯
void UpdatePulseCountDisplay(void); // 将脉冲数显示在 n3, n4, n5 控件上
void InitializePulsesPerLiterDisplay(void); // 初始化脉冲数，并显示在串口屏的 n0, n1, n2 控件上
void UpdateWaterFlowDisplay(float ro_water_flow, float mixed_water_flow, float waste_water_flow); // 实时更新换水显示
void ManualFlowControl(uint8_t control_id);
void UpdateFlowCalculations();


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint32_t pluse1L;
extern GOLBAL_FLOW golbal_flow[3];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  initRingBuff();		//初始化环形缓冲区
  HAL_UART_Receive_IT(&huart1, RxBuff, 1);	//打开串口接收中断
//  char dis1[25];
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);
  initStateMachine(&stateMachine);
    // 调用脉冲数初始化并显示在串口屏上
    InitializePulsesPerLiterDisplay();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//stm32f103的GND接串口屏或串口工具的GND,共地
		//stm32f103的TX1(PA9)接串口屏或串口工具的RX
		//stm32f103的RX1(PA10)接串口屏或串口工具的TX
		//stm32f103的5V接串口屏的5V,如果是串口工具,不用接5V也可以

	  processData();// 处理接收到的数据并更新相关控件
	  UpdatePulseCountDisplay();// 更新脉冲计数的显示

	  // 更新 OLED 显示，根据最新的流量计数据
	    for (int i = 0; i < 3; i++) {
	        Flow_Read(&golbal_flow[i],i);

	        if (golbal_flow[i].acculat != last_acculat[i]) {  // 只有在累计流量发生变化时才更新显示
	            last_acculat[i] = golbal_flow[i].acculat;
	            int flow_value = (int)(golbal_flow[i].acculat * 100);
	            TJCPrintf("x%d.val=%d", i, flow_value);  // 发送到 x0, x1, x2
	        }
	    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 处理接收到的数据帧，根据帧头调用相应的处理函数
void processData(void) {
    while (getRingBuffLenght() >= 6) {  // 确保至少有最小数据包长度的数据可处理
        uint8_t frame_head = read1BFromRingBuff(0);
        switch (frame_head) {
            case 0x55:
                processFrame55();
                break;
            case 0x56:
                processFrame56();
                break;
            case 0x57:
                processFrame57();
                break;
            case 0x58:
                processFrame58();  // 处理校准按钮的逻辑
                break;
            case 0x59:
                processFrame59();  // 处理 x6, x7, x8 的输入
                break;
            case 0x60:
                processFrame60();  // 处理 n0, n1, n2 的输入
                break;
            case 0x61:
                processFrame61();  // 处理 x11, x12, x13, x14 的输入
                break;

            default:
                udelete(1);  // 如果帧头不匹配，删除1字节
                break;
        }
    }
}
void processFrame55(void) // 处理 0x55 帧头的数据帧，处理 b0, b1, b2 控件的清零命令
{
    uint8_t command = read1BFromRingBuff(1);
    uint8_t status = read1BFromRingBuff(2);
    if (command >= 0x05 && command <= 0x07 && status == 0x01) {  // 清零命令
        if (command == 0x05) {
            golbal_flow[0].acculat = 0.0f;  // 清零第一个流量传感器的累计流量
            TJCPrintf("t7.txt=\"RO水已清零\"\xff\xff\xff");
        } else if (command == 0x06) {
            golbal_flow[1].acculat = 0.0f;  // 清零第二个流量传感器的累计流量
            TJCPrintf("t7.txt=\"混水已清零\"\xff\xff\xff");
        } else if (command == 0x07) {
            golbal_flow[2].acculat = 0.0f;  // 清零第三个流量传感器的累计流量
            TJCPrintf("t7.txt=\"废水已清零\"\xff\xff\xff");
        }
    }
    udelete(6);  // 删除处理后的6字节数据
}
// 处理 0x56 帧头的数据帧，处理流量控制逻辑和流量输入的通讯
void processFrame56(void) {
    uint8_t control_id = read1BFromRingBuff(1);  // 从缓冲区读取控件ID
    uint16_t value = 0;

    // 从缓冲区中读取两个字节的数据并合并为一个16位的值
    value = (read1BFromRingBuff(3) << 8) | read1BFromRingBuff(2);

    // 将接收到的数值反馈到 t7 文本控件上显示
    TJCPrintf("t7.txt=\"接收到的数值: %d\"\xff\xff\xff", value);

    if (control_id >= 0x00 && control_id <= 0x02) {  // 如果控件ID在0x00到0x02之间，则表示是双态按钮用于控制继电器
        bool new_relay_state = (value == 1);  // 判断新的继电器状态，value 为 1 时表示打开

        // 只有当状态发生变化时才切换继电器状态
        if (stateMachine.relayStatus[control_id] != new_relay_state) {
            stateMachine.relayStatus[control_id] = new_relay_state;  // 更新状态机中的继电器状态

            // 调用 relayControl 函数控制相应的继电器
            relayControl(&stateMachine, control_id, new_relay_state);

            // 将继电器的状态反馈到 t7 文本控件中
            TJCPrintf("t7.txt=\"继电器%d %s\"\xff\xff\xff", control_id + 1, new_relay_state ? "打开" : "关闭");
        }
    } else if (control_id >= 0x03 && control_id <= 0x07) {  // 如果控件ID在0x03到0x07之间，则表示是虚拟浮点数控件
        stateMachine.setpoint[control_id - 0x03] = value / 10.0f;  // 将接收到的数值转换为浮点数并存储在状态机的设定值数组中

        if (control_id == 0x03 || control_id == 0x04 || control_id == 0x05) {
            // 如果是 x3, x4, x5，对应 bt4, bt5, bt6 的设定值
            manual_setpoint[control_id - 0x03] = value / 10.0f;  // 缓存到 manual_setpoint 数组中
            TJCPrintf("t7.txt=\"手动控制设定值%d: %.2f L\"\xff\xff\xff", control_id - 0x02, manual_setpoint[control_id - 0x03]);
        }

        // 根据不同的 control_id，分别处理不同的虚拟浮点数控件
        switch (control_id) {
            case 0x06:  // x9 控件
                stateMachine.totalExchangeVolume = stateMachine.setpoint[control_id - 0x03];  // 更新总换水量
                // 将总换水量反馈到 t7 文本控件中
                TJCPrintf("t7.txt=\"总换水量: %.2f L\"\xff\xff\xff", stateMachine.totalExchangeVolume);
                break;
            case 0x07:  // x10 控件
                stateMachine.mix_ratio = stateMachine.setpoint[control_id - 0x03];  // 更新混水比例
                // 将混水比例反馈到 t7 文本控件中
                TJCPrintf("t7.txt=\"混水比例: %.2f\"\xff\xff\xff", stateMachine.mix_ratio);
                break;
        }
    }

    udelete(6);  // 删除处理后的6字节数据
}

// 处理 0x57 帧头的数据帧，处理 bt0 bt1 bt4, bt5, bt6 控件功能
void processFrame57(void) {
    uint8_t control_id = read1BFromRingBuff(1);  // 获取按键编号
    uint8_t value = read1BFromRingBuff(2);       // 获取按键状态

    if (control_id >= 0x00 && control_id <= 0x02) {
            if (value == 0x01) {
                relayControl(&stateMachine, control_id, true);  // 打开继电器
                golbal_flow[control_id].acculat = 0.0f;  // 清零累计流量
                TJCPrintf("x%d.val=0\xff\xff\xff", control_id);  // 重置x0, x1, x2
                stateMachine.currentState = STATE_MANUAL_CONTROL;  // 切换到手动控制状态
                stateMachine.currentStep = control_id;  // 保存当前手动控制的继电器
            } else {
                relayControl(&stateMachine, control_id, false);  // 关闭继电器
                golbal_flow[control_id].acculat = 0.0f;  // 清零累计流量
                TJCPrintf("x%d.val=0\xff\xff\xff", control_id);
                TJCPrintf("bt%d.val=0\xff\xff\xff", control_id + 4);
            }
        }
    // 新增: 检查是否为 0x08 (对应 bt0 - 开始换水)
    else if (control_id == 0x08) {
        if (value == 0x01) {  // 按下bt0
            startExchange(&stateMachine);  // 开始换水过程
        } else {
            finishExchange(&stateMachine);   // 完成换水过程
        }
    }
    else if (control_id == 0x07) {
        stateMachine.waterMode = value;  // 更新水模式到状态机中
        TJCPrintf("t7.txt=\"模式: %s\"\xff\xff\xff", stateMachine.waterMode ? "先出后进" : "同进同出");
    }
    udelete(6);  // 删除处理后的6字节数据
}
// 处理 0x58 帧头的数据帧，处理 bt7, bt8, bt9 控件的流量校准逻辑
void processFrame58(void)
{
    uint8_t control_id = read1BFromRingBuff(1);  // 获取按键编号
    uint8_t value = read1BFromRingBuff(2);       // 获取按键状态

    if (control_id >= 0x00 && control_id <= 0x02) {  // 检查编号是否为 0x00, 0x01, 0x02 (对应 bt7, bt8, bt9)
        // 使用状态机的 relayControl 函数来控制继电器状态
        relayControl(&stateMachine, control_id, (value == 0x01));

        if (value == 0x01) {  // 按下按钮开始校准
            startCalibration(&stateMachine, control_id);  // 开始校准流程

            // 清零脉冲计数和累计流量
            golbal_flow[control_id].accumulated_pulse_count = 0;  // 清零累计脉冲计数
            golbal_flow[control_id].acculat = 0.0f;  // 清零累计流量
            golbal_flow[control_id].pluse_1s = 0;  // 清零每秒脉冲计数

            // 重置对应的 x6, x7, x8 数值
            TJCPrintf("main.x%d.val=0", control_id + 6);  // 重置 x6, x7, x8 为 0

            // 更新 t7 中校准状态
            TJCPrintf("t7.txt=\"流量计%d校准开始\"\xff\xff\xff", control_id + 1);
        } else {
            // 释放按钮停止校准
            stopCalibration(&stateMachine, control_id,true);  // 停止校准，确保倒计时结束
        }
    }
    udelete(6);  // 删除处理后的6字节数据
}

// 处理 0x59 帧头的数据帧，处理流量校准计数的输入通讯
void processFrame59(void) {
    uint8_t control_id = read1BFromRingBuff(1);
    uint16_t value = (read1BFromRingBuff(3) << 8) | read1BFromRingBuff(2);

    if (control_id >= 0x03 && control_id <= 0x05) {
        float volume = value / 10.0f;  // 应该得到 0.6

        uint16_t calibration_pulse_count = golbal_flow[control_id - 0x03].saved_pulse_count;

        if (volume > 0.0f) {
            uint16_t calculated_pulses_per_liter = (uint16_t)(calibration_pulse_count / volume);  // 应该得到 1043

            TJCPrintf("main.n%d.val=%d", control_id - 0x03, calculated_pulses_per_liter);
            TJCPrintf("t7.txt=\"流量计%d校准完成，每升脉冲数: %d\"\xff\xff\xff", control_id - 0x02, calculated_pulses_per_liter);

            pulses_per_liter[control_id - 0x03] = calculated_pulses_per_liter;

            golbal_flow[control_id - 0x03].saved_pulse_count = 0;
        } else {
            TJCPrintf("t7.txt=\"输入的体积无效，请重新输入\"\xff\xff\xff");
        }
    }

    udelete(6);
}
// 处理 0x60 帧头的数据帧，处理 n0, n1, n2 n6 控件的脉冲输入通讯
void processFrame60(void) {
    uint8_t control_id = read1BFromRingBuff(1);  // 获取控件编号
    uint16_t pulse_value = (read1BFromRingBuff(3) << 8) | read1BFromRingBuff(2);  // 获取输入的脉冲数

    // 显示在 t7 中
    if (control_id >= 0x00 && control_id <= 0x03) {  // 对应 n0, n1, n2, n6
        switch (control_id) {
            case 0x00:  // n0
            case 0x01:  // n1
            case 0x02:  // n2
                // 更新 pulses_per_liter 数组中的值
                pulses_per_liter[control_id] = pulse_value;
                TJCPrintf("t7.txt=\"脉冲数%d输入: %d\"\xff\xff\xff", control_id + 1, pulse_value);
                break;
            case 0x03:  // n6 对应的 control_id
                stateMachine.exchange_steps = pulse_value;
                TJCPrintf("t7.txt=\"拆分次数: %d\"\xff\xff\xff", stateMachine.exchange_steps);
                break;
        }
    }

    udelete(6);  // 删除处理后的6字节数据
}
void processFrame61(void) {
    uint8_t control_id = read1BFromRingBuff(1);
    uint16_t value = 0;

    // 保留帧头后面的两个字节，处理这两个字节，无论它们是否为00
    value = (read1BFromRingBuff(3) << 8) | read1BFromRingBuff(2);

    // 处理 x11, x12, x13, x14 对应的控件
    switch (control_id) {
        case 0x00:  // 对应 x11
            stateMachine.accumulatedROVolume = value / 100.0f;
            TJCPrintf("x11.val=%d", value);
            break;

        case 0x01:  // 对应 x12
            stateMachine.accumulatedMixedVolume = value / 100.0f;
            TJCPrintf("x12.val=%d", value);
            break;

        case 0x02:  // 对应 x13
            stateMachine.accumulatedInflow = value / 100.0f;
            TJCPrintf("x13.val=%d", value);
            break;

        case 0x03:  // 对应 x14
            stateMachine.accumulatedOutflow = value / 100.0f;
            TJCPrintf("x14.val=%d", value);
            break;

        default:
            TJCPrintf("t7.txt=\"无效的控制ID: %d\"\xff\xff\xff", control_id);
            break;
    }
    udelete(6);  // 删除处理后的6字节数据
}





// 将脉冲数显示在 n3, n4, n5 控件上
void UpdatePulseCountDisplay(void)

{
    for (int i = 0; i < 3; i++) {
        if (update_display_flag[i]) {
            TJCPrintf("main.n%d.val=%d", i + 3, golbal_flow[i].accumulated_pulse_count);
            update_display_flag[i] = false;  // 重置标志位
        }
    }
}
void ManualFlowControl(uint8_t control_id) {
    if (stateMachine.relayStatus[control_id]) {
        if (golbal_flow[control_id].acculat >= manual_setpoint[control_id]) {
            relayControl(&stateMachine, control_id, false);  // 关闭继电器
            golbal_flow[control_id].acculat = 0.0f;  // 清零累计流量
            TJCPrintf("x%d.val=0\xff\xff\xff", control_id);  // 重置x0, x1, x2
            TJCPrintf("bt%d.val=0\xff\xff\xff", control_id + 4);  // 重置bt4, bt5, bt6
            TJCPrintf("t7.txt=\"流量计%d已达到设定值，继电器已关闭\"\xff\xff\xff", control_id + 1);

            stateMachine.currentState = STATE_IDLE;  // 切换回空闲状态
        } else {
            // 实时更新流量并显示在x0, x1, x2
            TJCPrintf("x%d.val=%d\xff\xff\xff", control_id, (int)(golbal_flow[control_id].acculat * 100));
        }
    }
}

//一键换水更新数据
void UpdateWaterFlowDisplay(float ro_water_flow, float mixed_water_flow, float waste_water_flow) {
    static int last_ro_value = -1;
    static int last_mixed_value = -1;
    static int last_inflow_value = -1;
    static int last_waste_value = -1;

    int current_ro_value = (int)(ro_water_flow * 10);
    int current_mixed_value = (int)(mixed_water_flow * 10);
    int current_inflow_value = (int)((ro_water_flow + mixed_water_flow) * 10);
    int current_waste_value = (int)(waste_water_flow * 10);
    // 调试信息：显示当前准备更新的值
    TJCPrintf("t10.txt=\"RO=%d, Mixed=%d, Inflow=%d, Waste=%d\"\xff\xff\xff",
              current_ro_value, current_mixed_value, current_inflow_value, current_waste_value);


    // 更新 RO 水累计流量显示
    if (current_ro_value != last_ro_value) {
        TJCPrintf("x11.val=%d", current_ro_value);
        last_ro_value = current_ro_value;
    }

    // 更新混水累计流量显示
    if (current_mixed_value != last_mixed_value) {
        TJCPrintf("x12.val=%d", current_mixed_value);
        last_mixed_value = current_mixed_value;
    }

    // 更新进水总量（RO 水 + 混水）累计流量显示
    if (current_inflow_value != last_inflow_value) {
        TJCPrintf("x13.val=%d", current_inflow_value);
        last_inflow_value = current_inflow_value;
    }

    // 更新排水累计流量显示
    if (current_waste_value != last_waste_value) {
        TJCPrintf("x14.val=%d", current_waste_value);
        last_waste_value = current_waste_value;
    }
}

// 在需要时调用 Flow_Read 来计算累计流量
void UpdateFlowCalculations(void) {
    for (int i = 0; i < 3; i++) {
        Flow_Read(&golbal_flow[i], i);
    }
    updateStateMachine(&stateMachine);  // 更新状态机，处理当前状态
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == YFS401_Pin) {
        golbal_flow[0].pluse_1s++;
        golbal_flow[0].accumulated_pulse_count++;
        update_display_flag[0] = true;
    } else if(GPIO_Pin == YFS401A2_Pin) {
        golbal_flow[1].pluse_1s++;
        golbal_flow[1].accumulated_pulse_count++;
        update_display_flag[1] = true;
    } else if(GPIO_Pin == YFS401A3_Pin) {
        golbal_flow[2].pluse_1s++;
        golbal_flow[2].accumulated_pulse_count++;
        update_display_flag[2] = true;
    }

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        for (int i = 0; i < 3; i++) {
            golbal_flow[i].receive_flag = 1;
            Flow_Read(&golbal_flow[i], i);

        }
    }
    else if (htim->Instance == TIM2)  // 处理 TIM2 的中断
    {
        // 直接调用状态机的更新函数，让状态机来处理校准逻辑
        updateStateMachine(&stateMachine);  // 传递状态机结构体
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
