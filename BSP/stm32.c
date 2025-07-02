/**
 * @file stm32_servo_control.c
 * @brief STM32微控制器程序，用于控制舵机、传感器数据采集和处理，包括温湿度、光照强度、酒精浓度等数据的采集与传输，
 *        以及通过串口通信控制舵机运动和动作组运行。
 * @author 郑中岳 (Zhongyue Zheng)
 * @date 2025-07-01
 * @version 1.0
 * @note 该代码已通过测试，可用于实际硬件环境中。确保硬件初始化正确并连接相关传感器和舵机。
 * @license MIT License
 */

/* 头文件包含 */
#include "stm32.h"

/* 全局变量定义 */
bool start, isUartRxCompleted;						// 标志位：程序启动和串口接收完成
unsigned char LobotTxBuf[128];						// 舵机发送数据缓冲区
unsigned short int batteryVolt, LobotRxBuf[16];		// 电池电压和接收数据缓冲区
unsigned char RX_len, RX_buf[len2], jsonData[200];	// 串口接收长度、接收缓冲区和JSON数据缓冲区
extern UART_HandleTypeDef huart2;					// UART2句柄
extern UART_HandleTypeDef huart6;					// UART6句柄
extern TIM_HandleTypeDef htim3;						// 定时器3句柄
extern TIM_HandleTypeDef htim4;						// 定时器4句柄
extern ADC_HandleTypeDef hadc1;						// ADC1句柄
float Temperature, Temperature_c, Humidity = 0;		// 温度（空气）、温度（醋液）、湿度
unsigned int adcValue = 0;							// ADC采集值
float Alcohol = 0;									// 酒精浓度
int lux, lux_cnt = 0;								// 光照强度和计数器
unsigned short int count3, count4, ice_cnt, buz_cnt = 0;	// 定时器计数和蜂鸣器计数

/**
 * @brief 初始化所有外设和传感器
 * @details 初始化GY30光照传感器、SHT30温湿度传感器、DS18B20温度传感器，
 *          启用UART2中断，启动定时器3，并初始化舵机位置。
 */
void All_Init(void)
{
    GY30_Init();               // 初始化GY30光照传感器
    Init_SHT30();              // 初始化SHT30温湿度传感器
    DS18B20_Init();            // 初始化DS18B20温度传感器
    
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); // 启用UART2空闲中断
    HAL_UART_Receive_IT(&huart2, RX_buf, len2);  // 启动UART2接收中断
    HAL_TIM_Base_Start_IT(&htim3);               // 启动定时器3中断

    moveServo(1, 0, 500);                        // 初始化舵机1到0度，时间500ms
    moveServo(2, 1000, 1000);                    // 初始化舵机2到特定位置，时间1000ms
}

/**
 * @brief 重定向标准输出函数，将字符通过UART6发送
 * @param ch 待发送的字符
 * @param f 文件指针
 * @return 返回发送的字符
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 1); // 通过UART6发送单个字符
    return ch;
}

/**
 * @brief 定时器中断回调函数
 * @details 处理定时器3和定时器4的中断。定时器3用于采集传感器数据并通过JSON格式发送，
 *          定时器4用于控制舵机运动序列和蜂鸣器。
 * @param htim 定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        count3++;
        lux_cnt++;
        if (count3 >= 10000) {
            start = 0;
            // 格式化传感器数据为JSON并通过UART2发送
            snprintf((char*)jsonData, sizeof(jsonData),
                     "{\"services\": [{\"service_id\": \"2025_Ecc\", \"properties\": {\"Humidity\": %.2f, \"Temperature\": %.2f, \"Temperature_C\": %.2f, \"Alcohol\": %.2f, \"Light\": %d}}]}",
                     Humidity, Temperature, Temperature_c, Alcohol, lux);
            HAL_UART_Transmit(&huart2, (const unsigned char*)jsonData, sizeof(jsonData), 50);
            count3 = 0;
        }
        if (lux_cnt == 0) {
            Single_Write_BH1750(CONT_H_RES_MODE); // 启动BH1750光照传感器连续高分辨率模式
        } else if (lux_cnt > 181) {
            lux = Multiple_read_BH1750(); // 读取光照强度
            lux_cnt = 0;
        }
    } else if (htim->Instance == TIM4) {
        ice_cnt++;
        buz_cnt++;
        // 舵机运动序列控制
        if (ice_cnt == 0)       moveServo(2, 810, 900);   // 上舵机转至45°
        else if (ice_cnt == 1000) moveServo(2, 620, 900); // 上舵机转至90°
        else if (ice_cnt == 3000) moveServo(1, 750, 1900);// 下舵机转至180°
        else if (ice_cnt == 5500) moveServo(2, 1000, 400);// 上舵机转至0°
        else if (ice_cnt == 6000) moveServo(2, 620, 800); // 上舵机转至90°
        else if (ice_cnt == 7000) moveServo(2, 1000, 400);// 上舵机转至0°
        else if (ice_cnt == 8000) moveServo(2, 620, 800); // 上舵机转至90°
        else if (ice_cnt == 9000) moveServo(1, 0, 600);   // 下舵机转至0°
        else if (ice_cnt == 10800) moveServo(2, 1000, 1000); // 上舵机转至0°
        else if (ice_cnt == 11000) {
            ice_cnt = 0;
            buz_cnt = 0;
            HAL_TIM_Base_Stop_IT(&htim4); // 停止定时器4
        }
        
        // 蜂鸣器控制
        if (buz_cnt > 1 && buz_cnt < 666)
            HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief 读取ADC值
 * @details 启动ADC转换，轮询获取结果并返回转换值。
 * @return ADC转换值
 */
uint32_t ADC_Read(void)
{
    uint32_t adcValue = 0;
    HAL_ADC_Start(&hadc1); // 启动ADC
    if (HAL_ADC_PollForConversion(&hadc1, 200) == HAL_OK)
        adcValue = HAL_ADC_GetValue(&hadc1); // 获取ADC值
    HAL_ADC_Stop(&hadc1); // 停止ADC
    return adcValue;
}

/**
 * @brief 控制单个舵机转动
 * @details 根据舵机ID、目标位置和转动时间生成控制指令并通过串口发送。
 * @param servoID 舵机ID (0 <= servoID <= 31)
 * @param position 目标位置
 * @param time 转动时间 (time > 0)
 */
void moveServo(uint8_t servoID, uint16_t position, uint16_t time)
{
    if (servoID > 31 || !(time > 0)) {
        return;
    }
    
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;    // 填充帧头
    LobotTxBuf[2] = 8;                               // 数据长度
    LobotTxBuf[3] = CMD_SERVO_MOVE;                  // 舵机移动指令
    LobotTxBuf[4] = 1;                               // 要控制的舵机个数
    LobotTxBuf[5] = (uint8_t)(time);                 // 时间低八位
    LobotTxBuf[6] = (uint8_t)(time >> 8);            // 时间高八位
    LobotTxBuf[7] = servoID;                         // 舵机ID
    LobotTxBuf[8] = (uint8_t)(position);             // 目标位置低八位
    LobotTxBuf[9] = (uint8_t)(position >> 8);        // 目标位置高八位
    
    uartWriteBuf(LobotTxBuf, 10);                    // 发送数据
}

/**
 * @brief 控制多个舵机转动（通过数组）
 * @details 根据舵机结构体数组、舵机数量和转动时间生成控制指令并发送。
 * @param servos 舵机结构体数组
 * @param num 舵机个数 (0 < num <= 32)
 * @param time 转动时间 (time > 0)
 */
void moveServosByArray(LobotServo servos[], uint8_t num, uint16_t time)
{
    uint8_t index = 7;
    uint8_t i = 0;

    if (num < 1 || num > 32 || !(time > 0)) {
        return;
    }
    
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      // 填充帧头
    LobotTxBuf[2] = num * 3 + 5;                       // 数据长度
    LobotTxBuf[3] = CMD_SERVO_MOVE;                    // 舵机移动指令
    LobotTxBuf[4] = num;                               // 要控制的舵机个数
    LobotTxBuf[5] = (uint8_t)(time);                   // 时间低八位
    LobotTxBuf[6] = (uint8_t)(time >> 8);              // 时间高八位

    for (i = 0; i < num; i++) {
        LobotTxBuf[index++] = servos[i].id;            // 填充舵机ID
        LobotTxBuf[index++] = (uint8_t)(servos[i].position); // 目标位置低八位
        LobotTxBuf[index++] = (uint8_t)(servos[i].position >> 8); // 目标位置高八位
    }

    uartWriteBuf(LobotTxBuf, LobotTxBuf[2] + 2);       // 发送数据
}

/**
 * @brief 控制多个舵机转动（可变参数）
 * @details 根据舵机数量、转动时间和可变参数（舵机ID和目标位置交替排列）生成控制指令并发送。
 * @param num 舵机个数 (0 < num <= 32)
 * @param time 转动时间 (time > 0)
 * @param ... 舵机ID和目标位置交替排列
 */
void moveServos(uint8_t num, uint16_t time, ...)
{
    uint8_t index = 7;
    uint8_t i = 0;
    uint16_t temp;
    va_list arg_ptr;

    va_start(arg_ptr, time);
    if (num < 1 || num > 32) {
        va_end(arg_ptr);
        return;
    }
    
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      // 填充帧头
    LobotTxBuf[2] = num * 3 + 5;                       // 数据长度
    LobotTxBuf[3] = CMD_SERVO_MOVE;                    // 舵机移动指令
    LobotTxBuf[4] = num;                               // 要控制的舵机个数
    LobotTxBuf[5] = (uint8_t)(time);                   // 时间低八位
    LobotTxBuf[6] = (uint8_t)(time >> 8);              // 时间高八位

    for (i = 0; i < num; i++) {
        temp = va_arg(arg_ptr, int);                   // 获取舵机ID
        LobotTxBuf[index++] = (uint8_t)temp;
        temp = va_arg(arg_ptr, int);                   // 获取目标位置
        LobotTxBuf[index++] = (uint8_t)temp;           // 目标位置低八位
        LobotTxBuf[index++] = (uint8_t)(temp >> 8);    // 目标位置高八位
    }

    va_end(arg_ptr);
    uartWriteBuf(LobotTxBuf, LobotTxBuf[2] + 2);       // 发送数据
}

/**
 * @brief 运行指定动作组
 * @details 生成运行指定动作组的指令并发送，动作组可循环执行指定次数。
 * @param numOfAction 动作组序号
 * @param times 执行次数 (times = 0 时无限循环)
 */
void runActionGroup(uint8_t numOfAction, uint16_t times)
{
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;  // 填充帧头
    LobotTxBuf[2] = 5;                              // 数据长度
    LobotTxBuf[3] = CMD_ACTION_GROUP_RUN;           // 运行动作组命令
    LobotTxBuf[4] = numOfAction;                    // 动作组号
    LobotTxBuf[5] = (uint8_t)(times);               // 执行次数低八位
    LobotTxBuf[6] = (uint8_t)(times >> 8);          // 执行次数高八位

    uartWriteBuf(LobotTxBuf, 7);                    // 发送数据
}

/**
 * @brief 停止动作组运行
 * @details 生成停止动作组的指令并发送。
 */
void stopActionGroup(void)
{
    LobotTxBuf[0] = FRAME_HEADER;     // 填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;                // 数据长度
    LobotTxBuf[3] = CMD_ACTION_GROUP_STOP;   // 停止动作组命令

    uartWriteBuf(LobotTxBuf, 4);      // 发送数据
}

/**
 * @brief 设置指定动作组的运行速度
 * @details 生成设置动作组速度的指令并发送。
 * @param numOfAction 动作组序号
 * @param speed 目标速度
 */
void setActionGroupSpeed(uint8_t numOfAction, uint16_t speed)
{
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;   // 填充帧头
    LobotTxBuf[2] = 5;                              // 数据长度
    LobotTxBuf[3] = CMD_ACTION_GROUP_SPEED;         // 设置动作组速度命令
    LobotTxBuf[4] = numOfAction;                    // 动作组号
    LobotTxBuf[5] = (uint8_t)(speed);               // 速度低八位
    LobotTxBuf[6] = (uint8_t)(speed >> 8);          // 速度高八位

    uartWriteBuf(LobotTxBuf, 7);                    // 发送数据
}

/**
 * @brief 设置所有动作组的运行速度
 * @details 调用setActionGroupSpeed函数，设置所有动作组的速度（组号0xFF）。
 * @param speed 目标速度
 */
void setAllActionGroupSpeed(uint16_t speed)
{
    setActionGroupSpeed(0xFF, speed);  // 组号为0xFF时设置所有组的速度
}

/**
 * @brief 发送获取电池电压命令
 * @details 生成获取电池电压的指令并发送。
 */
void getBatteryVoltage(void)
{
    LobotTxBuf[0] = FRAME_HEADER;  // 填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;             // 数据长度
    LobotTxBuf[3] = CMD_GET_BATTERY_VOLTAGE;  // 获取电池电压命令

    uartWriteBuf(LobotTxBuf, 4);   // 发送数据
}

/**
 * @brief 处理串口接收数据
 * @details 根据接收到的命令处理数据，目前支持获取电池电压的处理。
 */
void receiveHandle(void)
{
    if (isUartRxCompleted) {
        isUartRxCompleted = false;
        switch (LobotRxBuf[3]) {
            case CMD_GET_BATTERY_VOLTAGE: // 获取电压
                batteryVolt = ((uint16_t)(LobotRxBuf[5]) << 8) | LobotRxBuf[4];
                break;
            default:
                break;
        }
    }
}

/**
 * @brief 通过串口发送数据
 * @details 使用UART6发送指定长度的数据。
 * @param buf 数据缓冲区
 * @param len 数据长度
 */
void uartWriteBuf(uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart6, buf, len, 1000); // 通过UART6发送数据
}
