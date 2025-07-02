/**
 * @file stm32.h
 * @brief STM32微控制器通用头文件，定义了舵机控制、传感器数据处理和串口通信相关的宏、结构体和函数声明。
 * @author 郑中岳 (Zhongyue Zheng)
 * @date 2025-07-01
 * @version 1.0
 * @note 该头文件适用于STM32微控制器环境，需配合相关驱动文件（如bh1750.c、sht30.c等）使用。
 *       确保外设和传感器正确初始化。
 * @license MIT License
 */

#ifndef __STM32_H__
#define __STM32_H__

/* 头文件包含 */
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "adc.h"
#include "tim.h"
#include "bh1750.h"
#include "sht30.h"
#include "ds18b20.h"
#include <stdarg.h>
#include <stdbool.h>

/* 宏定义 */
#define len2                        100      // 串口接收缓冲区长度
#define FRAME_HEADER                0x55     // 通信协议帧头
#define CMD_SERVO_MOVE              0x03     // 舵机移动命令
#define CMD_ACTION_GROUP_RUN        0x04     // 运行动作组命令
#define CMD_ACTION_GROUP_STOP       0x05     // 停止动作组命令
#define CMD_ACTION_GROUP_SPEED      0x06     // 设置动作组速度命令
#define CMD_GET_BATTERY_VOLTAGE     0x0F     // 获取电池电压命令

/* 结构体定义 */
/**
 * @brief 舵机控制结构体
 * @details 包含舵机ID和目标位置，用于多舵机控制。
 */
typedef struct {
    uint8_t id;         // 舵机ID (0 <= id <= 31)
    uint16_t position;  // 目标位置 (0-1000，对应0-180度)
} LobotServo;

/* 函数声明 */
/**
 * @brief 初始化所有外设和传感器
 * @details 初始化GY30光照传感器、SHT30温湿度传感器、DS18B20温度传感器，启用UART2中断，启动定时器3，并初始化舵机位置。
 */
void All_Init(void);

/**
 * @brief 读取ADC值
 * @details 启动ADC转换，轮询获取结果并返回转换值。
 * @return ADC转换值
 */
uint32_t ADC_Read(void);

/**
 * @brief 控制单个舵机转动
 * @details 根据舵机ID、目标位置和转动时间生成控制指令并通过串口发送。
 * @param servoID 舵机ID (0 <= servoID <= 31)
 * @param position 目标位置
 * @param time 转动时间 (time > 0)
 */
void moveServo(uint8_t servoID, uint16_t position, uint16_t time);

/**
 * @brief 控制多个舵机转动（通过数组）
 * @details 根据舵机结构体数组、舵机数量和转动时间生成控制指令并发送。
 * @param servos 舵机结构体数组
 * @param num 舵机个数 (0 < num <= 32)
 * @param time 转动时间 (time > 0)
 */
void moveServosByArray(LobotServo servos[], uint8_t num, uint16_t time);

/**
 * @brief 控制多个舵机转动（可变参数）
 * @details 根据舵机数量、转动时间和可变参数（舵机ID和目标位置交替排列）生成控制指令并发送。
 * @param num 舵机个数 (0 < num <= 32)
 * @param time 转动时间 (time > 0)
 * @param ... 舵机ID和目标位置交替排列
 */
void moveServos(uint8_t num, uint16_t time, ...);

/**
 * @brief 运行指定动作组
 * @details 生成运行指定动作组的指令并发送，动作组可循环执行指定次数。
 * @param numOfAction 动作组序号
 * @param times 执行次数 (times = 0 时无限循环)
 */
void runActionGroup(uint8_t numOfAction, uint16_t times);

/**
 * @brief 停止动作组运行
 * @details 生成停止动作组的指令并发送。
 */
void stopActionGroup(void);

/**
 * @brief 设置指定动作组的运行速度
 * @details 生成设置动作组速度的指令并发送。
 * @param numOfAction 动作组序号
 * @param speed 目标速度
 */
void setActionGroupSpeed(uint8_t numOfAction, uint16_t speed);

/**
 * @brief 设置所有动作组的运行速度
 * @details 调用setActionGroupSpeed函数，设置所有动作组的速度（组号0xFF）。
 * @param speed 目标速度
 */
void setAllActionGroupSpeed(uint16_t speed);

/**
 * @brief 发送获取电池电压命令
 * @details 生成获取电池电压的指令并发送。
 */
void getBatteryVoltage(void);

/**
 * @brief 处理串口接收数据
 * @details 根据接收到的命令处理数据，目前支持获取电池电压的处理。
 */
void receiveHandle(void);

/**
 * @brief 通过串口发送数据
 * @details 使用UART6发送指定长度的数据。
 * @param buf 数据缓冲区
 * @param len 数据长度
 */
void uartWriteBuf(uint8_t *buf, uint16_t len);

#endif /* __STM32_H__ */
