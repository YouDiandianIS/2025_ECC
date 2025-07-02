/**
 * @file sht30.h
 * @brief SHT30温湿度传感器驱动头文件，定义了相关类型、宏和函数声明，用于初始化和操作SHT30传感器。
 * @author 郑中岳 (Zhongyue Zheng)
 * @date 2025-07-01
 * @version 1.0
 * @note 该头文件适用于STM32F4xx微控制器通过I2C接口与SHT30传感器通信的环境。
 *       确保I2C外设正确初始化并包含必要的STM32 HAL库。
 * @license MIT License
 */

#ifndef __SHT30_H
#define __SHT30_H

/* 头文件包含 */
#include "main.h"
#include "i2c.h"
#include "stdio.h"
#include <stdint.h>

/* 宏定义 */
#define SHT30_ADDR         (0x44)          // SHT30 I2C地址
#define SHT30_ADDR_W       (SHT30_ADDR << 1) // SHT30写地址（左移1位适配HAL库）
#define SHT30_ADDR_R       (SHT30_ADDR << 1) // SHT30读地址（与写地址相同，HAL库自动处理读写位）

/* 函数声明 */
/**
 * @brief 复位SHT30传感器
 * @details 通过I2C接口发送复位命令，重置传感器状态。
 */
void SHT30_reset(void);

/**
 * @brief 初始化SHT30传感器
 * @details 配置传感器为高精度周期测量模式。
 */
void Init_SHT30(void);

/**
 * @brief 读取SHT30温湿度数据
 * @details 通过I2C接口读取温度和湿度数据，并进行CRC校验。
 * @param Humidity 指向存储湿度值的指针（单位：%RH）
 * @param Temperature 指向存储温度值的指针（单位：摄氏度）
 */
void SHT30_ReadData(float *Humidity, float *Temperature);

/**
 * @brief 计算SHT30数据的CRC校验值
 * @details 使用多项式对指定数据进行CRC-8校验，比较计算结果与接收到的校验值。
 * @param data 数据数组指针
 * @param nbrOfBytes 数据字节数
 * @param checksum 接收到的校验值
 * @return 0表示校验成功，1表示校验失败
 */
unsigned char SHT30_CheckCrc(char data[], char nbrOfBytes, char checksum);

/**
 * @brief 计算温度值（摄氏度）
 * @details 根据SHT30的公式将16位原始数据转换为温度值。
 * @param u16sT 16位温度原始数据
 * @return 温度值（摄氏度）
 */
float SHT30_CalcTemperatureC(unsigned short u16sT);

/**
 * @brief 计算相对湿度值
 * @details 根据SHT30的公式将16位原始数据转换为相对湿度值。
 * @param u16sRH 16位湿度原始数据
 * @return 湿度值（%RH）
 */
float SHT30_CalcRH(unsigned short u16sRH);

#endif /* __SHT30_H */
