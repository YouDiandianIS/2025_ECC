/**
 * @file bh1750.h
 * @brief BH1750光照传感器驱动头文件，定义了相关宏、外部变量和函数声明，用于初始化和操作BH1750传感器。
 * @author 郑中岳 (Zhongyue Zheng)
 * @date 2025-07-01
 * @version 1.0
 * @note 该头文件适用于STM32微控制器通过I2C接口与BH1750传感器通信的环境。
 *       确保I2C外设正确初始化并包含必要的依赖库。
 * @license MIT License
 */
#ifndef __BH1750_H_
#define __BH1750_H_

/* 头文件包含 */
#include "i2c.h"
#include "stdint.h"

/* 宏定义 */
#define BH1750_ADDRESS       (0x23 << 1)  // BH1750 I2C地址（左移1位以适配HAL库）
#define POWER_ON             0x01        // 电源开启命令
#define RESET                0x07        // 复位命令
#define CONT_H_RES_MODE      0x10        // 连续高分辨率模式命令

/* 函数声明 */
/**
 * @brief 初始化BH1750光照传感器
 * @details 配置传感器进入连续高分辨率模式。
 */
void GY30_Init(void);

/**
 * @brief 读取BH1750光照强度数据
 * @details 通过I2C接口读取光照强度值（单位：勒克斯）。
 * @return 光照强度值（勒克斯），若读取失败返回-1.0f。
 */
float Multiple_read_BH1750(void);

/**
 * @brief 向BH1750发送单字节命令
 * @details 通过I2C接口向BH1750发送指定的命令字节。
 * @param cmd 待发送的命令字节
 */
void Single_Write_BH1750(unsigned char cmd);

#endif /* __BH1750_H_ */
