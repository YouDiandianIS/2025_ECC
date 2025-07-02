/**
 * @file bh1750.c
 * @brief BH1750光照传感器驱动程序，用于初始化传感器和读取光照强度数据。
 * @author 郑中岳 (Zhongyue Zheng)
 * @date 2025-07-01
 * @version 1.0
 * @note 该代码已通过测试，适用于STM32微控制器通过I2C接口与BH1750传感器通信的环境。
 *       确保I2C外设正确初始化并连接BH1750传感器。
 * @license MIT License
 */

/* 头文件包含 */
#include "bh1750.h"

/**
 * @brief 初始化BH1750光照传感器
 * @details 通过I2C接口发送电源开启命令和连续高分辨率模式命令，初始化传感器。
 */
void GY30_Init(void)
{
    uint8_t cmd;
    HAL_Delay(10); // 等待10ms以确保传感器稳定

    cmd = POWER_ON; // 设置电源开启命令
    HAL_I2C_Master_Transmit(&hi2c2, BH1750_ADDRESS, &cmd, 1, HAL_MAX_DELAY); // 发送电源开启命令
    HAL_Delay(10); // 等待10ms

    cmd = CONT_H_RES_MODE; // 设置连续高分辨率模式命令
    HAL_I2C_Master_Transmit(&hi2c2, BH1750_ADDRESS, &cmd, 1, HAL_MAX_DELAY); // 发送模式命令
}

/**
 * @brief 读取BH1750光照强度数据
 * @details 通过I2C接口读取2字节原始数据，转换为光照强度值（单位：勒克斯）。
 * @return 光照强度值（勒克斯），若读取失败返回-1.0f。
 */
float Multiple_read_BH1750(void)
{
    uint8_t buf[2];
    if (HAL_I2C_Master_Receive(&hi2c2, BH1750_ADDRESS, buf, 2, HAL_MAX_DELAY) != HAL_OK)
        return -1.0f; // I2C读取失败，返回错误值

    uint16_t raw = (buf[0] << 8) | buf[1]; // 合并高低字节为16位原始值
    return raw / 1.2f; // 转换为光照强度（勒克斯）
}

/**
 * @brief 向BH1750发送单字节命令
 * @details 通过I2C接口向BH1750发送指定的命令字节。
 * @param cmd 待发送的命令字节
 */
void Single_Write_BH1750(unsigned char cmd)
{
    HAL_I2C_Master_Transmit(&hi2c2, BH1750_ADDRESS, &cmd, 1, HAL_MAX_DELAY); // 发送单字节命令
}
