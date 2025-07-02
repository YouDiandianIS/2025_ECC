/**
 * @file sht30.c
 * @brief SHT30温湿度传感器驱动程序，用于初始化传感器、复位、读取温湿度数据，并进行CRC校验。
 * @author 郑中岳 (Zhongyue Zheng)
 * @date 2025-07-01
 * @version 1.0
 * @note 该代码已通过测试，适用于STM32微控制器通过I2C接口与SHT30传感器通信的环境。
 *       确保I2C外设正确初始化并连接SHT30传感器。
 * @license MIT License
 */

/* 头文件包含 */
#include "sht30.h"

/* 宏定义和常量 */
const int16_t SHT30_POLYNOMIAL = 0x131; // CRC校验多项式
#define SHT30_ADDR      (0x44)          // SHT30 I2C地址
#define SHT30_ADDR_W    (SHT30_ADDR << 1) // SHT30写地址（左移1位适配HAL库）
#define SHT30_ADDR_R    (SHT30_ADDR << 1) // SHT30读地址（与写地址相同，HAL库自动处理读写位）

/**
 * @brief 复位SHT30传感器
 * @details 通过I2C接口发送复位命令（0x30A2），并等待50ms以确保复位完成。
 */
void SHT30_reset(void)
{
    unsigned char cmd[2] = {0x30, 0xA2}; // 复位命令
    if (HAL_I2C_Master_Transmit(&hi2c1, SHT30_ADDR_W, cmd, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        // 错误处理（留空，留给用户扩展）
    }
    HAL_Delay(50); // 等待50ms以完成复位
}

/**
 * @brief 初始化SHT30传感器
 * @details 复位传感器并发送初始化命令（0x2236），配置为高精度周期测量模式。
 */
void Init_SHT30(void)
{
    unsigned char cmd[2] = {0x22, 0x36}; // 初始化命令（高精度周期测量）
    SHT30_reset(); // 执行复位
    if (HAL_I2C_Master_Transmit(&hi2c1, SHT30_ADDR_W, cmd, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        printf("Init_SHT30_error: init fail\r\n"); // 初始化失败，打印错误信息
    }
    HAL_Delay(200); // 等待200ms以确保初始化完成
}

/**
 * @brief 读取SHT30温湿度数据
 * @details 发送读取命令（0xE000），接收6字节数据（温度+CRC、湿度+CRC），
 *          进行CRC校验后计算温度和湿度值。
 * @param Humidity 指向存储湿度值的指针（单位：%RH）
 * @param Temperature 指向存储温度值的指针（单位：摄氏度）
 */
void SHT30_ReadData(float *Humidity, float *Temperature)
{
    unsigned char cmd[2] = {0xE0, 0x00}; // 读取命令
    unsigned char rx_buffer[6]; // 接收缓冲区（温度2字节+CRC1字节+湿度2字节+CRC1字节）
    unsigned short int dat;

    if (HAL_I2C_Master_Transmit(&hi2c1, SHT30_ADDR_W, cmd, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        printf("SHT30_ReadData error: transmit fail\r\n"); // 发送命令失败，打印错误信息
        return;
    }
    HAL_Delay(30); // 等待30ms以确保数据准备好

    if (HAL_I2C_Master_Receive(&hi2c1, SHT30_ADDR_R, rx_buffer, 6, HAL_MAX_DELAY) != HAL_OK)
    {
        return; // 接收数据失败，直接返回
    }

    // 校验温度数据
    if (SHT30_CheckCrc((char *)rx_buffer, 2, rx_buffer[2]) == 0)
    {
        dat = ((unsigned short int)rx_buffer[0] << 8) | rx_buffer[1]; // 合并温度高低字节
        *Temperature = SHT30_CalcTemperatureC(dat); // 计算温度值
    }
    else
    {
        printf("SHT30_ReadData error: temperature CRC fail\r\n"); // 温度CRC校验失败
    }

    // 校验湿度数据
    if (SHT30_CheckCrc((char *)(rx_buffer + 3), 2, rx_buffer[5]) == 0)
    {
        dat = ((unsigned short int)rx_buffer[3] << 8) | rx_buffer[4]; // 合并湿度高低字节
        *Humidity = SHT30_CalcRH(dat); // 计算湿度值
    }
    else
    {
        printf("SHT30_ReadData error: humidity CRC fail\r\n"); // 湿度CRC校验失败
    }
}

/**
 * @brief 计算SHT30数据的CRC校验值
 * @details 使用多项式0x131对指定数据进行CRC-8校验，比较计算结果与接收到的校验值。
 * @param data 数据数组指针
 * @param nbrOfBytes 数据字节数
 * @param checksum 接收到的校验值
 * @return 0表示校验成功，1表示校验失败
 */
unsigned char SHT30_CheckCrc(char data[], char nbrOfBytes, char checksum)
{
    char crc = 0xFF; // 初始CRC值
    char bit;
    char byteCtr;

    for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
    {
        crc ^= data[byteCtr]; // 对每个字节进行异或
        for (bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ SHT30_POLYNOMIAL; // 如果最高位为1，进行多项式除法
            else
                crc = (crc << 1); // 否则左移
        }
    }
    return (crc != checksum) ? 1 : 0; // 比较计算的CRC与接收的校验值
}

/**
 * @brief 计算温度值（摄氏度）
 * @details 根据SHT30的公式将16位原始数据转换为温度值（单位：摄氏度）。
 * @param u16sT 16位温度原始数据
 * @return 温度值（摄氏度）
 */
float SHT30_CalcTemperatureC(unsigned short u16sT)
{
    u16sT &= ~0x0003; // 清除最低2位（状态位）
    return (175 * (float)u16sT / 65535 - 45); // 转换为摄氏度
}

/**
 * @brief 计算相对湿度值
 * @details 根据SHT30的公式将16位原始数据转换为相对湿度值（单位：%RH）。
 * @param u16sRH 16位湿度原始数据
 * @return 湿度值（%RH）
 */
float SHT30_CalcRH(unsigned short u16sRH)
{
    u16sRH &= ~0x0003; // 清除最低2位（状态位）
    return (100 * (float)u16sRH / 65535); // 转换为相对湿度
}
