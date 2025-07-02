#include "ds18b20.h"

// 硬件配置
#define DS18B20_PORT    GPIOB
#define DS18B20_PIN     GPIO_PIN_1

// 引脚操作宏
#define DS18B20_OUT_LOW()   HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET)
#define DS18B20_OUT_HIGH()  HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET)
#define DS18B20_IN()        HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)

// 私有函数声明
static void DS18B20_IO_Input(void);
static void DS18B20_IO_Output(void);
static uint8_t DS18B20_Reset(void);

/**
  * @brief  精确微秒延时（100MHz时钟优化）
  * @param  us: 微秒数 (0~65535)
  */
void DS18B20_Delay(uint32_t us)
{
    uint32_t ticks = us * (100 / 8);  // 100MHz时钟计算
    do {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    } while(--ticks);
}

/**
  * @brief  GPIO设置为输入模式（带上拉）
  */
static void DS18B20_IO_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
}

/**
  * @brief  GPIO设置为输出模式（推挽）
  */
static void DS18B20_IO_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
}

/**
  * @brief  发送复位脉冲并检测存在信号
  * @retval 0: 成功 1: 无响应
  */
static uint8_t DS18B20_Reset(void)
{
    DS18B20_IO_Output();
    
    // 拉低总线480us
    DS18B20_OUT_LOW();
    DS18B20_Delay(480);
    
    // 释放总线
    DS18B20_OUT_HIGH();
    DS18B20_IO_Input();
    DS18B20_Delay(70);  // 等待15-60us
    
    // 检测存在脉冲
    if(DS18B20_IN() != GPIO_PIN_RESET) return 1;
    
    // 等待存在脉冲结束（60-240us）
    uint32_t timeout = 240;
    while(DS18B20_IN() == GPIO_PIN_RESET && timeout--) 
        DS18B20_Delay(1);
    
    return (timeout > 0) ? 0 : 1;
}

/**
  * @brief  写入一个字节
  * @param  data: 要写入的数据
  */
void DS18B20_WriteByte(uint8_t data)
{
    DS18B20_IO_Output();
    for(uint8_t i=0; i<8; i++){
        DS18B20_OUT_LOW();
        DS18B20_Delay(2);  // 至少1us
        
        if(data & 0x01) DS18B20_OUT_HIGH();
        else DS18B20_OUT_LOW();
        
        DS18B20_Delay(60);  // 保持60us
        DS18B20_OUT_HIGH();
        data >>= 1;
        DS18B20_Delay(2);
    }
}

/**
  * @brief  读取一个字节
  * @retval 读取到的数据
  */
uint8_t DS18B20_ReadByte(void)
{
    uint8_t data = 0;
    for(uint8_t i=0; i<8; i++){
        data >>= 1;
        DS18B20_IO_Output();
        DS18B20_OUT_LOW();
        DS18B20_Delay(2);  // 至少1us
        
        DS18B20_IO_Input();
        DS18B20_Delay(12);  // 等待15us采样
        
        if(DS18B20_IN() == GPIO_PIN_SET) data |= 0x80;
        
        DS18B20_Delay(50);  // 完成整个时隙
    }
    return data;
}

/**
  * @brief  启动温度转换
  */
void DS18B20_StartConversion(void)
{
    DS18B20_Reset();
    DS18B20_WriteByte(0xCC);  // 跳过ROM
    DS18B20_WriteByte(0x44);  // 启动转换
}

/**
  * @brief  读取温度值（阻塞式等待转换完成）
  * @retval 温度值（带两位小数，单位：℃）
  */
float DS18B20_ReadTemperature(void)
{
    // 启动转换并等待完成
    DS18B20_StartConversion();
    DS18B20_Delay(750000);    // 750ms等待转换完成
    
    // 读取暂存器
    DS18B20_Reset();
    DS18B20_WriteByte(0xCC);  // 跳过ROM
    DS18B20_WriteByte(0xBE);  // 读暂存器
    
    // 读取温度数据
    uint8_t lsb = DS18B20_ReadByte();
    uint8_t msb = DS18B20_ReadByte();
    
    // 转换为实际温度
    int16_t temp = (msb << 8) | lsb;
    return temp * 0.0625f;
}

/**
  * @brief  硬件初始化
  * @retval 0: 成功 1: 设备未响应
  */
uint8_t DS18B20_Init(void)
{
    // 确保GPIO时钟已使能
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // 初始化为高电平
    DS18B20_IO_Output();
    DS18B20_OUT_HIGH();
    
    // 发送复位脉冲
    return DS18B20_Reset();
}
