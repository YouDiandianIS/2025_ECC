#ifndef __DS18B20_H
#define __DS18B20_H

#include "main.h"
#include <stdint.h>

uint8_t DS18B20_Init(void);
float DS18B20_ReadTemperature(void);

#endif
