#ifndef CMSIS_H
#define CMSIS_H

#define STM32F103xB

#include "stm32f1xx.h"

#define BIT(n) (1UL << (n))
#define FLAG_SET(reg, bit) (((reg) & (bit)) == 0UL)

#endif

