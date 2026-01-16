#include <stdint.h>

#define STM32F103xB

#include "cmsis/cmsis_f1/Source/Templates/system_stm32f1xx.c"

/* Workaround */
void __libc_init_array(void) {};

