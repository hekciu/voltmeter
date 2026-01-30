#include "system_clock.h"

#include "cmsis.h"


/* Defined inside system.c */
extern uint32_t SystemCoreClock;
extern void SystemCoreClockUpdate(void);

/*
    We configure HSE through PLL as system clock input to get 48MHz (minimum for USB)
*/
void system_clock_configure(void) {
    // enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0);

    // set flash latency <- needed if we want 72MHz clock
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    // disable PLL
    RCC->CR &= ~RCC_CR_PLLON;
    while(RCC->CR & RCC_CR_PLLRDY);

    // configure PLL
    RCC->CFGR &= ~RCC_CFGR_PLLMULL_Msk;
    RCC->CFGR |= RCC_CFGR_PLLMULL9;
    RCC->CFGR |= RCC_CFGR_PLLSRC;

    // enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0);

    // set PLL as system clock source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);

    // configure AHB and APB prescalers <- probably needed if we want 72MHz
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

    // update clock variable
    SystemCoreClockUpdate();
}
