#include "stm32f4xx.h"

#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    ((uint32_t)8000000) /* Kristal frekansı (Hz) */
#endif 

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000) /* Dahili osilatör frekansı (Hz) */
#endif

uint32_t SystemCoreClock = 168000000; // 168 MHz varsayılan saat
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

void SystemInit(void) {
    // FPU Yapılandırması (Coprocessor Access Control Register)
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* CP10 ve CP11 tam erişim yetkisi */
#endif

    // Reset clock registers to default state
    RCC->CR |= (uint32_t)0x00000001; // Enable HSI
    RCC->CFGR = 0x00000000;          // Reset CFGR
    RCC->CR &= (uint32_t)0xFEF6FFFF; // Reset HSEON, CSSON, PLLON
    RCC->PLLCFGR = 0x24003010;       // Reset PLLCFGR
    RCC->CR &= (uint32_t)0xFFFBFFFF; // Reset HSEBYP
    RCC->CIR = 0x00000000;           // Disable all interrupts in CIR

    // Vector Table Offset Register
#ifdef VECT_TAB_SRAM
    SCB->VTOR = SRAM_BASE | 0x00; /* Vector Table in Internal SRAM */
#else
    SCB->VTOR = FLASH_BASE | 0x00; /* Vector Table in Internal FLASH */
#endif
}

void SystemCoreClockUpdate(void) {
    uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
    
    tmp = RCC->CFGR & RCC_CFGR_SWS;
    
    switch (tmp) {
        case 0x00:  /* HSI used as system clock source */
            SystemCoreClock = HSI_VALUE;
            break;
        case 0x04:  /* HSE used as system clock source */
            SystemCoreClock = HSE_VALUE;
            break;
        case 0x08:  /* PLL used as system clock source */
            pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
            pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
            
            if (pllsource != 0) {
                /* HSE used as PLL clock source */
                pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
            } else {
                /* HSI used as PLL clock source */
                pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
            }
            
            pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
            SystemCoreClock = pllvco/pllp;
            break;
        default:
            SystemCoreClock = HSI_VALUE;
            break;
    }
    
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    SystemCoreClock >>= tmp;
}
