#include "main.h"
#include "stm32f4xx_it.h"
#include "protocol.h"

// Dışarıdan Alınan Çevre Birimleri
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern TIM_HandleTypeDef htim3;

// FreeRTOS zamanlayıcı fonksiyonları
extern void xPortSysTickHandler(void);
extern int xTaskGetSchedulerState(void);
#define taskSCHEDULER_NOT_STARTED 0

/******************************************************************************/
/*           Cortex-M4 İşlemci Hata ve Sistem Kesme Servis Rutinleri          */
/******************************************************************************/

void NMI_Handler(void) {
    while (1) {}
}

void HardFault_Handler(void) {
    // Motorları anında kilitli modda kapat
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
    while (1) {}
}

void MemManage_Handler(void) {
    while (1) {}
}

void BusFault_Handler(void) {
    while (1) {}
}

void UsageFault_Handler(void) {
    while (1) {}
}

void DebugMon_Handler(void) {
}

// Not: SVC_Handler ve PendSV_Handler rutinleri, FreeRTOS port.c 
// dosyası tarafından doğrudan tanımlanmaktadır (Assembly seviyesinde).
// Makefile veya IDE'de çakışma olmaması için burada tanımlanmamıştır.

void SysTick_Handler(void) {
    HAL_IncTick(); // HAL Zamanlayıcısını Güncelle (1 ms)
    
    // Eğer FreeRTOS zamanlayıcısı başladıysa FreeRTOS tick'ini de ilerlet
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

/******************************************************************************/
/*                 STM32F405 Çevre Birimi Kesme Servis Rutinleri              */
/******************************************************************************/

// USART1 RX DMA2 Stream 5 Kesmesi
void DMA2_Stream5_IRQHandler(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return;
    }
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

// USART1 Global Kesmesi (Telefon Haberleşmesi)
void USART1_IRQHandler(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return;
    }
    uint32_t srval = huart1.Instance->SR;
    // ORE (Overrun), FE (Framing), NE (Noise), PE (Parity) hataları kontrolü
    if (srval & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        // Hata bayraklarını temizlemek için SR ardından DR okunur
        volatile uint32_t dummy = huart1.Instance->DR;
        (void)dummy;
        
        // DMA alımını durdur, parser durumunu sıfırla ve circular DMA'yı yeniden başlat
        HAL_UART_DMAStop(&huart1);
        extern uint16_t usart1_rx_read_ptr;
        usart1_rx_read_ptr = 0;
        extern uint8_t usart1_rx_buf[];
        extern ProtocolParser_t serial_parser;
        protocol_parser_init(&serial_parser);
        
        HAL_UART_Receive_DMA(&huart1, usart1_rx_buf, USART1_RX_BUF_SIZE);
        huart1.ErrorCode = HAL_UART_ERROR_NONE;
        return; // Hata temizlendi, HAL Handler'a girmeye gerek yok
    }
    HAL_UART_IRQHandler(&huart1);
}

// USART2 Global Kesmesi (GPS Haberleşmesi)
void USART2_IRQHandler(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return;
    }
    uint32_t srval = huart2.Instance->SR;
    if (srval & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        volatile uint32_t dummy = huart2.Instance->DR;
        (void)dummy;
        
        // Kesmeli GPS alımını iptal et ve sıfırdan yeniden başlat
        HAL_UART_AbortReceive(&huart2);
        extern uint8_t gps_rx_byte;
        HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);
        huart2.ErrorCode = HAL_UART_ERROR_NONE;
        return;
    }
    HAL_UART_IRQHandler(&huart2);
}

// HAL UART Hata Callback Fonksiyonu (Yedekli Hata Kurtarma)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        volatile uint32_t dummy = huart->Instance->SR;
        dummy = huart->Instance->DR;
        (void)dummy;
        
        HAL_UART_DMAStop(huart);
        extern uint16_t usart1_rx_read_ptr;
        usart1_rx_read_ptr = 0;
        extern uint8_t usart1_rx_buf[];
        extern ProtocolParser_t serial_parser;
        protocol_parser_init(&serial_parser);
        
        HAL_UART_Receive_DMA(huart, usart1_rx_buf, USART1_RX_BUF_SIZE);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
    }
    else if (huart->Instance == USART2) {
        volatile uint32_t dummy = huart->Instance->SR;
        dummy = huart->Instance->DR;
        (void)dummy;
        
        HAL_UART_AbortReceive(huart);
        extern uint8_t gps_rx_byte;
        HAL_UART_Receive_IT(huart, &gps_rx_byte, 1);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
    }
}

// EXTI15_10 (PC13 Acil Durdurma Butonu)
void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_IRQHandler(EMERGENCY_STOP_PIN);
}
