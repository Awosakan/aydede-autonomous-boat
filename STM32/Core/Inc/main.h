#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Donanım Pin Tanımları
#define EMERGENCY_STOP_PIN       GPIO_PIN_13
#define EMERGENCY_STOP_PORT      GPIOC

#define MOTOR_LEFT_PIN           GPIO_PIN_6
#define MOTOR_LEFT_PORT          GPIOA
#define MOTOR_RIGHT_PIN          GPIO_PIN_7
#define MOTOR_RIGHT_PORT         GPIOA

// Hata İşleme
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif // MAIN_H
