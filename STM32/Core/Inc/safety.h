#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Emniyet Parametreleri
#define BATTERY_CRITICAL_VOLTAGE 10.5f  // Kritik voltaj eşiği (V)
#define BATTERY_SAG_DURATION_MS  3000   // Voltajın kritik eşik altında kalabileceği maks süre (ms)
#define WATCHDOG_TIMEOUT_MS      500    // Telefondan komut gelme zaman aşımı (ms)
#define STALL_DURATION_MS        4000   // Motor kilitlenme/yosun tespiti süresi (ms)
#define STALL_MIN_STEER_DIFF     0.2f   // Dönüş komutu eşiği (yosun tespiti için fark)
#define STALL_MAX_YAW_CHANGE     1.0f   // 4 saniyede beklenen minimum sapma açısı değişimi (derece)

// Emniyet Durum Yapısı
typedef struct {
    uint8_t system_mode;          // protocol.h altındaki MODE_IDLE, MODE_AUTO vb.
    float filtered_voltage;       // EMA filtresinden geçmiş batarya voltajı
    uint32_t low_voltage_timer;   // Düşük voltajda geçen süre sayacı (ms)
    uint32_t watchdog_timer;      // Telemetri/Komut zaman aşımı sayacı (ms)
    uint32_t stall_timer;         // Motor kilitlenme (stuck) sayacı (ms)
    float last_yaw_for_stall;     // Stall kontrolü için kaydedilen son yaw açısı
    uint8_t emergency_triggered;  // PC13 EXTI fiziksel kesme bayrağı
} SafetyStatus_t;

// Emniyet modülü fonksiyonları
void safety_init(float initial_voltage, float initial_yaw);
void safety_update(float raw_voltage, float current_yaw, float left_cmd, float right_cmd, uint32_t dt_ms);
uint8_t safety_is_ok(void);
void safety_trigger_emergency(void);
void safety_feed_watchdog(void);
uint8_t safety_get_mode(void);
void safety_set_mode(uint8_t mode);
SafetyStatus_t safety_get_status(void);

#ifdef __cplusplus
}
#endif

#endif // SAFETY_H
