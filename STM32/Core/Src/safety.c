#include "safety.h"
#include "protocol.h"
#include <math.h>

static volatile SafetyStatus_t safety_state;
static volatile uint32_t telemetry_wd_timer = 0;
static volatile uint32_t navigation_wd_timer = 0;
static volatile uint32_t safety_wd_timer = 0;

// EMA filtresi katsayısı (0.05f yavaş/pürüzsüz tepki sağlar, ani dalgalanmaları önler)
#define EMA_ALPHA 0.05f

void safety_init(float initial_voltage, float initial_yaw) {
    safety_state.system_mode = MODE_IDLE;
    safety_state.filtered_voltage = initial_voltage;
    safety_state.low_voltage_timer = 0;
    safety_state.watchdog_timer = 0;
    safety_state.stall_timer = 0;
    safety_state.last_yaw_for_stall = initial_yaw;
    safety_state.emergency_triggered = 0;
    telemetry_wd_timer = 0;
    navigation_wd_timer = 0;
    safety_wd_timer = 0;
}

void safety_update(float raw_voltage, float current_yaw, float left_cmd, float right_cmd, uint32_t dt_ms) {
    // 1. Acil Durdurma Kesmesi kontrolü (Latching - kilitli koruma)
    if (safety_state.emergency_triggered) {
        safety_state.system_mode = MODE_EMERGENCY;
        return;
    }

    // 2. Batarya Voltaj Filtresi (EMA)
    safety_state.filtered_voltage = (EMA_ALPHA * raw_voltage) + ((1.0f - EMA_ALPHA) * safety_state.filtered_voltage);

    // 3. Batarya Voltaj Sag Koruması
    if (safety_state.filtered_voltage < BATTERY_CRITICAL_VOLTAGE) {
        safety_state.low_voltage_timer += dt_ms;
        if (safety_state.low_voltage_timer >= BATTERY_SAG_DURATION_MS) {
            safety_state.system_mode = MODE_FAILSAFE;
        }
    } else {
        safety_state.low_voltage_timer = 0;
    }

    // 4. Haberleşme Watchdog Sayacı
    if (safety_state.system_mode == MODE_AUTO || safety_state.system_mode == MODE_MANUAL) {
        safety_state.watchdog_timer += dt_ms;
        if (safety_state.watchdog_timer >= WATCHDOG_TIMEOUT_MS) {
            safety_state.system_mode = MODE_FAILSAFE;
        }
    } else {
        safety_state.watchdog_timer = 0;
    }

    // 5. Yosun/Motor Stall Koruması
    // Eğer otonom veya manuel modda isek ve dümen komutu motorları döndürmek için fark yaratıyorsa:
    if ((safety_state.system_mode == MODE_AUTO || safety_state.system_mode == MODE_MANUAL) &&
        (fabsf(left_cmd - right_cmd) > STALL_MIN_STEER_DIFF)) {
        
        // Mevcut yaw açısı ile son stall kontrol açısı arasındaki fark
        float yaw_diff = current_yaw - safety_state.last_yaw_for_stall;
        while (yaw_diff > 180.0f)  yaw_diff -= 360.0f;
        while (yaw_diff < -180.0f) yaw_diff += 360.0f;
        yaw_diff = fabsf(yaw_diff);

        if (yaw_diff >= STALL_MAX_YAW_CHANGE) {
            // İDA döndüğü için stall durumunda değil, timer'ı sıfırla ve referans açıyı güncelle
            safety_state.stall_timer = 0;
            safety_state.last_yaw_for_stall = current_yaw;
        } else {
            // İDA dönmeye çalışıyor ama açı değişmiyor, yosun sarma ihtimali var
            safety_state.stall_timer += dt_ms;
            if (safety_state.stall_timer >= STALL_DURATION_MS) {
                safety_state.system_mode = MODE_FAILSAFE;
            }
        }
    } else {
        // Dümen döndürme çabası yoksa stall kontrolünü devre dışı tut
        safety_state.stall_timer = 0;
        safety_state.last_yaw_for_stall = current_yaw;
    }
}

uint8_t safety_is_ok(void) {
    if (safety_state.system_mode == MODE_FAILSAFE || 
        safety_state.system_mode == MODE_EMERGENCY) {
        return 0; // Güvenli değil
    }
    return 1; // Güvenli
}

void safety_trigger_emergency(void) {
    // ISR bağlamından çağrılabilir (EXTI callback) — atomik yazma yeterli (uint8_t)
    safety_state.emergency_triggered = 1;
    safety_state.system_mode = MODE_EMERGENCY;
}

void safety_feed_watchdog(void) {
    safety_state.watchdog_timer = 0;
}

void safety_task_feed(uint8_t task_bit) {
    if (task_bit & TASK_WD_TELEMETRY)  telemetry_wd_timer = 0;
    if (task_bit & TASK_WD_NAVIGATION) navigation_wd_timer = 0;
    if (task_bit & TASK_WD_SAFETY)     safety_wd_timer = 0;
}

uint8_t safety_check_task_watchdogs(uint32_t dt_ms) {
    telemetry_wd_timer += dt_ms;
    navigation_wd_timer += dt_ms;
    safety_wd_timer += dt_ms;

    // Görev zaman aşımı limiti: 1500ms (Görev 87)
    if (telemetry_wd_timer > 1500 || navigation_wd_timer > 1500 || safety_wd_timer > 1500) {
        return 0; // Görev freeze algılandı
    }
    return 1; // Tüm görevler sağlıklı
}

uint8_t safety_get_mode(void) {
    return safety_state.system_mode;
}

void safety_set_mode(uint8_t mode) {
    if (safety_state.system_mode != MODE_EMERGENCY) {
        safety_state.system_mode = mode;
    }
}

SafetyStatus_t safety_get_status(void) {
    return safety_state;
}
