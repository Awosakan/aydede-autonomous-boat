#ifndef SENSORS_H
#define SENSORS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// IMU Tanımları
#define MPU_ADDR 0x68 << 1
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_GYRO_XOUT_H 0x43

// GPS outlier filtresi limiti (m/s cinsinden izin verilen maks konum sıçraması hızı)
#define GPS_OUTLIER_SPEED_LIMIT 6.0f 

// Sensör Yapıları
typedef struct {
    double latitude;
    double longitude;
    float sog;          // Speed over ground (m/s)
    float cog;          // Course over ground (derece, 0-360)
    uint8_t gps_lock;
    uint32_t last_update_time;
    uint8_t has_first_fix;
} GPS_Data_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float roll_rate;    // dps (deg/sec)
    float pitch_rate;   // dps
    float yaw_rate;     // dps
} IMU_Data_t;

// Fonksiyon bildirimleri
uint8_t sensors_imu_init(I2C_HandleTypeDef *hi2c);
void I2C_RecoverBus(I2C_HandleTypeDef *hi2c);
void sensors_imu_update(I2C_HandleTypeDef *hi2c, float dt);
void sensors_gps_init(void);
void sensors_gps_feed(uint8_t data);
void sensors_gps_update_tick(uint32_t current_time_ms);
float sensors_battery_read(ADC_HandleTypeDef *hadc);

// Getter'lar
GPS_Data_t sensors_get_gps(void);
IMU_Data_t sensors_get_imu(void);
float sensors_get_yaw(void); // GPS COG Fallback dahil yönelim bilgisi

#ifdef __cplusplus
}
#endif

#endif // SENSORS_H
