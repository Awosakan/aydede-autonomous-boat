#include "sensors.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Sensör Durumları
static volatile GPS_Data_t gps_data;
static volatile IMU_Data_t imu_data;
static float battery_voltage = 12.0f;

// GPS NMEA Parser Tamponu
static char nmea_buf[128];
static uint8_t nmea_idx = 0;
static volatile char nmea_process_buf[128];
static volatile uint8_t nmea_sentence_ready = 0;

static void sensors_delay(uint32_t ms) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        HAL_Delay(ms);
    }
}

// Complementary Filtre Katsayıları
#define COMP_FILTER_ALPHA 0.98f   // Roll/Pitch için jiroskop ağırlığı
#define GPS_COG_ALIGN_BETA 0.995f  // Hareket halindeyken yaw'ı GPS COG'a kaydıran katsayı

// ADC Batarya Gerilim Bölücü Oranı (Örn: 10K / 1K direnç bölücü -> (10+1)/1 = 11)
#define VOLTAGE_DIVIDER_RATIO 11.0f
#define ADC_REF_VOLTAGE 3.3f

// IMU Sınırlayıcı ve Çözünürlükleri
// FS_SEL = 0 (Gyro: +-250 dps -> 131 LSB/dps)
// AFS_SEL = 0 (Accel: +-2g -> 16384 LSB/g)
#define GYRO_SCALE 131.0f
#define ACCEL_SCALE 16384.0f

uint8_t sensors_imu_init(I2C_HandleTypeDef *hi2c) {
    uint8_t temp = 0;
    
    // MPU6050/9250 Resetleme
    temp = 0x80;
    if (HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100) != HAL_OK) {
        return 0; // I2C Hatası
    }
    sensors_delay(100);
    
    // Uyku modundan çıkarma, Dahili 8MHz osilatör seçimi
    temp = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100) != HAL_OK) {
        return 0;
    }
    sensors_delay(100);
    
    // Gyro Config: +-250 dps (0x00)
    temp = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);
    
    // Accel Config: +-2g (0x00)
    temp = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);
    
    // DLPF Config: 42Hz Low Pass Filter (0x03)
    temp = 0x03;
    HAL_I2C_Mem_Write(hi2c, MPU_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);
    
    // Değişkenleri sıfırla
    imu_data.roll = 0.0f;
    imu_data.pitch = 0.0f;
    imu_data.yaw = 0.0f;
    imu_data.roll_rate = 0.0f;
    imu_data.pitch_rate = 0.0f;
    imu_data.yaw_rate = 0.0f;
    
    return 1; // Başarılı
}

void I2C_RecoverBus(I2C_HandleTypeDef *hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 1. I2C Birimini De-ilklendir
    HAL_I2C_DeInit(hi2c);
    
    // GPIOB pin saatini etkinleştir (PB6/PB7 GPIOB üzerindedir)
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // 2. SCL ve SDA pinlerini GPIO Open-Drain Çıkış olarak yapılandır
    // İlk olarak pinleri HIGH (serbest bırakılmış) seviyeye çekelim
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Kısa bir bekleme
    sensors_delay(1); 
    
    // 3. SDA hattı LOW (kilitli) ise SCL hattına 9 clock darbesi gönder
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) {
        for (int i = 0; i < 9; i++) {
            // SCL LOW
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            sensors_delay(1); 
            
            // SCL HIGH
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            sensors_delay(1); 
            
            // Eğer SDA HIGH olduysa köle hattı serbest bırakmıştır
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) {
                break;
            }
        }
    }
    
    // 4. Manuel STOP Koşulu Üret
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // SDA LOW
    sensors_delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // SCL HIGH
    sensors_delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   // SDA HIGH (STOP)
    sensors_delay(1);
    
    // 5. Pinleri I2C AF4 (Alternatif Fonksiyon) moduna geri getir
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 6. I2C donanımını sıfırla ve yeniden ilklendir
    __HAL_RCC_I2C1_FORCE_RESET();
    sensors_delay(1);
    __HAL_RCC_I2C1_RELEASE_RESET();
    sensors_delay(1);
    
    HAL_I2C_Init(hi2c);
}

void sensors_imu_update(I2C_HandleTypeDef *hi2c, float dt) {
    uint8_t data[14];
    static uint32_t consecutive_failures = 0;
    
    if (HAL_I2C_Mem_Read(hi2c, MPU_ADDR, MPU_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 14, 50) != HAL_OK) {
        consecutive_failures++;
        // Üst üste 3 kez okuma hatası alınırsa I2C hattını ve IMU'yu sıfırla (Görev 4.2)
        if (consecutive_failures >= 3) {
            I2C_RecoverBus(hi2c);
            sensors_imu_init(hi2c);
            consecutive_failures = 0;
        }
        return; // Okuma başarısız ise filtreyi çalıştırma
    }
    consecutive_failures = 0;
    
    // Ham verileri 16-bit işaretli tamsayıya dönüştür
    int16_t raw_ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_az = (int16_t)((data[4] << 8) | data[5]);
    
    int16_t raw_gx = (int16_t)((data[8] << 8) | data[9]);
    int16_t raw_gy = (int16_t)((data[10] << 8) | data[11]);
    int16_t raw_gz = (int16_t)((data[12] << 8) | data[13]);
    
    // Ölçeklendir
    float ax = (float)raw_ax / ACCEL_SCALE;
    float ay = (float)raw_ay / ACCEL_SCALE;
    float az = (float)raw_az / ACCEL_SCALE;
    
    // Jiroskop oranları derece/saniye (deg/sec)
    imu_data.roll_rate = (float)raw_gx / GYRO_SCALE;
    imu_data.pitch_rate = (float)raw_gy / GYRO_SCALE;
    imu_data.yaw_rate = (float)raw_gz / GYRO_SCALE;
    
    // İvmeölçer yardımıyla roll ve pitch açılarını hesapla
    float roll_acc = atan2f(ay, az) * 180.0f / M_PI;
    float pitch_acc = -atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    
    // Complementary Filter (Bütünleyici Filtre) ile Roll/Pitch süzme
    imu_data.roll = COMP_FILTER_ALPHA * (imu_data.roll + imu_data.roll_rate * dt) + (1.0f - COMP_FILTER_ALPHA) * roll_acc;
    imu_data.pitch = COMP_FILTER_ALPHA * (imu_data.pitch + imu_data.pitch_rate * dt) + (1.0f - COMP_FILTER_ALPHA) * pitch_acc;
    
    // Yaw açısını entegre et
    imu_data.yaw += imu_data.yaw_rate * dt;
    
    // Yaw açısını 0-360 dereceye sarmala
    while (imu_data.yaw >= 360.0f) imu_data.yaw -= 360.0f;
    while (imu_data.yaw < 0.0f)   imu_data.yaw += 360.0f;
    
    // GPS Rota Açısı (COG) Fallback:
    // Eğer araç 0.6 m/s üzerinde hareket ediyorsa, manyetik bozunmalar veya jiroskop sapması (drift) 
    // yerine GPS COG referans alınarak yaw açısı hizalanır (Kötü Senaryo 2 koruması).
    // Ancak yan yan sürüklenmelerde (crab walk) ve sert dönüşlerde COG ile Heading farklılaşacağından,
    // yanal ivme (ay) ve dönüş hızı (yaw_rate) için eşik filtresi uyguluyoruz (Görev 4.3).
    // Ucuz GPS gürültüsünü engellemek için hız eşiği 1.2 m/s'ye yükseltildi.
    if (gps_data.gps_lock && gps_data.sog > 1.2f && fabsf(ay) < 0.12f && fabsf(imu_data.yaw_rate) < 15.0f) {
        float cog = gps_data.cog;
        float diff = cog - imu_data.yaw;
        while (diff > 180.0f)  diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;
        
        // Yavaşça COG değerine çek (Yön ani sıçramasın)
        imu_data.yaw += (1.0f - GPS_COG_ALIGN_BETA) * diff;
        
        // Tekrar sarmalama
        while (imu_data.yaw >= 360.0f) imu_data.yaw -= 360.0f;
        while (imu_data.yaw < 0.0f)   imu_data.yaw += 360.0f;
    }
}

void sensors_gps_init(void) {
    memset(&gps_data, 0, sizeof(GPS_Data_t));
    gps_data.gps_lock = 0;
    gps_data.has_first_fix = 0;
    nmea_idx = 0;
}

// NMEA verisinden belirli bir alanı çeken yardımcı fonksiyon
static void get_nmea_field(const char *sentence, int field_idx, char *out_val, int max_len) {
    int cur_field = 0;
    int src_idx = 0;
    int dst_idx = 0;
    
    out_val[0] = '\0';
    
    while (sentence[src_idx] != '\0' && sentence[src_idx] != '*') {
        if (sentence[src_idx] == ',') {
            cur_field++;
            if (cur_field > field_idx) {
                break;
            }
        } else if (cur_field == field_idx) {
            if (dst_idx < max_len - 1) {
                out_val[dst_idx++] = sentence[src_idx];
            }
        }
        src_idx++;
    }
    out_val[dst_idx] = '\0';
}

// NMEA Derece.Dakika biçimini Ondalık Dereceye çeviren yardımcı fonksiyon
static double nmea_to_decimal(const char *nmea_str, char direction) {
    if (nmea_str[0] == '\0') return 0.0;
    
    double raw = atof(nmea_str);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);
    
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

// NMEA Checksum Doğrulaması
static uint8_t verify_nmea_checksum(const char *sentence) {
    // Cümlenin başındaki '$' işaretini geç, '*' işaretine kadar olan karakterlerin XOR toplamını bul
    if (sentence[0] != '$') return 0;
    
    int i = 1;
    uint8_t xor_sum = 0;
    while (sentence[i] != '\0' && sentence[i] != '*') {
        xor_sum ^= (uint8_t)sentence[i];
        i++;
    }
    
    if (sentence[i] == '*') {
        char hex_str[3];
        hex_str[0] = sentence[i+1];
        hex_str[1] = sentence[i+2];
        hex_str[2] = '\0';
        uint8_t expected_xor = (uint8_t)strtol(hex_str, NULL, 16);
        return (xor_sum == expected_xor);
    }
    return 0;
}

// GPS NMEA satır ayrıştırıcı ($GPRMC veya $GNRMC)
static void parse_nmea_sentence(const char *sentence) {
    if (!verify_nmea_checksum(sentence)) {
        return; // Checksum hatalı ise paket iptal
    }
    
    char header[10];
    get_nmea_field(sentence, 0, header, 10);
    
    if (strcmp(header, "$GPRMC") == 0 || strcmp(header, "$GNRMC") == 0) {
        char status[2];
        get_nmea_field(sentence, 2, status, 2);
        
        if (status[0] == 'A') { // Lock var (Active)
            char lat_str[20], lat_dir[2], lon_str[20], lon_dir[2];
            char speed_knots[15], course_deg[15];
            
            get_nmea_field(sentence, 3, lat_str, 20);
            get_nmea_field(sentence, 4, lat_dir, 2);
            get_nmea_field(sentence, 5, lon_str, 20);
            get_nmea_field(sentence, 6, lon_dir, 2);
            get_nmea_field(sentence, 7, speed_knots, 15);
            get_nmea_field(sentence, 8, course_deg, 15);
            
            double parsed_lat = nmea_to_decimal(lat_str, lat_dir[0]);
            double parsed_lon = nmea_to_decimal(lon_str, lon_dir[0]);
            float parsed_speed = atof(speed_knots) * 0.514444f; // knots -> m/s çevrim
            float parsed_cog = atof(course_deg);
            
            uint32_t now_ms = HAL_GetTick();
            
            // GPS Outlier Sıçrama Filtresi (Kötü Senaryo 1):
            // Eğer ilk konum kilitlenmesi ise doğrudan ata.
            // Sonraki okumalarda ardışık iki veri arasındaki değişimi zamansal delta ile oranlayıp 
            // 6 m/s üzerindeki sıçramaları konum hatası olarak eliyoruz.
            // GPS Outlier Sıçrama Filtresi (Kötü Senaryo 1) & İlk Fix Tuzağı Önlemi (Görev 16):
            // İlk kilitlemede doğrudan tek bir veriyi almak yerine, ilk 5 verinin ortalamasını 
            // referans konum olarak kurup outlier filtresini başlatıyoruz.
            static double init_lat_sum = 0.0;
            static double init_lon_sum = 0.0;
            static uint8_t init_fix_count = 0;

            if (!gps_data.has_first_fix) {
                if (init_fix_count < 5) {
                    init_lat_sum += parsed_lat;
                    init_lon_sum += parsed_lon;
                    init_fix_count++;
                    
                    // Ortalama oluşana kadar geçici olarak her gelen veriyi doğrudan yazıyoruz
                    gps_data.latitude = parsed_lat;
                    gps_data.longitude = parsed_lon;
                    gps_data.sog = parsed_speed;
                    gps_data.cog = parsed_cog;
                    gps_data.gps_lock = 1;
                    gps_data.last_update_time = now_ms;
                    
                    if (init_fix_count == 5) {
                        gps_data.latitude = init_lat_sum / 5.0;
                        gps_data.longitude = init_lon_sum / 5.0;
                        gps_data.has_first_fix = 1;
                    }
                }
            } else {
                float dt = (float)(now_ms - gps_data.last_update_time) / 1000.0f;
                if (dt <= 0.0f) dt = 0.1f; // Bölme hatasını önle
                
                // İki konum arasındaki mesafeyi metre cinsinden hesapla (Flat Earth)
                double dx = (parsed_lon - gps_data.longitude) * 111320.0 * cos(gps_data.latitude * M_PI / 180.0);
                double dy = (parsed_lat - gps_data.latitude) * 110574.0;
                float distance = (float)sqrt(dx*dx + dy*dy);
                float calculated_speed = distance / dt;
                
                // Eğer son veri alımından bu yana 5 saniyeden fazla zaman geçtiyse GPS kesilmiş olabilir,
                // filtreyi geçici olarak gevşetip yeni konumu doğrudan kabul ediyoruz. 
                // Aksi takdirde 6 m/s sıçrama kontrolü devrededir.
                if (calculated_speed < GPS_OUTLIER_SPEED_LIMIT || dt > 5.0f) {
                    gps_data.latitude = parsed_lat;
                    gps_data.longitude = parsed_lon;
                    gps_data.sog = parsed_speed;
                    gps_data.cog = parsed_cog;
                    gps_data.gps_lock = 1;
                    gps_data.last_update_time = now_ms;
                } else {
                    // Outlier tespit edildi: Konumu güncelleme, eski konumu koru. Hız değerini eski değere sabitle.
                    // GPS kilit durumunu bozmuyoruz ancak bu sıçrayan paketi yoksayıyoruz.
                }
            }
        } else {
            gps_data.gps_lock = 0; // GPS sinyali koptu
        }
    }
}

void sensors_gps_feed(uint8_t data) {
    if (data == '\n' || data == '\r') {
        if (nmea_idx > 5) {
            nmea_buf[nmea_idx] = '\0';
            if (!nmea_sentence_ready) {
                strcpy((char *)nmea_process_buf, nmea_buf);
                nmea_sentence_ready = 1;
            }
        }
        nmea_idx = 0;
    } else {
        if (nmea_idx < sizeof(nmea_buf) - 1) {
            nmea_buf[nmea_idx++] = data;
        } else {
            nmea_idx = 0; // Taşma koruması
        }
    }
}

void sensors_gps_update_tick(uint32_t current_time_ms) {
    char local_nmea[128];
    uint8_t process_now = 0;
    
    taskENTER_CRITICAL();
    if (nmea_sentence_ready) {
        strcpy(local_nmea, (char *)nmea_process_buf);
        nmea_sentence_ready = 0;
        process_now = 1;
    }
    taskEXIT_CRITICAL();
    
    if (process_now) {
        parse_nmea_sentence(local_nmea);
    }

    // Eğer son GPS verisinden bu yana 2.0 saniyeden fazla geçmişse kilit durumunu otomatik kayıp olarak ata
    if (gps_data.gps_lock && (current_time_ms - gps_data.last_update_time > 2000)) {
        gps_data.gps_lock = 0;
    }
}

float sensors_battery_read(ADC_HandleTypeDef *hadc) {
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
        uint32_t raw_val = HAL_ADC_GetValue(hadc);
        // 12-bit ADC -> 0-4095
        float pin_voltage = ((float)raw_val / 4095.0f) * ADC_REF_VOLTAGE;
        battery_voltage = pin_voltage * VOLTAGE_DIVIDER_RATIO;
    }
    HAL_ADC_Stop(hadc);
    
    return battery_voltage;
}

GPS_Data_t sensors_get_gps(void) {
    taskENTER_CRITICAL();
    GPS_Data_t copy;
    copy.latitude = gps_data.latitude;
    copy.longitude = gps_data.longitude;
    copy.sog = gps_data.sog;
    copy.cog = gps_data.cog;
    copy.gps_lock = gps_data.gps_lock;
    copy.last_update_time = gps_data.last_update_time;
    copy.has_first_fix = gps_data.has_first_fix;
    taskEXIT_CRITICAL();
    return copy;
}

IMU_Data_t sensors_get_imu(void) {
    return imu_data;
}

float sensors_get_yaw(void) {
    return imu_data.yaw;
}
