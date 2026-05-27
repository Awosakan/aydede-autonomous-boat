#include "main.h"
#include "protocol.h"
#include "control.h"
#include "sensors.h"
#include "safety.h"
#include <string.h>

// Çevre Birimleri Tanımları
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1; // Telefon Haberleşmesi
UART_HandleTypeDef huart2; // GPS Haberleşmesi
DMA_HandleTypeDef hdma_usart1_rx;
IWDG_HandleTypeDef hiwdg;

// FreeRTOS Görev Tanımları
TaskHandle_t TelemetryTaskHandle;
TaskHandle_t NavigationTaskHandle;
TaskHandle_t SafetyTaskHandle;

// Haberleşme Değişkenleri
// USART1_RX_BUF_SIZE artık main.h'de tanımlı (C1 düzeltmesi)
uint8_t usart1_rx_buf[USART1_RX_BUF_SIZE];
uint16_t usart1_rx_read_ptr = 0;
ProtocolParser_t serial_parser;

volatile PhoneCommands_t latest_commands = {
    .control_mode = 0,
    .target_speed = 0.0f,
    .target_heading = 0.0f
};

// GPS Kesme Değişkeni
uint8_t gps_rx_byte = 0;

// Motor Çıkış Değişkenleri (Safety Task tarafından denetlenmesi için global yapılmıştır)
volatile float global_left_thrust = 0.0f;
volatile float global_right_thrust = 0.0f;
volatile float global_battery_voltage = 12.0f; // Batarya voltajı (TelemetryTask tarafından güncellenir)

// Fonksiyon Bildirimleri
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);

void StartTelemetryTask(void *argument);
void StartNavigationTask(void *argument);
void StartSafetyTask(void *argument);

int main(void) {
    // STM32 HAL İlklendirmesi
    HAL_Init();

    // Sistem Saatini Yapılandır (168 MHz)
    SystemClock_Config();

    // Donanım Birimlerini İlklendir
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init(); // Telefon Seri Portu
    MX_USART2_UART_Init(); // GPS Seri Portu
    MX_I2C1_Init();        // IMU (MPU6050/9250)
    MX_ADC1_Init();        // Batarya Voltajı
    MX_TIM3_Init();        // Motor PWM Sinyalleri
    MX_IWDG_Init();        // Donanımsal Watchdog (Görev 25 & 87 & 143)

    // Yazılım Kontrolörlerini İlklendir
    control_init();
    protocol_parser_init(&serial_parser);
    sensors_gps_init();
    
    // Sensörlerin ilk okumasını alarak Emniyet Katmanını ilklendir
    float init_volts = sensors_battery_read(&hadc1);
    sensors_imu_init(&hi2c1);
    HAL_Delay(50);
    sensors_imu_update(&hi2c1, 0.01f);
    safety_init(init_volts, sensors_get_yaw());

    // Motor PWM Sinyallerini Başlat (50Hz ESC tetikleme)
    // Başlangıçta ESC'lere 1500us (Boşta / Neutral) gönder
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);

    // UART Kesme Önceliklerini FreeRTOS Güvenli Seviyeye Ayarla (Görev 68)
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    // Telefon Haberleşmesi için DMA RX Circular modunu başlat
    HAL_UART_Receive_DMA(&huart1, usart1_rx_buf, USART1_RX_BUF_SIZE);

    // GPS Kesmeli Alımı Başlat (1 bayt kesmeli)
    HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);

    // FreeRTOS Görevlerini Tanımla ve Zamanlayıcıyı Başlat
    xTaskCreate(StartTelemetryTask, "TelemetryTask", 256, NULL, 2, &TelemetryTaskHandle);
    xTaskCreate(StartNavigationTask, "NavTask", 512, NULL, 3, &NavigationTaskHandle);
    xTaskCreate(StartSafetyTask, "SafetyTask", 256, NULL, 4, &SafetyTaskHandle);

    vTaskStartScheduler();

    while (1) {
        // FreeRTOS zamanlayıcısı devrede olduğu için buraya erişilmemelidir.
    }
}

// 1. Telemetri ve Haberleşme Görevi (50 Hz - Görev 3.1)
void StartTelemetryTask(void *argument) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms = 50Hz (Görev 3.1)
    
    uint8_t tx_packet[128];
    
    for (;;) {
        // A. DMA Circular Buffer'daki yeni gelen baytları ayrıştırıcıya besle
        // DMA yazma işaretçisini bul (kalan veri miktarından hesaplanır)
        uint16_t dma_write_ptr = USART1_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        
        while (usart1_rx_read_ptr != dma_write_ptr) {
            uint8_t byte = usart1_rx_buf[usart1_rx_read_ptr];
            usart1_rx_read_ptr = (usart1_rx_read_ptr + 1) % USART1_RX_BUF_SIZE;
            
            if (protocol_parser_feed(&serial_parser, byte, HAL_GetTick())) {
                // Paket başarıyla doğrulandı (MsgID & CRC16 OK)
                safety_feed_watchdog(); // Watchdog'u besle
                
                if (serial_parser.msg_id == MSG_PHONE_COMMANDS && serial_parser.payload_len == sizeof(PhoneCommands_t)) {
                    PhoneCommands_t cmd;
                    memcpy(&cmd, serial_parser.payload, sizeof(PhoneCommands_t));
                    
                    static uint8_t last_command_seq = 0;
                    static uint8_t first_command_received = 0;
                    
                    // Sequence ID kontrolü (Bayat komutları ezme - Görev 3.5)
                    if (!first_command_received || ((uint8_t)(cmd.sequence_id - last_command_seq) > 0 && (uint8_t)(cmd.sequence_id - last_command_seq) < 128)) {
                        last_command_seq = cmd.sequence_id;
                        first_command_received = 1;
                        
                        // Komutları kritik bölge koruması altında güncelle (Görev 3.3)
                        taskENTER_CRITICAL();
                        latest_commands.control_mode = cmd.control_mode;
                        latest_commands.target_speed = cmd.target_speed;
                        latest_commands.target_heading = cmd.target_heading;
                        taskEXIT_CRITICAL();
                    }
                } 
                else if (serial_parser.msg_id == MSG_HEARTBEAT && serial_parser.payload_len == sizeof(Heartbeat_t)) {
                    Heartbeat_t hb;
                    memcpy(&hb, serial_parser.payload, sizeof(Heartbeat_t));
                    
                    // Telefonun otonom mod isteğine göre modu güncelle
                    safety_set_mode(hb.mode);
                }
                else if (serial_parser.msg_id == MSG_PID_TUNING && serial_parser.payload_len == sizeof(PIDTuning_t)) {
                    PIDTuning_t pid;
                    memcpy(&pid, serial_parser.payload, sizeof(PIDTuning_t));
                    
                    // Canlı PID katsayılarını güncelle (Görev 3.2)
                    control_set_pid_gains(pid.kp, pid.ki, pid.kd);
                }
            }
        }

        // B. GPS Zaman Aşımı Kontrolü
        sensors_gps_update_tick(HAL_GetTick());

        // C. Batarya Okumasını Kritik Bölge DIŞINDA Yap (ADC polling 10ms'e kadar bloklar)
        float bat = sensors_battery_read(&hadc1);
        global_battery_voltage = bat; // SafetyTask için paylaşılan değişken (A3 düzeltmesi)
        
        // Diğer Sensör Verilerini Kritik Bölge Korumasıyla Oku (Görev 3.3)
        taskENTER_CRITICAL();
        GPS_Data_t gps = sensors_get_gps();
        IMU_Data_t imu = sensors_get_imu();
        uint8_t current_mode = safety_get_mode();
        taskEXIT_CRITICAL();

        // D. Telemetri Paketini Oluştur
        Telemetry_t telem;
        telem.lat = gps.latitude;
        telem.lon = gps.longitude;
        telem.sog = gps.sog;
        telem.cog = gps.cog;
        telem.gps_lock = gps.gps_lock;
        telem.roll = imu.roll;
        telem.pitch = imu.pitch;
        telem.yaw = imu.yaw;
        telem.roll_rate = imu.roll_rate;
        telem.pitch_rate = imu.pitch_rate;
        telem.yaw_rate = imu.yaw_rate;
        telem.battery = bat;
        telem.mode = current_mode;

        // E. Paketi Seri Port Formatına Dönüştür ve Gönder (SYNC1 + SYNC2 + MsgID + Len + Payload + CRC16)
        tx_packet[0] = SYNC_BYTE_1;
        tx_packet[1] = SYNC_BYTE_2;
        tx_packet[2] = MSG_STM32_TELEMETRY;
        tx_packet[3] = sizeof(Telemetry_t);
        memcpy(tx_packet + 4, &telem, sizeof(Telemetry_t));
        
        uint16_t crc = calculate_crc16(tx_packet, 4 + sizeof(Telemetry_t));
        tx_packet[4 + sizeof(Telemetry_t)] = (uint8_t)(crc & 0xFF);
        tx_packet[5 + sizeof(Telemetry_t)] = (uint8_t)((crc >> 8) & 0xFF);
        
        uint16_t total_len = 6 + sizeof(Telemetry_t); // 54 + 6 = 60 bayt
        
        HAL_UART_Transmit(&huart1, tx_packet, total_len, 15);

        safety_task_feed(TASK_WD_TELEMETRY);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 2. Seyrüsefer Kontrol Görevi (50 Hz)
void StartNavigationTask(void *argument) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms = 50Hz
    
    for (;;) {
        float dt = 0.02f; // Sabit 20ms adım süresi
        
        // A. IMU Verisini Oku ve Complementary Filtreyi Çalıştır
        sensors_imu_update(&hi2c1, dt);
        
        // B. Hedef ve Mevcut Değerleri Al
        float current_yaw = sensors_get_yaw();
        
        taskENTER_CRITICAL();
        float target_heading = latest_commands.target_heading;
        float target_speed = latest_commands.target_speed;
        taskEXIT_CRITICAL();
        
        // C. PID Dümenleme ve Diferansiyel İtkiyi Güncelle
        MotorOutput_t motors;
        uint8_t current_mode = safety_get_mode();
        
        if (current_mode == MODE_AUTO) {
            // Tam otonom mod: PID devrededir
            motors = control_update(current_yaw, target_heading, target_speed, dt);
        } 
        else if (current_mode == MODE_MANUAL) {
            // Manuel mod: Telefonun doğrudan diferansiyel komutları geçerlidir
            // target_speed -> Sol itki, target_heading -> Sağ itki olarak taşınır
            motors.left_thrust = target_speed;
            motors.right_thrust = target_heading;
        } 
        else {
            // Idle, Failsafe veya Acil Durum modları: Motorları durdur
            motors.left_thrust = 0.0f;
            motors.right_thrust = 0.0f;
        }
        
        // Global motor komutlarını güncelle (Safety task denetimi için)
        global_left_thrust = motors.left_thrust;
        global_right_thrust = motors.right_thrust;

        // D. PWM Dürtü Genişliği Hesaplama ve Uygulama (ESC Sinyali: 1000us - 2000us)
        // Kavitasyon ve mekanik yıpranmayı önlemek için Slew Rate Limiter (ivme rampası) uygulanır.
        // Ancak emniyet durumu (safety_is_ok() == 0) kritik ise motorlar anında kesilmelidir (bypass).
        static float current_left_pulse = 1500.0f;
        static float current_right_pulse = 1500.0f;
        
        int16_t left_pulse = 1500;
        int16_t right_pulse = 1500;
        
        if (!safety_is_ok()) {
            left_pulse = 1500;
            right_pulse = 1500;
            current_left_pulse = 1500.0f;
            current_right_pulse = 1500.0f;
        } else {
            float target_left = 1500.0f + (global_left_thrust * 500.0f);
            float target_right = 1500.0f + (global_right_thrust * 500.0f);
            
            // Adım başına maks 25us değişim (~1.25 saniyede tam gazdan durma)
            #define MOTOR_SLEW_LIMIT_US 25.0f 
            
            float diff_l = target_left - current_left_pulse;
            if (diff_l > MOTOR_SLEW_LIMIT_US) diff_l = MOTOR_SLEW_LIMIT_US;
            else if (diff_l < -MOTOR_SLEW_LIMIT_US) diff_l = -MOTOR_SLEW_LIMIT_US;
            current_left_pulse += diff_l;
            
            float diff_r = target_right - current_right_pulse;
            if (diff_r > MOTOR_SLEW_LIMIT_US) diff_r = MOTOR_SLEW_LIMIT_US;
            else if (diff_r < -MOTOR_SLEW_LIMIT_US) diff_r = -MOTOR_SLEW_LIMIT_US;
            current_right_pulse += diff_r;
            
            left_pulse = (int16_t)current_left_pulse;
            right_pulse = (int16_t)current_right_pulse;
            
            // Sınır koruması
            if (left_pulse > 2000)  left_pulse = 2000;
            if (left_pulse < 1000)  left_pulse = 1000;
            if (right_pulse > 2000) right_pulse = 2000;
            if (right_pulse < 1000) right_pulse = 1000;
        }

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_pulse);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_pulse);

        safety_task_feed(TASK_WD_NAVIGATION);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 3. Emniyet ve Failsafe Denetleyici Görevi (100 Hz)
void StartSafetyTask(void *argument) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz
    
    for (;;) {
        float raw_volts = global_battery_voltage; // ADC çakışmasını önlemek için TelemetryTask'ın okumasını kullan (A3)
        float current_yaw = sensors_get_yaw();
        
        // Emniyet durumunu güncelle (10 ms adım süresi ile)
        safety_update(raw_volts, current_yaw, global_left_thrust, global_right_thrust, 10);
        
        // Eğer emniyet durumu kritik bir hata tespit ettiyse, motorları anında durdur
        if (!safety_is_ok()) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
        }

        safety_task_feed(TASK_WD_SAFETY);
        if (safety_check_task_watchdogs(10)) {
            HAL_IWDG_Refresh(&hiwdg);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// GPS Veri Alımı (UART2 RX) Kesme Servis Rutini Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Gelen baytı GPS parser'a gönder
        sensors_gps_feed(gps_rx_byte);
        
        // Bir sonraki karakter kesmesini başlat
        HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);
    }
}

// UART Hata Yönetim Callback (ORE - Overrun, FE - Frame Error, NE - Noise Error Temizleme)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // DMA durdur, bayrakları temizle, SR/DR oku ve alımı yeniden başlat
        HAL_UART_DMAStop(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        
        volatile uint32_t tmpreg = huart->Instance->SR;
        tmpreg = huart->Instance->DR;
        (void)tmpreg;
        
        HAL_UART_Receive_DMA(huart, usart1_rx_buf, USART1_RX_BUF_SIZE);
    }
    else if (huart->Instance == USART2) {
        // Bayrakları temizle, SR/DR oku ve kesmeli alımı yeniden başlat
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        
        volatile uint32_t tmpreg = huart->Instance->SR;
        tmpreg = huart->Instance->DR;
        (void)tmpreg;
        
        HAL_UART_Receive_IT(huart, &gps_rx_byte, 1);
    }
}

// Acil Durdurma Butonu Fiziksel Kesmesi (EXTI PC13)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == EMERGENCY_STOP_PIN) {
        // Fiziksel acil durum kesmesi algılandı: Motorları anında kilitli modda kapat
        safety_trigger_emergency();
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
    }
}

// Sistem Saat Yapılandırması (168 MHz SYSCLK, 42 MHz APB1, 84 MHz APB2)
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // FPU Aktifleştir (Cortex-M4 Donanımsal Float İşlemci)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

    // Regülatör çıkış voltaj ayarı
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE_1);

    // Donanımsal ART Hızlandırıcı ve Prefetch aktif et (Flash erişim gecikmesini sıfırlamak için)
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
    __HAL_FLASH_DATA_CACHE_ENABLE();

    // HSI/HSE PLL Yapılandırması (HSE = 8MHz kristal varsayılmıştır)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    // Bus saatlerini yapılandır
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; // 168/4 = 42 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; // 168/2 = 84 MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    // Batarya okuma kanalı PA1 (ADC_CHANNEL_1)
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000; // 400 kHz Hızlı Mod
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM3_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83; // 84MHz APB1 timer clock -> 1MHz counter saati
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 20000; // 50 Hz PWM periyodu (20ms)
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.RepetitionCounter = 0;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500; // 1.5ms durma pozisyonu
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    // Sol Motor (PA6 - TIM3 CH1)
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    // Sağ Motor (PA7 - TIM3 CH2)
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600; // GPS standart baud hızı (NMEA)
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_DMA_Init(void) {
    __HAL_RCC_DMA2_CLK_ENABLE();

    // USART1 RX DMA2 Stream 5 Channel 4
    hdma_usart1_rx.Instance = DMA2_Stream5;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) {
        Error_Handler();
    }

    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);
    
    // Kesme önceliklerini ayarla (Görev 68)
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Hot Reboot Thrashing Önlemi (Görev 65): Motor PWM pinlerini (PA6, PA7) önce Çıkış Low olarak tut
    HAL_GPIO_WritePin(GPIOA, MOTOR_LEFT_PIN | MOTOR_RIGHT_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = MOTOR_LEFT_PIN | MOTOR_RIGHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // EXTI PC13 Acil Durdurma Kesme Yapılandırması (Pull-up, Düşen Kenar Tetikleme)
    GPIO_InitStruct.Pin = EMERGENCY_STOP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(EMERGENCY_STOP_PORT, &GPIO_InitStruct);

    // Motor PWM Pinleri (PA6, PA7 TIM3) Alternatif Fonksiyon (AF2) olarak yapılandırılmalıdır
    GPIO_InitStruct.Pin = MOTOR_LEFT_PIN | MOTOR_RIGHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(MOTOR_LEFT_PORT, &GPIO_InitStruct);

    // I2C1 Pinleri (PB6 -> SCL, PB7 -> SDA) Alternatif Fonksiyon (AF4)
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // UART1 Pinleri (PA9 -> TX, PA10 -> RX) Alternatif Fonksiyon (AF7)
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // UART2 Pinleri (PA2 -> TX, PA3 -> RX) Alternatif Fonksiyon (AF7)
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // EXTI Kesmesini etkinleştir
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void MX_IWDG_Init(void) {
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64; // LSI (32kHz) / 64 = 500Hz clock
    hiwdg.Init.Reload = 1000; // Timeout = 2.0 seconds (Görev 87 & 143)
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void) {
    // Acil durum tetikle ve motorları anında durdur
    safety_trigger_emergency();
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
    while (1) {
        // Hata durumunda sistemi sonsuz döngüde kilitle
    }
}
