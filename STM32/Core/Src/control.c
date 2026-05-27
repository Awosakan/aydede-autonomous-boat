#include "control.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

static PID_t yaw_pid;

// Motor Asimetri Kalibrasyon Katsayıları (Varsayılan: 1.0f)
float left_motor_scaling = 1.0f;
float right_motor_scaling = 1.0f;

void control_init(void) {
    // Varsayılan PID katsayıları (Saha testlerinde optimize edilebilir)
    yaw_pid.kp = 0.8f;
    yaw_pid.ki = 0.05f;
    yaw_pid.kd = 0.2f;
    yaw_pid.integrator = 0.0f;
    yaw_pid.last_error = 0.0f;
    yaw_pid.max_integrator = 0.3f; // Integrator doyumu (Anti-windup)
    yaw_pid.last_yaw = -999.0f;
    
    // Kalibrasyon değerlerini sıfırla/başlat
    left_motor_scaling = 1.0f;
    right_motor_scaling = 1.0f;
}

void control_set_pid_gains(float kp, float ki, float kd) {
    taskENTER_CRITICAL();
    yaw_pid.kp = kp;
    yaw_pid.ki = ki;
    yaw_pid.kd = kd;
    yaw_pid.integrator = 0.0f;
    yaw_pid.last_yaw = -999.0f;
    taskEXIT_CRITICAL();
}

void control_set_motor_scaling(float left_scale, float right_scale) {
    taskENTER_CRITICAL();
    left_motor_scaling = left_scale;
    right_motor_scaling = right_scale;
    taskEXIT_CRITICAL();
}

static float linearize_thrust(float thrust) {
    if (thrust == 0.0f) return 0.0f;
    float sign = (thrust > 0.0f) ? 1.0f : -1.0f;
    float abs_thrust = fabsf(thrust);
    // Fırçasız motor itki eğrisi doğrusallaştırma
    return sign * (0.6f * abs_thrust + 0.4f * abs_thrust * abs_thrust);
}

MotorOutput_t control_update(float current_yaw, float target_yaw, float target_speed, float dt) {
    MotorOutput_t output = {0.0f, 0.0f};
    
    if (dt <= 0.0f) {
        output.left_thrust = 0.0f;
        output.right_thrust = 0.0f;
        return output;
    }

    if (yaw_pid.last_yaw == -999.0f) {
        yaw_pid.last_yaw = current_yaw;
    }

    // 1. Açısal Hata Hesaplama ve Sarmalama (Yaw Wrapping)
    // 359 derece ile 1 derece arasındaki hatanın 358 değil, -2 derece olmasını sağlar.
    float error = target_yaw - current_yaw;
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // 2. Oransal Terim (Proportional)
    float p_term = yaw_pid.kp * error;

    // 3. İntegral Terim (Integral) ve Anti-Windup (Doyum Sınırı)
    yaw_pid.integrator += error * dt;
    if (yaw_pid.integrator > yaw_pid.max_integrator) {
        yaw_pid.integrator = yaw_pid.max_integrator;
    } else if (yaw_pid.integrator < -yaw_pid.max_integrator) {
        yaw_pid.integrator = -yaw_pid.max_integrator;
    }
    float i_term = yaw_pid.ki * yaw_pid.integrator;

    // 4. Türev Terim (Derivative-on-Measurement)
    // Hata türevi yerine yön açısının negatif türevini kullanarak setpoint sıçramalarını önler.
    float yaw_diff = current_yaw - yaw_pid.last_yaw;
    while (yaw_diff > 180.0f)  yaw_diff -= 360.0f;
    while (yaw_diff < -180.0f) yaw_diff += 360.0f;

    float derivative = -yaw_diff / dt;
    float d_term = yaw_pid.kd * derivative;
    
    yaw_pid.last_yaw = current_yaw;
    yaw_pid.last_error = error;

    // 5. Toplam Dümen Düzeltme Komutu (Steering Command)
    float steer_cmd = p_term + i_term + d_term;
    
    // Dümen düzeltmesini makul limitlerde sınırla (-1.0 ile 1.0 arası)
    if (steer_cmd > 1.0f)  steer_cmd = 1.0f;
    if (steer_cmd < -1.0f) steer_cmd = -1.0f;

    // 6. Diferansiyel İtki Eşleme ve Doyum Koruması (Görev 4.4)
    // Motor itki limitleri aşıldığında (Thrust Saturation), dümen farkını (steer_cmd) koruyacak şekilde 
    // nominal hızı (target_speed) orantılı olarak sınırlandırıyoruz.
    float max_speed_allowed = 1.0f - fabsf(steer_cmd);
    float min_speed_allowed = -1.0f + fabsf(steer_cmd);
    
    float adjusted_speed = target_speed;
    if (adjusted_speed > max_speed_allowed) {
        adjusted_speed = max_speed_allowed;
    } else if (adjusted_speed < min_speed_allowed) {
        adjusted_speed = min_speed_allowed;
    }
    
    output.left_thrust = adjusted_speed + steer_cmd;
    output.right_thrust = adjusted_speed - steer_cmd;

    // 7. Ölü Bölge (Deadband) Telafisi
    if (output.left_thrust > 0.0f) {
        output.left_thrust = MOTOR_DEADBAND + output.left_thrust * (1.0f - MOTOR_DEADBAND);
    } else if (output.left_thrust < 0.0f) {
        output.left_thrust = -MOTOR_DEADBAND + output.left_thrust * (1.0f - MOTOR_DEADBAND);
    }
    
    if (output.right_thrust > 0.0f) {
        output.right_thrust = MOTOR_DEADBAND + output.right_thrust * (1.0f - MOTOR_DEADBAND);
    } else if (output.right_thrust < 0.0f) {
        output.right_thrust = -MOTOR_DEADBAND + output.right_thrust * (1.0f - MOTOR_DEADBAND);
    }

    // 8. İtki Eğrisi Doğrusallaştırma (Non-Linear Thrust Mapping)
    output.left_thrust = linearize_thrust(output.left_thrust);
    output.right_thrust = linearize_thrust(output.right_thrust);

    // 9. Motor Asimetri Kalibrasyonu Eşlemesi
    output.left_thrust *= left_motor_scaling;
    output.right_thrust *= right_motor_scaling;

    // Sınırlandırma (Clamping) -> Motor çıkışlarının [-1.0, 1.0] aralığında olmasını garantiler
    if (output.left_thrust > 1.0f)  output.left_thrust = 1.0f;
    if (output.left_thrust < -1.0f) output.left_thrust = -1.0f;
    
    if (output.right_thrust > 1.0f)  output.right_thrust = 1.0f;
    if (output.right_thrust < -1.0f) output.right_thrust = -1.0f;

    // Emniyet Koruması: Eğer hedef ileri hız sıfır ise ve yön değişimi gereksiz küçükse motorları kapat
    if (fabsf(target_speed) < 0.05f && fabsf(error) < 5.0f) {
        output.left_thrust = 0.0f;
        output.right_thrust = 0.0f;
    }

    return output;
}
