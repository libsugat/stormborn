#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "utils.h"

#include "control_loop.h"

extern QueueHandle_t control_data_queue;

static const char *TAG = "CONTROL_LOOP_TASK";

// --- Servo Configuration (Timer 0) ---
#define SERVO_TIMER             LEDC_TIMER_0
#define SERVO_SPEED_MODE        LEDC_LOW_SPEED_MODE
#define SERVO_DUTY_RES          LEDC_TIMER_14_BIT   // Max value: 16383
#define SERVO_FREQ_HZ           50                  // 20ms period
#define NUM_SERVOS              8
#define HPOINT_STEP             2048

// --- Motor Configuration (Timer 1) ---
#define MOTOR_TIMER             LEDC_TIMER_1
#define MOTOR_SPEED_MODE        LEDC_HIGH_SPEED_MODE
#define MOTOR_DUTY_RES          LEDC_TIMER_8_BIT   // Max value: 255
#define MOTOR_FREQ_HZ           20000               // 20 kHz ultrasonic (silent)
#define NUM_MOTOR_PINS          4

#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1

#define MOTOR_DIR_F  0
#define MOTOR_DIR_B  1

static const gpio_num_t s_servo_gpios[NUM_SERVOS] = {
    GPIO_NUM_18,
    GPIO_NUM_19,
    GPIO_NUM_21,
    GPIO_NUM_22,
    GPIO_NUM_23,
    GPIO_NUM_25,
    GPIO_NUM_4,
    GPIO_NUM_16 // Gemini: Use GPIO_NUM_17 if GPIO 16 is reserved by custom UART
};

static const gpio_num_t s_motor_gpios[NUM_MOTOR_PINS] = {
    // LEFT
    GPIO_NUM_26,  // Motor 1 RPWM
    GPIO_NUM_27,  // Motor 1 LPWM

    // Right
    GPIO_NUM_32,  // Motor 2 RPWM
    GPIO_NUM_33   // Motor 2 LPWM
};


IRAM_ATTR void set_servo_angle(uint8_t servo_idx, int angle)
{
    if (servo_idx >= NUM_SERVOS) return;
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // Linear pulse interpolation: 500us to 2500us
    uint32_t pulse_us = 500 + (angle * 2000) / 180;
    
    // Duty = (pulse_us / 20000us) * 16383
    uint32_t duty = (pulse_us * ((1 << SERVO_DUTY_RES) - 1)) / 20000;

    ledc_set_duty(SERVO_SPEED_MODE, (ledc_channel_t)servo_idx, duty);
    ledc_update_duty(SERVO_SPEED_MODE, (ledc_channel_t)servo_idx);
}

IRAM_ATTR void set_motor_speed(uint8_t motor_id, uint8_t speed, int dir)
{
    if (motor_id > 1) return; // 0 = Motor 1, 1 = Motor 2
    
    // Motor 0 -> Channels 0 & 1; Motor 1 -> Channels 2 & 3
    ledc_channel_t rpwm_ch = (ledc_channel_t)(motor_id << 1);
    ledc_channel_t lpwm_ch = (ledc_channel_t)((motor_id << 1) + 1);

    uint32_t duty = speed;

    uint32_t forward_duty = 0;
    uint32_t reverse_duty = 0;

    if (speed > 0) {
        if (dir == MOTOR_DIR_F) {
            forward_duty = duty;
            reverse_duty = 0;
        } else {
            forward_duty = 0;
            reverse_duty = duty;
        }
    } // speed == 0 leaves both at 0 (neutral / coast)

    ledc_set_duty(MOTOR_SPEED_MODE, rpwm_ch, forward_duty);
    ledc_update_duty(MOTOR_SPEED_MODE, rpwm_ch);
    ledc_set_duty(MOTOR_SPEED_MODE, lpwm_ch, reverse_duty);
    ledc_update_duty(MOTOR_SPEED_MODE, lpwm_ch);
}

void handle_packet_drive(uint8_t speed, uint8_t dir_bits) {
    // dir_bits: [Bit 3: Right] [Bit 2: Left] [Bit 1: Back] [Bit 0: Fwd]
    int t = (dir_bits & 1) - ((dir_bits >> 1) & 1);
    int s = ((dir_bits >> 3) & 1) - ((dir_bits >> 2) & 1);

    int l = t + s;
    int r = t - s;

    ESP_LOGI(TAG, "left: %d, right: %d      dir_bits: %u, t: %d, s:%d", l < 0, r < 0, dir_bits, t,s);

    // speed: 0 when stopped, untouched 'speed' when moving
    // dir:   0 for forward (l >= 0), 1 for backward (l < 0)
    set_motor_speed(MOTOR_LEFT,  speed & -(l != 0), l < 0);
    set_motor_speed(MOTOR_RIGHT, speed & -(r != 0), r < 0);
}

int init_pheripherals(void) {
    // Configure Timer 0 for Servos (50 Hz, 14-bit)
    ledc_timer_config_t servo_timer_cfg = {
        .speed_mode      = SERVO_SPEED_MODE,
        .duty_resolution = SERVO_DUTY_RES,
        .timer_num       = SERVO_TIMER,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer_cfg));

    ledc_timer_config_t motor_timer_cfg = {
        .speed_mode      = MOTOR_SPEED_MODE,
        .duty_resolution = MOTOR_DUTY_RES,
        .timer_num       = MOTOR_TIMER,
        .freq_hz         = MOTOR_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&motor_timer_cfg));

    for (int i = 0; i < NUM_SERVOS; i++) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = s_servo_gpios[i],
            .speed_mode = SERVO_SPEED_MODE,
            .channel    = (ledc_channel_t)i,
            .timer_sel  = SERVO_TIMER,
            .duty       = 0,
            .hpoint     = i * HPOINT_STEP
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    }

    // Bind 4 Channels to BTS7960 Motors (Timer 1, High Speed Mode)
    for (int i = 0; i < NUM_MOTOR_PINS; i++) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = s_motor_gpios[i],
            .speed_mode = MOTOR_SPEED_MODE,
            .channel    = (ledc_channel_t)i,
            .timer_sel  = MOTOR_TIMER,
            .duty       = 0,
            .hpoint     = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    }
    return 1;
}

void emergency_stop_motors() {
    set_motor_speed(MOTOR_LEFT, 0, MOTOR_DIR_F);
    set_motor_speed(MOTOR_RIGHT, 0, MOTOR_DIR_F);
}

void control_loop_task(void *pvParameters) {
    if (init_pheripherals() != 1) {
        ESP_LOGE(TAG, "Configuring LED GPIO failed...");
        vTaskDelete(NULL);
        return;
    }

    ControlData data;
    TickType_t timeout_ticks = pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS);

    while (1) {
        BaseType_t status = xQueueReceive(
                control_data_queue,
                &data,
                timeout_ticks
                );

        if (status == pdPASS) {
            // ESP_LOGI(TAG, "Received packet, speed: %d", data.speed);
            uint8_t dir = 0;
            if (data.forward > 0)      dir |= (1 << 0); // Bit 0: FWD
            else if (data.forward < 0) dir |= (1 << 1); // Bit 1: BACK

            // Turn Right / Left
            if (data.turn > 0)         dir |= (1 << 3); // Bit 3: RIGHT
            else if (data.turn < 0)    dir |= (1 << 2); // Bit 2: LEFT
            handle_packet_drive(data.speed, dir);
        }
        else {
            emergency_stop_motors();
        }
    }
    vTaskDelete(NULL);
}
