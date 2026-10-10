#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "utils.h"
#include "udp_receiver.h"
#include "control_loop.h"

QueueHandle_t control_data_queue = NULL;
TaskHandle_t led_blink_task_handle = NULL;

static const char *TAG = "STORMBORN_init";

static int init_builtin_led_gpio() {
    gpio_config_t my_gpio_config = {
        .pin_bit_mask = (1ULL << BLINK_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    // ESP_ERROR_CHECK(gpio_config(&my_gpio_config));
    if (gpio_config(&my_gpio_config) != ESP_OK) {
        ESP_LOGE(TAG, "Configuring LED GPIO failed...");
        return 0;
    }
    return 1;
}

static void blink_led_task(void *pvParameters) {
    while (1) {
        gpio_set_level(BLINK_GPIO, !gpio_get_level(BLINK_GPIO));
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void start_led_blink(void) {
    if (led_blink_task_handle != NULL) {
        vTaskResume(led_blink_task_handle);
    }
}
void stop_led_blink(void) {
    if (led_blink_task_handle != NULL) {
        vTaskSuspend(led_blink_task_handle);
    }
}
void set_led_on(void) {
    gpio_set_level(BLINK_GPIO, 1);
}


void app_main() {
    ESP_LOGI(TAG, "Configuring LED GPIO...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    control_data_queue = xQueueCreate(1, sizeof(ControlData));
    init_builtin_led_gpio();
    xTaskCreate(blink_led_task,
            "led_blink",
            1024,
            NULL,
            1, 
            &led_blink_task_handle);
    start_led_blink();

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    xTaskCreatePinnedToCore(
            control_loop_task,
            "control_loop_task",
            4096,
            NULL,
            4,
            NULL,
            1
            );

    // Wait 50ms
    vTaskDelay(pdMS_TO_TICKS(50));

    xTaskCreatePinnedToCore(
            udp_receiver_task,
            "udp_receiver_task",
            4096,
            NULL,
            5,
            NULL,
            0
            );
}
