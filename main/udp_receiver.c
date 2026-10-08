#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sys/socket.h"
#include "driver/gpio.h"

#include "utils.h"
#include "udp_receiver.h"


extern QueueHandle_t control_data_queue;

static const char *TAG = "UDP_RECEIVER_TASK";

static int sock = -1;

static void led_blink_task(void *pvParameters) {
    bool led_state = false;
    while (1) {
        gpio_set_level(BLINK_GPIO, led_state);
        led_state = !led_state;
        vTaskDelay(pdMS_TO_TICKS(200)); // Blinks every 200 ms
    }
}

extern void wifi_init_sta(void);
extern EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

int receiver_init(void) {
    wifi_init_sta();

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket creation failed");
        vTaskDelete(NULL);
        return 0;
    }

    struct sockaddr_in server_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed: errno %d", errno);
        close(sock);
        sock = -1;
        return 0;
    }

    struct timeval tv = {
        .tv_sec  = 0,
        .tv_usec = WATCHDOG_TIMEOUT_MS * 1000
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return 1;
}

void udp_receiver_task(void *pvParameters) {
    if (!receiver_init()) {
        // Log error message
        vTaskDelete(NULL);
        return;
    }

    ControlData packet;
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    while (1) {
        EventBits_t wifi_event_bits = xEventGroupGetBits(wifi_event_group);
        if (!(wifi_event_bits & WIFI_CONNECTED_BIT)) {
            ESP_LOGI(TAG, "Pausing till wifi reconnects...");
            xEventGroupWaitBits(
                wifi_event_group,
                WIFI_CONNECTED_BIT,
                pdFALSE,        // Don't clear bit on exit
                pdTRUE,
                portMAX_DELAY   // Block until connected
            );
        }
        int len = recvfrom(sock, &packet, sizeof(packet), 0,
                           (struct sockaddr *)&source_addr, &socklen);
        if (len == sizeof(ControlData)) {
            // Overwrite mailbox queue: always gives Core 1 the newest target
            xQueueOverwrite(control_data_queue, &packet);
        } else if (len < 0){
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                ESP_LOGI(TAG, "Packet receive error, errno %d", errno);
            }
            else {
                ESP_LOGE(TAG, "Socket error, errno: %d", errno);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }

    close(sock);
    vTaskDelete(NULL);
}
