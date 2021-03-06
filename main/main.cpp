#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "esp_log.h"
#include "esp_https_ota.h"
#include "nvs_flash.h"
#include "lwip/apps/sntp.h"
#include "wifi_helper.h"
#include "mqtt_helper.h"

static const char *TAG = "app";


#define GPIO_INPUT_IO_RS    GPIO_NUM_27
#define GPIO_INPUT_IO_E     GPIO_NUM_25

#define GPIO_INPUT_IO_D4    GPIO_NUM_18
#define GPIO_INPUT_IO_D5    GPIO_NUM_19
#define GPIO_INPUT_IO_D6    GPIO_NUM_22
#define GPIO_INPUT_IO_D7    GPIO_NUM_23

#define GPIO_OUTPUT_RELAY   GPIO_NUM_4

#define GPIO_INPUT_PIN_SEL ( \
    BIT(GPIO_INPUT_IO_RS) |  \
    BIT(GPIO_INPUT_IO_E)  |  \
    BIT(GPIO_INPUT_IO_D4) |  \
    BIT(GPIO_INPUT_IO_D5) |  \
    BIT(GPIO_INPUT_IO_D6) |  \
    BIT(GPIO_INPUT_IO_D7)    \
)

#define ESP_INTR_FLAG_DEFAULT 0

#define BYTES_PER_LINE 16

#define BIT_TO_POS(value, from, to) (((value & BIT(from)) >> from) << to)

static QueueHandle_t gpio_evt_queue = NULL;

static const char* ota_url = "http://raspberrypi.fritz.box:8032/esp32/PowSpy.bin";

static void ota_task(void * pvParameter) {
    ESP_LOGI(TAG, "Starting OTA update...");

    esp_http_client_config_t config = {
        .url = ota_url,
    };
    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware Upgrades Failed");
    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// NOTES:
// - High level interrupts

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    // This needs to be fast, < 4 us, at least
    uint32_t gpioValues = (REG_READ(GPIO_IN_REG) & GPIO_INPUT_PIN_SEL);
    xQueueSendFromISR(gpio_evt_queue, &gpioValues, NULL);
}

// Configure the used gpio pins, and setup the interrupt handler
// Important: Interrupt handler will run on the core which calls this method.
static void gpioSetupTask(void* arg) {
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_E, GPIO_INTR_NEGEDGE);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_E, gpio_isr_handler, NULL);

    vTaskDelete(NULL);
}

static void gpioTask(void* arg) {
    uint32_t lastMs = 0;
    uint32_t gpioValues = 0;
    uint32_t rs = 0;
    uint32_t bits = 0;

    uint32_t nibble = 0;

    uint8_t cmd = 0;
    uint8_t data = 0;

    uint8_t addr = 0;
    uint8_t line1[40] = {};
    uint8_t line2[40] = {};
    for(int pos = 0; pos < 40; pos++) {
        line1[pos] = 0x20;
        line2[pos] = 0x20;
    }

    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &gpioValues, portMAX_DELAY)) {
            rs = (gpioValues & BIT(GPIO_INPUT_IO_RS)) > 0;

            bits = 0;
            bits |= BIT_TO_POS(gpioValues, GPIO_INPUT_IO_D7, 3);
            bits |= BIT_TO_POS(gpioValues, GPIO_INPUT_IO_D6, 2);
            bits |= BIT_TO_POS(gpioValues, GPIO_INPUT_IO_D5, 1);
            bits |= BIT_TO_POS(gpioValues, GPIO_INPUT_IO_D4, 0);

            uint32_t nowMs = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if(lastMs == 0) {
                lastMs = nowMs;
            }

            // 200 ms between updates, 150 should be a save limit
            if(nowMs - lastMs > 150) {
                char hex_buffer1[2 * BYTES_PER_LINE + 1];
                for (int i = 0; i < BYTES_PER_LINE; i++) {
                    sprintf(hex_buffer1 + 2 * i, "%02x", line1[i]);
                }
                char hex_buffer2[2 * BYTES_PER_LINE + 1];
                for (int i = 0; i < BYTES_PER_LINE; i++) {
                    sprintf(hex_buffer2 + 2 * i, "%02x", line2[i]);
                }
                // ESP_LOGI(TAG, "80:%s", hex_buffer1);
                // ESP_LOGI(TAG, "c0:%s", hex_buffer2);

                char payload[50];
                snprintf(payload, sizeof(payload), "80:%s", hex_buffer1);
                publishNodeProp("display", "value", payload);

                snprintf(payload, sizeof(payload), "c0:%s", hex_buffer2);
                publishNodeProp("display", "value", payload);
            }
            lastMs = nowMs;


            if(rs == 0) {
                // Command
                if(nibble == 0 || nibble > 1) {
                    nibble = 0;
                    cmd = 0;
                }
                if(nibble == 0) {
                    cmd = bits << 4;
                } else if(nibble == 1) {
                    cmd |= bits;
                    if((cmd & 0x80) > 0) {
                        // Set DDRAM address
                        addr = cmd & 0x7f;
                    }
                }
            } else {
                // Data
                if(nibble % 2 == 0) {
                    data = bits << 4;
                } else {
                    data |= bits;
                    if(addr < 0x28) {
                        line1[addr] = data;
                    }
                    if(addr > 0x40 && addr < 0x68) {
                        line2[addr - 0x40] = data;
                    }
                    addr++;
                }
            }

            nibble++;
         }
    }

    vTaskDelete(NULL);
}

static void subscribeTopics() {
    subscribeDevTopic("$update");
    subscribeDevTopic("relay/value/set");
}

static void handleMessage(const char* topic1, const char* topic2, const char* topic3, const char* data) {
    if(
        strcmp(topic1, "$update") == 0 && 
        topic2 == NULL && 
        topic3 == NULL
    ) {
        xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
    }

    if(
        strcmp(topic1, "relay") == 0 && 
        strcmp(topic2, "value") == 0 && 
        strcmp(topic3, "set") == 0
    ) {
        if(strcmp(data, "true") == 0) {
            gpio_set_level(GPIO_OUTPUT_RELAY, 1);
            publishNodeProp("relay", "value", "true");
        }
        if(strcmp(data, "false") == 0) {
            gpio_set_level(GPIO_OUTPUT_RELAY, 0);
            publishNodeProp("relay", "value", "false");
        }
    }
}

extern "C" void app_main() {

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Set relay to safe state (off)
    gpio_set_direction(GPIO_OUTPUT_RELAY, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_OUTPUT_RELAY, 0);

    // Initialize WiFi
    wifiStart();

    ESP_LOGI(TAG, "Waiting for wifi");
    wifiWait();

    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    mqttStart(subscribeTopics, handleMessage, NULL);

    ESP_LOGI(TAG, "Waiting for MQTT");

    mqttWait();

    publishNodeProp("relay", "value", "false");

    // Create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(96, sizeof(uint32_t));

    // Start gpio setup task, sets up the interrupt handler too
    xTaskCreatePinnedToCore(gpioSetupTask, "gpio_setup_task", 2048, NULL, 10, NULL, 1);

    // Start gpio task
    xTaskCreatePinnedToCore(gpioTask, "gpio_task", 2048, NULL, 10, NULL, 0);



    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

}
