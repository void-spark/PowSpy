#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/apps/sntp.h"
#include "wifi_helper.h"
#include "mqtt_helper.h"

static const char *TAG = "app";


#define GPIO_INPUT_IO_RS    GPIO_NUM_27
#define GPIO_INPUT_IO_RW    GPIO_NUM_26
#define GPIO_INPUT_IO_E     GPIO_NUM_25

#define GPIO_INPUT_IO_D4    GPIO_NUM_18
#define GPIO_INPUT_IO_D5    GPIO_NUM_19
#define GPIO_INPUT_IO_D6    GPIO_NUM_22
#define GPIO_INPUT_IO_D7    GPIO_NUM_23

#define GPIO_INPUT_PIN_SEL ( \
    BIT(GPIO_INPUT_IO_RS) |  \
    BIT(GPIO_INPUT_IO_RW) |  \
    BIT(GPIO_INPUT_IO_E)  |  \
    BIT(GPIO_INPUT_IO_D4) |  \
    BIT(GPIO_INPUT_IO_D5) |  \
    BIT(GPIO_INPUT_IO_D6) |  \
    BIT(GPIO_INPUT_IO_D7)    \
)

#define ESP_INTR_FLAG_DEFAULT 0

#define BIT_TO_POS(value, from, to) (((value & BIT(from)) >> from) << to)

static xQueueHandle gpio_evt_queue = NULL;

// NOTES:
// - High level interrupts

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    // This needs to be fast, < 4 us, at least
    uint32_t gpioValues = (REG_READ(GPIO_IN_REG) & GPIO_INPUT_PIN_SEL);
    xQueueSendFromISR(gpio_evt_queue, &gpioValues, NULL);
}

// Configure the used gpio pins, and setup the interrupt handler
// Important: Interrupt handler will run on the core which calls this method.
static void setupGpio() {
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

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(96, sizeof(uint32_t));


    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_E, gpio_isr_handler, NULL);
}

static void gpioTask(void* arg) {

    setupGpio();

    uint32_t gpioValues = 0;
    uint32_t rs = 0;
    uint32_t rw = 0;
    uint32_t bits = 0;

    uint32_t nibble = 0;

    uint32_t pos = 0;
    uint8_t cmd = 0;
    uint8_t line[16] = {};

    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &gpioValues, portMAX_DELAY)) {
            rs = (gpioValues & BIT(GPIO_INPUT_IO_RS)) > 0;
            rw = (gpioValues & BIT(GPIO_INPUT_IO_RW)) > 0;

            if(rw != 0) {
                // Write: shouldn't happen, but ignore it (the bit is/might be on on bootup)
                continue;
            }

            bits = 0;
            bits |= BIT_TO_POS(gpioValues, GPIO_INPUT_IO_D7, 3);
            bits |= BIT_TO_POS(gpioValues, GPIO_INPUT_IO_D6, 2);
            bits |= BIT_TO_POS(gpioValues, GPIO_INPUT_IO_D5, 1);
            bits |= BIT_TO_POS(gpioValues, GPIO_INPUT_IO_D4, 0);

            if(rs == 0) {
                // Command
                if(nibble == 0 || nibble > 1) {
                    // Command resets everything, unless it's the second nibble of the first command.
                    nibble = 0;
                    pos = 0;
                    cmd = 0;
                }
                if(nibble == 0) {
                    cmd = bits << 4;
                } else if(nibble == 1) {
                    cmd |= bits;
                }
            } else {
                // Data
                if(pos == 16) {
                    // Ignore too much data
                    continue;
                }

                if(nibble % 2 == 0) {
                    line[pos] = bits << 4;
                } else {
                    line[pos] |= bits;
                    pos++;
                }

                if(pos == 16) {
                    char payload[50];
                    snprintf(payload, sizeof(payload), "%02x:%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", cmd,
                            line[0],  line[1],  line[2],  line[3],
                            line[4],  line[5],  line[6],  line[7],
                            line[8],  line[9],  line[10], line[11],
                            line[12], line[13], line[14], line[15]);
                    publishNodeProp("display", "value", payload);
                }
            }

            nibble++;
         }
    }

    vTaskDelete(NULL);
}

static void subscribeTopics() {
}

static void handleMessage(const char* topic1, const char* topic2, const char* topic3, const char* data) {
}

extern "C" void app_main() {

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi
    wifiStart();

    ESP_LOGI(TAG, "Waiting for wifi");
    wifiWait();

    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    mqttStart(subscribeTopics, handleMessage);

    ESP_LOGI(TAG, "Waiting for MQTT");

    mqttWait();


    //start gpio task
    xTaskCreatePinnedToCore(gpioTask, "gpio_task_example", 2048, NULL, 10, NULL, 1);



    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

}
