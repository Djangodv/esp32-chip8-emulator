#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MAIN";

extern "C" void app_main(void)
{

    ESP_LOGI(TAG, "Started running 'main' task");
    while (true) {
		vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
