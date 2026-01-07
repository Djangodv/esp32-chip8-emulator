#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "InterpreterControl.hpp"
#include "RomLoadingControl.hpp"

static const char *TAG = "Main";

extern "C" void app_main(void) {

	// Uncomment the below line to enable debug loggings
	// esp_log_level_set("*", ESP_LOG_INFO);   

  ESP_LOGI(TAG, "Started running main task");

	InterpreterControl interpreterControl("/littlefs/Corax.ch8", GPIO_NUM_21);
	interpreterControl.start();

	// RomLoadingControl romLoadingControl(interpreterControl);
	// romLoadingControl.loadRom("/littlefs/Corax.ch8");
	// romLoadingControl.start();

  // Infinte loop that prevents task deletion, because a FreeRTOS task is
  // automatically deleted after execution.
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
