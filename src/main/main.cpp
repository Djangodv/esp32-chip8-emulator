#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Ili9341Display.hpp"
#include <esp_random.h>

static const char* TAG = "MAIN";

extern "C" void app_main(void)
{

    ESP_LOGI(TAG, "Started running 'main' task");

	Ili9341Display display(13, 14, 15, 2, -1, 27); // MOSI, SCLK, CS, DC, RST, BCKL (BL on GPIO 27)
    display.init();
	ESP_LOGI(TAG, "Succesfully initialized display");
	// display.diagnostics();

	display.fillScreen(display.rbg565(0, 0, 0)); // clear the screen

}
