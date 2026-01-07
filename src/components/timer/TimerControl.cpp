#include "TimerControl.hpp"

static const char *TAG = "TimerControl";

TimerControl::TimerControl(TimerControlInterface *timerControlI)
    : _task_handle(nullptr), _running(false) {

	_timerControlI = timerControlI;

  init();
}
TimerControl::~TimerControl() { stop(); }

void TimerControl::start() {
  if (!_running) {
    _running = true;
    // 'this' in the function arguments is what is passed on to taskWrapper to
    // void* pvParameters
    xTaskCreatePinnedToCore(taskWrapper, TAG, 4096, this, 5, &_task_handle, 0);
  }
}

void TimerControl::stop() {
  if (_task_handle != nullptr) {
    _running = false;
    vTaskDelete(_task_handle);
    _task_handle = nullptr;
  }
}

// A task wrapper is a design pattern used to encapsulate a task or function
// that will be executed by a seperate thread or task scheduler 'void*' is a
// pointer (memory address) that can point to any data type
void TimerControl::taskWrapper(void *pvParameters) {
  // The void* pointer is cast back to the original type with static_cast
  TimerControl *reader = static_cast<TimerControl *>(pvParameters);
  reader->run();
}

void TimerControl::init() {

  ESP_LOGI(TAG, "Succesfully initialized '%s'", TAG);

  // Create a software timer
  xTimer = xTimerCreate(
      "Timer",                 // Timer name
      pdMS_TO_TICKS(interval), // Timer period in ticks (milliseconds to ticks)
      pdTRUE,                  // Auto-reload flag
      (void *)0,               // Timer ID
      timerCallback            // Callback function
  );

  if (xTimer == NULL) {
    ESP_LOGE(TAG, "Timer creation failed!");
    return;
  }
}

void TimerControl::run() {

  ESP_LOGI(TAG, "Started running '%s' task", TAG);

  while (_running) {
    vTaskDelay(pdMS_TO_TICKS(300)); // wait 300 ms
  }

}

void TimerControl::timerCallback(TimerHandle_t xTimer) {

	if (_timerControlI) {
		_timerControlI->timerFinished();
	}

  ESP_LOGW(TAG, "Timer finished");

  if (xTimerStop(xTimer, 0) != pdPASS) {
    ESP_LOGW(TAG, "Timer stop failed!");
  } else {
    ESP_LOGW(TAG, "Timer stopped succesfully");
  }
}

TimerControlInterface* TimerControl::_timerControlI = nullptr;
