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

void TimerControl::startTimer(uint8_t duration) {

  // The timer is one byte large (max. of 255) and because the timer is
  // decremented by 60 each seconds you get 255 / 60 = 4.25s. Converted to
  // ms that is 4250ms if the timer gets set to the maximum value of a byte.
  // 4250 / 255 = ~16.67, because (4250 / 255) * 255 = 4250ms. So each value
  // of 1 inside the byte is exactly ~16.67ms long amounting to a maximum
  // duration of 4.25 seconds.

  // SOURCE: https://saludpcb.com/esp32-tutorial-using-xtimer/
  // Start timer almost instantly (with small delay to avoid errors)
  if (xTimerStart(xTimer, interval) != pdPASS) {
    ESP_LOGE(TAG, "Timer start failed!");
  } else {
    ESP_LOGE(TAG, "Timer start succesful!");
  }

  // Below check is needed to prevent a crash when duration is 0, which
  // doesn't give enough time to timerCallback() to finish
  if (duration != 0) {
    // Change period of timer using above calculation
    if (xTimerChangePeriod(xTimer, pdMS_TO_TICKS(interval * duration), 0) !=
        pdPASS) {
      ESP_LOGE(TAG, "Timer change period failed!");
    }
  }
}

uint8_t TimerControl::getExpiryTime() {

  TickType_t expiryTime;

  expiryTime =
      pdTICKS_TO_MS(xTimerGetExpiryTime(xTimer) - xTaskGetTickCount()) /
      interval;
  ESP_LOGI(TAG, "Timer expiry time: %d ticks",
           static_cast<uint8_t>(expiryTime));

  return static_cast<uint8_t>(expiryTime);
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

  ESP_LOGW(TAG, "Succesfully created timer!");
}

void TimerControl::run() {

  ESP_LOGI(TAG, "Started running '%s' task", TAG);

  while (_running) {
    vTaskDelay(pdMS_TO_TICKS(200));
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

TimerControlInterface *TimerControl::_timerControlI = nullptr;
