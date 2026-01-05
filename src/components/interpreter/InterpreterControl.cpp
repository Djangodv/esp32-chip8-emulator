#include "InterpreterControl.hpp"

static const char *TAG = "InterpreterControl";

InterpreterControl::InterpreterControl()
    : _task_handle(nullptr), _running(false) {
  init();
};

InterpreterControl::~InterpreterControl() { stop(); }

void InterpreterControl::start() {
  if (!_running) {
    _running = true;
    xTaskCreatePinnedToCore(taskWrapper, "Test", 16384, this, 5, &_task_handle,
                            1);
  }
}

void InterpreterControl::stop() {
  if (_task_handle != nullptr) {
    _running = false;
    vTaskDelete(_task_handle);
    _task_handle = nullptr;
  }
}

void InterpreterControl::taskWrapper(void *pvParameters) {
  InterpreterControl *reader = static_cast<InterpreterControl *>(pvParameters);
  reader->run();
}

void InterpreterControl::run() {

  while (_running) {

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void InterpreterControl::init() {}
