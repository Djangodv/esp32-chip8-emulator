#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include <array>
#include <cstdint>
#include "Memory.hpp"

#include <sys/types.h>

class InterpreterControl {

public:
  InterpreterControl();
  ~InterpreterControl();

  void start();
  void stop();

	Memory memory;

  // NOTE: Has to be a static member, otherwise the task will end up crashing
  // static uint8_t memory[4096];

private:
  TaskHandle_t _task_handle;
  bool _running;

  static void taskWrapper(void *pvParameters);
  void run();
  void init();

};
