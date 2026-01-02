#pragma once

#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <functional>
#include <vector>

enum class Button { NONE, START, LEFT, RIGHT, UP, DOWN };

class KeypadListener {

public:
  // virtual ~KeypadListener() = default;
  virtual void buttonPressed(Button buttonId) = 0;
};

class KeypadControl {

public:
  KeypadControl(adc1_channel_t channel, gpio_num_t pin, uint32_t interval_ms);
  ~KeypadControl();

  void start();
  void stop();

  void assignButtonCallback(std::function<void(Button)> callback);

private:
  adc1_channel_t adc_channel_;
  gpio_num_t gpio_pin_;
  uint32_t interval_ms_;
  TaskHandle_t task_handle_;
  bool running_;

  static void taskWrapper(void *pvParameters);
  void run();
  Button detectButton(int raw_adc);

  std::function<void(Button)> _buttonCallback;
};
