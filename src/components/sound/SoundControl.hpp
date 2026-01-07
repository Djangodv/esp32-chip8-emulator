#pragma once

#include "driver/gpio.h"
#include "esp_log.h"
#include "soc/gpio_num.h"

class SoundControl {

public:
  SoundControl(const gpio_num_t gpio_num);
  void playSound();
  void stopSound();

private:
  void init();
  gpio_num_t _gpio_pin;
};
