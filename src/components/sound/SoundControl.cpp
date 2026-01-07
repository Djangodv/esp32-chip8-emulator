#include "SoundControl.hpp"

static const char *TAG = "SoundControl";

SoundControl::SoundControl(const gpio_num_t gpio_pin) : _gpio_pin(gpio_pin) {

  init();
};

void SoundControl::playSound() {

  ESP_LOGD(TAG, "playSound() executed succesfully");
  gpio_set_level(_gpio_pin, 1);
};

void SoundControl::stopSound() {

  ESP_LOGD(TAG, "stopSound() executed succesfully");
  gpio_set_level(_gpio_pin, 0);
};

void SoundControl::init() {

  // Source: https://randomnerdtutorials.com/esp-idf-esp32-blink-led/
  gpio_reset_pin(_gpio_pin);
  gpio_set_direction(_gpio_pin, GPIO_MODE_INPUT_OUTPUT);

}
