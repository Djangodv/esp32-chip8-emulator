#include "KeypadControl.hpp"

static const char *TAG = "KeypadControl";

struct ButtonThreshold {
  int min;
  int max;
  Button button;
};

static const std::vector<ButtonThreshold> button_thresholds = {
    {0, 50, Button::LEFT},       {51, 300, Button::UP},
    {301, 600, Button::DOWN},    {601, 1000, Button::RIGHT},
    {1001, 1450, Button::START},
};

KeypadControl::KeypadControl(adc1_channel_t channel, gpio_num_t pin,
                             uint32_t interval_ms)
    : adc_channel_(channel), gpio_pin_(pin), interval_ms_(interval_ms),
      task_handle_(nullptr), running_(false) {

  adc1_config_width(ADC_WIDTH_BIT_11);
  adc1_config_channel_atten(adc_channel_, ADC_ATTEN_DB_12);

  ESP_LOGI(TAG, "Succesfully initialized ADC keypad");
}

KeypadControl::~KeypadControl() { stop(); }

void KeypadControl::start() {
  if (!running_) {
    running_ = true;
    xTaskCreatePinnedToCore(taskWrapper, "KeypadControlTask", 4096, this, 5,
                            &task_handle_, 1);
  }
}

void KeypadControl::stop() {
  if (running_) {
    running_ = false;
    if (task_handle_) {
      vTaskDelete(task_handle_);
      task_handle_ = nullptr;
    }
  }
}

void KeypadControl::assignButtonCallback(std::function<void(Button)> callback) {
  _buttonCallback = callback;
}

void KeypadControl::taskWrapper(void *pvParameters) {
  KeypadControl *reader = static_cast<KeypadControl *>(pvParameters);
  reader->run();
}

void KeypadControl::run() {
  Button lastButton = Button::NONE;

  while (running_) {
    int raw = adc1_get_raw(adc_channel_);
    Button currentButton = detectButton(raw);

    if (currentButton != lastButton) {
      ESP_LOGI(TAG, "Button Changed: %d (ADC Raw: %d)",
               static_cast<int>(currentButton), raw);
      lastButton = currentButton;
      if (_buttonCallback) {
        _buttonCallback(currentButton);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(interval_ms_));
  }
}

Button KeypadControl::detectButton(int raw_adc) {
  for (const ButtonThreshold &threshold : button_thresholds) {
    if (raw_adc >= threshold.min && raw_adc <= threshold.max) {
      return threshold.button;
    }
  }
  return Button::NONE;
}
