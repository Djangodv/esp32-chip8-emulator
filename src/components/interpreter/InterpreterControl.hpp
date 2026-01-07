#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include <array>
#include <cstdint>

#include "Memory.hpp"
#include "SoundControl.hpp"
#include "TimerControl.hpp"

#include <sys/types.h>

#include "GraphicsControl.hpp"
#include "RomLoadingControl.hpp"
#include "KeypadControl.hpp"

class InterpreterControl : public TimerControlInterface,
                           public GraphicsControlInterface, public KeypadListener {

public:
  InterpreterControl(const gpio_num_t sound_pin);
  ~InterpreterControl();

  void start();
  void stop();

  Memory memory;

  void timerFinished() override;
  void setCollision(bool state) override;
	void buttonPressed(Button buttonId) override;
  // NOTE: Has to be a static member, otherwise the task will end up crashing
  // static uint8_t memory[4096];

private:
  static void taskWrapper(void *pvParameters);
  void run();
  void init();

  uint16_t fetch();
  void execute();

  RomLoadingControl romLoadingControl;
  GraphicsControl graphicsControl;
  TimerControl timerControl;
  SoundControl soundControl;
	KeypadControl keypadControl;

  uint16_t pc;
  uint16_t I;
  uint16_t stack[16];
  uint8_t sound, delay_;
  uint8_t sp; // Stack pointer?
  uint8_t v[16];

  uint8_t keypad[16];

  uint16_t opcode;

  TaskHandle_t _task_handle;
  bool _running;
  const gpio_num_t _sound_pin;

  // Colors
  uint16_t black;
  uint16_t white;
};
