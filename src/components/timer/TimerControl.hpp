#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>
#include <functional>

class TimerControlInterface {

public:
  // virtual ~TimerControlInterface() = default;
  virtual void timerFinished() = 0;
};

class TimerControl {

public:
  TimerControl(TimerControlInterface *timerControlI);
  ~TimerControl();

  void start();
  void stop();

  void startTimer(uint8_t duration);
	uint8_t getExpiryTime();

private:

  TimerHandle_t xTimer;
	static constexpr float interval = 4250.0 / 255.0;

  TaskHandle_t _task_handle;
  bool _running;

  static void taskWrapper(void *pvParameters);
  void init();
  void run();

	// void assignStatic(TimerControlInterface* timerControlI);
	static void timerCallback(TimerHandle_t xTimer);

	static TimerControlInterface *_timerControlI;

};
