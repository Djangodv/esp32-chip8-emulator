#pragma once

#include "freertos/idf_additions.h"
#include "esp_log.h"
#include "esp_littlefs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <fstream>
#include <string>
#include <iostream>

#include "Memory.hpp"

class RomLoadingControl {

public:

  RomLoadingControl();
  ~RomLoadingControl();

	void loadRom(const std::string &filename);

  void start();
  void stop();

	// TODO: remove
	// void assignSetMemory(std::function<void(uint8_t address, uint8_t value)> setMemory);

	Memory _memory;

private:

	void init();

	// Memory _memory;

  TaskHandle_t _task_handle;
	bool _running;

  esp_vfs_littlefs_conf_t conf;

	static void taskWrapper(void* pvParameters);
	void run();

};
