#include "freertos/idf_additions.h"
#include "esp_log.h"
#include "esp_littlefs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <fstream>
#include <string>
#include <iostream>

#include "InterpreterControl.hpp"

class RomLoadingControl {

public:

  RomLoadingControl(InterpreterControl& interpreterControl);
  ~RomLoadingControl();

	void loadRom(const std::string &filename);

  void start();
  void stop();

private:

	void init();

  TaskHandle_t _task_handle;
	bool _running;

  esp_vfs_littlefs_conf_t conf;

	static void taskWrapper(void* pvParameters);
	void run();

	InterpreterControl& _interpreterControl;

};
