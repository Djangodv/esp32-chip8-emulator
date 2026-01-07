#include "RomLoadingControl.hpp"

// using namespace std;

static const char *TAG = "RomLoadingControl";

RomLoadingControl::RomLoadingControl()
    : _memory(), _task_handle(nullptr), _running(false) {

  init();
};

RomLoadingControl::~RomLoadingControl() { stop(); };

void RomLoadingControl::loadRom(const std::string &filename) {

  std::ifstream Rom(filename, std::ios::binary);

  if (!Rom.is_open()) {
    ESP_LOGE(TAG, "Unable to open file");
  } else {
    ESP_LOGD(TAG, "Succesfully opened .ch8 ROM");
  }

  // Seek to end of the Rom to determine ROM size
  Rom.seekg(0, std::fstream::end);
  size_t size = Rom.tellg();
  Rom.seekg(0, std::fstream::beg);

  ESP_LOGE(TAG, "ROM size: %zu", size);

  // Allocate a temporary buffer to store the ROM contents the size of the file
  char buffer[size];
  Rom.read(buffer, size);

  for (int i = 0; i < size; i++) {
    _memory.setMemory((0x200 + i), buffer[i]);
    ESP_LOGD(TAG, "Byte %d: %X", i, buffer[i]);
    ESP_LOGD(TAG, "Byte %d: %X", i, _memory.getMemory(0x200 + i));
		// Small delay to avoid watchdog
		vTaskDelay(pdMS_TO_TICKS(5));
  }

  ESP_LOGE(TAG, "Loaded file: %s into memory", filename.c_str());

  // TODO: Update to work with ESP-IDF
  // Some simple tests for determining whether the output is correct
  // uint16_t opcode;
  // cout << endl << bitset<16>(opcode);
  // opcode = fetch();
  // cout << endl << bitset<16>(opcode);
  // opcode = opcode & 0xF000;
  // cout << endl << bitset<16>(opcode);

  // Don't forget to close the file after use
  Rom.close();

  // All done, unmount partition and disable LittleFS
  esp_vfs_littlefs_unregister(conf.partition_label);
  ESP_LOGI(TAG, "LittleFS unmounted");
}

void RomLoadingControl::start() {
  if (!_running) {
    _running = true;
    xTaskCreatePinnedToCore(taskWrapper, TAG, 4096, this, 5, &_task_handle, 0);
  }
}

void RomLoadingControl::stop() {
  if (_task_handle != nullptr) {
    _running = false;
    vTaskDelete(_task_handle);
    _task_handle = nullptr;
  }
}

void RomLoadingControl::init() {

  ESP_LOGD(TAG, "Initializing LittleFS");

  // SOURCE:
  // https://github.com/espressif/esp-idf/tree/master/examples/storage/littlefs
  conf = {.base_path = "/littlefs",
          .partition_label = "storage",
          .format_if_mount_failed = true,
          .dont_mount = false};

  // Use settings defined above to initialize and mount LittleFS filesystem.
  // Note: esp_vfs_littlefs_register is an all-in-one convenience function.
  esp_err_t ret = esp_vfs_littlefs_register(&conf);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE(TAG, "Failed to mount or format filesystem");
    } else if (ret == ESP_ERR_NOT_FOUND) {
      ESP_LOGE(TAG, "Failed to find LittleFS partition");
    } else {
      ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
    }
    return;
  }

  size_t total = 0, used = 0;
  ret = esp_littlefs_info(conf.partition_label, &total, &used);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)",
             esp_err_to_name(ret));
    esp_littlefs_format(conf.partition_label);
  } else {
    ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
  }
}

void RomLoadingControl::taskWrapper(void *pvParameters) {
  RomLoadingControl *reader = static_cast<RomLoadingControl *>(pvParameters);
  reader->run();
}

void RomLoadingControl::run() {}
