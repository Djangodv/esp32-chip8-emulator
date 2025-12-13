#include "Ili9341Display.hpp"
#include "esp_err.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>
#include <esp_random.h>

#include <fstream>
#include <iostream>
#include <unistd.h>

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include <bit>

static const char *TAG = "MAIN";

uint8_t memory[4096];
uint16_t pc;
uint16_t index_;
uint16_t stack[16];
uint8_t sound, delay;
uint8_t v[16];
uint8_t sp; // Stack pointer?

uint16_t opcode;

// Colors
uint16_t black;
uint16_t white;

Ili9341Display display(13, 14, 15, 2, -1,
                       27); // MOSI, SCLK, CS, DC, RST, BCKL (BL on GPIO 27)

void init() {

  pc = 0x200;

  // TODO: Initialize all memory type values to zero
};

uint16_t fetch() {

  uint16_t instruction;

  // Each memory address holds exactly one byte, an opcode is two bytes long.
  // Recall that Chip-8 uses big-endian to store opcodes First take the second
  // byte starting from the PC and shift it 8 bits to left, resulting in
  // 1111111100000000 after which you compare the current address of the PC with
  // the bitwise OR operator (|) setting the second part of the full opcode
  instruction = memory[pc] << 8 | memory[pc + 1]; // 0x200

	// TEST:
	// ESP_LOGI(TAG, "Instruction: %X", instruction);

  pc += 2;

  return instruction;
};

void execute() {

  opcode = fetch();

  ESP_LOGE(TAG, "Current instruction in memory: %X", opcode);

  switch (opcode & 0xF000) { // & 0xF000
  case 0x0000:
    switch (opcode & 0xFFFF) {
    // Clear screen (00E0)
    case 0x00E0:
      display.fillScreen(black);
      ESP_LOGE(TAG, "00E0");
      break;
    case 0x00EE:
      ESP_LOGE(TAG, "00EE");
      break;
      // Test for testing
      // case 0x0C60:
      //   ESP_LOGE(TAG, "0C60");
      //   break;
    }
    break;
  // Set PC to location NNN (1NNN)
  case 0x1000:
    ESP_LOGE(TAG, "Memory address: %X", opcode & 0x0FFF);
    ESP_LOGE(TAG, "1NNN");
    pc = opcode & 0x0FFF;
    ESP_LOGE(TAG, "Current PC: %X", pc);
    break;
  // Set register (6XNN)
  case 0x6000:

    uint16_t test;
    test = opcode & 0x0F00;
    ESP_LOGI(TAG, "Vx: %X", test);

    ESP_LOGE(TAG, "6XNN");
    break;
  case 0x7000:
    ESP_LOGE(TAG, "7XNN");
    break;
  case 0xA000:
    ESP_LOGE(TAG, "ANNN");
    break;
  case 0xD000:
    ESP_LOGE(TAG, "DXYN");
    break;
  default:
    ESP_LOGE(TAG, "No opcode implementation yet");
  }
};

void init_littlefs() {

  ESP_LOGI(TAG, "Initializing LittleFS");

  esp_vfs_littlefs_conf_t conf = {.base_path = "/littlefs",
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

  // All done, unmount partition and disable LittleFS
  // esp_vfs_littlefs_unregister(conf.partition_label);
  // ESP_LOGI(TAG, "LittleFS unmounted");
}

void load_rom() {

  std::ifstream Rom("/littlefs/IBM Logo.ch8", std::ios::binary);

  if (!Rom.is_open()) {
    ESP_LOGE(TAG, "Unable to open file");
  } else {
    ESP_LOGI(TAG, "Succesfully opened a .ch8 ROM");
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
    ESP_LOGE(TAG, "Byte %d: %X", i, buffer[i]);
    // TODO: Use PC instead of 0x200
    memory[0x200 + i] = buffer[i];
  }

  // Some simple tests for determining whether the output is correct
  // uint16_t opcode;
  // cout << endl << bitset<16>(opcode);
  // opcode = fetch();
  // cout << endl << bitset<16>(opcode);
  // opcode = opcode & 0xF000;
  // cout << endl << bitset<16>(opcode);

  // Don't forget to close the file after use
  Rom.close();
};

extern "C" void app_main(void) {

  ESP_LOGI(TAG, "Started running 'main' task");

  black = 0xFFFF;
  white = 0x0000;

  init();
  init_littlefs();
  load_rom();

  display.init();
  ESP_LOGI(TAG, "Initialized display");
  display.fillScreen(white); // Test

  while (true) {
    // Below line causes issues due to calling fetch twice, thereby incrementing PC twice
    // ESP_LOGE(TAG, "Current instruction in memory: %X", fetch());
    execute();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
