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

#include "driver/gpio.h"

#include <bit>
#include <bitset>

#include <KeypadControl.hpp>
// #include <SoundControl.hpp>>

static const char *TAG = "MAIN";

uint8_t memory[4096];
uint16_t pc;
uint16_t I;
uint16_t stack[16];
uint8_t sound, delay_;
uint8_t v[16];
uint8_t sp; // Stack pointer?

uint8_t keypad[16];

uint16_t opcode;

TimerHandle_t xTimer;
// TODO: Change to float (4080ms)
constexpr float interval = 4250.0 / 255.0;

constexpr gpio_num_t LED_GPIO = GPIO_NUM_21;

uint8_t fontset[80] = {
    0xF0, 0x90, 0x90, 0x90, 0xF0, // 0
    0x20, 0x60, 0x20, 0x20, 0x70, // 1
    0xF0, 0x10, 0xF0, 0x80, 0xF0, // 2
    0xF0, 0x10, 0xF0, 0x10, 0xF0, // 3
    0x90, 0x90, 0xF0, 0x10, 0x10, // 4
    0xF0, 0x80, 0xF0, 0x10, 0xF0, // 5
    0xF0, 0x80, 0xF0, 0x90, 0xF0, // 6
    0xF0, 0x10, 0x20, 0x40, 0x40, // 7
    0xF0, 0x90, 0xF0, 0x90, 0xF0, // 8
    0xF0, 0x90, 0xF0, 0x10, 0xF0, // 9
    0xF0, 0x90, 0xF0, 0x90, 0x90, // A
    0xE0, 0x90, 0xE0, 0x90, 0xE0, // B
    0xF0, 0x80, 0x80, 0x80, 0xF0, // C
    0xE0, 0x90, 0x90, 0x90, 0xE0, // D
    0xF0, 0x80, 0xF0, 0x80, 0xF0, // E
    0xF0, 0x80, 0xF0, 0x80, 0x80  // F
};

// Colors
uint16_t black;
uint16_t white;

Ili9341Display display(13, 14, 15, 2, -1,
                       27); // MOSI, SCLK, CS, DC, RST, BCKL (BL on GPIO 27)

void init() {

  pc = 0x200;

  // Load sprite data into memory
  for (int i = 0; i < sizeof(fontset); i++) {
    ESP_LOGE(TAG, "%X", fontset[i]);
    memory[0x50 + i] = fontset[i];
    ESP_LOGI(TAG, "%X: %X", 0x50 + i, memory[0x50 + i]);
  }

  sp = 0; // Initialize stack pointer to 0 (top of the stack)

  // Below lines could be deleted?
  display.fillScreen(black);
  display.present();

  // Source: https://randomnerdtutorials.com/esp-idf-esp32-blink-led/
  gpio_reset_pin(LED_GPIO);
  gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

  // TODO: Initialize all memory type values to zero
};

bool soundState = false;

void timerCallback(TimerHandle_t xTimer) {

  // Turn LED OFF
  if (soundState == true) {
    ESP_LOGW(TAG, "LED OFF");
    gpio_set_level(LED_GPIO, 0);
    // vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1 second
    soundState = false;
  }

  ESP_LOGW(TAG, "Timer finished");

  if (xTimerStop(xTimer, 0) != pdPASS) {
    ESP_LOGW(TAG, "Timer stop failed!");
  } else {
    ESP_LOGW(TAG, "Timer stopped succesfully");
  }
}

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
      // 00E0 (Clear screen)
    case 0x00E0:
      display.fillScreen(black);
      display.present();
      ESP_LOGE(TAG, "00E0");
      break;
    case 0x00EE:

      // uint16_t stack_value = stack[sp];

      // Sets PC to current value pointed to by stack pointer after 2NNN
      // subroutine has finished
      sp--;
      pc = stack[sp];

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
    ESP_LOGI(TAG, "Memory address: %X", opcode & 0x0FFF);
    ESP_LOGE(TAG, "1NNN");
    pc = opcode & 0x0FFF;
    ESP_LOGI(TAG, "Current PC: %X", pc);
    break;
  // 2NNN (call subroutine at NNN)
  case 0x2000:

    stack[sp] = pc;
    sp++;
    pc = opcode & 0x0FFF;

    ESP_LOGE(TAG, "2NNN");
    break;

  // 3XNN
  case 0x3000:

    // if (Vx == NN)
    if (v[(opcode & 0x0F00) >> 8] == (opcode & 0x00FF)) {
      ESP_LOGI(TAG, "PC: %d", pc);
      pc += 2;
      ESP_LOGI(TAG, "Expression true, incremented PC");
      ESP_LOGI(TAG, "PC: %d", pc);
    } else {
      ESP_LOGI(TAG, "Expression false, nothing happened");
    }

    ESP_LOGE(TAG, "3NNN: %d", (opcode & 0x0F00) >> 8);
    ESP_LOGE(TAG, "3NNN: %X", (opcode & 0x0F00) >> 8);
    ESP_LOGE(TAG, "3NNN");

    break;

  // 4XNN
  // Yet to be tested
  case 0x4000:

    if (v[(opcode & 0x0F00) >> 8] != (opcode & 0x00FF)) {
      pc += 2;
    }

    ESP_LOGE(TAG, "4NNN");
    break;
  // 5XY0
  case 0x5000:

    if (v[(opcode & 0x0F00) >> 8] == v[(opcode & 0x0F0) >> 4]) {
      pc += 2;
    }

    ESP_LOGE(TAG, "5NNN");
    break;
  // Set register (6XNN)
  case 0x6000:

    uint8_t reg;
    reg = (opcode & 0x0F00) >> 8;
    ESP_LOGI(TAG, "Vx: %X", reg);

    v[reg] = opcode & 0x00FF;

    ESP_LOGI(TAG, "Register %X: %X", reg, v[reg]);

    ESP_LOGE(TAG, "6XNN");
    break;
    // 7XNN
  case 0x7000:

    ESP_LOGI(TAG, "Register Vx: %d", v[(opcode & 0x0F00) >> 8]);
    v[(opcode & 0x0F00) >> 8] += (opcode & 0x00FF);

    ESP_LOGI(TAG, "Vx: %d", (opcode & 0x0F00) >> 8);
    ESP_LOGI(TAG, "NN: %d", opcode & 0x00FF);
    ESP_LOGI(TAG, "Register Vx: %d", v[(opcode & 0x0F00) >> 8]);

    ESP_LOGE(TAG, "7XNN");
    break;

  // 8XYN;
  case 0x8000:
    switch (opcode & 0x000F) {
    // 8XY0
    case 0x0000:
      ESP_LOGE(TAG, "Vx: %X", (opcode & 0x00F0) >> 4);
      ESP_LOGE(TAG, "Vx: %d", (opcode & 0x00F0) >> 4);
      v[(opcode & 0x0F00) >> 8] = v[(opcode & 0x00F0) >> 4];
      ESP_LOGE(TAG, "8XY0");
      break;
    // 8XY1
    case 0x0001:
      v[(opcode & 0x0F00) >> 8] |= v[(opcode & 0x00F0) >> 4];
      ESP_LOGE(TAG, "8XY0");
      break;
    // 8XY2
    case 0x0002:
      v[(opcode & 0x0F00) >> 8] &= v[(opcode & 0x00F0) >> 4];
      ESP_LOGE(TAG, "8XY0");
      break;
    // 8XY3
    case 0x0003:
      v[(opcode & 0x0F00) >> 8] ^= v[(opcode & 0x00F0) >> 4];
      ESP_LOGE(TAG, "8XY0");
      break;
    // 8XY4
    case 0x0004:
      uint16_t val;
      val = v[(opcode & 0x0F00) >> 8] + v[(opcode & 0x00F0) >> 4];

      v[(opcode & 0x0F00) >> 8] = 0xFF & val;

      if (val > 255) {
        v[0xf] = 1;
        ESP_LOGE(TAG, "Test: %d", 0xf);
      } else {
        v[0x0f] = 0;
      }

      break;
    case 0x0005:

      if (v[(opcode & 0x0F00) >> 8] > v[(opcode & 0x00F0) >> 4]) {
        v[0x0f] = 1;
      } else {
        v[0x0f] = 0;
      }

      v[(opcode & 0x0F00) >> 8] -= v[(opcode & 0x00F0) >> 4];

      break;

    case 0x0006:
      // Check whether the LSB of current register Vx is 1 or 0
      v[0x0f] = v[(opcode & 0x0F00) >> 8] & 0x1;

      v[(opcode & 0x0F00) >> 8] >>= 1;
      ESP_LOGE(TAG, "TEST1: %X", v[(opcode & 0x0F00) >> 8]);
      ESP_LOGE(TAG, "TEST2: %X", v[(opcode & 0x0F00) >> 8] & 0x1);
      break;

    // UNTESTED
    case 0x0007:

      ESP_LOGE(TAG, "TEST1: %X", v[(opcode & 0x0F00) >> 8]);
      ESP_LOGE(TAG, "TEST2: %X", v[(opcode & 0x0F00) >> 8] & 0x1);

      if (v[(opcode & 0x0F00) >> 8] < v[(opcode & 0x00F0) >> 4]) {
        v[0xf] = 1;
      } else {
        v[0xf] = 0;
      }

      v[(opcode & 0x0F00) >> 8] =
          v[(opcode & 0x00F0) >> 4] - v[(opcode & 0x0F00) >> 8];

      break;

    case 0x000E:

      v[0x0f] = v[(opcode & 0x0F00) >> 8] & 0x80;

      v[(opcode & 0x0F00) >> 8] <<= 1;

      break;
    }
    break;

  // 9XY0
  case 0x9000:

    if (v[(opcode & 0x0F00) >> 8] != v[(opcode & 0x0F0) >> 4]) {
      ESP_LOGI(TAG, "PC: %d", pc);
      pc += 2;
      ESP_LOGI(TAG, "Expression true, incremented PC");
      ESP_LOGI(TAG, "PC: %d", pc);
    } else {
      ESP_LOGI(TAG, "Expression false, nothing happened");
    }

    ESP_LOGE(TAG, "Vx: %X", (opcode & 0x00F0) >> 4);
    ESP_LOGE(TAG, "Vx: %d", (opcode & 0x00F0) >> 4);

    ESP_LOGE(TAG, "5NNN");

    break;
    // ANNN
  case 0xA000:
    ESP_LOGE(TAG, "ANNN");
    I = opcode & 0x0FFF;
    ESP_LOGI(TAG, "NNN: %X", opcode & 0x0FFF);
    ESP_LOGI(TAG, "Register I: %X", I);
    break;
  // BNNN
  case 0xB000:

    pc = (opcode & 0x0FFF) + v[0];

    break;
  // CXNN
  case 0xC000:
    ESP_LOGE(TAG, "Random number: %d", rand() % 256);
    v[(opcode & 0x0F00) >> 8] = (rand() % 256) & (opcode & 0x00FF);
    break;
    // Most involved instruction for drawing a sprite on the screen
  case 0xD000:
    ESP_LOGE(TAG, "DXYN");

    uint8_t height;
    height = opcode & 0x000F;

    ESP_LOGE(TAG, "N: %X", height);

    uint8_t x, y;

    x = v[(opcode & 0x0F00) >> 8];
    y = v[(opcode & 0x00F0) >> 4];

    ESP_LOGE(TAG, "Vx: %d", x);
    ESP_LOGE(TAG, "Vy: %d", y);

    // TEST:
    // std::bitset<8> byte_;
    uint8_t byte;

    // First loop over the height (h)
    for (int h = 0; h < height; h++) {

      byte = memory[I + h];

      ESP_LOGI(TAG, "Data in memory at %d: %X", I + h, memory[I + h]);
      // TEST:
      // byte_ = memory[I + h];
      // std::cout << byte_ << std::endl;

      // Width of the sprite to be drawn
      for (int w = 0; w < 8; w++) {
        // Check which of bits in the byte of the sprite are set
        // Invert the operation by starting on the left-side of the byte,
        // because of big-endiannes (else the sprites will be drawn in a mirror
        // image) Cause: if ((byte >> i) & 0x1) {
        if ((byte << w) & 0x80) {
          display.drawPixel(x + w, y + h, white);
        }
      }
    }

    display.present();

    break;
  case 0xE000:
    switch (opcode & 0x00FF) {
    // EX9E
    case 0x009E:
      if (keypad[v[(opcode & 0x0F00) >> 8]]) {
        pc += 2;
      }
      ESP_LOGE(TAG, "");
      break;
    // EXA1
    case 0x00A1:
      ESP_LOGE(TAG, "Register Vx: %X", v[(opcode & 0x0F00) >> 8]);
      ESP_LOGE(TAG, "Register Vx: %X", keypad[v[(opcode & 0x0F00) >> 8]]);
      if (keypad[v[(opcode & 0x0F00) >> 8]] == 0) {
        pc += 2;
      }
      ESP_LOGE(TAG, "EX0A");
      break;
    }
    break;
  case 0xF000:
    switch (opcode & 0x00FF) {
    // TODO: implement opcode
    case 0x0007:

      TickType_t expiryTime;

      expiryTime = pdTICKS_TO_MS(xTimerGetExpiryTime(xTimer) - xTaskGetTickCount()) / interval;
      ESP_LOGI(TAG, "Timer expiry time: %d ticks", static_cast<uint8_t>(expiryTime));

      v[(opcode & 0x0F00) >> 8] = static_cast<uint8_t>(expiryTime);

      break;
    case 0x000A:

      bool key_pressed;
      key_pressed = false;

      for (int i = 0; i < 16; i++) {
        if (keypad[i]) {
          v[(opcode & 0x0F00) >> 8] = i;
          key_pressed = true;
        }
      }

      if (key_pressed != true) {
        pc -= 2;
      }

      break;
    case 0x0015:

      v[(opcode & 0x0F00) >> 8] = 255;

      uint8_t duration;
      duration = v[(opcode & 0x0F00) >> 8];

      ESP_LOGW(TAG, "Duration: %d", duration);

      // SOURCE: https://saludpcb.com/esp32-tutorial-using-xtimer/
      // Start timer instantly (with delay of 0)
      if (xTimerStart(xTimer, interval) != pdPASS) {
        ESP_LOGE(TAG, "Timer start failed!");
      } else {
        ESP_LOGE(TAG, "Timer start succesful!");
      }

      ESP_LOGW(TAG, "Interval * duration: %f", interval * duration);

      // Below check is needed to prevent a crash when duration is 0, which
      // doesn't give enough time to timerCallback() to finish
      if (duration != 0) {
        // Change period of timer using above calculation
        if (xTimerChangePeriod(xTimer, pdMS_TO_TICKS(interval * duration), 0) !=
            pdPASS) {
          ESP_LOGE(TAG, "Timer change period failed!");
        }
      }

      break;

    case 0x0018:

      // NOTE: Is this correct?
      // Calculation for amount of timer ms:
      // The timer is one byte large (max. of 255) and because the timer is
      // decremented by 60 each seconds you get 255 / 60 = 4.25s. Converted to
      // ms that is 4250ms if the timer gets set to the maximum value of a byte.
      // 4250 / 255 = ~16.67, because (4250 / 255) * 255 = 4250ms. So each value
      // of 1 inside the byte is exactly ~16.67ms long amounting to a maximum
      // duration of 4.25 seconds.

      duration = v[(opcode & 0x0F00) >> 8];

      ESP_LOGW(TAG, "Duration: %d", duration);
      ESP_LOGW(TAG, "Interval: %f", interval);

      // SOURCE: https://saludpcb.com/esp32-tutorial-using-xtimer/
      // Start timer instantly (with delay of 0)
      if (xTimerStart(xTimer, interval) != pdPASS) {
        ESP_LOGE(TAG, "Timer start failed!");
      } else {
        ESP_LOGE(TAG, "Timer start succesful!");
      }

      // Below check is needed to prevent a crash when duration is 0, which
      // doesn't give enough time to timerCallback() to finish
      if (duration != 0) {
        // Change period of timer using above calculation
        if (xTimerChangePeriod(xTimer, pdMS_TO_TICKS(interval) * duration, 0) !=
            pdPASS) {
          ESP_LOGE(TAG, "Timer change period failed!");
        }
      }

      // Turn LED ON
      ESP_LOGW(TAG, "LED ON");
      gpio_set_level(LED_GPIO, 1);
      soundState = true;
      // vTaskDelay(pdMS_TO_TICKS(100)); // Delay 0.1 second

      break;
    }
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

  std::ifstream Rom("/littlefs/Beep.ch8", std::ios::binary);

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

  // Create a software timer
  xTimer = xTimerCreate(
      "Timer",                 // Timer name
      pdMS_TO_TICKS(interval), // Timer period in ticks (milliseconds to ticks)
      pdTRUE,                  // Auto-reload flag
      (void *)0,               // Timer ID
      timerCallback            // Callback function
  );

  if (xTimer == NULL) {
    ESP_LOGE(TAG, "Timer creation failed!");
    return;
  }

  black = 0xFFFF;
  white = 0x0000;

  init();
  init_littlefs();
  load_rom();

  // GPIO 35, 3.3V
  KeypadControl controller(ADC1_CHANNEL_7, GPIO_NUM_35, 50);

  controller.assignButtonCallback([&](Button button) {
    switch (button) {
    case Button::NONE:
      for (int i = 0; i < 16; i++) {
        // ESP_LOGI(TAG, "Key: %d", keypad[i]);
        keypad[i] = 0;
      }

      ESP_LOGI(TAG, "Button: NONE");
      break;
    case Button::START:
      keypad[0xb] = 1;
      ESP_LOGI(TAG, "Button: START");
      break;
    case Button::LEFT:
      keypad[4] = 1;
      ESP_LOGI(TAG, "Button: LEFT");
      break;
    case Button::RIGHT:
      keypad[6] = 1;
      ESP_LOGI(TAG, "Button: RIGHT");
      break;
    case Button::UP:
      keypad[5] = 1;
      ESP_LOGI(TAG, "Button: UP");
      break;
    case Button::DOWN:
      keypad[0xe] = 1;
      ESP_LOGI(TAG, "Button: DOWN");
      break;
    }
  });

  display.init();
  ESP_LOGI(TAG, "Initialized display");
  display.fillScreen(black); // Test
  display.present();

  TickType_t period;

  while (true) {
    // Below line causes issues due to calling fetch twice, thereby incrementing
    // PC twice ESP_LOGE(TAG, "Current instruction in memory: %X", fetch());
    execute();
    controller.start();
    vTaskDelay(pdMS_TO_TICKS(100));

    // period = xTimerGetPeriod(xTimer);
    // ESP_LOGI(TAG, "Timer expiry time: %d ticks", static_cast<int>(period));
  }
}
