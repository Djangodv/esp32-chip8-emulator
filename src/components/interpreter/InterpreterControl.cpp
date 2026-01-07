#include "InterpreterControl.hpp"

static const char *TAG = "InterpreterControl";

InterpreterControl::InterpreterControl()
    : graphicsControl(13, 14, 15, 2, -1, 27), timerControl(this),
      soundControl(GPIO_NUM_21), _task_handle(nullptr), _running(false) {
  init();
};

InterpreterControl::~InterpreterControl() { stop(); }

void InterpreterControl::start() {
  if (!_running) {
    _running = true;
    xTaskCreatePinnedToCore(taskWrapper, "Test", 16384, this, 5, &_task_handle,
                            1);
  }
}

void InterpreterControl::stop() {
  if (_task_handle != nullptr) {
    _running = false;
    vTaskDelete(_task_handle);
    _task_handle = nullptr;
  }
}

void InterpreterControl::taskWrapper(void *pvParameters) {
  InterpreterControl *reader = static_cast<InterpreterControl *>(pvParameters);
  reader->run();
}

void InterpreterControl::run() {

  ESP_LOGI(TAG, "Started running '%s' task", TAG);

  while (_running) {

    // vTaskDelay(pdMS_TO_TICKS(2));
    vTaskDelay(pdMS_TO_TICKS(1000));

    execute();
  }
}

void InterpreterControl::init() {

  black = 0xFFFF;
  white = 0x0000;

  pc = 0x200;

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

  // Load sprite data into memory
  for (int i = 0; i < sizeof(fontset); i++) {
    ESP_LOGW(TAG, "%X", fontset[i]);
    memory.setMemory(0x50 + i, fontset[i]);
    ESP_LOGW(TAG, "%X: %X", 0x50 + i, memory.getMemory(0x50 + i));
  }

  sp = 0; // Initialize stack pointer to 0 (top of the stack)

  graphicsControl.init();

  graphicsControl.fillScreen(white);
  graphicsControl.present();

  // Load keypad test 3 automatically (temporary)
  // memory[0x1FF] = 3;

  // TODO: Initialize all memory type values to zero
}

uint16_t InterpreterControl::fetch() {

  uint16_t instruction;

  // Each memory address holds exactly one byte, an opcode is two bytes long.
  // Recall that Chip-8 uses big-endian to store opcodes First take the second
  // byte starting from the PC and shift it 8 bits to left, resulting in
  // 1111111100000000 after which you compare the current address of the PC with
  // the bitwise OR operator (|) setting the second part of the full opcode
  instruction = memory.getMemory(pc) << 8 | memory.getMemory(pc + 1); // 0x200

  // TEST:
  // ESP_LOGW(TAG, "Instruction: %X", instruction);

  pc += 2;

  return instruction;
};

void InterpreterControl::execute() {

  opcode = fetch();

  ESP_LOGW(TAG, "Current instruction in memory: %X", opcode);

  switch (opcode & 0x0F000) {

  case 0x0000:
    switch (opcode & 0xFFFF) {
      // 00E0 (Clear screen)
    case 0x00E0:
      graphicsControl.fillScreen(black);
      graphicsControl.present();
      break;
      // 00EE (Set PC to current value pointed to by stack pointer)
    case 0x00EE:
      sp--;
      pc = stack[sp];
      break;
    }
    break;
  // 1NNN (Set PC to location NNN)
  case 0x1000:
    ESP_LOGW(TAG, "NNN: %X", opcode & 0x0FFF);
    pc = opcode & 0x0FFF;
    ESP_LOGW(TAG, "Current PC: %X", pc);
    break;
  // 2NNN (Call subroutine at NNN)
  case 0x2000:
    stack[sp] = pc;
    sp++;
    pc = opcode & 0x0FFF;
    break;
  // 3XNN (Skips instruction if Vx == NN)
  case 0x3000:
    if (v[(opcode & 0x0F00) >> 8] == (opcode & 0x00FF)) {
      ESP_LOGW(TAG, "PC: %d", pc);
      pc += 2;
      ESP_LOGW(TAG, "Expression true, incremented PC");
      ESP_LOGW(TAG, "PC: %d", pc);
    } else {
      ESP_LOGW(TAG, "Expression false, nothing happened");
    }
    ESP_LOGW(TAG, "3NNN: %d", (opcode & 0x0F00) >> 8);
    ESP_LOGW(TAG, "3NNN: %X", (opcode & 0x0F00) >> 8);
    break;
  // 4XNN ()
  case 0x4000:
    if (v[(opcode & 0x0F00) >> 8] != (opcode & 0x00FF)) {
      pc += 2;
    }
    break;
  // 5XY0 ()
  case 0x5000:
    if (v[(opcode & 0x0F00) >> 8] == v[(opcode & 0x0F0) >> 4]) {
      pc += 2;
    }
    break;
  // 6XNN (Set register)
  case 0x6000:
    uint8_t reg;
    reg = (opcode & 0x0F00) >> 8;
    ESP_LOGW(TAG, "Vx: %X", reg);
    v[reg] = opcode & 0x00FF;
    ESP_LOGW(TAG, "Register %X: %X", reg, v[reg]);
    break;
	// 7XNN ()
  case 0x7000:
    ESP_LOGW(TAG, "Register Vx: %d", v[(opcode & 0x0F00) >> 8]);
    v[(opcode & 0x0F00) >> 8] += (opcode & 0x00FF);
    ESP_LOGW(TAG, "Vx: %d", (opcode & 0x0F00) >> 8);
    ESP_LOGW(TAG, "NN: %d", opcode & 0x00FF);
    ESP_LOGW(TAG, "Register Vx: %d", v[(opcode & 0x0F00) >> 8]);
    break;
	// ANNN ()
  case 0xA000:
    I = opcode & 0x0FFF;
    ESP_LOGW(TAG, "NNN: %X", opcode & 0x0FFF);
    ESP_LOGW(TAG, "Register I: %X", I);
    break;
	// DXYN (Draw to screen)
  case 0xD000:
    uint8_t height;
    height = opcode & 0x000F;

    ESP_LOGW(TAG, "N: %X", height);

    uint8_t x, y;

    x = v[(opcode & 0x0F00) >> 8];
    y = v[(opcode & 0x00F0) >> 4];

    ESP_LOGW(TAG, "Vx: %d", x);
    ESP_LOGW(TAG, "Vy: %d", y);

    // TEST:
    // std::bitset<8> byte_;
    uint8_t byte;

    // First loop over the height (h)
    for (int h = 0; h < height; h++) {

      byte = memory.getMemory(I + h);

      ESP_LOGW(TAG, "Data in memory at %d: %X", I + h, memory.getMemory(I + h));
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
          if (graphicsControl.drawPixel(x + w, y + h, white)) {
            v[0xf] = 1;
          } else {
            v[0xf] = 0;
          }
        }
      }
    }

    graphicsControl.present();

    break;
  }
};

void InterpreterControl::timerFinished() { ESP_LOGW(TAG, "TESTSTST"); };
