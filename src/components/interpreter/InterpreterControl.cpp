#include "InterpreterControl.hpp"

static const char *TAG = "InterpreterControl";

InterpreterControl::InterpreterControl(const gpio_num_t sound_pin)
    : romLoadingControl(), graphicsControl(this, 13, 14, 15, 2, -1, 27),
      timerControl(this), soundControl(sound_pin),
      keypadControl(this, ADC1_CHANNEL_7, GPIO_NUM_35, 5),
      _task_handle(nullptr), _running(false), _sound_pin(sound_pin) {
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

    vTaskDelay(pdMS_TO_TICKS(2));

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
    ESP_LOGE(TAG, "%X", fontset[i]);
    memory.setMemory(0x50 + i, fontset[i]);
    ESP_LOGE(TAG, "%X: %X", 0x50 + i, memory.getMemory(0x50 + i));
  }

  I = 0;

  sp = 0; // Initialize stack pointer to 0 (top of the stack)

  romLoadingControl.loadRom("/littlefs/Pong.ch8");

  graphicsControl.init();

  graphicsControl.fillScreen(black);
  graphicsControl.present();

  keypadControl.start();

  // Load keypad test 3 automatically (temporary)
  memory.setMemory(0x1FF, 1);

  // TODO: Initialize all memory type values to zero
}

uint16_t InterpreterControl::fetch() {

  uint16_t instruction;

  // Each memory address holds exactly one byte, an opcode is two bytes long.
  // Recall that Chip-8 uses big-endian to store opcodes First take the second
  // byte starting from the PC and shift it 8 bits to left, resulting in
  // 1111111100000000 after which you compare the current address of the PC with
  // the bitwise OR operator (|) setting the second part of the full opcode
  instruction =
      (memory.getMemory(pc) << 8) | (memory.getMemory(pc + 1)); // 0x200

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
  // 8XYN;
  case 0x8000:
    switch (opcode & 0x000F) {
    // 8XY0
    case 0x0000:
      ESP_LOGW(TAG, "Vx: %X", (opcode & 0x00F0) >> 4);
      ESP_LOGW(TAG, "Vx: %d", (opcode & 0x00F0) >> 4);
      v[(opcode & 0x0F00) >> 8] = v[(opcode & 0x00F0) >> 4];
      break;
    // 8XY1
    case 0x0001:
      v[(opcode & 0x0F00) >> 8] |= v[(opcode & 0x00F0) >> 4];
      break;
    // 8XY2
    case 0x0002:
      v[(opcode & 0x0F00) >> 8] &= v[(opcode & 0x00F0) >> 4];
      break;
    // 8XY3
    case 0x0003:
      v[(opcode & 0x0F00) >> 8] ^= v[(opcode & 0x00F0) >> 4];
      break;
    // 8XY4
    case 0x0004:
      uint16_t val;
      val = v[(opcode & 0x0F00) >> 8] + v[(opcode & 0x00F0) >> 4];

      v[(opcode & 0x0F00) >> 8] = 0xFF & val;

      if (val > 255) {
        v[0xf] = 1;
      } else {
        v[0xf] = 0;
      }

      break;
      // 8XY5
    case 0x0005:
      // Save value in vX before carrying out operation to check carry flag (vF)
      uint8_t vX;
      vX = v[(opcode & 0x0F00) >> 8];

      v[(opcode & 0x0F00) >> 8] -= v[(opcode & 0x00F0) >> 4];

      if (vX >= v[(opcode & 0x00F0) >> 4]) {
        v[0xf] = 1;
      } else {
        v[0xf] = 0;
      }

      break;
      // 8XY6 (Shifts Vx to the right: >> 1)
    case 0x0006:

      vX = v[(opcode & 0x0F00) >> 8];

      v[(opcode & 0x0F00) >> 8] >>= 1;

      // Check whether the LSB of current register Vx is 1 or 0
      v[0x0f] = vX & 0x1;
      ESP_LOGW(TAG, "TEST1: %X", v[(opcode & 0x0F00) >> 8]);
      ESP_LOGW(TAG, "TEST2: %X", v[(opcode & 0x0F00) >> 8] & 0x1);
      break;

    // 8XY7 ()
    case 0x0007:

      ESP_LOGW(TAG, "TEST1: %X", v[(opcode & 0x0F00) >> 8]);
      ESP_LOGW(TAG, "TEST2: %X", v[(opcode & 0x0F00) >> 8] & 0x1);

      vX = v[(opcode & 0x0F00) >> 8];

      v[(opcode & 0x0F00) >> 8] =
          v[(opcode & 0x00F0) >> 4] - v[(opcode & 0x0F00) >> 8];

      if (vX <= v[(opcode & 0x00F0) >> 4]) {
        v[0xf] = 1;
      } else {
        v[0xf] = 0;
      }

      break;

      // 8XYE ()
    case 0x000E:

      vX = v[(opcode & 0x0F00) >> 8];

      v[(opcode & 0x0F00) >> 8] <<= 1;

      if (vX & 0x80) {
        v[0xf] = 1;
      } else {
        v[0xf] = 0;
      }
      // v[0xf] = vX & 0x80;

      break;
    }
    break;
  // 9XY0
  case 0x9000:

    if (v[(opcode & 0x0F00) >> 8] != v[(opcode & 0x0F0) >> 4]) {
      ESP_LOGW(TAG, "PC: %d", pc);
      pc += 2;
      ESP_LOGW(TAG, "Expression true, incremented PC");
      ESP_LOGW(TAG, "PC: %d", pc);
    } else {
      ESP_LOGW(TAG, "Expression false, nothing happened");
    }

    ESP_LOGW(TAG, "Vx: %X", (opcode & 0x00F0) >> 4);
    ESP_LOGW(TAG, "Vx: %d", (opcode & 0x00F0) >> 4);

    break;
    // ANNN ()
  case 0xA000:
    I = opcode & 0x0FFF;
    ESP_LOGW(TAG, "NNN: %X", opcode & 0x0FFF);
    ESP_LOGW(TAG, "Register I: %X", I);
    break;
  // BNNN ()
  case 0xB000:
    pc = (opcode & 0x0FFF) + v[0];
    break;
  // CXNN ()
  case 0xC000:
    ESP_LOGW(TAG, "Random number: %d", rand() % 256);
    v[(opcode & 0x0F00) >> 8] = (rand() % 256) & (opcode & 0x00FF);
    break;
    // DXYN (Draw to screen)
  case 0xD000:
    uint8_t height;
    height = opcode & 0x000F;

    ESP_LOGW(TAG, "N (height): %X", height);
    ESP_LOGW(TAG, "Register I (memory address of sprite): %X, decimal: %d", I,
             I);

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

      // Width of the sprite to be drawn
      for (int w = 0; w < 8; w++) {
        // Check which of bits in the byte of the sprite are set
        // Invert the operation by starting on the left-side of the byte,
        // because of big-endiannes (else the sprites will be drawn in a mirror
        // image) Cause: if ((byte >> i) & 0x1) {
        if ((byte << w) & 0x80) {
          graphicsControl.drawPixel(x + w, y + h, white);
        }
      }
    }

    graphicsControl.present();

    break;
  case 0xE000:
    switch (opcode & 0x00FF) {
    // EX9E
    case 0x009E:
      if (keypad[v[(opcode & 0x0F00) >> 8]]) {
        pc += 2;
      }
      break;
    // EXA1
    case 0x00A1:
      ESP_LOGW(TAG, "Register Vx: %X", v[(opcode & 0x0F00) >> 8]);
      ESP_LOGW(TAG, "Register Vx: %X", keypad[v[(opcode & 0x0F00) >> 8]]);
      if (keypad[v[(opcode & 0x0F00) >> 8]] == 0) {
        pc += 2;
      }
      break;
    }
    break;
  case 0xF000:
    switch (opcode & 0x00FF) {
    case 0x0007:
      v[(opcode & 0x0F00) >> 8] = timerControl.getExpiryTime();
      break;
      // FX0A (Await a key press and set Vx to it)
    case 0x000A:
      bool key_pressed;
      key_pressed = false;

      for (int i = 0; i < 16; i++) {
        if (keypad[i] != 0) {
          v[(opcode & 0x0F00) >> 8] = i;
          key_pressed = true;
        }
      }

      if (key_pressed != true) {
        pc -= 2;
      }

      break;
      // FX15 (Sets delay timer to Vx)
    case 0x0015:
      uint8_t duration;
      duration = v[(opcode & 0x0F00) >> 8];
      ESP_LOGW(TAG, "Duration: %d", duration);
      timerControl.startTimer(duration);
      break;
      // FX18 (Sets sound timer to Vx)
    case 0x0018:
      duration = v[(opcode & 0x0F00) >> 8];
      ESP_LOGW(TAG, "Duration: %d", duration);
      timerControl.startTimer(duration);
      soundControl.playSound();
      break;
      // FX1E (Add Vx to I)
    case 0x001E:
      I += v[(opcode & 0x0F00) >> 8];
      break;
      // FX29 (Set I to location of sprite in Vx)
    case 0x0029:
      uint8_t offset;
      offset = 0x50 + (v[(opcode & 0x0F00) >> 8] * 5);
      I = offset;
      break;
      // FX33 (Store binary representation of number in Vx)
    case 0x0033:
      uint8_t number;
      number = v[(opcode & 0x0F00) >> 8];

      for (int i = 0; i < 3; i++) {
        memory.setMemory(I + (2 - i), number % 10);
        number /= 10;
      }

      break;
    case 0x0055:
      for (int i = 0; i <= ((opcode & 0x0F00) >> 8); i++) {
        memory.setMemory(I + i, v[i]);
      }
      break;
    case 0x0065:
      for (int i = 0; i <= ((opcode & 0x0F00) >> 8); i++) {
        v[i] = memory.getMemory(I + i);
      }
      break;
    }
    break;
  default:
    ESP_LOGE(TAG, "No opcode implementation yet");
  }
};

void InterpreterControl::timerFinished() {

  if (gpio_get_level(_sound_pin)) {
    soundControl.stopSound();
  }

  ESP_LOGW(TAG, "Timer finished");
};

void InterpreterControl::setCollision(bool state) {

  if (state == true) {
    v[0xf] = 1;
  } else {
    v[0xf] = 0;
  }

  ESP_LOGD(TAG, "Set collision register v[0xf] to %d", state);
};

void InterpreterControl::buttonPressed(Button buttonId) {
  switch (buttonId) {
  case Button::NONE:
    for (int i = 0; i < 16; i++) {
      keypad[i] = 0;
    }
    ESP_LOGI(TAG, "Button: NONE");
    break;
  case Button::START:
    keypad[0xa] = 1;
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
    keypad[0xe] = 1;
    ESP_LOGI(TAG, "Button: UP");
    break;
  case Button::DOWN:
    keypad[0xf] = 1;
    ESP_LOGI(TAG, "Button: DOWN");
    break;
  }
};
