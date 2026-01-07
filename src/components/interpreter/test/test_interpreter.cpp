#define private public

#include "InterpreterControl.hpp"
#include "esp_log.h"
#include "unity.h"
#include <stdio.h>

static const char *TAG = "Unit tests";

static void print_banner(const char *text) {
  printf("\n#### %s #####\n\n", text);
}

void test_opcodes(void) {

  auto interpreterControl =
      InterpreterControl("/littlefs/test_opcode.ch8", GPIO_NUM_0);

  ESP_LOGI(TAG, "Running tests");

  // Fill screen with white before testing opcode 00E0 (clear display)
  interpreterControl.graphicsControl.fillScreen(0x0000);
  interpreterControl.graphicsControl.present();

  interpreterControl.execute();
  TEST_ASSERT_EQUAL(interpreterControl.graphicsControl.backbuffer_[0], 0xFFFF);
	
  interpreterControl.execute();
  TEST_ASSERT_EQUAL(interpreterControl.v[0], 0xc);

  interpreterControl.execute();
  TEST_ASSERT_EQUAL(interpreterControl.v[0], 0xc + 9);

  interpreterControl.execute();
  TEST_ASSERT_EQUAL(interpreterControl.I, 0x50);

  interpreterControl.execute();
  TEST_ASSERT_EQUAL(interpreterControl.pc, 0x228);

  print_banner("Unity test runner");
}

extern "C" void app_main(void) {

  UNITY_BEGIN();
  RUN_TEST(test_opcodes);
  UNITY_END();
}
