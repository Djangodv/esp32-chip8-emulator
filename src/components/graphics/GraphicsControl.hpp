/*! \brief Headerfile for Ili9341 Display wrapper
 *
 *  Basic drawing primitives
 */
#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h" // for heap_caps_malloc
#include "esp_lcd_ili9341.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
// #include "esp_timer.h" // for esp_timer_get_time for fine grain timing

// #include "Line.hpp"
// #include "Rectangle.hpp"
// #include "Square.hpp"
// #include "Circle.hpp"
// #include "Triangle.hpp"
#include <vector>

class GraphicsControlInterface {

public:
  // virtual ~GraphicsControlInterface() = default;
  virtual void setCollision(bool state) = 0;
};

class GraphicsControl {
public:
  GraphicsControl(GraphicsControlInterface *graphicsControlI, int mosi,
                  int sclk, int cs, int dc, int rst, int bl);
  ~GraphicsControl();

  // CYD in landscape: 320x240
  static constexpr int WIDTH = 320;
  static constexpr int HEIGHT = 240;

  void init();
  void fillScreen(uint16_t color);
  void backlightOn();
  void backlightOff();
  void diagnostics();
  uint16_t rbg565(uint8_t r, uint8_t b, uint8_t g);

  // Drawing Primitives
  void drawPixel(int x, int y, uint16_t color);

  void present();

private:
  void setupSPI();
  void setupPanel();

  int _mosi, _sclk, _cs, _dc, _rst, _bl;
  spi_host_device_t _host = SPI2_HOST;
  esp_lcd_panel_io_handle_t _io = nullptr;
  esp_lcd_panel_handle_t _panel = nullptr;

  // For the backbuffer
  uint16_t *backbuffer_ = nullptr; // WIDTH * HEIGHT entries
  size_t backbuffer_len_ = 0;      // number of uint16_t entries

	GraphicsControlInterface *_graphicsControlI;

};

