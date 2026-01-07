#pragma once

#include <cstdint>
class Memory {

public:
  Memory();
  ~Memory();

	uint8_t getMemory(uint16_t address);
	void setMemory(uint16_t address, uint8_t value);

private:

	static uint8_t _memory[4096];

	void init();

};
