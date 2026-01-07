#include "Memory.hpp"

Memory::Memory() { init(); };
Memory::~Memory() {};

uint8_t Memory::_memory[4096]{0};

void Memory::init() {
	// TODO: initialize all memory at 0
}

uint8_t Memory::getMemory(uint16_t address) {

	return _memory[address];

};
void Memory::setMemory(uint16_t address, uint8_t value) {

	_memory[address] = value;

};

