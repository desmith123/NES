#ifndef MEMORY_INTERFACE_H
#define MEMORY_INTERFACE_H

#include "Processor.h"
#include "Memory.h"

class Processor::MemoryInterface {

public:
	MemoryInterface();

	void loadRom();

	status_t fetchBytesFromMemory(uchar_t *memData, uint32_t offset);
	status_t fetchBytesFromMemory(std::vector<uchar_t> *memData, int numBytes, uint32_t offset);
	status_t writeBytesToMemory(uchar_t memData, uint32_t offset);

	status_t PushToStack(std::vector<uchar_t> *memData, uchar_t *sp, int numBytes);
	status_t PopFromStack(std::vector<uchar_t> *memData, uchar_t *sp, int numBytes);


private:
	Memory *mMemory;

	void switchEndian(std::vector<uchar_t> *memData);
};


#endif  //  MEMORY_INTERFACE_H
