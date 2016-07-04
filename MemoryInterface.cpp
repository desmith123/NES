#include "MemoryInterface.h"

Processor::MemoryInterface::MemoryInterface() {

}


void Processor::MemoryInterface::loadRom() {
	mMemory = new Memory("nestest.nes");
}


status_t Processor::MemoryInterface::fetchBytesFromMemory(uchar_t *memData, uint32_t offset) {

	if (offset < 0) {
		return ERROR_UNKNOWN;
	}

	uchar_t data;
	if (mMemory->fetchByte(&data, offset) == OK) {
		*memData = data;
	}
	else {
		return ERROR_UNKNOWN;
	}

	return OK;
}


status_t Processor::MemoryInterface::fetchBytesFromMemory(std::vector<uchar_t> *memData, int numBytes, uint32_t offset) {

	if (numBytes <= 0 || offset < 0) {
		return ERROR_UNKNOWN;
	}

	for (int i = 0; i < numBytes; i++) {

		uchar_t data;
		if (mMemory->fetchByte(&data, offset+i) == OK) {
			memData->push_back(data);
		}
		else {
			return ERROR_UNKNOWN;
		}
	}

	// Convert from big endian to little endian
	switchEndian(memData);

	return OK;
}


status_t Processor::MemoryInterface::PushToStack(std::vector<uchar_t> *memData, uchar_t *sp, int numBytes) {
	
	status_t ret = ERROR_UNKNOWN;

	if (*sp < 0 || *sp >  0xFF - 1) {
		printf("Cannot push to stack, stack pointer is invalid\n");
		return ERROR_UNKNOWN;
	}

	// Convert from big to little endian
	switchEndian(memData);

	// Write data to stack and increment stack pointer
	for (int i = 0; i < numBytes; i++) {
		 ret = mMemory->WriteToStack(memData->at(i), *sp+i);
	}

	if (ret != OK)
	{
		printf("Error reading stack data from memory\n");
		return ERROR_UNKNOWN;
	}
	
	*sp += numBytes;

	if (ret != OK)
	{
		printf("Error writing stack data to memory\n");
		return ERROR_UNKNOWN;
	}

	return OK;
}

status_t Processor::MemoryInterface::PopFromStack(std::vector<uchar_t> *memData, uchar_t *sp, int numBytes) {

	status_t ret = ERROR_UNKNOWN;

	if (*sp < 1 || *sp >  0xFF) {
		printf("Cannot push to stack, stack pointer is invalid\n");
		return ERROR_UNKNOWN;
	}

	// Read the data on the stack
	for (int i = 0; i < numBytes; i++) {
		uchar_t val;
		// Need to read leftmost byte first, farthest from stack pointer
		ret = mMemory->ReadFromStack(&val, *sp-numBytes+i);
		memData->push_back(val);
	}

	if (ret != OK)
	{
		printf("Error reading stack data from memory\n");
		return ERROR_UNKNOWN;
	}

	*sp -= numBytes;

	// Erase the data on the stack
	std::vector<uchar_t> empty;
	for (int i = 0; i < numBytes; i++) {
		empty.push_back(0);
	}

	for (int i = 0; i < numBytes; i++) {
		ret = mMemory->WriteToStack(empty.at(i), *sp+i);
	}

	if (ret != OK)
	{
		printf("Error writing stack data to memory\n");
		return ERROR_UNKNOWN;
	}

	// Convert from little to big endian
	switchEndian(memData);
	return OK;
}


status_t Processor::MemoryInterface::writeBytesToMemory(uchar_t memData, uint32_t offset) {
	
	if (offset < 0) {
		return ERROR_UNKNOWN;
	}

	mMemory->writeByte(memData, offset);

	return OK;
}


#define LOG_SWITCH_ENDIAN 1
/*
* Converts memory stored in a vector from little endian to big endian
*/
void Processor::MemoryInterface::switchEndian(std::vector<uchar_t> *memData) {

#if LOG_SWITCH_ENDIAN
	printf("Start:\n");
	std::vector<uchar_t>::const_iterator iter = memData->begin();
	while (iter != memData->end()) {
		printf(" 0x%x", *iter);
		iter++;
	}
	printf("\n");
#endif

	int numBytes = memData->size();
	for (int i = 0; i < numBytes / 2; i++) {
		uchar_t temp = (*memData)[i];
		(*memData)[i] = (*memData)[numBytes - 1 - i];
		(*memData)[numBytes - 1 - i] = temp;
	}

#if LOG_SWITCH_ENDIAN
	printf("End:\n");
	iter = memData->begin();
	while (iter != memData->end()) {
		printf(" 0x%x", *iter);
		iter++;
	}
	printf("\n");
#endif
}
