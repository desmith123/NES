#include "Memory.h"
#include <fstream>
#include <iostream>

Memory::Memory(std::string fileName) {

	std::ifstream *input = new std::ifstream();
	input->open(fileName, std::ios::binary);

	if (input->fail()) {
		printf("Error opening file: %s", fileName);
		return;
	}

	mRam = new std::vector<uchar_t>();
	std::vector<uchar_t>::const_iterator iter = mRam->begin();

	// NES ROM starts at 0xC000
	mRam->insert(iter, 0xC000, 0);

	// Skip first 16 bytes (NES HEADER)
	input->seekg(16);

	while (input->good()) {
		mRam->push_back(input->get());
	}
}

Memory::~Memory() {
	delete mRam;
}


status_t Memory::fetchByte(uchar_t* data, uint32_t index) {

	if (index >= 0 && index <= mRam->size()) {
		*data = mRam->at(index);
	}
	else {
		return ERROR_UNKNOWN;
	}

	return OK;
}


status_t Memory::writeByte(uchar_t data, uint32_t index) {

	if (index >= 0xC000) {
		printf("Cannot write to memory ROM");
		return ERROR_UNKNOWN;
	}

	mRam->at(index) = data;
	return OK;
}

/*
 * Stack is at 0x100 to 0x1FF, contains 2 byte addresses
 */
status_t Memory::WriteToStack(uchar_t data, uint32_t offset) {

	if (offset < 0 || offset > 0xFF - 1) {
		printf("Invalid write to stack\n");
		return ERROR_UNKNOWN;
	}

	mRam->at(0x100 + offset) = data;
	
	return OK;
}


/*
 * Stack is at 0x100 to 0x1FF, contains 2 byte addresses
 */
status_t Memory::ReadFromStack(uchar_t* data, uint32_t offset) {

	if (offset < 0 || offset > 0xFF) {
		printf("Invalid read from stack\n");
		return ERROR_UNKNOWN;
	}

	*data = mRam->at(0x100 + offset);

	return OK;

}