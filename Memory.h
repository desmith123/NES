#ifndef MEMORY_H
#define MEMORY_H

#include "Types.h"
#include <vector>
#include <iostream>

class Memory {

public:
	Memory(std::string filename);
	Memory::~Memory();
	status_t fetchByte(uchar_t* data, uint32_t index);
	status_t writeByte(uchar_t data, uint32_t index);

	status_t WriteToStack(uchar_t memData, uint32_t offset);
	status_t ReadFromStack(uchar_t* memData, uint32_t offset);


private:
	std::vector<uchar_t> *mRam;
};



#endif  //  MEMORY_H
