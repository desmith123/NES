#include "Registers.h"

Processor::Registers::Registers():
	accumulator(0),
	programCounter(0),
	stackPointer(0xFD), // Starts at 0xFD in test??
	indexX(0),
	indexY(0)
{
	memset(&statusFlags, 0, sizeof(statusFlags));
	statusFlags.U = 1;
}

void Processor::Registers::dumpRegisters() {

}

void Processor::Registers::dumpStatusRegisters() {

}

