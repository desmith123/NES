#ifndef REGISTERS_H
#define REGISTERS_H

#include "Processor.h"

class Processor::Registers {

public:
	Registers::Registers();

	void dumpRegisters();

	void dumpStatusRegisters();

	/*
	* Accumulator is used in all ALU operations except bit shifting.
	* Contents of accumulator stored/retrieved from stack or memory.
	*/
	uchar_t accumulator;


	/*
	* Points to next instruction to be executed.
	*/
	ushort_t programCounter;

	/*
	* Holds low 8 bits of the next free location on the stack.
	*/
	uchar_t stackPointer;

	/*
	* Commonly used to hold counters or offsets in memory.
	*/
	uchar_t indexX;

	/*
	* Commonly used to hold counters or offsets in memory.
	*/
	uchar_t indexY;

	struct StatusFlag {

		bool C;    // Set if an overflow or underflow occurs.
		bool Z;    // Set if result of last operation was zero.
		bool I;    // Set if program requested interrupt disable.
		bool D;    // Set if program requested binary coded decimal.
		bool B;    // Set when break command executed.
		bool O;    // Set if operation produced invalid 2's compliment result.
		bool N;    // Set if operation set bit 7 to 1.
		bool U;    // Unused. Always logical 1.
	};

	StatusFlag statusFlags;

};


#endif  //  REGISTERS_H
