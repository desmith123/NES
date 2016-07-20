#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <vector>
#include <map>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Types.h"

typedef status_t(*opfunc)();

class Processor {

public:
	Processor();
	void start(ushort_t initialPC);
	void stop();
	void test();

	status_t executeNextInstruction();

private:
	class MemoryInterface;
	class Registers;

	uint32_t mCycleCount;

	MemoryInterface *mMemoryInterface;
	Registers *mRegisters;

	status_t JMP(uchar_t opcode);
	status_t LDX(uchar_t opcode);
	status_t LDY(uchar_t opcode);
	status_t STX(uchar_t opcode);
	status_t STA(uchar_t opcode);
	status_t JSR(uchar_t opcode);
	status_t NOP(uchar_t opcode);
	status_t SEC(uchar_t opcode);
	status_t BCS(uchar_t opcode);
	status_t BCC(uchar_t opcode);
	status_t BEQ(uchar_t opcode);
	status_t BNE(uchar_t opcode);
	status_t BVS(uchar_t opcode);
	status_t BVC(uchar_t opcode);
	status_t BPL(uchar_t opcode);
    status_t BMI(uchar_t opcode);
	status_t CLC(uchar_t opcode);
	status_t LDA(uchar_t opcode);
	status_t BIT(uchar_t opcode);
	status_t RTS(uchar_t opcode);
	status_t SEI(uchar_t opcode);
	status_t SED(uchar_t opcode);
	status_t PHP(uchar_t opcode);
    status_t PLP(uchar_t opcode);
    status_t PHA(uchar_t opcode);
	status_t PLA(uchar_t opcode);
	status_t LSR(uchar_t opcode);
	status_t ROR(uchar_t opcode);
        status_t ROL(uchar_t opcode);
	status_t ASL(uchar_t opcode);
	status_t AND(uchar_t opcode);
    status_t ORA(uchar_t opcode);
	status_t EOR(uchar_t opcode);
	status_t CMP(uchar_t opcode);
	status_t CPY(uchar_t opcode);
	status_t CPX(uchar_t opcode);
    status_t CLD(uchar_t opcode);
	status_t CLV(uchar_t opcode);
	status_t ADC(uchar_t opcode);
	status_t SBC(uchar_t opcode);
	status_t INY(uchar_t opcode);
	status_t INX(uchar_t opcode);
	status_t DEY(uchar_t opcode);
	status_t DEX(uchar_t opcode);
	status_t TAX(uchar_t opcode);
	status_t TAY(uchar_t opcode);
	status_t TYA(uchar_t opcode);
	status_t TXA(uchar_t opcode);
	status_t TSX(uchar_t opcode);
	status_t TXS(uchar_t opcode);
	status_t RTI(uchar_t opcode);

};

struct Instruction {
	uchar_t opcode;
	std::vector<uchar_t> extension;
};

#endif  //  PROCESSOR_H
