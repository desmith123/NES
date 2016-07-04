#include "Processor.h"
#include "MemoryInterface.h"
#include "Registers.h"
#include "opcodes.h"

Processor::Processor() {
	mMemoryInterface = new MemoryInterface();
	mMemoryInterface->loadRom();

	mRegisters = new Registers();
	mCycleCount = 0;
}

void Processor::start(ushort_t initialPC) {

	mRegisters->programCounter = initialPC;

	for (int i = 0; i < 160; i++) {
		printf("----------------\n");
		printf("Executing next instruction!\n");
		status_t ret = executeNextInstruction();
		if (ret != OK) {
			printf("Error executing instruction");
			break;
		}
	}
	stop();
}

void Processor::stop() {

}

status_t Processor::executeNextInstruction() {

	uchar_t opcode;
	status_t ret = mMemoryInterface->fetchBytesFromMemory(&opcode, mRegisters->programCounter);
	if (ret != OK) {
		printf("Error reading memory at 0x%x\n", mRegisters->programCounter);
		return ret;
	}

	printf("Next opcode: 0x%x\n", opcode);

	switch (opcode) {

		case JMP_ABSOLUTE:
		case JMP_INDIRECT:
		{
			printf("Jump Instruction\n");
			ret = JMP(opcode);
			break;
		}

		case LDX_IMMEDIATE:
		case LDX_ZERO_PAGE:
		case LDX_ZERO_PAGE_Y:
		case LDX_ABSOLUTE:
		case LDX_ABSOLUTE_Y:
		{
			printf("Load X Instruction\n");
			ret = LDX(opcode);
			break;
		}

		case LDA_IMMEDIATE:
		case LDA_ZERO_PAGE:
		case LDA_ZERO_PAGE_X:
		case LDA_ABSOLUTE:
		case LDA_ABSOLUTE_X:
		case LDA_ABSOLUTE_Y:
		case LDA_INDIRECT_X:
		case LDA_INDIRECT_Y:
		{
			printf("Load Accumulator Instruction\n");
			ret = LDA(opcode);
			break;
		}

		case STX_ZERO_PAGE:
		case STX_ZERO_PAGE_Y:
		case STX_ABSOLUTE:
		{
			printf("Jump Instruction\n");
			ret = STX(opcode);
			break;
		}

		case STA_ZERO_PAGE:
		case STA_ZERO_PAGE_X:
		case STA_ABSOLUTE:
		case STA_ABSOLUTE_X:
		case STA_ABSOLUTE_Y:
		case STA_INDIRECT_X:
		case STA_INDIRECT_Y:
		{
			printf("Store Accumulator Instruction\n");
			ret = STA(opcode);
			break;
		}

		case AND_IMMEDIATE:
		case AND_ZERO_PAGE:
		case AND_ZERO_PAGE_X:
		case AND_ABSOLUTE:
		case AND_ABSOLUTE_X:
		case AND_ABSOLUTE_Y:
		case AND_INDIRECT_X:
		case AND_INDIRECT_Y:
		{
			printf("Logical AND Instruction\n");
			ret = AND(opcode);
			break;
		}

		case CMP_IMMEDIATE:
		case CMP_ZERO_PAGE:
		case CMP_ZERO_PAGE_X:
		case CMP_ABSOLUTE:
		case CMP_ABSOLUTE_X:
		case CMP_ABSOLUTE_Y:
		case CMP_INDIRECT_X:
		case CMP_INDIRECT_Y:
		{
			printf("Compare Instruction\n");
			ret = CMP(opcode);
			break;
		}

		case JSR_ABSOLUTE:
		{
			printf("Jump to subroutine Instruction\n");
			ret = JSR(opcode);
			break;
		}

		case NOP_IMPLIED:
		{
			printf("NOP Instruction\n");
			ret = NOP(opcode);
			break;
		}

		case SEC_IMPLIED:
		{
			printf("Set carry instruction\n");
			ret = SEC(opcode);
			break;
		}

		case BCS_RELATIVE:
		{
			printf("Branch if carry set instruction\n");
			ret = BCS(opcode);
			break;
		}

		case BCC_RELATIVE:
		{
			printf("Branch if carry clear instruction\n");
			ret = BCC(opcode);
			break;
		}

		case BEQ_RELATIVE:
		{
			printf("Branch if equal instruction\n");
			ret = BEQ(opcode);
			break;
		}

		case BNE_RELATIVE:
		{
			printf("Branch if not equal instruction\n");
			ret = BNE(opcode);
			break;
		}

		case BVS_RELATIVE:
		{
			printf("Branch if overflow set instruction\n");
			ret = BVS(opcode);
			break;
		}

		case BVC_RELATIVE:
		{
			printf("Branch if overflow clear instruction\n");
			ret = BVC(opcode);
			break;
		}

		case BPL_RELATIVE:
		{
			printf("Branch if positive instruction\n");
			ret = BPL(opcode);
			break;
		}

                case BMI_RELATIVE:
                {
                        printf("Branch if negative instruction\n");
                        ret = BMI(opcode);
                        break;
                }

		case CLC_IMPLIED:
		{
			printf("Clear carry instruction\n");
			ret = CLC(opcode);
			break;
		}

		case RTS_IMPLIED:
		{
			printf("Return from subroutine instruction\n");
			ret = RTS(opcode);
			break;
		}

		case SEI_IMPLIED:
		{
			printf("Set interrupt disable instruction\n");
			ret = SEI(opcode);
			break;
		}

		case SED_IMPLIED:
		{
			printf("Set decimal instruction\n");
			ret = SED(opcode);
			break;
		}

                case CLD_IMPLIED:
                {
                        printf("Clear decimal instruction\n");
                        ret = CLD(opcode);
                        break;
                }

		case PHP_IMPLIED:
		{
			printf("Push processor status instruction\n");
			ret = PHP(opcode);
			break;
		}

                case PHA_IMPLIED:
                {
                        printf("Push accumulator instruction\n");
                        ret = PHA(opcode);
                        break;
                }

		case PLA_IMPLIED:
		{
			printf("Pull accumulator instruction\n");
			ret = PLA(opcode);
			break;
		}

                case PLP_IMPLIED:
                {
                       printf("Pull processor status instruction\n");
                       ret = PLP(opcode);
                       break;
                }

		case BIT_ZERO_PAGE:
		case BIT_ABSOLUTE:
		{
			printf("Bit set instruction\n");
			ret = BIT(opcode);
			break;
		}

		default:
		{
			printf("Opcode 0x%x not implemented\n", opcode);
			ret = ERROR_UNKNOWN;
		}
	}

	return ret;
}


/*
 * Sets the program counter to the address specified by the operand.
 * 
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Absolute         |  0x4C   |  3     |  3
 * Indirect         |  0x6C   |  3     |  5
 */
status_t Processor::JMP(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Jumps to the address contained in the proceeding two bytes.
		case JMP_ABSOLUTE:
		{
			std::vector<uchar_t> instrData;
			ret = mMemoryInterface->fetchBytesFromMemory(&instrData, 2, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			ushort_t memAddr = (instrData[0] << 8) | instrData[1];

			printf("Jumping to address 0x%x\n", memAddr);
			mRegisters->programCounter = memAddr;

			mCycleCount += 3;
			ret = OK;
			break;
		}

		/*
		 * 
		 */
		case JMP_INDIRECT:
		{
			printf("Instruction JMP_INDIRECT not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default: 
		{
			printf("No JMP implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Loads a byte of memory into the X register setting the zero and negative flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Set if X = 0
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of X is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Immediate        |  0xA2   |  2     |  2
 * Zero Page        |  0xA6   |  2     |  3
 * Zero Page,Y      |  0xB6   |  2     |  4
 * Absolute         |  0xAE   |  3     |  4
 * Absolute,Y       |  0xBE   |  3     |  4 (+1 if page crossed)
 */
status_t Processor::LDX(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Loads the value of the next byte into register X.
		case LDX_IMMEDIATE:
		{
			uchar_t instrData;
			ret = mMemoryInterface->fetchBytesFromMemory(&instrData, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			mRegisters->indexX = instrData;

			// Set status registers
			if (mRegisters->indexX == 0) {
				printf("Zero result: Setting zero flag\n");
				mRegisters->statusFlags.Z = 1;
			}
			else {
				mRegisters->statusFlags.Z = 0;
			}
			if ((mRegisters->indexX & 0x80) == 0x80) {
				printf("Negative result: Setting negative flag\n");
				mRegisters->statusFlags.N = 1;
			}
			else {
				mRegisters->statusFlags.N = 0;
			}

			mRegisters->programCounter += 2;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		case LDX_ZERO_PAGE:
		{
			printf("Instruction LDX_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		case LDX_ZERO_PAGE_Y:
		{
			printf("Instruction LDX_ZERO_PAGE_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		case LDX_ABSOLUTE:
		{
			printf("Instruction LDX_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		case LDX_ABSOLUTE_Y:
		{
			printf("Instruction LDX_ABSOLUTE_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No LDX implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Loads a byte of memory into the accumulator setting the zero and negative flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Set if A = 0
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of A is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Immediate        |  0xA9   |  2     |  2
 * Zero Page        |  0xA5   |  2     |  3
 * Zero Page,X      |  0xB5   |  2     |  4
 * Absolute         |  0xAD   |  3     |  4
 * Absolute,X       |  0xBD   |  3     |  4 (+1 if page crossed)
 * Absolute,Y       |  0xB9   |  3     |  4 (+1 if page crossed)
 * (Indirect,X)     |  0xA1   |  2     |  6
 * (Indirect),Y     |  0xB1   |  2     |  5 (+1 if page crossed)
 */
status_t Processor::LDA(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {
  
    // Loads the next 8 bit constant into the accumulator
	case LDA_IMMEDIATE:
	{
		uchar_t instrData;
		ret = mMemoryInterface->fetchBytesFromMemory(&instrData, mRegisters->programCounter + 1);
		if (ret != OK) {
			printf("Error reading memory\n");
			return ret;
		}

		printf("Setting accumulator to 0x%x\n", instrData);
		mRegisters->accumulator = instrData;

		// Set status registers
		if (mRegisters->accumulator == 0) {
			printf("Zero result: Setting zero flag\n");
			mRegisters->statusFlags.Z = 1;
		}
		else {
			mRegisters->statusFlags.Z = 0;
		}
		if ((mRegisters->accumulator & 0x80) == 0x80) {
			printf("Negative result: Setting negative flag\n");
			mRegisters->statusFlags.N = 1;
		}
		else {
			mRegisters->statusFlags.N = 0;
		}

		mRegisters->programCounter += 2;

		mCycleCount += 2;
		ret = OK;
		break;
	}

	//
	case LDA_ZERO_PAGE:
	{
		printf("Instruction LDA_ZERO_PAGE not implemented\n");
		ret = ERROR_UNKNOWN;
		break;
	}

	//
	case LDA_ZERO_PAGE_X:
	{
		printf("Instruction LDA_ZERO_PAGE_X not implemented\n");
		ret = ERROR_UNKNOWN;
		break;
	}

	//
	case LDA_ABSOLUTE:
	{
		printf("Instruction LDA_ABSOLUTE not implemented\n");
		ret = ERROR_UNKNOWN;
		break;
	}

	//
	case LDA_ABSOLUTE_X:
	{
		printf("Instruction LDA_ABSOLUTE_X not implemented\n");
		ret = ERROR_UNKNOWN;
		break;
	}

	//
	case LDA_ABSOLUTE_Y:
	{
		printf("Instruction LDA_ABSOLUTE_Y not implemented\n");
		ret = ERROR_UNKNOWN;
		break;
	}

	//
	case LDA_INDIRECT_X:
	{
		printf("Instruction LDA_INDIRECT_X not implemented\n");
		ret = ERROR_UNKNOWN;
		break;
	}

	//
	case LDA_INDIRECT_Y:
	{
		printf("Instruction LDA_INDIRECT_Y not implemented\n");
		ret = ERROR_UNKNOWN;
		break;
	}

	default:
	{
		printf("No LDA implementation for opcode 0x%x\n", opcode);
		ret = ERROR_UNKNOWN;
		break;
	}
	}

	return ret;
}


/*
 * Stores the contents of the accumulator into memory.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Zero Page        |  0x85   |  2     |  3
 * Zero Page,X      |  0x95   |  2     |  4
 * Absolute         |  0x8D   |  3     |  4
 * Absolute,X       |  0x9D   |  3     |  4 (+1 if page crossed)
 * Absolute,Y       |  0x99   |  3     |  4 (+1 if page crossed)
 * (Indirect,X)     |  0x81   |  2     |  6
 * (Indirect),Y     |  0x91   |  2     |  5 (+1 if page crossed)
 */
status_t Processor::STA(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// The next byte has the zero page address to store the accumulator register into
		case STA_ZERO_PAGE:
		{
			uchar_t addr;
			ret = mMemoryInterface->fetchBytesFromMemory(&addr, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			uchar_t accu = mRegisters->accumulator;
			ret = mMemoryInterface->writeBytesToMemory(accu, addr);
			if (ret != OK) {
				printf("Error writing to memory\n");
				return ret;
			}

			mRegisters->programCounter += 2;

			mCycleCount += 3;
			ret = OK;
			break;
		}

		//
		case STA_ZERO_PAGE_X:
		{
			printf("Instruction STA_ZERO_PAGE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case STA_ABSOLUTE:
		{
			printf("Instruction STA_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case STA_ABSOLUTE_X:
		{
			printf("Instruction STA_ABSOLUTE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case STA_ABSOLUTE_Y:
		{
			printf("Instruction STA_ABSOLUTE_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case STA_INDIRECT_X:
		{
			printf("Instruction STA_INDIRECT_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case STA_INDIRECT_Y:
		{
			printf("Instruction STA_INDIRECT_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No STA implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * A logical AND is performed, bit by bit, on the accumulator contents using the contents of a byte of memory.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Set if A = 0
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Immediate        |  0x29   |  2     |  2
 * Zero Page        |  0x25   |  2     |  3
 * Zero Page,X      |  0x35   |  2     |  4
 * Absolute         |  0x2D   |  3     |  4
 * Absolute,X       |  0x3D   |  3     |  4 (+1 if page crossed)
 * Absolute,Y       |  0x39   |  3     |  4 (+1 if page crossed)
 * (Indirect,X)     |  0x21   |  2     |  6
 * (Indirect),Y     |  0x31   |  2     |  5 (+1 if page crossed)
 */
status_t Processor::AND(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Get accumulator and bit by bit AND it with next byte
		case AND_IMMEDIATE:
		{
			uchar_t accumulator = mRegisters->accumulator;
			
			uchar_t nextByte;
			ret = mMemoryInterface->fetchBytesFromMemory(&nextByte, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			uchar_t andResult = nextByte & accumulator;
			mRegisters->accumulator = andResult;
			printf("Setting accumulator to 0x%x\n", andResult);

			// Set status registers
			if (mRegisters->accumulator == 0) {
				printf("Zero result: Setting zero flag\n");
				mRegisters->statusFlags.Z = 1;
			}
			else {
				mRegisters->statusFlags.Z = 0;
			}
			if ((mRegisters->accumulator & 0x80) == 0x80) {
				printf("Negative result: Setting negative flag\n");
				mRegisters->statusFlags.N = 1;
			}
			else {
				mRegisters->statusFlags.N = 0;
			}

			mRegisters->programCounter += 2;

			mCycleCount += 2;
			ret = OK;
			break;
		}


		//
		case AND_ZERO_PAGE:
		{
			printf("Instruction AND_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case AND_ZERO_PAGE_X:
		{
			printf("Instruction AND_ZERO_PAGE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case AND_ABSOLUTE:
		{
			printf("Instruction AND_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case AND_ABSOLUTE_X:
		{
			printf("Instruction AND_ABSOLUTE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case AND_ABSOLUTE_Y:
		{
			printf("Instruction STA_ABSOLUTE_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}
	
		//
		case AND_INDIRECT_X:
		{
			printf("Instruction AND_INDIRECT_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case AND_INDIRECT_Y:
		{
			printf("Instruction AND_INDIRECT_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No AND implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Compares the contents of the accumulator with another memory held value and sets the zero and carry flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Set if A >= M
 * Z 	Zero Flag 	        Set if A = M
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of the result is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Immediate        |  0xC9   |  2     |  2
 * Zero Page        |  0xC5   |  2     |  3
 * Zero Page,X      |  0xD5   |  2     |  4
 * Absolute         |  0xCD   |  3     |  4
 * Absolute,X       |  0xDD   |  3     |  4 (+1 if page crossed)
 * Absolute,Y       |  0xD9   |  3     |  4 (+1 if page crossed)
 * (Indirect,X)     |  0xC1   |  2     |  6
 * (Indirect),Y     |  0xD1   |  2     |  5 (+1 if page crossed)
 */
status_t Processor::CMP(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Get accumulator and compare it with next byte, set flags.
		case CMP_IMMEDIATE:
		{
			uchar_t accumulator = mRegisters->accumulator;
			uchar_t nextByte;
			ret = mMemoryInterface->fetchBytesFromMemory(&nextByte, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			int cmp = (int)accumulator - (int)nextByte;

			// Set status registers
			if (cmp == 0) {
				printf("Zero result: Setting zero flag\n");
				mRegisters->statusFlags.Z = 1;
			}
			else {
				mRegisters->statusFlags.Z = 0;
			}
			if (cmp >= 0) {
				printf("Carry result: Setting carry flag\n");
				mRegisters->statusFlags.C = 1;
			}
			else {
				mRegisters->statusFlags.C = 0;
			}
			if (cmp < 0) {
				printf("Negative result: Setting negative flag\n");
				mRegisters->statusFlags.N = 1;
			}
			else {
				mRegisters->statusFlags.N = 0;
			}

			mRegisters->programCounter += 2;

			mCycleCount += 2;
			ret = OK;
			break;
		}


		//
		case CMP_ZERO_PAGE:
		{
			printf("Instruction CMP_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case CMP_ZERO_PAGE_X:
		{
			printf("Instruction CMP_ZERO_PAGE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case CMP_ABSOLUTE:
		{
			printf("Instruction CMP_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case CMP_ABSOLUTE_X:
		{
			printf("Instruction CMP_ABSOLUTE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case CMP_ABSOLUTE_Y:
		{
			printf("Instruction CMP_ABSOLUTE_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case CMP_INDIRECT_X:
		{
			printf("Instruction CMP_INDIRECT_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case CMP_INDIRECT_Y:
		{
			printf("Instruction CMP_INDIRECT_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No CMP implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Stores the contents of the X register into memory.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Zero Page        |  0x86   |  2     |  3
 * Zero Page,Y      |  0x96   |  2     |  4
 * Absolute         |  0x8E   |  3     |  4
 */
status_t Processor::STX(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Writes the value of Register X into zero page memory address contained in the next byte.
		case STX_ZERO_PAGE:
		{
			uchar_t instrData;
			ret = mMemoryInterface->fetchBytesFromMemory(&instrData, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory");
				return ret;
			}

			uchar_t regXVal = mRegisters->indexX;
			printf("Writing 0x%x to zero page address 0x%x\n", regXVal, instrData);
			ret = mMemoryInterface->writeBytesToMemory(regXVal, instrData);
			if (ret != OK) {
				printf("Error writing to memory");
				return ret;
			}

			mRegisters->programCounter += 2;

			mCycleCount += 3;
			ret = OK;
			break;
		}

		//
		case STX_ZERO_PAGE_Y:
		{
			printf("Instruction STX_ZERO_PAGE_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case STX_ABSOLUTE:
		{
			printf("Instruction STX_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}
		
		default:
		{
			printf("No STX implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Pushes the address (minus one) of the return point on to the stack
 * and then sets the program counter to the target memory address.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Absolute         |  0x20   |  3     |  6
 */
status_t Processor::JSR(uchar_t opcode) {
	
	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Jumps to address contained in next 2 bytes and puts current PC into stack.
		case JSR_ABSOLUTE:
		{
			// Put address of next instruction - 1 into stack
			ushort_t popAddr = mRegisters->programCounter + 2;
			std::vector<uchar_t> popAddrContainer;
			popAddrContainer.push_back((popAddr & 0xff00) >> 8);
			popAddrContainer.push_back(popAddr & 0x00ff);

			ret = mMemoryInterface->PushToStack(&popAddrContainer, &mRegisters->stackPointer, 2);
			if (ret != OK) {
				printf("Error pushing address to stack\n");
				return ret;
			}


			// Jump to new address
			std::vector<uchar_t> instrData;
			ret = mMemoryInterface->fetchBytesFromMemory(&instrData, 2, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			ushort_t memAddr = (instrData[0] << 8) | instrData[1];

			printf("Jumping to address 0x%x\n", memAddr);
			mRegisters->programCounter = memAddr;

			mCycleCount += 6;
			ret = OK;
			break;
		}

		default:
		{
			printf("No JSR implementation for opcode 0x%x", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Causes no changes to the processor other than the normal incrementing of the program counter to the next instruction.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0xEA   |  1     |  2
 */
status_t Processor::NOP(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// NOP
		case NOP_IMPLIED:
		{
			mRegisters->programCounter += 1;
			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No NOP implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Set the carry flag to one.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Set to 1
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0x38   |  1     |  2
 */
status_t Processor::SEC(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Sets the Carry Register to true
		case SEC_IMPLIED:
		{
			mRegisters->statusFlags.C = 1;
			mRegisters->programCounter += 1;
			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No SEC implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * If the carry flag is set then add the relative displacement to the program counter to cause a branch to a new location.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Relative         |  0xB0   |  2     |  2 (+1 if branch succeeds, +2 if to a new page)
 */
status_t Processor::BCS(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Adds the value (positive or negative) to the program counter and jumps to that address if carry set
		case BCS_RELATIVE:
		{
			if (mRegisters->statusFlags.C == 0) {
				mRegisters->programCounter += 2;
				mCycleCount += 2;
			}
			else {
				// Get displacement
				uchar_t disp;
				ret = mMemoryInterface->fetchBytesFromMemory(&disp, mRegisters->programCounter + 1);
				if (ret != OK) {
					printf("Error reading memory\n");
					return ret;
				}
				printf("Disp is 0x%02x\n", disp);

				// Account for negative displacement
				mRegisters->programCounter += (char)disp + 2;

				mCycleCount += 3;
			}

			ret = OK;
			break;
		}

		default:
		{
			printf("No BCS implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * If the carry flag is clear then add the relative displacement to the program counter to cause a branch to a new location.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Relative         |  0x90   |  2     |  2 (+1 if branch succeeds, +2 if to a new page)
 */
status_t Processor::BCC(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Adds the value (positive or negative) to the program counter and jumps to that address if not carry set
		case BCC_RELATIVE:
		{
			if (mRegisters->statusFlags.C == 1) {
				mRegisters->programCounter += 2;
				mCycleCount += 2;
			}
			else {
				// Get displacement
				uchar_t disp;
				ret = mMemoryInterface->fetchBytesFromMemory(&disp, mRegisters->programCounter + 1);
				if (ret != OK) {
					printf("Error reading memory\n");
					return ret;
				}
				printf("Disp is 0x%02x\n", disp);

				// Account for negative displacement
				mRegisters->programCounter += (char)disp + 2;

				mCycleCount += 3;
			}

			ret = OK;
			break;
		}

		default:
		{
			printf("No BCC implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * If the zero flag is set then add the relative displacement to the program counter to cause a branch to a new location.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Relative         |  0xf0   |  2     |  2 (+1 if branch succeeds, +2 if to a new page)
 */
status_t Processor::BEQ(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Adds the value (positive or negative) to the program counter and jumps to that address if zero flag is set
		case BEQ_RELATIVE:
		{
			if (mRegisters->statusFlags.Z == 0) {
				mRegisters->programCounter += 2;
				mCycleCount += 2;
			}
			else {
				// Get displacement
				uchar_t disp;
				ret = mMemoryInterface->fetchBytesFromMemory(&disp, mRegisters->programCounter + 1);
				if (ret != OK) {
					printf("Error reading memory\n");
					return ret;
				}
				printf("Disp is 0x%02x\n", disp);

				// Account for negative displacement
				mRegisters->programCounter += (char)disp + 2;

				mCycleCount += 3;
			}

			ret = OK;
			break;
		}

		default:
		{
			printf("No BEQ implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * If the zero flag is clear then add the relative displacement to the program counter to cause a branch to a new location.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Relative         |  0xd0   |  2     |  2 (+1 if branch succeeds, +2 if to a new page)
 */
status_t Processor::BNE(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Adds the value (positive or negative) to the program counter and jumps to that address if zero flag is set
		case BNE_RELATIVE:
		{
			if (mRegisters->statusFlags.Z == 1) {
				mRegisters->programCounter += 2;
				mCycleCount += 2;
			}
			else {
				// Get displacement
				uchar_t disp;
				ret = mMemoryInterface->fetchBytesFromMemory(&disp, mRegisters->programCounter + 1);
				if (ret != OK) {
					printf("Error reading memory\n");
					return ret;
				}
				printf("Disp is 0x%02x\n", disp);

				// Account for negative displacement
				mRegisters->programCounter += (char)disp + 2;

				mCycleCount += 3;
			}

			ret = OK;
			break;
		}

		default:
		{
			printf("No BNE implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * If the overflow flag is set then add the relative displacement to the program counter to cause a branch to a new location.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Relative         |  0x70   |  2     |  2 (+1 if branch succeeds, +2 if to a new page)
 */
status_t Processor::BVS(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		//
		case BVS_RELATIVE:
		{
			if (mRegisters->statusFlags.O == 0) {
				mRegisters->programCounter += 2;
				mCycleCount += 2;
			}
			else {
				// Get displacement
				uchar_t disp;
				ret = mMemoryInterface->fetchBytesFromMemory(&disp, mRegisters->programCounter + 1);
				if (ret != OK) {
					printf("Error reading memory\n");
					return ret;
				}
				printf("Disp is 0x%02x\n", disp);

				// Account for negative displacement
				mRegisters->programCounter += (char)disp + 2;

				mCycleCount += 3;
			}

			ret = OK;
			break;
		}

		default:
		{
			printf("No BVS implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * If the overflow flag is clear then add the relative displacement to the program counter to cause a branch to a new location.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Relative         |  0x50   |  2     |  2 (+1 if branch succeeds, +2 if to a new page)
 */
status_t Processor::BVC(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		//
		case BVC_RELATIVE:
		{
			if (mRegisters->statusFlags.O == 1) {
				mRegisters->programCounter += 2;
				mCycleCount += 2;
			}
			else {
				// Get displacement
				uchar_t disp;
				ret = mMemoryInterface->fetchBytesFromMemory(&disp, mRegisters->programCounter + 1);
				if (ret != OK) {
					printf("Error reading memory\n");
					return ret;
				}
				printf("Disp is 0x%02x\n", disp);

				// Account for negative displacement
				mRegisters->programCounter += (char)disp + 2;

				mCycleCount += 3;
			}

			ret = OK;
			break;
		}
	
		default:
		{
			printf("No BVC implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * If the negative flag is clear then add the relative displacement to the program counter to cause a branch to a new location.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Relative         |  0x10   |  2     |  2 (+1 if branch succeeds, +2 if to a new page)
 */
status_t Processor::BPL(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Pull displacement from next byte. Jump there if N flag not 1.
		case BPL_RELATIVE:
		{
			if (mRegisters->statusFlags.N == 1) {
				mRegisters->programCounter += 2;
				mCycleCount += 2;
			}
			else {
				// Get displacement
				uchar_t disp;
				ret = mMemoryInterface->fetchBytesFromMemory(&disp, mRegisters->programCounter + 1);
				if (ret != OK) {
					printf("Error reading memory\n");
					return ret;
				}
				printf("Disp is 0x%02x\n", disp);

				// Account for negative displacement
				mRegisters->programCounter += (char)disp + 2;

				mCycleCount += 3;
			}

			ret = OK;
			break;
		}

		default:
		{
			printf("No BPL implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * If the negative flag is set then add the relative displacement to the program counter to cause a branch to a new location.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Relative         |  0x30   |  2     |  2 (+1 if branch succeeds, +2 if to a new page)
 */
status_t Processor::BMI(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		//
		case BMI_RELATIVE:
		{
			if (mRegisters->statusFlags.N == 0) {
				mRegisters->programCounter += 2;
				mCycleCount += 2;
                                printf("Not branching, flag is not negative\n");
			}
			else {
				// Get displacement
				uchar_t disp;
				ret = mMemoryInterface->fetchBytesFromMemory(&disp, mRegisters->programCounter + 1);
				if (ret != OK) {
					printf("Error reading memory\n");
					return ret;
				}
				printf("Disp is 0x%02x\n", disp);

				// Account for negative displacement
				mRegisters->programCounter += (char)disp + 2;

				mCycleCount += 3;
			}

			ret = OK;
			break;
		}

		default:
		{
			printf("No BMI implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * This instructions is used to test if one or more bits are set in a target memory location. 
 * The mask pattern in A is ANDed with the value in memory to set or clear the zero flag, 
 * but the result is not kept. Bits 7 and 6 of the value from memory are copied into the N and V flags.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Set if the result if the AND is zero
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Set to bit 6 of the memory value
 * N 	Negative Flag    	Set to bit 7 of the memory value
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Zero Page        |  0x24   |  2     |  3 
 * Absolute         |  0x2C   |  3     |  4  
 */
status_t Processor::BIT(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Find the bit differences between the accumulator and the value in zero page memory
		case BIT_ZERO_PAGE:
		{
			// Get zero page address
			uchar_t addr;
			ret = mMemoryInterface->fetchBytesFromMemory(&addr, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			// Get value stored in memory
			uchar_t compr;
			ret = mMemoryInterface->fetchBytesFromMemory(&compr, addr);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			// Compare memory value is accumulator
			uchar_t accu = mRegisters->accumulator;
			uchar_t testValue = compr & accu;
			
			// Zero flag
			if (testValue == 0) {
				mRegisters->statusFlags.Z = 1;
				printf("Setting zero flag\n");
			}
			else {
				mRegisters->statusFlags.Z = 0;
			}

			//Overflow Flag
			if (testValue & 0x40 == 0x40) {
				mRegisters->statusFlags.O = 1;
				printf("Setting overflow flag\n");
			}
			else {
				mRegisters->statusFlags.O = 0;
			}

			// Negative Flag
			if (testValue & 0x80 == 0x80) {
				mRegisters->statusFlags.N = 1;
				printf("Setting negative flag\n");
			}
			else {
				mRegisters->statusFlags.N = 0;
			}

			mRegisters->programCounter += 2;

			mCycleCount += 3;
			ret = OK;
			break;
		}

		//
		case BIT_ABSOLUTE:
		{
			printf("Instruction BIT_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}
	
		default:
		{
			printf("No BIT implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Set the carry flag to zero.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Set to 0
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0x18   |  1     |  2
 */
status_t Processor::CLC(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Sets the Carry Register to false
		case CLC_IMPLIED:
		{
			mRegisters->statusFlags.C = 0;
			mRegisters->programCounter += 1;
			mCycleCount += 2;
			ret = OK;
			break;
		}
	
		default:
		{
			printf("No CLC implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Is used at the end of a subroutine to return to the calling routine. 
 * It pulls the program counter (minus one) from the stack.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0x60   |  1     |  6
 */
status_t Processor::RTS(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Grab the last address push to the stack and set the program counter to it.
		case RTS_IMPLIED:
		{
			std::vector<uchar_t> stackData;
			ret = mMemoryInterface->PopFromStack(&stackData, &mRegisters->stackPointer, 2);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			ushort_t returnAddr = (stackData[0] << 8) | stackData[1];
			printf("return addr is 0x%x\n", returnAddr);

			// Return address stored as PC - 1;
			mRegisters->programCounter = returnAddr + 1;

			mCycleCount += 6;
			ret = OK;
			break;
		}
	
		default:
		{
			printf("No RTS implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Set the interrupt disable flag to one.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Set to 1
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0x78   |  1     |  2
 */
status_t Processor::SEI(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Set the interrupt disable flag
		case SEI_IMPLIED:
		{
			mRegisters->statusFlags.I = 1;
			mRegisters->programCounter += 1;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No SEI implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Set the decimal mode flag to one.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Set to 1
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0xF8   |  1     |  2
 */
status_t Processor::SED(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Set the decimal flag
		case SED_IMPLIED:
		{
			mRegisters->statusFlags.D = 1;
			mRegisters->programCounter += 1;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No SED implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Clears the Binary Coded Decimal flag
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Set to 0
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0xD8   |  1     |  2
 */
status_t Processor::CLD(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Clear the decimal flag
		case CLD_IMPLIED:
		{
			mRegisters->statusFlags.D = 0;
			mRegisters->programCounter += 1;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No CLD implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Pushes a copy of the status flags on to the stack.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0x08   |  1     |  3
 */
status_t Processor::PHP(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Pushes an 8 bit value containing processor flags to stack
		case PHP_IMPLIED:
		{
			// 7 	6 	5 	4 	3 	2 	1 	0
			// N 	V   -   B 	D 	I 	Z 	C
			// Bit 5 is always 1.
			uchar_t flagStatus = 0;
			flagStatus = flagStatus | mRegisters->statusFlags.C;
			flagStatus = flagStatus | mRegisters->statusFlags.Z << 1;
			flagStatus = flagStatus | mRegisters->statusFlags.I << 2;
			flagStatus = flagStatus | mRegisters->statusFlags.D << 3;
			flagStatus = flagStatus | mRegisters->statusFlags.B << 4;
			flagStatus = flagStatus | mRegisters->statusFlags.U << 5;
			flagStatus = flagStatus | mRegisters->statusFlags.O << 6;
			flagStatus = flagStatus | mRegisters->statusFlags.N << 7;

			printf("Pushing status flags 0x%x to stack\n", flagStatus);

			std::vector<uchar_t> memData;
			memData.push_back(flagStatus);
			ret = mMemoryInterface->PushToStack(&memData, &mRegisters->stackPointer, 1);
			if (ret != OK) {
				printf("Error writing to memory\n");
				return ret;
			}

			mRegisters->programCounter += 1;

			mCycleCount += 3;
			ret = OK;
			break;
		}

		default:
		{
			printf("No PHP implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Pushes a copy of the accumulator on to the stack.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0x48   |  1     |  3
 */
status_t Processor::PHA(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Pushes an the accumulator onto stack
		case PHA_IMPLIED:
		{
			std::vector<uchar_t> memData;
			memData.push_back(mRegisters->accumulator);
			ret = mMemoryInterface->PushToStack(&memData, &mRegisters->stackPointer, 1);
			if (ret != OK) {
				printf("Error writing to memory\n");
				return ret;
			}
                        printf("Pushed 0x%x to stack\n", mRegisters->accumulator);
                       
			mRegisters->programCounter += 1;

			mCycleCount += 3;
			ret = OK;
			break;
		}

		default:
		{
			printf("No PHA implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Pulls an 8 bit value from the stack and into the accumulator. The zero and negative flags are set as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Set if A = 0
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of A is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0x68   |  1     |  4
 */
status_t Processor::PLA(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Pushes an 8 bit value containing processor flags to stack
		case PLA_IMPLIED:
		{
			std::vector<uchar_t> stackValue;
                        printf("SP is 0x%x\n", mRegisters->stackPointer);
			ret = mMemoryInterface->PopFromStack(&stackValue, &mRegisters->stackPointer, 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			printf("Setting accumulator to 0x%x\n", stackValue[0]);
			mRegisters->accumulator = stackValue[0];

			// Set status registers
			if (mRegisters->accumulator == 0) {
				printf("Zero result: Setting zero flag\n");
				mRegisters->statusFlags.Z = 1;
			}
			else {
				mRegisters->statusFlags.Z = 0;
			}
			if ((mRegisters->accumulator & 0x80) == 0x80) {
				printf("Negative result: Setting negative flag\n");
				mRegisters->statusFlags.N = 1;
			}
			else {
				mRegisters->statusFlags.N = 0;
			}

			mRegisters->programCounter += 1;

			mCycleCount += 4;
			ret = OK;
			break;
		}

		default:
		{
			printf("No PLA implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Pulls an 8 bit value from the stack and into the status registers.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0x28   |  1     |  4
 */
status_t Processor::PLP(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Pushes an 8 bit value off stack and sets status registers to it
		case PLP_IMPLIED:
		{
			std::vector<uchar_t> stackValue;
			ret = mMemoryInterface->PopFromStack(&stackValue, &mRegisters->stackPointer, 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			printf("Setting register status flags to 0x%x\n", stackValue[0]);

                       	// 7 	6 	5 	4 	3 	2 	1 	0
			// N 	V       -       B 	D 	I       Z       C
			// Bit 5 is always 1.
                        mRegisters->statusFlags.C = (stackValue[0] & 0x01) >> 0;
		        mRegisters->statusFlags.Z = (stackValue[0] & 0x02) >> 1;
			mRegisters->statusFlags.I = (stackValue[0] & 0x04) >> 2;
                        mRegisters->statusFlags.D = (stackValue[0] & 0x08) >> 3;
			mRegisters->statusFlags.B = (stackValue[0] & 0x10) >> 4;
			mRegisters->statusFlags.U = 1;
			mRegisters->statusFlags.O = (stackValue[0] & 0x40) >> 6;
			mRegisters->statusFlags.N = (stackValue[0] & 0x80) >> 7;
			
                        mRegisters->programCounter += 1;

			mCycleCount += 4;
			ret = OK;
			break;
		}

		default:
		{
			printf("No PLP implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Misc testing.
 */
void Processor::test() {
	mMemoryInterface = new MemoryInterface();
	mMemoryInterface->loadRom();

	std::vector<uchar_t> data = std::vector<uchar_t>();
	mMemoryInterface->fetchBytesFromMemory(&data, 3, 0);

	for (int i = 0; i < 3; i++) {
		printf("0x%x\n", data[i]);
	}
}
