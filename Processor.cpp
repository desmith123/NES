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

	for (int i = 0; i < 1150; i++) {
		printf("----------------\n");
		printf("Executing next instruction %d!\n", i);
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

		case LDY_IMMEDIATE:
		case LDY_ZERO_PAGE:
		case LDY_ZERO_PAGE_X:
		case LDY_ABSOLUTE:
		case LDY_ABSOLUTE_X:
		{
			printf("Load Y Instruction\n");
			ret = LDY(opcode);
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
			printf("Store X Instruction\n");
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

		case LSR_ACCUMULATOR:
		case LSR_ZERO_PAGE:
		case LSR_ZERO_PAGE_X:
		case LSR_ABSOLUTE:
		case LSR_ABSOLUTE_X:
		{
			printf("Logical shift right Instruction\n");
			ret = LSR(opcode);
			break;
		}

		case ROR_ACCUMULATOR:
		case ROR_ZERO_PAGE:
		case ROR_ZERO_PAGE_X:
		case ROR_ABSOLUTE:
		case ROR_ABSOLUTE_X:
		{
			printf("Rotate right Instruction\n");
			ret = ROR(opcode);
			break;
		}

                case ROL_ACCUMULATOR:
		case ROL_ZERO_PAGE:
		case ROL_ZERO_PAGE_X:
		case ROL_ABSOLUTE:
		case ROL_ABSOLUTE_X:
		{
			printf("Rotate left Instruction\n");
			ret = ROL(opcode);
			break;
		}

		case ASL_ACCUMULATOR:
		case ASL_ZERO_PAGE:
		case ASL_ZERO_PAGE_X:
		case ASL_ABSOLUTE:
		case ASL_ABSOLUTE_X:
		{
			printf("Arithmetic shift left Instruction\n");
			ret = ASL(opcode);
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

		case ORA_IMMEDIATE:
		case ORA_ZERO_PAGE:
		case ORA_ZERO_PAGE_X:
		case ORA_ABSOLUTE:
		case ORA_ABSOLUTE_X:
		case ORA_ABSOLUTE_Y:
		case ORA_INDIRECT_X:
		case ORA_INDIRECT_Y:
		{
			printf("Logical Inclusive OR Instruction\n");
			ret = ORA(opcode);
			break;
		}

		case EOR_IMMEDIATE:
		case EOR_ZERO_PAGE:
		case EOR_ZERO_PAGE_X:
		case EOR_ABSOLUTE:
		case EOR_ABSOLUTE_X:
		case EOR_ABSOLUTE_Y:
		case EOR_INDIRECT_X:
		case EOR_INDIRECT_Y:
		{
			printf("Exclusive OR Instruction\n");
			ret = EOR(opcode);
			break;
		}

		case ADC_IMMEDIATE:
		case ADC_ZERO_PAGE:
		case ADC_ZERO_PAGE_X:
		case ADC_ABSOLUTE:
		case ADC_ABSOLUTE_X:
		case ADC_ABSOLUTE_Y:
		case ADC_INDIRECT_X:
		case ADC_INDIRECT_Y:
		{
			printf("Add with carry Instruction\n");
			ret = ADC(opcode);
			break;
		}

		case SBC_IMMEDIATE:
		case SBC_ZERO_PAGE:
		case SBC_ZERO_PAGE_X:
		case SBC_ABSOLUTE:
		case SBC_ABSOLUTE_X:
		case SBC_ABSOLUTE_Y:
		case SBC_INDIRECT_X:
		case SBC_INDIRECT_Y:
		{
			printf("Substract with carry Instruction\n");
			ret = SBC(opcode);
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

		case CPY_IMMEDIATE:
		case CPY_ZERO_PAGE:
		case CPY_ABSOLUTE:
		{
			printf("Compare Y register Instruction\n");
			ret = CPY(opcode);
			break;
		}

		case CPX_IMMEDIATE:
		case CPX_ZERO_PAGE:
		case CPX_ABSOLUTE:
		{
			printf("Compare X register Instruction\n");
			ret = CPX(opcode);
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

		case RTI_IMPLIED:
		{
			printf("Return from interrupt instruction\n");
			ret = RTI(opcode);
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

		case CLV_IMPLIED:
		{
			printf("Clear overflow instruction\n");
			ret = CLV(opcode);
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

		case INY_IMPLIED:
		{
			printf("Increment Y register instruction\n");
			ret = INY(opcode);
			break;
		}

		case DEY_IMPLIED:
		{
			printf("Decrement Y register instruction\n");
			ret = DEY(opcode);
			break;
		}

		case DEX_IMPLIED:
		{
			printf("Decrement X register instruction\n");
			ret = DEX(opcode);
			break;
		}

		case TAX_IMPLIED:
		{
			printf("Transfer accumulator to X instruction\n");
			ret = TAX(opcode);
			break;
		}

		case TAY_IMPLIED:
		{
			printf("Transfer accumulator to Y instruction\n");
			ret = TAY(opcode);
			break;
		}

		case TYA_IMPLIED:
		{
			printf("Transfer Y to accumulator instruction\n");
			ret = TYA(opcode);
			break;
		}

		case TXA_IMPLIED:
		{
			printf("Transfer X to accumulator instruction\n");
			ret = TXA(opcode);
			break;
		}

		case TSX_IMPLIED:
		{
			printf("Transfer stack pointer to X instruction\n");
			ret = TSX(opcode);
			break;
		}

		case TXS_IMPLIED:
		{
			printf("Transfer X to stack pointer instruction\n");
			ret = TXS(opcode);
			break;
		}

		case INX_IMPLIED:
		{
			printf("Increment X register instruction\n");
			ret = INX(opcode);
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
			std::vector<uchar_t> storedAddress;
			mMemoryInterface->fetchBytesFromMemory(&storedAddress, 2, mRegisters->programCounter + 1);

			ushort_t addr = (storedAddress[0] << 8) | storedAddress[1];

			uchar_t data;
			mMemoryInterface->fetchBytesFromMemory(&data, addr);

			printf("Loaded X with 0x%x from 0x%x\n", data, addr);
 
			mRegisters->indexX = data;

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

			mRegisters->programCounter += 3;

			mCycleCount += 4;
			ret = OK;
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
 * Loads a byte of memory into the Y register setting the zero and negative flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Set if Y = 0
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of Y is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Immediate        |  0xA0   |  2     |  2
 * Zero Page        |  0xA4   |  2     |  3
 * Zero Page,Y      |  0xB4   |  2     |  4
 * Absolute         |  0xAC   |  3     |  4
 * Absolute,Y       |  0xBC   |  3     |  4 (+1 if page crossed)
 */
status_t Processor::LDY(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Loads the value of the next byte into register Y.
		case LDY_IMMEDIATE:
		{
			uchar_t instrData;
			ret = mMemoryInterface->fetchBytesFromMemory(&instrData, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			mRegisters->indexY = instrData;
			printf("Set index Y Reg to: 0x%x\n", mRegisters->indexY);

			// Set status registers
			if (mRegisters->indexY == 0) {
				printf("Zero result: Setting zero flag\n");
				mRegisters->statusFlags.Z = 1;
			}
			else {
				mRegisters->statusFlags.Z = 0;
			}
			if ((mRegisters->indexY & 0x80) == 0x80) {
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
		case LDY_ZERO_PAGE:
		{
			printf("Instruction LDY_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case LDY_ZERO_PAGE_X:
		{
			printf("Instruction LDY_ZERO_PAGE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case LDY_ABSOLUTE:
		{
			printf("Instruction LDY_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case LDY_ABSOLUTE_X:
		{
			printf("Instruction LDY_ABSOLUTE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No LDY implementation for opcode 0x%x\n", opcode);
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
                uchar_t instrAddr;
		ret = mMemoryInterface->fetchBytesFromMemory(&instrAddr, mRegisters->programCounter + 1);
		if (ret != OK) {
			printf("Error reading memory\n");
			return ret;
		}

		printf("Zero page address: 0x%x\n", instrAddr);

		uchar_t instrData;
                ret = mMemoryInterface->fetchBytesFromMemory(&instrData, instrAddr);

                printf("Setting accumulator to: 0x%x\n", instrData);
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
	case LDA_ZERO_PAGE_X:
	{
		printf("Instruction LDA_ZERO_PAGE_X not implemented\n");
		ret = ERROR_UNKNOWN;
		break;
	}

	//
	case LDA_ABSOLUTE:
	{
		std::vector<uchar_t> storedAddress;
		mMemoryInterface->fetchBytesFromMemory(&storedAddress, 2, mRegisters->programCounter + 1);

		ushort_t addr = (storedAddress[0] << 8) | storedAddress[1];

		uchar_t data;
		mMemoryInterface->fetchBytesFromMemory(&data, addr);

		printf("sp 0x%x\n", mRegisters->stackPointer);

		printf("Loaded accumulator with 0x%x from 0x%x\n", data, addr);

		mRegisters->accumulator = data;

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

		mRegisters->programCounter += 3;

		mCycleCount += 4;
		ret = OK;
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
                uchar_t offset;
		mMemoryInterface->fetchBytesFromMemory(&offset, mRegisters->programCounter + 1);

                ushort_t pageOffset = offset + mRegisters->indexX;
                printf("Offset is 0x%x\n", pageOffset);
 
                std::vector<uchar_t> addr;
		mMemoryInterface->fetchBytesFromMemory(&addr, 2, pageOffset);

                ushort_t indirectAddr = (addr[0] << 8) | addr[1];

                printf("Indirect Addr: 0x%x\n", indirectAddr);

                uchar_t val;
                mMemoryInterface->fetchBytesFromMemory(&val, indirectAddr);
                printf("Setting accumulator to: 0x%x\n", val);
		mRegisters->accumulator = val;

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

		mCycleCount += 6;
		ret = OK;
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
                        std::vector<uchar_t> addr;
			ret = mMemoryInterface->fetchBytesFromMemory(&addr, 2, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

                        ushort_t dataAddr = (addr[0] << 8) | addr[1];

                        uchar_t accu = mRegisters->accumulator;
                        printf("Storing: 0x%x in: 0x%x\n, ", accu, dataAddr);
			ret = mMemoryInterface->writeBytesToMemory(accu, dataAddr);
			if (ret != OK) {
				printf("Error writing to memory\n");
				return ret;
			}

			mRegisters->programCounter += 3;

			mCycleCount += 3;
			ret = OK;
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
                	uchar_t offset;
			mMemoryInterface->fetchBytesFromMemory(&offset, mRegisters->programCounter + 1);

                	ushort_t pageOffset = offset + mRegisters->indexX;
                	printf("Offset is 0x%x\n", pageOffset);
 
                	std::vector<uchar_t> addr;
			mMemoryInterface->fetchBytesFromMemory(&addr, 2, pageOffset);

                	ushort_t indirectAddr = (addr[0] << 8) | addr[1];

                	printf("Setting 0x%x to 0x%x\n", indirectAddr, mRegisters->accumulator);

                	uchar_t accu = mRegisters->accumulator;
			ret = mMemoryInterface->writeBytesToMemory(accu, indirectAddr);
			if (ret != OK) {
				printf("Error writing to memory\n");
				return ret;
			}

			mRegisters->programCounter += 2;

			mCycleCount += 6;
			ret = OK;
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
			printf("Next byte: 0x%x, accumulator: 0x%x\n", nextByte, accumulator);
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
 * Each of the bits in A or M is shift one place to the right. 
 * The bit that was in bit 0 is shifted into the carry flag. Bit 7 is set to zero.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Set to contents of old bit 0
 * Z 	Zero Flag 	        Set if result = 0
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of the result is set
 *
 * Addressing Modes   |  Opcode |  Bytes |  Cycles
 * Accumulator        |  0x4A   |  1     |  2
 * Zero Page          |  0x46   |  2     |  5
 * Zero Page,X        |  0x56   |  2     |  6
 * Absolute           |  0x4E   |  3     |  6
 * Absolute,X         |  0x5E   |  3     |  7
 */
status_t Processor::LSR(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Get accumulator and bit by bit AND it with next byte
		case LSR_ACCUMULATOR:
		{
			uchar_t accumulator = mRegisters->accumulator;

			if ((accumulator & 0x01) == 0x01) {
				printf("Carry: Setting carry flag\n");
				mRegisters->statusFlags.C = 1;
			}
			else {
				mRegisters->statusFlags.C = 0;
			}

			// Shift right by one and ensure bit 7 is 0
			uchar_t shift = accumulator >> 1;
			accumulator = accumulator & 0x7f;

			mRegisters->accumulator = shift;
			printf("Setting accumulator to 0x%x\n", shift);

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

			mCycleCount += 2;
			ret = OK;
			break;
		}


		//
		case LSR_ZERO_PAGE:
		{
			printf("Instruction LSR_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case LSR_ZERO_PAGE_X:
		{
			printf("Instruction LSR_ZERO_PAGE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case LSR_ABSOLUTE:
		{
			printf("Instruction LSR_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case LSR_ABSOLUTE_X:
		{
			printf("Instruction LSR_ABSOLUTE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No LSR implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Move each of the bits in either A or M one place to the left. 
 * Bit 0 is filled with the current value of the carry flag whilst the old bit 1 becomes the new carry flag value.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Set to contents of old bit 7
 * Z 	Zero Flag 	        Set if A = 0
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of the result is set
 *
 * Addressing Modes   |  Opcode |  Bytes |  Cycles
 * Accumulator        |  0x2A   |  1     |  2
 * Zero Page          |  0x26   |  2     |  5
 * Zero Page,X        |  0x36   |  2     |  6
 * Absolute           |  0x2E   |  3     |  6
 * Absolute,X         |  0x3E   |  3     |  7
 */
status_t Processor::ROL(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Get accumulator and bit by bit AND it with next byte
		case ROL_ACCUMULATOR:
		{
			uchar_t accu = mRegisters->accumulator;
			uchar_t rotateOne = mRegisters->statusFlags.C;

			if ((accu & 0x80) == 0x80) {
				printf("Carry: Setting carry flag\n");
				mRegisters->statusFlags.C = 1;
			}
			else {
				mRegisters->statusFlags.C = 0;
			}

			// Shift right by one and ensure bit 7 is 0
			uchar_t shift = accu << 1;

			if (rotateOne) {
				shift = shift | 0x01;
			}
			else {
				shift = shift & 0xfe;
			}

			mRegisters->accumulator = shift;
			printf("Setting accumulator to 0x%x\n", shift);

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

			mCycleCount += 2;
			ret = OK;
			break;
		}


		//
		case ROL_ZERO_PAGE:
		{
			printf("Instruction ROL_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ROL_ZERO_PAGE_X:
		{
			printf("Instruction ROL_ZERO_PAGE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ROL_ABSOLUTE:
		{
			printf("Instruction ROL_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ROL_ABSOLUTE_X:
		{
			printf("Instruction ROL_ABSOLUTE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No ROL implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Move each of the bits in either A or M one place to the right. 
 * Bit 7 is filled with the current value of the carry flag whilst the old bit 0 becomes the new carry flag value.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Set to contents of old bit 0
 * Z 	Zero Flag 	        Set if A = 0
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of the result is set
 *
 * Addressing Modes   |  Opcode |  Bytes |  Cycles
 * Accumulator        |  0x6A   |  1     |  2
 * Zero Page          |  0x66   |  2     |  5
 * Zero Page,X        |  0x76   |  2     |  6
 * Absolute           |  0x6E   |  3     |  6
 * Absolute,X         |  0x7E   |  3     |  7
 */
status_t Processor::ROR(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Get accumulator and bit by bit AND it with next byte
		case ROR_ACCUMULATOR:
		{
			uchar_t accu = mRegisters->accumulator;
			uchar_t rotateOne = mRegisters->statusFlags.C;

			if ((accu & 0x01) == 0x01) {
				printf("Carry: Setting carry flag\n");
				mRegisters->statusFlags.C = 1;
			}
			else {
				mRegisters->statusFlags.C = 0;
			}

			// Shift right by one and ensure bit 7 is 0
			uchar_t shift = accu >> 1;

			if (rotateOne) {
				shift = shift | 0x80;
			}
			else {
				shift = shift & 0x7f;
			}

			mRegisters->accumulator = shift;
			printf("Setting accumulator to 0x%x\n", shift);

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

			mCycleCount += 2;
			ret = OK;
			break;
		}


		//
		case ROR_ZERO_PAGE:
		{
			printf("Instruction ROR_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ROR_ZERO_PAGE_X:
		{
			printf("Instruction ROR_ZERO_PAGE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ROR_ABSOLUTE:
		{
			printf("Instruction ROR_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ROR_ABSOLUTE_X:
		{
			printf("Instruction ROR_ABSOLUTE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No ROR implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Shifts all the bits of the accumulator or memory contents one bit left. 
 * Bit 0 is set to 0 and bit 7 is placed in the carry flag. 
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Set to contents of old bit 7
 * Z 	Zero Flag 	        Set if result = 0
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of the result is set
 *
 * Addressing Modes   |  Opcode |  Bytes |  Cycles
 * Accumulator        |  0x0A   |  1     |  2
 * Zero Page          |  0x06   |  2     |  5
 * Zero Page,X        |  0x16   |  2     |  6
 * Absolute           |  0x0E   |  3     |  6
 * Absolute,X         |  0x1E   |  3     |  7
 */
status_t Processor::ASL(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Get accumulator and bit by bit AND it with next byte
		case ASL_ACCUMULATOR:
		{
			uchar_t accumulator = mRegisters->accumulator;

			if ((accumulator & 0x80) == 0x80) {
				printf("Carry: Setting carry flag\n");
				mRegisters->statusFlags.C = 1;
			}
			else {
				mRegisters->statusFlags.C = 0;
			}

			// Shift left by one and ensure bit 0 is 0
			uchar_t shift = accumulator << 1;
			accumulator = accumulator & 0xfe;

			mRegisters->accumulator = shift;
			printf("Setting accumulator to 0x%x\n", shift);

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

			mCycleCount += 2;
			ret = OK;
			break;
		}


		//
		case ASL_ZERO_PAGE:
		{
			printf("Instruction ASL_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ASL_ZERO_PAGE_X:
		{
			printf("Instruction ASL_ZERO_PAGE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ASL_ABSOLUTE:
		{
			printf("Instruction ASL_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ASL_ABSOLUTE_X:
		{
			printf("Instruction ASL_ABSOLUTE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No ASL implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * An inclusive OR is performed, bit by bit, on the accumulator contents using the contents of a byte of memory.
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
 * Immediate        |  0x09   |  2     |  2
 * Zero Page        |  0x05   |  2     |  3
 * Zero Page,X      |  0x15   |  2     |  4
 * Absolute         |  0x0D   |  3     |  4
 * Absolute,X       |  0x1D   |  3     |  4 (+1 if page crossed)
 * Absolute,Y       |  0x19   |  3     |  4 (+1 if page crossed)
 * (Indirect,X)     |  0x01   |  2     |  6
 * (Indirect),Y     |  0x11   |  2     |  5 (+1 if page crossed)
 */
status_t Processor::ORA(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Get accumulator and bit by bit AND it with next byte
		case ORA_IMMEDIATE:
		{
			uchar_t accumulator = mRegisters->accumulator;

			uchar_t nextByte;
			ret = mMemoryInterface->fetchBytesFromMemory(&nextByte, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			uchar_t andResult = nextByte | accumulator;
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
		case ORA_ZERO_PAGE:
		{
			printf("Instruction ORA_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ORA_ZERO_PAGE_X:
		{
			printf("Instruction ORA_ZERO_PAGE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ORA_ABSOLUTE:
		{
			printf("Instruction ORA_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ORA_ABSOLUTE_X:
		{
			printf("Instruction ORA_ABSOLUTE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ORA_ABSOLUTE_Y:
		{
			printf("Instruction ORA_ABSOLUTE_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ORA_INDIRECT_X:
		{
                 uchar_t offset;
		mMemoryInterface->fetchBytesFromMemory(&offset, mRegisters->programCounter + 1);

                ushort_t pageOffset = offset + mRegisters->indexX;
                printf("Offset is 0x%x\n", pageOffset);
 
                std::vector<uchar_t> addr;
		mMemoryInterface->fetchBytesFromMemory(&addr, 2, pageOffset);

                ushort_t indirectAddr = (addr[0] << 8) | addr[1];

                printf("Indirect Addr: 0x%x\n", indirectAddr);

                uchar_t val;
                mMemoryInterface->fetchBytesFromMemory(&val, indirectAddr);
                printf("Setting accumulator to: 0x%x\n", val);
		mRegisters->accumulator = val;

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

			mCycleCount += 6;
			ret = OK;
			break;
		}

		//
		case ORA_INDIRECT_Y:
		{
			printf("Instruction ORA_INDIRECT_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No ORA implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * An exclusive OR is performed, bit by bit, on the accumulator contents using the contents of a byte of memory.
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
 * Immediate        |  0x49   |  2     |  2
 * Zero Page        |  0x45   |  2     |  3
 * Zero Page,X      |  0x55   |  2     |  4
 * Absolute         |  0x4D   |  3     |  4
 * Absolute,X       |  0x5D   |  3     |  4 (+1 if page crossed)
 * Absolute,Y       |  0x59   |  3     |  4 (+1 if page crossed)
 * (Indirect,X)     |  0x41   |  2     |  6
 * (Indirect),Y     |  0x51   |  2     |  5 (+1 if page crossed)
 */
status_t Processor::EOR(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Get accumulator and bit by bit XOR it with next byte
		case EOR_IMMEDIATE:
		{
			uchar_t accumulator = mRegisters->accumulator;

			uchar_t nextByte;
			ret = mMemoryInterface->fetchBytesFromMemory(&nextByte, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			uchar_t andResult = nextByte ^ accumulator;
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
		case EOR_ZERO_PAGE:
		{
			printf("Instruction EOR_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case EOR_ZERO_PAGE_X:
		{
			printf("Instruction EOR_ZERO_PAGE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case EOR_ABSOLUTE:
		{
			printf("Instruction EOR_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case EOR_ABSOLUTE_X:
		{
			printf("Instruction EOR_ABSOLUTE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case EOR_ABSOLUTE_Y:
		{
			printf("Instruction EOR_ABSOLUTE_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case EOR_INDIRECT_X:
		{
			printf("Instruction EOR_INDIRECT_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case EOR_INDIRECT_Y:
		{
			printf("Instruction EOR_INDIRECT_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No EOR implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Adds the contents of a memory location to the accumulator together with the carry bit. 
 * If overflow occurs the carry bit is set, this enables multiple byte addition to be performed.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Set if overflow in bit 7
 * Z 	Zero Flag 	        Set if A = 0
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Set if sign bit is incorrect
 * N 	Negative Flag    	Set if bit 7 set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Immediate        |  0x69   |  2     |  2
 * Zero Page        |  0x65   |  2     |  3
 * Zero Page,X      |  0x75   |  2     |  4
 * Absolute         |  0x6D   |  3     |  4
 * Absolute,X       |  0x7D   |  3     |  4 (+1 if page crossed)
 * Absolute,Y       |  0x79   |  3     |  4 (+1 if page crossed)
 * (Indirect,X)     |  0x61   |  2     |  6
 * (Indirect),Y     |  0x71   |  2     |  5 (+1 if page crossed)
 */
status_t Processor::ADC(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Get accumulator and add with the carry flag
		case ADC_IMMEDIATE:
		{
			uchar_t accumulator = mRegisters->accumulator;
			uchar_t carry = mRegisters->statusFlags.C;

			uchar_t nextByte;
			ret = mMemoryInterface->fetchBytesFromMemory(&nextByte, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			printf("Accum: 0x%x, Mem: 0x%x, carry: 0x%x\n", accumulator, nextByte, carry);

			uchar_t result = accumulator + nextByte + carry;
			mRegisters->accumulator = result;
			printf("Setting accumulator to 0x%x\n", result);

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

			// Set the carry and overflow flags
			if ((accumulator <= 0x7f) && (nextByte + carry <= 0x7f) && (result >= 0x80) ||
			    (accumulator >= 0x80) && (nextByte + carry >= 0x80) && (result <= 0x80)) {
				printf("Overflow Set - Positive -> Negative\n");
				mRegisters->statusFlags.O = 1;
			}
			else {
				mRegisters->statusFlags.O = 0;
			}
			if ((accumulator >= 0x80) || (nextByte + carry >= 0x80) && (result <= 0x7f)) {
					printf("Carry set\n");
					mRegisters->statusFlags.C = 1;
			}
			else {
				mRegisters->statusFlags.C = 0;
			}

			mRegisters->programCounter += 2;

			mCycleCount += 2;
			ret = OK;
			break;
		}


		//
		case ADC_ZERO_PAGE:
		{
			printf("Instruction ADC_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ADC_ZERO_PAGE_X:
		{
			printf("Instruction ADC_ZERO_PAGE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ADC_ABSOLUTE:
		{
			printf("Instruction ADC_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ADC_ABSOLUTE_X:
		{
			printf("Instruction ADC_ABSOLUTE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ADC_ABSOLUTE_Y:
		{
			printf("Instruction ADC_ABSOLUTE_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}
	
		//
		case ADC_INDIRECT_X:
		{
			printf("Instruction ADC_INDIRECT_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case ADC_INDIRECT_Y:
		{
			printf("Instruction ADC_INDIRECT_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No ADC implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Subtracts the contents of a memory location to the accumulator together with the not of the carry bit. 
 * If overflow occurs the carry bit is clear, this enables multiple byte subtraction to be performed.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Clear if overflow in bit 7
 * Z 	Zero Flag 	        Set if A = 0
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Set if sign bit is incorrect
 * N 	Negative Flag    	Set if bit 7 set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Immediate        |  0xE9   |  2     |  2
 * Zero Page        |  0xE5   |  2     |  3
 * Zero Page,X      |  0xF5   |  2     |  4
 * Absolute         |  0xED   |  3     |  4
 * Absolute,X       |  0xFD   |  3     |  4 (+1 if page crossed)
 * Absolute,Y       |  0xF9   |  3     |  4 (+1 if page crossed)
 * (Indirect,X)     |  0xE1   |  2     |  6
 * (Indirect),Y     |  0xF1   |  2     |  5 (+1 if page crossed)
 */
status_t Processor::SBC(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Get accumulator and subtract with the carry flag
		case SBC_IMMEDIATE:
		{
			uchar_t accumulator = mRegisters->accumulator;
			uchar_t notCarry = !mRegisters->statusFlags.C;

			uchar_t nextByte;
			ret = mMemoryInterface->fetchBytesFromMemory(&nextByte, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			printf("Accum: 0x%x, Mem: 0x%x, carry: 0x%x\n", accumulator, nextByte, notCarry);

			uchar_t result = accumulator - nextByte - notCarry;
			mRegisters->accumulator = result;
			printf("Setting accumulator to 0x%x\n", result);

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

			// Set the carry and overflow flags
			if ((accumulator <= 0x7f) && ((nextByte + notCarry) >= 0x80) && (result >= 0x80) ||
				(accumulator >= 0x80) && ((nextByte + notCarry) <= 0x7f) && (result <= 0x7f)) {
				printf("Overflow set\n");
				mRegisters->statusFlags.O = 1;
			}
			else {
				mRegisters->statusFlags.O = 0;
			}
			if (accumulator < (nextByte + notCarry)) {
				printf("Carry set\n");
				mRegisters->statusFlags.C = 0;
			}
			else {
				mRegisters->statusFlags.C =	1;
			}

			mRegisters->programCounter += 2;

			mCycleCount += 2;
			ret = OK;
			break;
		}


		//
		case SBC_ZERO_PAGE:
		{
			printf("Instruction SBC_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case SBC_ZERO_PAGE_X:
		{
			printf("Instruction SBC_ZERO_PAGE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case SBC_ABSOLUTE:
		{
			printf("Instruction SBC_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case SBC_ABSOLUTE_X:
		{
			printf("Instruction SBC_ABSOLUTE_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case SBC_ABSOLUTE_Y:
		{
			printf("Instruction SBC_ABSOLUTE_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case SBC_INDIRECT_X:
		{
			printf("Instruction SBC_INDIRECT_X not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case SBC_INDIRECT_Y:
		{
			printf("Instruction SBC_INDIRECT_Y not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No SBC implementation for opcode 0x%x\n", opcode);
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

			printf("Accu 0x%x, next 0x%x\n", accumulator, nextByte);
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
 * Compares the contents of the Y register with another memory held value and sets the zero and carry flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Set if Y >= M
 * Z 	Zero Flag 	        Set if Y = M
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of the result is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Immediate        |  0xC0   |  2     |  2
 * Zero Page        |  0xC4   |  2     |  3
 * Absolute         |  0xCC   |  3     |  4
 */
status_t Processor::CPY(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Get index register Y and compare it with next byte, set flags.
		case CPY_IMMEDIATE:
		{
			uchar_t indexY = mRegisters->indexY;
			uchar_t nextByte;
			ret = mMemoryInterface->fetchBytesFromMemory(&nextByte, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			printf("indexY 0x%x, next 0x%x\n", indexY, nextByte);
			int cmp = (int)indexY - (int)nextByte;

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
		case CPY_ZERO_PAGE:
		{
			printf("Instruction CPY_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case CPY_ABSOLUTE:
		{
			printf("Instruction CPY_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}
	
		default:
		{
			printf("No CPY implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Compares the contents of the X register with another memory held value and sets the zero and carry flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Set if X >= M
 * Z 	Zero Flag 	        Set if X = M
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of the result is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Immediate        |  0xE0   |  2     |  2
 * Zero Page        |  0xE4   |  2     |  3
 * Absolute         |  0xEC   |  3     |  4
 */
status_t Processor::CPX(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Get index register X and compare it with next byte, set flags.
		case CPX_IMMEDIATE:
		{
			uchar_t indexX = mRegisters->indexX;
			uchar_t nextByte;
			ret = mMemoryInterface->fetchBytesFromMemory(&nextByte, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory\n");
				return ret;
			}

			printf("indexX 0x%x, next 0x%x\n", indexX, nextByte);
			int cmp = (int)indexX - (int)nextByte;

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
		case CPX_ZERO_PAGE:
		{
			printf("Instruction CPX_ZERO_PAGE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		//
		case CPX_ABSOLUTE:
		{
			printf("Instruction CPX_ABSOLUTE not implemented\n");
			ret = ERROR_UNKNOWN;
			break;
		}

		default:
		{
			printf("No CPX implementation for opcode 0x%x\n", opcode);
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

		// Stores the value of zero page memory address contained in the next byte into register X.
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
			std::vector<uchar_t> instrData;
			ret = mMemoryInterface->fetchBytesFromMemory(&instrData, 2, mRegisters->programCounter + 1);
			if (ret != OK) {
				printf("Error reading memory");
				return ret;
			}

			ushort_t memaddr = (instrData[0] << 8) | instrData[1];

			uchar_t regXVal = mRegisters->indexX;
			printf("Writing 0x%x to address 0x%x\n", regXVal, memaddr);
			ret = mMemoryInterface->writeBytesToMemory(regXVal, memaddr);
			if (ret != OK) {
				printf("Error writing to memory");
				return ret;
			}

			mRegisters->programCounter += 3;

			mCycleCount += 4;
			ret = OK;
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
			// 
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
				printf("Setting PC to 0x%x\n", mRegisters->programCounter);

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
 * Used at the end of an interrupt processing routine. 
 * It pulls the processor flags from the stack followed by the program counter.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Set from stack
 * Z 	Zero Flag 	        Set from stack
 * I 	Interrupt Disable 	Set from stack
 * D 	Decimal Mode Flag 	Set from stack
 * B 	Break Command    	Set from stack
 * V 	Overflow Flag   	Set from stack
 * N 	Negative Flag    	Set from stack
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * IMPLIED          |  0x40   |  1     |  6
 */
status_t Processor::RTI(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		case RTI_IMPLIED:
		{
			
			std::vector<uchar_t> stackValues;
			mMemoryInterface->PopFromStack(&stackValues, &mRegisters->stackPointer, 3);

			mRegisters->statusFlags.C = (stackValues[2] & 0x01) >> 0;
			mRegisters->statusFlags.Z = (stackValues[2] & 0x02) >> 1;
			mRegisters->statusFlags.I = (stackValues[2] & 0x04) >> 2;
			mRegisters->statusFlags.D = (stackValues[2] & 0x08) >> 3;
			mRegisters->statusFlags.B = (stackValues[2] & 0x10) >> 4;
			mRegisters->statusFlags.U = 1;
			mRegisters->statusFlags.O = (stackValues[2] & 0x40) >> 6;
			mRegisters->statusFlags.N = (stackValues[2] & 0x80) >> 7;

			ushort_t pc = stackValues[0] << 8 | stackValues[1];
			printf("Setting PC to 0x%x\n", pc);
			mRegisters->programCounter = pc;

			mCycleCount += 6;
			ret = OK;
			break;
		}

		default:
		{
			printf("No RTI implementation for opcode 0x%x\n", opcode);
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
			printf("compr 0x%x\n", compr);

			// Compare memory value and accumulator
			uchar_t accu = mRegisters->accumulator;
			uchar_t testValue = compr & accu;
			printf("test 0x%x\n", testValue);
			
			// Zero flag
			if (testValue == 0) {
				mRegisters->statusFlags.Z = 1;
				printf("Setting zero flag\n");
			}
			else {
				mRegisters->statusFlags.Z = 0;
			}

			//Overflow Flag
			if ((compr & 0x40) == 0x40) {
				mRegisters->statusFlags.O = 1;
				printf("Setting overflow flag\n");
			}
			else {
				mRegisters->statusFlags.O = 0;
			}

			// Negative Flag
			if ((compr & 0x80) == 0x80) {
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
 * Clears the overflow flag.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Not affected
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Set to 0
 * N 	Negative Flag    	Not affected
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0xB8   |  1     |  2
 */
status_t Processor::CLV(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// Clear the overflow flag
		case CLV_IMPLIED:
		{
			mRegisters->statusFlags.O = 0;
			mRegisters->programCounter += 1;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No CLV implementation for opcode 0x%x\n", opcode);
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
 * Adds one to the Y register setting the zero and negative flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Set if Y is zero
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of Y is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0xC8   |  1     |  2
 */
status_t Processor::INY(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// 
		case INY_IMPLIED:
		{
			mRegisters->indexY += 1;
			printf("Set Register Y to 0x%x\n", mRegisters->indexY);

			// Set status registers
			if (mRegisters->indexY == 0) {
				printf("Zero result: Setting zero flag\n");
				mRegisters->statusFlags.Z = 1;
			}
			else {
				mRegisters->statusFlags.Z = 0;
			}
			if ((mRegisters->indexY & 0x80) == 0x80) {
				printf("Negative result: Setting negative flag\n");
				mRegisters->statusFlags.N = 1;
			}
			else {
				mRegisters->statusFlags.N = 0;
			}

			mRegisters->programCounter += 1;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No INY implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Subtracts one from the Y register setting the zero and negative flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Set if Y is zero
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of Y is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0x88   |  1     |  2
 */
status_t Processor::DEY(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// 
		case DEY_IMPLIED:
		{
			mRegisters->indexY -= 1;
			printf("Set Register Y to 0x%x\n", mRegisters->indexY);

			// Set status registers
			if (mRegisters->indexY == 0) {
					printf("Zero result: Setting zero flag\n");
					mRegisters->statusFlags.Z = 1;
			}
			else {
				mRegisters->statusFlags.Z = 0;
			}
			if ((mRegisters->indexY & 0x80) == 0x80) {
				printf("Negative result: Setting negative flag\n");
				mRegisters->statusFlags.N = 1;
			}
			else {
				mRegisters->statusFlags.N = 0;
			}

			mRegisters->programCounter += 1;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No DEY implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Subtracts one from the X register setting the zero and negative flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Set if X is zero
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of X is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0x88   |  1     |  2
 */
status_t Processor::DEX(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// 
		case DEX_IMPLIED:
		{
			mRegisters->indexX -= 1;
			printf("Set Register X to 0x%x\n", mRegisters->indexX);

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

			mRegisters->programCounter += 1;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No DEX implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Copies the current contents of the accumulator into the X register and sets the zero and negative flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Set if X is zero
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of X is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0xAA   |  1     |  2
 */
status_t Processor::TAX(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// 
		case TAX_IMPLIED:
		{
			mRegisters->indexX = mRegisters->accumulator;
			printf("Set Register X to 0x%x\n", mRegisters->indexX);

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

			mRegisters->programCounter += 1;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No TAX implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Copies the current contents of the accumulator into the Y register and sets the zero and negative flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Set if Y is zero
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of Y is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0xA8   |  1     |  2
 */
status_t Processor::TAY(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// 
		case TAY_IMPLIED:
		{
			mRegisters->indexY = mRegisters->accumulator;
			printf("Set Register Y to 0x%x\n", mRegisters->indexY);

			// Set status registers
			if (mRegisters->indexY == 0) {
				printf("Zero result: Setting zero flag\n");
				mRegisters->statusFlags.Z = 1;
			}
			else {
				mRegisters->statusFlags.Z = 0;
			}
			if ((mRegisters->indexY & 0x80) == 0x80) {
				printf("Negative result: Setting negative flag\n");
				mRegisters->statusFlags.N = 1;
			}
			else {
				mRegisters->statusFlags.N = 0;
			}

			mRegisters->programCounter += 1;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No TAY implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Copies the current contents of the stack register into the X register and sets the zero and negative flags as appropriate.
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
 * Implied          |  0xBA   |  1     |  2
 */
status_t Processor::TSX(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// 
		case TSX_IMPLIED:
		{
			mRegisters->indexX = mRegisters->stackPointer;
			printf("Set Register X to 0x%x\n", mRegisters->indexX);

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

			mRegisters->programCounter += 1;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No TSX implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Copies the current contents of the stack register into the X register and sets the zero and negative flags as appropriate.
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
 * Implied          |  0x9A   |  1     |  2
 */
status_t Processor::TXS(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// 
		case TXS_IMPLIED:
		{
			mRegisters->stackPointer = mRegisters->indexX;
			printf("Set SP to 0x%x\n", mRegisters->stackPointer);

			mRegisters->programCounter += 1;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No TXS implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Copies the current contents of the X register into the accumulator and sets the zero and negative flags as appropriate.
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
 * Implied          |  0x8A   |  1     |  2
 */
status_t Processor::TXA(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// 
		case TXA_IMPLIED:
		{
			mRegisters->accumulator = mRegisters->indexX;
			printf("Set Accumulator to 0x%x\n", mRegisters->accumulator);

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

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No TXA implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Copies the current contents of the Y register into the accumulator and sets the zero and negative flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	       	Set if A = 0
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of A is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0x98   |  1     |  2
 */
status_t Processor::TYA(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// 
		case TYA_IMPLIED:
		{
			mRegisters->accumulator = mRegisters->indexY;
			printf("Set Accumulator to 0x%x\n", mRegisters->accumulator);

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

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No TYA implementation for opcode 0x%x\n", opcode);
			ret = ERROR_UNKNOWN;
			break;
		}
	}

	return ret;
}


/*
 * Adds one to the X register setting the zero and negative flags as appropriate.
 *
 * Processor Status after use:
 *
 * C 	Carry Flag       	Not affected
 * Z 	Zero Flag 	        Set if X is zero
 * I 	Interrupt Disable 	Not affected
 * D 	Decimal Mode Flag 	Not affected
 * B 	Break Command    	Not affected
 * V 	Overflow Flag   	Not affected
 * N 	Negative Flag    	Set if bit 7 of X is set
 *
 * Addressing Modes |  Opcode |  Bytes |  Cycles
 * Implied          |  0xE8   |  1     |  2
 */
status_t Processor::INX(uchar_t opcode) {

	status_t ret = ERROR_UNKNOWN;
	switch (opcode) {

		// 
		case INX_IMPLIED:
		{
			mRegisters->indexX += 1;
			printf("Set Register X to 0x%x\n", mRegisters->indexX);

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

			mRegisters->programCounter += 1;

			mCycleCount += 2;
			ret = OK;
			break;
		}

		default:
		{
			printf("No INX implementation for opcode 0x%x\n", opcode);
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
