#ifndef OPCODES_H
#define OPCODES_H

enum {

	// Load X Register
	LDX_IMMEDIATE = 0xA2,
	LDX_ZERO_PAGE = 0xA6,
	LDX_ZERO_PAGE_Y = 0xB6,
	LDX_ABSOLUTE = 0xAE,
	LDX_ABSOLUTE_Y = 0xBE,

        // Load Accumulator
	LDA_IMMEDIATE = 0xA9,
	LDA_ZERO_PAGE = 0xA5,
	LDA_ZERO_PAGE_X = 0xB5,
	LDA_ABSOLUTE = 0xAD,
	LDA_ABSOLUTE_X = 0xBD,
	LDA_ABSOLUTE_Y = 0xB9,
	LDA_INDIRECT_X = 0xA1,
	LDA_INDIRECT_Y = 0xB1,

	// Store X Register
	STX_ZERO_PAGE = 0x86,
	STX_ZERO_PAGE_Y = 0x96,
	STX_ABSOLUTE = 0x8E,

	// Store Accumulator
	STA_ZERO_PAGE = 0x85,
	STA_ZERO_PAGE_X = 0x95,
	STA_ABSOLUTE = 0x8D,
	STA_ABSOLUTE_X = 0x9D,
	STA_ABSOLUTE_Y = 0x99,
	STA_INDIRECT_X = 0x81,
	STA_INDIRECT_Y = 0x91,

	// Jump
	JMP_ABSOLUTE = 0x4C,
	JMP_INDIRECT = 0x6C,

	// Jump to Subroutine
	JSR_ABSOLUTE = 0x20,

	// No Operation
	NOP_IMPLIED = 0xEA,

	// Set Carry Flag
	SEC_IMPLIED = 0x38,

	// Branch if Carry Set
	BCS_RELATIVE = 0xB0,

	// Branch if Carry Clear
	BCC_RELATIVE = 0x90,

	// Branch if Equal
	BEQ_RELATIVE = 0xf0,

	// Branch if Not Equal
	BNE_RELATIVE = 0xd0,

	// Branch if Overflow Set
	BVS_RELATIVE = 0x70,

	// Branch if Overflow Clear
	BVC_RELATIVE = 0x50,

	// Branch if Positive
	BPL_RELATIVE = 0x10,

        // Branch if negative
        BMI_RELATIVE = 0x30,

	// Clear Carry Flag
	CLC_IMPLIED = 0x18,

	// Return from Subroutine
	RTS_IMPLIED = 0x60,

	// Set Interrupt Disable
	SEI_IMPLIED = 0x78,

	// Set Decimal Flag
	SED_IMPLIED = 0xf8,

        // Clear Decimal Flag
        CLD_IMPLIED = 0xd8,

	// Push Processor Status
	PHP_IMPLIED = 0x08,

        // Pull Processor Status
        PLP_IMPLIED = 0x28,

        // Push Accumulator
        PHA_IMPLIED = 0x48,

	// Pull Accumulator
	PLA_IMPLIED = 0x68,

	// Logical AND
	AND_IMMEDIATE = 0x29,
	AND_ZERO_PAGE = 0x25,
	AND_ZERO_PAGE_X = 0x35,
	AND_ABSOLUTE = 0x2D,
	AND_ABSOLUTE_X = 0x3D,
	AND_ABSOLUTE_Y = 0x39,
	AND_INDIRECT_X = 0x21,
	AND_INDIRECT_Y = 0x31,

    	// Logical OR
	ORA_IMMEDIATE = 0x29,
	ORA_ZERO_PAGE = 0x05,
	ORA_ZERO_PAGE_X = 0x15,
	ORA_ABSOLUTE = 0x0D,
        ORA_ABSOLUTE_X = 0x1D,
        ORA_ABSOLUTE_Y = 0x19,
	ORA_INDIRECT_X = 0x01,
	ORA_INDIRECT_Y = 0x11,

	// Compare
	CMP_IMMEDIATE = 0xC9,
	CMP_ZERO_PAGE = 0xC5,
	CMP_ZERO_PAGE_X = 0xD5,
	CMP_ABSOLUTE = 0xCD,
	CMP_ABSOLUTE_X = 0xDD,
	CMP_ABSOLUTE_Y = 0xD9,
	CMP_INDIRECT_X = 0xC1,
	CMP_INDIRECT_Y = 0xD1,

	// Bit Test
	BIT_ZERO_PAGE = 0x24,
	BIT_ABSOLUTE = 0x2C

};

#endif  //  OPCODES_H
