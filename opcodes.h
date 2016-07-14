#ifndef OPCODES_H
#define OPCODES_H

enum {

	// Load X Register
	LDX_IMMEDIATE = 0xA2,
	LDX_ZERO_PAGE = 0xA6,
	LDX_ZERO_PAGE_Y = 0xB6,
	LDX_ABSOLUTE = 0xAE,
	LDX_ABSOLUTE_Y = 0xBE,

	// Load Y Register
	LDY_IMMEDIATE = 0xA0,
	LDY_ZERO_PAGE = 0xA4,
	LDY_ZERO_PAGE_X = 0xB4,
	LDY_ABSOLUTE = 0xAC,
	LDY_ABSOLUTE_X = 0xBC,

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

	// Clear Overflow Flag
	CLV_IMPLIED = 0xb8,

	// Push Processor Status
	PHP_IMPLIED = 0x08,

    // Pull Processor Status
    PLP_IMPLIED = 0x28,

    // Push Accumulator
    PHA_IMPLIED = 0x48,

	// Pull Accumulator
	PLA_IMPLIED = 0x68,

	// Increment Y Register
	INY_IMPLIED = 0xC8,

	// Increment X Register
	INX_IMPLIED = 0xE8,

	// Decrement Y Register
	DEY_IMPLIED = 0x88,

	// Decrement X Register
	DEX_IMPLIED = 0xCA,

	// Transfer Accumulator to Y
	TAY_IMPLIED = 0xA8,

	// Transfer Accumulator to X
	TAX_IMPLIED = 0xAA,

	// Transfer Y to Accumulator
	TYA_IMPLIED = 0x98,

	// Transfer X to Accumulator
	TXA_IMPLIED = 0x8A,

	// Transfer Stack Pointer to X
	TSX_IMPLIED = 0xBA,

	// Transfer X to Stack Pointer
	TXS_IMPLIED = 0x9A,

	// Return From Interrupt
	RTI_IMPLIED = 0x40,

	// Logical Shift Right
	LSR_ACCUMULATOR = 0x4A,
	LSR_ZERO_PAGE = 0x46,
	LSR_ZERO_PAGE_X = 0x56,
	LSR_ABSOLUTE = 0x4E,
	LSR_ABSOLUTE_X = 0x5E,

	// Rotate Right
	ROR_ACCUMULATOR = 0x6A,
	ROR_ZERO_PAGE = 0x66,
	ROR_ZERO_PAGE_X = 0x76,
	ROR_ABSOLUTE = 0x6E,
	ROR_ABSOLUTE_X = 0x7E,

	// Arithmetic Shift Left
	ASL_ACCUMULATOR = 0x0A,
	ASL_ZERO_PAGE = 0x06,
	ASL_ZERO_PAGE_X = 0x16,
	ASL_ABSOLUTE = 0x0E,
	ASL_ABSOLUTE_X = 0x1E,

	// Logical AND
	AND_IMMEDIATE = 0x29,
	AND_ZERO_PAGE = 0x25,
	AND_ZERO_PAGE_X = 0x35,
	AND_ABSOLUTE = 0x2D,
	AND_ABSOLUTE_X = 0x3D,
	AND_ABSOLUTE_Y = 0x39,
	AND_INDIRECT_X = 0x21,
	AND_INDIRECT_Y = 0x31,

    // Logical Inclusive OR
	ORA_IMMEDIATE = 0x09,
	ORA_ZERO_PAGE = 0x05,
	ORA_ZERO_PAGE_X = 0x15,
	ORA_ABSOLUTE = 0x0D,
    ORA_ABSOLUTE_X = 0x1D,
    ORA_ABSOLUTE_Y = 0x19,
	ORA_INDIRECT_X = 0x01,
	ORA_INDIRECT_Y = 0x11,

	// Exclusive OR
	EOR_IMMEDIATE = 0x49,
	EOR_ZERO_PAGE = 0x45,
	EOR_ZERO_PAGE_X = 0x55,
	EOR_ABSOLUTE = 0x4D,
	EOR_ABSOLUTE_X = 0x5D,
	EOR_ABSOLUTE_Y = 0x59,
	EOR_INDIRECT_X = 0x41,
	EOR_INDIRECT_Y = 0x51,

	// Add with Carry
	ADC_IMMEDIATE = 0x69,
	ADC_ZERO_PAGE = 0x65,
	ADC_ZERO_PAGE_X = 0x75,
	ADC_ABSOLUTE = 0x6D,
	ADC_ABSOLUTE_X = 0x7D,
	ADC_ABSOLUTE_Y = 0x79,
	ADC_INDIRECT_X = 0x61,
	ADC_INDIRECT_Y = 0x71,

	// Subtract with Carry
	SBC_IMMEDIATE = 0xE9,
	SBC_ZERO_PAGE = 0xE5,
	SBC_ZERO_PAGE_X = 0xF5,
	SBC_ABSOLUTE = 0xED,
	SBC_ABSOLUTE_X = 0xFD,
	SBC_ABSOLUTE_Y = 0xF9,
	SBC_INDIRECT_X = 0xE1,
	SBC_INDIRECT_Y = 0xF1,

	// Compare
	CMP_IMMEDIATE = 0xC9,
	CMP_ZERO_PAGE = 0xC5,
	CMP_ZERO_PAGE_X = 0xD5,
	CMP_ABSOLUTE = 0xCD,
	CMP_ABSOLUTE_X = 0xDD,
	CMP_ABSOLUTE_Y = 0xD9,
	CMP_INDIRECT_X = 0xC1,
	CMP_INDIRECT_Y = 0xD1,

	// Compare Y register
	CPY_IMMEDIATE = 0xC0,
	CPY_ZERO_PAGE = 0xC4,
	CPY_ABSOLUTE = 0xCC,

	// Compare X register
	CPX_IMMEDIATE = 0xE0,
	CPX_ZERO_PAGE = 0xE4,
	CPX_ABSOLUTE = 0xEC,

	// Bit Test
	BIT_ZERO_PAGE = 0x24,
	BIT_ABSOLUTE = 0x2C

};

#endif  //  OPCODES_H
