#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>

#define TODO \
  fprintf(stderr, "TODO! %s - %s:%d\n", __func__, __FILE__, __LINE__); \
  exit(-1);

enum flag {
  // Carry
  F_C = (1 << 0),

  // Zero
  F_Z = (1 << 1),

  // Disable interrupt
  F_I = (1 << 2),

  // (Unused) Decimal mode
  F_D = (1 << 3),

  // Break
  F_B = (1 << 4),

  // Unused (?)
  F_U = (1 << 5),

  // Overflow
  F_V = (1 << 6),

  // Negative
  F_N = (1 << 7),
};

uint8_t ram[0x0800];

void cpu_write(uint16_t addr, uint8_t b) {
  if (addr < 0x2000) {
	ram[addr & 0x07FF] = b;
  }
  TODO
}

uint8_t cpu_read(uint16_t addr) {
  if (addr < 0x2000) {
	return ram[addr & 0x07FF];
  }
  TODO
}

typedef enum opcode {
/* ADC */
/* Add Memory to Accumulator with Carry */

/* A + M + C -> A, C */
/* N	Z	C	I	D	V */
/* +	+	+	-	-	+ */
/* addressing	assembler	opc	bytes	cycles */
/* immediate	ADC #oper	69	2	2   */
/* zeropage	ADC oper	65	2	3   */
/* zeropage,X	ADC oper,X	75	2	4   */
/* absolute	ADC oper	6D	3	4   */
/* absolute,X	ADC oper,X	7D	3	4*  */
/* absolute,Y	ADC oper,Y	79	3	4*  */
/* (indirect,X)	ADC (oper,X)	61	2	6   */
/* (indirect),Y	ADC (oper),Y	71	2	5*  */
/* AND */
/* AND Memory with Accumulator */

/* A AND M -> A */
/* N	Z	C	I	D	V */
/* +	+	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* immediate	AND #oper	29	2	2   */
/* zeropage	AND oper	25	2	3   */
/* zeropage,X	AND oper,X	35	2	4   */
/* absolute	AND oper	2D	3	4   */
/* absolute,X	AND oper,X	3D	3	4*  */
/* absolute,Y	AND oper,Y	39	3	4*  */
/* (indirect,X)	AND (oper,X)	21	2	6   */
/* (indirect),Y	AND (oper),Y	31	2	5*  */
/* ASL */
/* Shift Left One Bit (Memory or Accumulator) */

/* C <- [76543210] <- 0 */
/* N	Z	C	I	D	V */
/* +	+	+	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* accumulator	ASL A	0A	1	2   */
/* zeropage	ASL oper	06	2	5   */
/* zeropage,X	ASL oper,X	16	2	6   */
/* absolute	ASL oper	0E	3	6   */
/* absolute,X	ASL oper,X	1E	3	7   */
/* BCC */
/* Branch on Carry Clear */

/* branch on C = 0 */
/* N	Z	C	I	D	V */
/* -	-	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* relative	BCC oper	90	2	2** */
/* BCS */
/* Branch on Carry Set */

/* branch on C = 1 */
/* N	Z	C	I	D	V */
/* -	-	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* relative	BCS oper	B0	2	2** */
/* BEQ */
/* Branch on Result Zero */

/* branch on Z = 1 */
/* N	Z	C	I	D	V */
/* -	-	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* relative	BEQ oper	F0	2	2** */
/* BIT */
/* Test Bits in Memory with Accumulator */

/* bits 7 and 6 of operand are transfered to bit 7 and 6 of SR (N,V); */
/* the zero-flag is set according to the result of the operand AND */
/* the accumulator (set, if the result is zero, unset otherwise). */
/* This allows a quick check of a few bits at once without affecting */
/* any of the registers, other than the status register (SR). */

/* A AND M -> Z, M7 -> N, M6 -> V */
/* N	Z	C	I	D	V */
/* M7	+	-	-	-	M6 */
/* addressing	assembler	opc	bytes	cycles */
/* zeropage	BIT oper	24	2	3   */
/* absolute	BIT oper	2C	3	4   */
/* BMI */
/* Branch on Result Minus */

/* branch on N = 1 */
/* N	Z	C	I	D	V */
/* -	-	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* relative	BMI oper	30	2	2** */
/* BNE */
/* Branch on Result not Zero */

/* branch on Z = 0 */
/* N	Z	C	I	D	V */
/* -	-	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* relative	BNE oper	D0	2	2** */
/* BPL */
/* Branch on Result Plus */

/* branch on N = 0 */
/* N	Z	C	I	D	V */
/* -	-	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* relative	BPL oper	10	2	2** */
/* BRK */
/* Force Break */

/* BRK initiates a software interrupt similar to a hardware */
/* interrupt (IRQ). The return address pushed to the stack is */
/* PC+2, providing an extra byte of spacing for a break mark */
/* (identifying a reason for the break.) */
/* The status register will be pushed to the stack with the break */
/* flag set to 1. However, when retrieved during RTI or by a PLP */
/* instruction, the break flag will be ignored. */
/* The interrupt disable flag is not set automatically. */

/* interrupt, */
/* push PC+2, push SR */
/* N	Z	C	I	D	V */
/* -	-	-	1	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* implied	BRK	00	1	7   */
/* BVC */
/* Branch on Overflow Clear */

/* branch on V = 0 */
/* N	Z	C	I	D	V */
/* -	-	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* relative	BVC oper	50	2	2** */
/* BVS */
/* Branch on Overflow Set */

/* branch on V = 1 */
/* N	Z	C	I	D	V */
/* -	-	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* relative	BVS oper	70	2	2** */
/* CLC */
/* Clear Carry Flag */

/* 0 -> C */
/* N	Z	C	I	D	V */
/* -	-	0	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* implied	CLC	18	1	2   */
/* CLD */
/* Clear Decimal Mode */

/* 0 -> D */
/* N	Z	C	I	D	V */
/* -	-	-	-	0	- */
/* addressing	assembler	opc	bytes	cycles */
/* implied	CLD	D8	1	2   */
/* CLI */
/* Clear Interrupt Disable Bit */

/* 0 -> I */
/* N	Z	C	I	D	V */
/* -	-	-	0	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* implied	CLI	58	1	2   */
/* CLV */
/* Clear Overflow Flag */

/* 0 -> V */
/* N	Z	C	I	D	V */
/* -	-	-	-	-	0 */
/* addressing	assembler	opc	bytes	cycles */
/* implied	CLV	B8	1	2   */
/* CMP */
/* Compare Memory with Accumulator */

/* A - M */
/* N	Z	C	I	D	V */
/* +	+	+	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* immediate	CMP #oper	C9	2	2   */
/* zeropage	CMP oper	C5	2	3   */
/* zeropage,X	CMP oper,X	D5	2	4   */
/* absolute	CMP oper	CD	3	4   */
/* absolute,X	CMP oper,X	DD	3	4*  */
/* absolute,Y	CMP oper,Y	D9	3	4*  */
/* (indirect,X)	CMP (oper,X)	C1	2	6   */
/* (indirect),Y	CMP (oper),Y	D1	2	5*  */

  // CPX (compare memory and X)
  CPX_IMM = 0xE0,
  CPX_ZPG = 0xE4,
  CPX_ABS = 0xEC,

  // CPY (compare memory and Y) - NZC
  CPY_IMM = 0xC0,
  CPY_ZPG = 0xC4,
  CPY_ABS = 0xCC,

  // DEC (decrement memory)
  DEC_ZPG = 0xC6,
  DEC_ZPX = 0xD6,
  DEC_ABS = 0xCE,
  DEC_ABX = 0xDE,

  // DEX (decrement X)
  DEX_IMP = 0xCA,

  // DEY (decrement Y)
  DEY_IMP = 0x88,


/* Y - 1 -> Y */
/* N	Z	C	I	D	V */
/* +	+	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* implied	DEY	88	1	2   */
/* EOR */
/* Exclusive-OR Memory with Accumulator */

/* A EOR M -> A */
/* N	Z	C	I	D	V */
/* +	+	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* immediate	EOR #oper	49	2	2   */
/* zeropage	EOR oper	45	2	3   */
/* zeropage,X	EOR oper,X	55	2	4   */
/* absolute	EOR oper	4D	3	4   */
/* absolute,X	EOR oper,X	5D	3	4*  */
/* absolute,Y	EOR oper,Y	59	3	4*  */
/* (indirect,X)	EOR (oper,X)	41	2	6   */
/* (indirect),Y	EOR (oper),Y	51	2	5*  */

  // INC (increment memory)
  INC_ZPG = 0xE6,
  INC_ZPX = 0xF6,
  INC_ABS = 0xEE,
  INC_ABX = 0xFE,

  // INX (increment X)
  INX_IMP = 0xE8,

  // INY (increment Y)
  INY_IMP = 0xC8,

  // JMP
  JMP_ABS = 0x4C,
  JMP_IND = 0x6C,

  // JSR
  JSR_ABS = 0x20,

  // LDA - NZ
  LDA_IMM = 0xA9,
  LDA_ZPG = 0xA5,
  LDA_ZPX = 0xB5,
  LDA_ABS = 0xAD,
  LDA_ABX = 0xBD,
  LDA_ABY = 0xB9,
  LDA_INX = 0xA1,
  LDA_INY = 0xB1,

  // LDX - NZ
  LDA_IMM = 0xA2,
  LDA_ZPG = 0xA6,
  LDA_ZPY = 0xB6,
  LDA_ABS = 0xAE,
  LDA_ABY = 0xBE,

/* LDY */
/* Load Index Y with Memory */

/* M -> Y */
/* N	Z	C	I	D	V */
/* +	+	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* immediate	LDY #oper	A0	2	2   */
/* zeropage	LDY oper	A4	2	3   */
/* zeropage,X	LDY oper,X	B4	2	4   */
/* absolute	LDY oper	AC	3	4   */
/* absolute,X	LDY oper,X	BC	3	4*  */
/* LSR */
/* Shift One Bit Right (Memory or Accumulator) */

/* 0 -> [76543210] -> C */
/* N	Z	C	I	D	V */
/* 0	+	+	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* accumulator	LSR A	4A	1	2   */
/* zeropage	LSR oper	46	2	5   */
/* zeropage,X	LSR oper,X	56	2	6   */
/* absolute	LSR oper	4E	3	6   */
/* absolute,X	LSR oper,X	5E	3	7   */

  NOP_IMP = 0xEA,

/* ORA */
/* OR Memory with Accumulator */

/* A OR M -> A */
/* N	Z	C	I	D	V */
/* +	+	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* immediate	ORA #oper	09	2	2   */
/* zeropage	ORA oper	05	2	3   */
/* zeropage,X	ORA oper,X	15	2	4   */
/* absolute	ORA oper	0D	3	4   */
/* absolute,X	ORA oper,X	1D	3	4*  */
/* absolute,Y	ORA oper,Y	19	3	4*  */
/* (indirect,X)	ORA (oper,X)	01	2	6   */
/* (indirect),Y	ORA (oper),Y	11	2	5*  */

  // push A register to stack
  PHA_IMP = 0x48,

  // push P register to stack
  PHP_IMP = 0x08,

  // pull A register from stack
  PLA_IMP = 0x68,

  // pull P register from stack, woah
  PLP_IMP = 0x28,

/* ROL */
/* Rotate One Bit Left (Memory or Accumulator) */

/* C <- [76543210] <- C */
/* N	Z	C	I	D	V */
/* +	+	+	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* accumulator	ROL A	2A	1	2   */
/* zeropage	ROL oper	26	2	5   */
/* zeropage,X	ROL oper,X	36	2	6   */
/* absolute	ROL oper	2E	3	6   */
/* absolute,X	ROL oper,X	3E	3	7   */
/* ROR */
/* Rotate One Bit Right (Memory or Accumulator) */

/* C -> [76543210] -> C */
/* N	Z	C	I	D	V */
/* +	+	+	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* accumulator	ROR A	6A	1	2   */
/* zeropage	ROR oper	66	2	5   */
/* zeropage,X	ROR oper,X	76	2	6   */
/* absolute	ROR oper	6E	3	6   */
/* absolute,X	ROR oper,X	7E	3	7   */

  // Return from interrupt
  RTI_IMP = 0x40,

  // Return from subroutine
  RTS_IMP = 0x60,

/* SBC */
/* Subtract Memory from Accumulator with Borrow */

/* A - M - C̅ -> A */
/* N	Z	C	I	D	V */
/* +	+	+	-	-	+ */
/* addressing	assembler	opc	bytes	cycles */
/* immediate	SBC #oper	E9	2	2   */
/* zeropage	SBC oper	E5	2	3   */
/* zeropage,X	SBC oper,X	F5	2	4   */
/* absolute	SBC oper	ED	3	4   */
/* absolute,X	SBC oper,X	FD	3	4*  */
/* absolute,Y	SBC oper,Y	F9	3	4*  */
/* (indirect,X)	SBC (oper,X)	E1	2	6   */
/* (indirect),Y	SBC (oper),Y	F1	2	5*  */

  // Set carry flag
  SEC_IMP = 0x38,

  // Set decimal flag
  SED_IMP = 0xF8,

  // Set interrupt disable
  SEI_IMP = 0x78,

/* STA */
/* Store Accumulator in Memory */

/* A -> M */
/* N	Z	C	I	D	V */
/* -	-	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* zeropage	STA oper	85	2	3   */
/* zeropage,X	STA oper,X	95	2	4   */
/* absolute	STA oper	8D	3	4   */
/* absolute,X	STA oper,X	9D	3	5   */
/* absolute,Y	STA oper,Y	99	3	5   */
/* (indirect,X)	STA (oper,X)	81	2	6   */
/* (indirect),Y	STA (oper),Y	91	2	6   */

/* STX */
/* Store Index X in Memory */

/* X -> M */
/* N	Z	C	I	D	V */
/* -	-	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* zeropage	STX oper	86	2	3   */
/* zeropage,Y	STX oper,Y	96	2	4   */
/* absolute	STX oper	8E	3	4   */

/* STY */
/* Sore Index Y in Memory */

/* Y -> M */
/* N	Z	C	I	D	V */
/* -	-	-	-	-	- */
/* addressing	assembler	opc	bytes	cycles */
/* zeropage	STY oper	84	2	3   */
/* zeropage,X	STY oper,X	94	2	4   */
/* absolute	STY oper	8C	3	4   */

  // TAX (transfer A to X)
  TAX_IMP = 0xAA,

  // TAY (transfer A to Y)
  TAY_IMP = 0xA8,

  // TSX (transfer S to X)
  TSX_IMP = 0xBA,

  // TXA (transfer X to Y)
  TXA_IMP = 0x8A,

  // TXS (transfer X to S)
  TXS_IMP = 0x9A,

  // TYA (transfer Y to A)
  TYA_IMP = 0x98,
} opcode;

typedef enum AM {
  AM_ABS,
  AM_ABX,
  AM_ABY,
  AM_IMM,
  AM_IMP,
  AM_IND,
  AM_INX,
  AM_INY,
  AM_REL,
  AM_ZPG,
  AM_ZPX,
  AM_ZPY,
  AM_LEN,
} AM;


typedef struct instruction {
  char *fmt;
  AM am;
} instruction;

instruction instruction[256] = {
  [0x00] = {"BRK", AM_IMP},
  [0x01] = {"ORA ($%02X, X)", AM_INX},
  [0x05] = {"ORA $%02X", AM_ZPG},
  [0x08] = {"PHP", AM_IMP},
  [0x09] = {"ORA #%02X", AM_IMM},
  [0x0A] = {"ASL A", AM_IMP},
  [0x0D] = {"ORA $%04X", AM_ABS},
  [0x0E] = {"ASL $%04X", AM_ABS},

  [0x10] = {"BPL %02X", AM_REL},
  [0x11] = {"ORA (%02X), Y", AM_INY},
  [0x15] = {"ORA %02X,X", AM_ZPX},
  [0x16] = {"ASL %02X,X", AM_ZPX},
  [0x18] = {"CLC", AM_IMP},
  [0x19] = {"ORA $%04X, Y", AM_ABY},
  [0x1D] = {"ORA $%04X, X", AM_ABX},
  [0x1E] = {"ASL $%04X, X", AM_ABX},

  [0x20] = {"JSR $%04X", AM_ABS},
  [0x21] = {"AND ($%02X, X)", AM_INX},
  [0x24] = {"BIT $%02X", AM_ZPG},
  [0x25] = {"AND $%02X", AM_ZPG},
  [0x26] = {"ROL $%02X", AM_ZPG},
  [0x28] = {"PLP", AM_IMP},
  [0x29] = {"AND #%02X", AM_IMM},
  [0x2A] = {"ROL A", AM_IMP},
  [0x2D] = {"AND $%04X", AM_ABS},

  [0x38] = {"SEC", AM_IMP},

  [0x40] = {"RTI", AM_IMP},
  [0x4A] = {"LSR A", AM_IMP},
  [0x48] = {"PHA", AM_IMP},

  [0x58] = {"CLI", AM_IMP},

  [0x60] = {"RTS", AM_IMP},
  [0x68] = {"PLA", AM_IMP},
  [0x6A] = {"ROR A", AM_IMP},

  [0x78] = {"SEI", AM_IMP},

  [0x88] = {"DEY", AM_IMP},
  [0x8A] = {"TXA", AM_IMP},

  [0x98] = {"TYA", AM_IMP},
  [0x9A] = {"TXS", AM_IMP},

  [0xA8] = {"TAY", AM_IMP},
  [0xAA] = {"TAX", AM_IMP},

  [0xB8] = {"CLV", AM_IMP},
  [0xBA] = {"TSX", AM_IMP},

  [0xC8] = {"INY", AM_IMP},
  [0xCA] = {"DEX", AM_IMP},

  [0xD8] = {"CLD", AM_IMP},

  [0xE8] = {"INX", AM_IMP},
  [0xEA] = {"NOP", AM_IMP},

  [0xF8] = {"SED", AM_IMP},
};

typedef struct cpu {
  // accumulator register
  uint8_t a;

  // general purpose register X, which has a special functionality of
  // retrieving and modifying the stack pointer
  uint8_t x;

  // general purpose register Y
  uint8_t y;

  // processor status register
  uint8_t p;

  // stack pointer
  uint8_t s;

  // program counter
  uint16_t pc;

  // helper variable to store byte value read
  uint8_t b;

  // helper variable to store address value read
  uint8_t addr;
} cpu;

void cpu_write_state(struct cpu *cpu, FILE* f) {
  char xxx[] = {'N', 'V', '-', 'B', 'D', 'I', 'Z', 'C', '\0'};
  for (int i = 0; i < 8; i++) {
	if (!((cpu->p >> (7-i)) & 1)) {
	  xxx[i] = '.';
	}
  }

  fprintf(f, "A: %02X | X: %02X | Y: %02X | S: %02X | PC: %04X | P: %s\n",
		  cpu->a, cpu->x, cpu->y, cpu->s, cpu->pc, xxx);
}

uint8_t cpu_fetch(cpu *cpu) {
  uint8_t b = cpu_read(cpu->pc);
  cpu->pc++;
  return b;
}

void cpu_lda_nn(cpu *cpu) {
  uint8_t b = cpu_fetch(cpu);
  printf("LDA, #%02X\n", b);
  cpu->a = b;

  if (b == 0) {
    cpu->p |= F_Z;
  } else {
    cpu->p &= ~F_Z;
  }

  if (b & F_N) {
    cpu->p |= F_N;
  } else {
    cpu->p &= ~F_N;
  }
}

void cpu_set_flag(cpu *cpu, enum flag flag, uint8_t val) {
  val &= 1;
  TODO
  /* cpu->p */
}

void cpu_get_flag(cpu *cpu, enum flag flag) {
  TODO
  /* return cpu->p & flag */
  /* cpu->p */
}

int main() {
  cpu cpu = {0};
  cpu.p = 0xFF;
  cpu.pc = 0;
  ram[0] = 0xA9;
  ram[1] = 0x01;

  uint8_t instr = cpu_fetch(&cpu);
  assert(instr == 0xA9);
  switch (instr) {
	
  }

  cpu_write_state(&cpu, stderr);
  cpu_lda_nn(&cpu);
  cpu_write_state(&cpu, stderr);
}
