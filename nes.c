#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>
#include <string.h>

#define TODO                                                            \
  fprintf(stderr, "TODO! %s - %s:%d\n", __func__, __FILE__, __LINE__);  \
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
    return;
  }
  TODO;
}

uint8_t cpu_read(uint16_t addr) {
  if (addr < 0x2000) {
    return ram[addr & 0x07FF];
  }
  TODO;
}

typedef enum addr_mode_t {
  AM_ACC,
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
} addr_mode_t;

typedef enum instruction_t {
  I_LDA,
} instruction_t;


typedef struct instruction {
  char *fmt;
  addr_mode_t addr_mode;
} instruction;

// ASL A
// b = A (read AM_ACC)
// b <<= 1
// set flags
// A = b

enum II {
  I_ADC_IMM = 0x69,
  I_ADC_ZPG = 0x65,
  I_ADC_ZPX = 0x75,
  I_ADC_ABS = 0x6D,
  I_ADC_ABX = 0x7D,
  I_ADC_ABY = 0x79,

  I_AND_IMM = 0x29,
  I_AND_ZPG = 0x25,
  I_AND_ZPX = 0x35,
  I_AND_ABS = 0x2D,
  I_AND_ABX = 0x3D,
  I_AND_ABY = 0x39,
  I_AND_INX = 0x21,
  I_AND_INY = 0x31,

  I_ASL_ACC = 0x0A,
  I_ASL_ZPG = 0x06,
  I_ASL_ZPX = 0x16,
  I_ASL_ABS = 0x0E,
  I_ASL_ABX = 0x1E,

  I_BCC_REL = 0x90,

  I_BCS_REL = 0xB0,

  I_BEQ_REL = 0xF0,

  I_BIT_ZPG = 0x24,
  I_BIT_ABS = 0x2C,

  I_BMI_ZPG = 0x30,

  I_BNE_REL = 0xD0,

  I_BPL_REL = 0x10,

  I_BRK_IMP = 0x00,

  I_BVC_REL = 0x50,

  I_CLC_IMP = 0x18,

  I_CLD_IMP = 0xD8,

  I_CLI_IMP = 0x58,

  I_CLV_IMP = 0xB8,

  I_CMP_IMM = 0xC9,
  I_CMP_ZPG = 0xC5,
  I_CMP_ZPX = 0xD5,
  I_CMP_ABS = 0xCD,
  I_CMP_ABX = 0xDD,
  I_CMP_ABY = 0xD9,
  I_CMP_INX = 0xC1,
  I_CMP_INY = 0xD1,

  CPX_IMM = 0xE0,
  CPX_ZPG = 0xE4,
  CPX_ABS = 0xEC,

  CPY_IMM = 0xC0,
  CPY_ZPG = 0xC4,
  CPY_ABS = 0xCC,

  DEC_ZPG = 0xC6,
  DEC_ZPX = 0xD6,
  DEC_ABS = 0xCE,
  DEC_ABX = 0xDE,

  DEX_IMP = 0xCA,

  DEY_IMP = 0x88,

  I_JSR_ABS = 0x20,

  I_LDA_IMM = 0xA9,
  I_LDA_ZPG = 0xA5,
  I_LDA_ZPX = 0xB5,
  I_LDA_ABS = 0xAD,
  I_LDA_ABX = 0xBD,
  I_LDA_ABY = 0xB9,
  I_LDA_INX = 0xA1,
  I_LDA_INY = 0xB1,

  I_ORA_ABS = 0x0D,
  I_ORA_ABY = 0x19,
  I_ORA_ABX = 0x1D,
  I_ORA_INX = 0x01,
  I_ORA_ZPG = 0x05,
  I_ORA_IMM = 0x09,
  I_ORA_INY = 0x11,
  I_ORA_ZPX = 0x15,

  I_PHP_IMP = 0x08,

  I_PLP_IMP = 0x28,


  I_ROL_ZPG = 0x26,
  I_ROL_ACC = 0x2A,
  I_ROL_ABS = 0x2E,

};

instruction instructions[256] = {
  [I_BRK_IMP] = {"BRK",            AM_IMP},
  [I_ORA_INX] = {"ORA ($%02X, X)", AM_INX},
  [I_ORA_ZPG] = {"ORA $%02X",      AM_ZPG},
  [I_PHP_IMP] = {"PHP",            AM_IMP},
  [I_ORA_IMM] = {"ORA #%02X",      AM_IMM},
  [I_ASL_ACC] = {"ASL A",          AM_ACC},
  [I_ORA_ABS] = {"ORA $%04X",      AM_ABS},
  [I_ASL_ABS] = {"ASL $%04X",      AM_ABS},

  [I_BPL_REL] = {"BPL %02X",      AM_REL},
  [I_ORA_INY] = {"ORA (%02X), Y", AM_INY},
  [I_ORA_ZPX] = {"ORA %02X,X",    AM_ZPX},
  [I_ASL_ZPX] = {"ASL %02X,X",    AM_ZPX},
  [I_CLC_IMP] = {"CLC",           AM_IMP},
  [I_ORA_ABY] = {"ORA $%04X, Y",  AM_ABY},
  [I_ORA_ABX] = {"ORA $%04X, X",  AM_ABX},
  [I_ASL_ABX] = {"ASL $%04X, X",  AM_ABX},

  [I_JSR_ABS] = {"JSR $%04X",      AM_ABS},
  [I_AND_INX] = {"AND ($%02X, X)", AM_INX},
  [I_BIT_ZPG] = {"BIT $%02X",      AM_ZPG},
  [I_AND_ZPG] = {"AND $%02X",      AM_ZPG},
  [I_ROL_ZPG] = {"ROL $%02X",      AM_ZPG},
  [I_PLP_IMP] = {"PLP",            AM_IMP},
  [I_AND_IMM] = {"AND #%02X",      AM_IMM},
  [I_ROL_ACC] = {"ROL A",          AM_ACC},
  [I_AND_ABS] = {"AND $%04X",      AM_ABS},
  [I_ROL_ABS] = {"ROL $%04X",      AM_ABS},

  [0x38] = {"SEC", AM_IMP},

  [0x40] = {"RTI",   AM_IMP},
  [0x4A] = {"LSR A", AM_ACC},
  [0x48] = {"PHA",   AM_IMP},

  [0x58] = {"CLI", AM_IMP},

  [0x60] = {"RTS",   AM_IMP},
  [0x68] = {"PLA",   AM_IMP},
  [0x6A] = {"ROR A", AM_ACC},

  [0x78] = {"SEI", AM_IMP},

  [0x88] = {"DEY", AM_IMP},
  [0x8A] = {"TXA", AM_IMP},

  [0x98] = {"TYA", AM_IMP},
  [0x9A] = {"TXS", AM_IMP},

  [0xA8] = {"TAY",       AM_IMP},
  [I_LDA_IMM] = {"LDA #%02X", AM_IMM},
  [0xAA] = {"TAX",       AM_IMP},

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

  // helper variable to store current opcode being executed
  uint8_t op;

  // helper variable to store byte value read
  uint8_t b;

  // helper variable to store address value read
  uint16_t addr;
} cpu;

void cpu_write_state(cpu *cpu, FILE* f) {
  char xxx[] = {'N', 'V', '-', 'B', 'D', 'I', 'Z', 'C', '\0'};
  for (int i = 0; i < 8; i++) {
  if (!((cpu->p >> (7-i)) & 1)) {
    xxx[i] = '.';
  }
  }

  fprintf(f, "A: %02X | X: %02X | Y: %02X | S: %02X | PC: %04X | P: %s\n",
      cpu->a, cpu->x, cpu->y, cpu->s, cpu->pc, xxx);
}

void cpu_fetch(cpu *cpu) {
  uint8_t op = cpu_read(cpu->pc);
  cpu->op = op;
  cpu->pc++;

  instruction instr = instructions[op];
  if (!instr.fmt) {
    fprintf(stderr, "ilegal opcode: %02X\n", op);
    exit(1);
  }

  int8_t d;

  switch (instr.addr_mode) {
  default:
    fprintf(stderr, "ilegal addressing mode: %d\n", instr.addr_mode);
    exit(1);
    break;

  case AM_IMP:
    fprintf(stderr, "%s\n", instr.fmt);
    break;

  // TODO
  case AM_INX:
  case AM_INY:
  case AM_ZPX:
  case AM_ZPY:
    break;

  case AM_REL:
    // byte read should be treated as a signed offset to current position (PC).
    d = cpu->b;
    // minus 1 because pc moved to read byte b
    cpu->addr = cpu->pc - 1 + d;
    break;

  case AM_ZPG:
    cpu->addr = cpu_read(cpu->pc);
    fprintf(stderr, instr.fmt, cpu->addr);
    break;

  case AM_IMM:
    cpu->b = cpu_read(cpu->pc);
    fprintf(stderr, instr.fmt, cpu->b);
    cpu->pc++;
    break;

  // TODO
  case AM_IND:
  case AM_ABX:
  case AM_ABY:
    break;

  case AM_ABS:
    cpu->addr = cpu_read(cpu->pc);
    cpu->pc++;
    cpu->addr |= ((uint16_t) cpu_read(cpu->pc)) << 8;
    cpu->pc++;

    fprintf(stderr, instr.fmt, cpu->b);
    break;
  }

  fprintf(stderr, "\n");
}

void cpu_read_addr_mode(cpu *cpu, addr_mode_t am) {
  switch (am) {
  default:
    fprintf(stderr, "ilegal addressing mode: %d\n", am);
    exit(1);

  case AM_IMP:
    fprintf(stderr, "called cpu_read_addr_mode but addressing mode is 'implied'");
    exit(1);

  case AM_ACC:
    cpu->b = cpu->a;
    break;

  case AM_IMM:
    // value is already in cpu->b. nothing to do.
    break;

  case AM_ABS:
  case AM_ZPG:
    // address was already fetched and parsed by cpu_fetch, just read from it
    cpu->b = cpu_read(cpu->addr);
    break;

    // TODO
  case AM_INX:
  case AM_INY:
  case AM_REL:
  case AM_ZPX:
  case AM_ZPY:
  case AM_IND:
  case AM_ABX:
  case AM_ABY:
    break;
  }
}

void cpu_write_addr_mode(cpu *cpu, addr_mode_t am) {
  switch (am) {
  default:
    fprintf(stderr, "ilegal addressing mode: %d\n", am);
    exit(1);

  case AM_IMM:
  case AM_IMP:
    fprintf(stderr, "called cpu_write_addr_mode but addressing mode is 'implied' or 'immediate'");
    exit(1);

  case AM_ACC:
    cpu->a = cpu->b;
    break;

  case AM_ZPG:
  case AM_ABS:
    // address was already parsed by cpu_fetch, just read from it
    printf("addr = %04X, b = %02X\n", cpu->addr, cpu->b);
    cpu_write(cpu->addr, cpu->b);
    break;

    // TODO
  case AM_INX:
  case AM_INY:
  case AM_REL:
  case AM_ZPX:
  case AM_ZPY:
  case AM_IND:
  case AM_ABX:
  case AM_ABY:
    break;
  }
}

void cpu_exec(cpu *cpu) {
  instruction instr = instructions[cpu->op];

  switch (cpu->op) {
  case I_LDA_IMM:
  case I_LDA_ZPG:
  case I_LDA_ZPX:
  case I_LDA_ABS:
  case I_LDA_ABX:
  case I_LDA_ABY:
  case I_LDA_INX:
  case I_LDA_INY:
    cpu_read_addr_mode(cpu, instr.addr_mode);
    cpu->a = cpu->b;
    cpu->p |= cpu->a & F_N;
    break;

  case I_ROL_ZPG:
  case I_ROL_ABS:
  case I_ROL_ACC:
    cpu_read_addr_mode(cpu, instr.addr_mode);
    cpu->b = (cpu->b << 1) | (cpu->b >> 7);
    cpu_write_addr_mode(cpu, instr.addr_mode);
    break;
  }
}

void test_lda() {
  printf("--> [%s] %s\n", __FILE__, __func__);
  cpu cpu = {0};
  cpu.p = 0x26;
  cpu.pc = 0;
  ram[0x00] = I_LDA_IMM;
  ram[0x01] = 0x85;
  cpu_fetch(&cpu);
  cpu_exec(&cpu);
  puts("load accumulator - immediate - 0x85 -> N=1");
  assert(cpu.p & F_N);
}

void test_rol() {
  printf("--> [%s] %s\n", __FILE__, __func__);
  cpu cpu = {0};
  cpu.p = 0x26;
  cpu.pc = 0;
  ram[0x00] = I_ROL_ZPG;
  ram[0x01] = 0x02;
  ram[0x02] = 0x80;
  cpu_fetch(&cpu);
  cpu_exec(&cpu);
  puts("rotate left - zero page - 0x08 -> 0x01");
  assert(ram[0x02] == 0x01);
}

int main() {
  test_lda();
  test_rol();

  cpu cpu = {0};
  cpu.p = 0x26;
  cpu.pc = 0;
  ram[0x00] = I_ROL_ZPG;
  ram[0x01] = 0x02;
  ram[0x02] = 0x80;

  /* cpu_write_state(&cpu, stderr); */
  /* cpu_fetch(&cpu); */
  /* cpu_exec(&cpu); */
  /* cpu_write_state(&cpu, stderr); */
}
