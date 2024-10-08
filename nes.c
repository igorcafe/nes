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
  I_INVALID = 0,
  I_ADC,
  I_AND,
  I_ASL,
  I_BCC,
  I_BCS,
  I_BEQ,
  I_BIT,
  I_BMI,
  I_BNE,
  I_BPL,
  I_BRK,
  I_BVC,
  I_CLC,
  I_CLD,
  I_CLI,
  I_CLV,
  I_CMP,
  I_CPX,
  I_CPY,
  I_DEC,
  I_DEX,
  I_DEY,
  I_EOR,
  I_INC,
  I_INX,
  I_INY,
  I_JMP,
  I_JSR,
  I_LDA,
  I_LDX,
  I_LDY,
  I_LSR,
  I_NOP,
  I_ORA,
  I_PHA,
  I_PHP,
  I_PLA,
  I_PLP,
  I_ROL,
  I_ROR,
  I_RTI,
  I_RTS,
  I_SBC,
  I_SEC,
  I_SED,
  I_SEI,
  I_STA,
  I_STX,
  I_STY,
  I_TAX,
  I_TAY,
  I_TSX,
  I_TXA,
  I_TXS,
  I_TYA,
} instruction_t;

typedef struct instruction {
  char *fmt;
  uint8_t opcode;
  instruction_t instruction;
  addr_mode_t addr_mode;
} instruction;

instruction instructions[] = {
  {"ADC", 0x69, I_ADC, AM_IMM},
  {"ADC", 0x65, I_ADC, AM_ZPG},
  {"ADC", 0x75, I_ADC, AM_ZPX},
  {"ADC", 0x6D, I_ADC, AM_ABS},
  {"ADC", 0x7D, I_ADC, AM_ABX},
  {"ADC", 0x79, I_ADC, AM_ABY},
  {"AND", 0x29, I_AND, AM_IMM},
  {"AND", 0x25, I_AND, AM_ZPG},
  {"AND", 0x35, I_AND, AM_ZPX},
  {"AND", 0x2D, I_AND, AM_ABS},
  {"AND", 0x3D, I_AND, AM_ABX},
  {"AND", 0x39, I_AND, AM_ABY},
  {"AND", 0x21, I_AND, AM_INX},
  {"AND", 0x31, I_AND, AM_INY},
  {"ASL", 0x0A, I_ASL, AM_ACC},
  {"ASL", 0x06, I_ASL, AM_ZPG},
  {"ASL", 0x16, I_ASL, AM_ZPX},
  {"ASL", 0x0E, I_ASL, AM_ABS},
  {"ASL", 0x1E, I_ASL, AM_ABX},
  {"BCC", 0x90, I_BCC, AM_REL},
  {"BCS", 0xB0, I_BCS, AM_REL},
  {"BEQ", 0xF0, I_BEQ, AM_REL},
  {"BIT", 0x24, I_BIT, AM_ZPG},
  {"BIT", 0x2C, I_BIT, AM_ABS},
  {"BMI", 0x30, I_BMI, AM_ZPG},
  {"BNE", 0xD0, I_BNE, AM_REL},
  {"BPL", 0x10, I_BPL, AM_REL},
  {"BRK", 0x00, I_BRK, AM_IMP},
  {"BVC", 0x50, I_BVC, AM_REL},
  {"CLC", 0x18, I_CLC, AM_IMP},
  {"CLD", 0xD8, I_CLD, AM_IMP},
  {"CLI", 0x58, I_CLI, AM_IMP},
  {"CLV", 0xB8, I_CLV, AM_IMP},
  {"CMP", 0xC9, I_CMP, AM_IMM},
  {"CMP", 0xC5, I_CMP, AM_ZPG},
  {"CMP", 0xD5, I_CMP, AM_ZPX},
  {"CMP", 0xCD, I_CMP, AM_ABS},
  {"CMP", 0xDD, I_CMP, AM_ABX},
  {"CMP", 0xD9, I_CMP, AM_ABY},
  {"CMP", 0xC1, I_CMP, AM_INX},
  {"CMP", 0xD1, I_CMP, AM_INY},
  {"CPX", 0xE0, I_CPX, AM_IMM},
  {"CPX", 0xE4, I_CPX, AM_ZPG},
  {"CPX", 0xEC, I_CPX, AM_ABS},
  {"CPY", 0xC0, I_CPY, AM_IMM},
  {"CPY", 0xC4, I_CPY, AM_ZPG},
  {"CPY", 0xCC, I_CPY, AM_ABS},
  {"DEC", 0xC6, I_DEC, AM_ZPG},
  {"DEC", 0xD6, I_DEC, AM_ZPX},
  {"DEC", 0xCE, I_DEC, AM_ABS},
  {"DEC", 0xDE, I_DEC, AM_ABX},
  {"DEX", 0xCA, I_DEX, AM_IMP},
  {"DEY", 0x88, I_DEY, AM_IMP},
  {"EOR", 0x49, I_EOR, AM_IMM},
  {"EOR", 0x45, I_EOR, AM_ZPG},
  {"EOR", 0x55, I_EOR, AM_ZPX},
  {"EOR", 0x4D, I_EOR, AM_ABS},
  {"EOR", 0x5D, I_EOR, AM_ABX},
  {"EOR", 0x59, I_EOR, AM_ABY},
  {"EOR", 0x41, I_EOR, AM_INX},
  {"EOR", 0x51, I_EOR, AM_INY},
  {"INC", 0xE6, I_INC, AM_ZPG},
  {"INC", 0xF6, I_INC, AM_ZPX},
  {"INC", 0xEE, I_INC, AM_ABS},
  {"INC", 0xFE, I_INC, AM_ABX},
  {"INX", 0xE8, I_INX, AM_IMP},
  {"INY", 0xC8, I_INY, AM_IMP},
  {"JMP", 0x4C, I_JMP, AM_ABS},
  {"JMP", 0x6C, I_JMP, AM_IND},
  {"JSR", 0x20, I_JSR, AM_ABS},
  {"LDA", 0xA9, I_LDA, AM_IMM},
  {"LDA", 0xA5, I_LDA, AM_ZPG},
  {"LDA", 0xB5, I_LDA, AM_ZPX},
  {"LDA", 0xAD, I_LDA, AM_ABS},
  {"LDA", 0xBD, I_LDA, AM_ABX},
  {"LDA", 0xB9, I_LDA, AM_ABY},
  {"LDA", 0xA1, I_LDA, AM_INX},
  {"LDA", 0xB1, I_LDA, AM_INY},
  {"LDX", 0xA2, I_LDX, AM_IMM},
  {"LDX", 0xA6, I_LDX, AM_ZPG},
  {"LDX", 0xB6, I_LDX, AM_ZPY},
  {"LDX", 0xAE, I_LDX, AM_ABS},
  {"LDX", 0xBE, I_LDX, AM_ABY},
  {"LDY", 0xA0, I_LDY, AM_IMM},
  {"LDY", 0xA4, I_LDY, AM_ZPG},
  {"LDY", 0xB4, I_LDY, AM_ZPX},
  {"LDY", 0xAC, I_LDY, AM_ABS},
  {"LDY", 0xBC, I_LDY, AM_ABX},
  {"LSR", 0x4A, I_LSR, AM_ACC},
  {"LSR", 0x46, I_LSR, AM_ZPG},
  {"LSR", 0x56, I_LSR, AM_ZPX},
  {"LSR", 0x4E, I_LSR, AM_ABS},
  {"LSR", 0x5E, I_LSR, AM_ABX},
  {"NOP", 0xEA, I_NOP, AM_IMP},
  {"ORA", 0x09, I_ORA, AM_IMM},
  {"ORA", 0x05, I_ORA, AM_ZPG},
  {"ORA", 0x15, I_ORA, AM_ZPX},
  {"ORA", 0x0D, I_ORA, AM_ABS},
  {"ORA", 0x1D, I_ORA, AM_ABX},
  {"ORA", 0x19, I_ORA, AM_ABY},
  {"ORA", 0x01, I_ORA, AM_INX},
  {"ORA", 0x11, I_ORA, AM_INY},
  {"PHA", 0x48, I_PHA, AM_IMP},
  {"PHP", 0x08, I_PHP, AM_IMP},
  {"PLA", 0x68, I_PLA, AM_IMP},
  {"PLP", 0x28, I_PLP, AM_IMP},
  {"ROL", 0x2A, I_ROL, AM_ACC},
  {"ROL", 0x26, I_ROL, AM_ZPG},
  {"ROL", 0x36, I_ROL, AM_ZPX},
  {"ROL", 0x2E, I_ROL, AM_ABS},
  {"ROL", 0x3E, I_ROL, AM_ABX},
  {"ROR", 0x6A, I_ROR, AM_ACC},
  {"ROR", 0x66, I_ROR, AM_ZPG},
  {"ROR", 0x76, I_ROR, AM_ZPX},
  {"ROR", 0x6E, I_ROR, AM_ABS},
  {"ROR", 0x7E, I_ROR, AM_ABX},
  {"RTI", 0x40, I_RTI, AM_IMP},
  {"RTS", 0x60, I_RTS, AM_IMP},
  {"SBC", 0xE9, I_SBC, AM_IMM},
  {"SBC", 0xE5, I_SBC, AM_ZPG},
  {"SBC", 0xF5, I_SBC, AM_ZPX},
  {"SBC", 0xED, I_SBC, AM_ABS},
  {"SBC", 0xFD, I_SBC, AM_ABX},
  {"SBC", 0xF9, I_SBC, AM_ABY},
  {"SBC", 0xE1, I_SBC, AM_INX},
  {"SBC", 0xF1, I_SBC, AM_INY},
  {"SEC", 0x38, I_SEC, AM_IMP},
  {"SED", 0xF8, I_SED, AM_IMP},
  {"SEI", 0x78, I_SEI, AM_IMP},
  {"STA", 0x85, I_STA, AM_ZPG},
  {"STA", 0x95, I_STA, AM_ZPX},
  {"STA", 0x8D, I_STA, AM_ABS},
  {"STA", 0x9D, I_STA, AM_ABX},
  {"STA", 0x99, I_STA, AM_ABY},
  {"STA", 0x81, I_STA, AM_INX},
  {"STA", 0x91, I_STA, AM_INY},
  {"STX", 0x86, I_STX, AM_ZPG},
  {"STX", 0x96, I_STX, AM_ZPY},
  {"STX", 0x8E, I_STX, AM_ABS},
  {"STY", 0x84, I_STY, AM_ZPG},
  {"STY", 0x94, I_STY, AM_ZPX},
  {"STY", 0x8C, I_STY, AM_ABS},
  {"TAX", 0xAA, I_TAX, AM_IMP},
  {"TAY", 0xA8, I_TAY, AM_IMP},
  {"TSX", 0xBA, I_TSX, AM_IMP},
  {"TXA", 0x8A, I_TXA, AM_IMP},
  {"TXS", 0x9A, I_TXS, AM_IMP},
  {"TYA", 0x98, I_TYA, AM_IMP},
  {0},
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

  // helper variable to store current instruction being executed
  instruction instr;

  // helper variable to store byte value read
  uint8_t b;

  // helper variable to store address value read
  uint16_t addr;
} cpu;

// TODO: check the right way to do it
void cpu_init(cpu *cpu) {
  cpu->pc = 0xFFFC;
  cpu->s = 0xFF;
  cpu->p = 0;
}

void stack_push(cpu *cpu, uint8_t b) {
  cpu_write(0x0100 | cpu->s, b);
  cpu->s--;
}

void stack_push16(cpu *cpu, uint16_t n) {
  stack_push(cpu, n & 0xFF);
  stack_push(cpu, n >> 8);
}

uint8_t stack_pop(cpu *cpu) {
  cpu->s++;
  return cpu_read(0x0100 | cpu->s);
}

uint16_t stack_pop16(cpu *cpu) {
  uint16_t n = stack_pop(cpu);
  n <<= 8;
  n |= stack_pop(cpu);
  return n;
}

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
  instruction instr = {0};
  for (int i = 0; ; i++) {
    if (instructions[i].instruction == I_INVALID) {
      break;
    }
    if (instructions[i].opcode == op) {
      instr = instructions[i];
      break;
    }
  }

  if (instr.instruction == I_INVALID) {
    fprintf(stderr, "ilegal opcode: %02X\n", op);
    exit(1);
  }

  cpu->instr = instr;
  cpu->pc++;

  int8_t d;

  switch (instr.addr_mode) {
  default:
    fprintf(stderr, "ilegal addressing mode: %d\n", instr.addr_mode);
    exit(1);
    break;

  case AM_IMP:
    fprintf(stderr, "%s", instr.fmt);
    break;

  case AM_REL:
    // byte read should be treated as a signed offset to current position (PC).
    d = cpu->b;
    // minus 1 because pc moved to read byte b
    cpu->addr = cpu->pc - 1 + d;
    fprintf(stderr, "%s $%02X ; $%04X", instr.fmt, cpu->b, cpu->addr);
    break;

  case AM_ZPG:
    cpu->addr = cpu_read(cpu->pc);
    fprintf(stderr, "%s $%02X ; $%04X", instr.fmt, cpu->addr, cpu->addr);
    break;

  case AM_ZPX:
    cpu->addr = cpu_read(cpu->pc) + cpu->x;
    fprintf(stderr, "%s $%02X,X ; $%04X", instr.fmt, cpu->addr, cpu->addr);
    break;

  case AM_ZPY:
    cpu->addr = cpu_read(cpu->pc) + cpu->y;
    fprintf(stderr, "%s $%02X,Y ; $%04X", instr.fmt, cpu->addr, cpu->addr);
    break;

  case AM_IMM:
    cpu->b = cpu_read(cpu->pc);
    fprintf(stderr, "%s #%02X", instr.fmt, cpu->b);
    cpu->pc++;
    break;

  case AM_ABS:
    cpu->addr = cpu_read(cpu->pc);
    cpu->pc++;
    cpu->addr |= ((uint16_t) cpu_read(cpu->pc)) << 8;
    cpu->pc++;

    fprintf(stderr, "%s $%04X", instr.fmt, cpu->addr);
    break;

  case AM_ABX:
    cpu->addr = ((uint16_t) cpu_read(cpu->pc)) + cpu->x;
    cpu->pc++;
    cpu->addr |= ((uint16_t) cpu_read(cpu->pc)) << 8;
    cpu->pc++;

    fprintf(stderr, "%s $%04X,X", instr.fmt, cpu->addr);
    break;

  case AM_ABY:
    cpu->addr = ((uint16_t) cpu_read(cpu->pc)) + cpu->y;
    cpu->pc++;
    cpu->addr |= ((uint16_t) cpu_read(cpu->pc)) << 8;
    cpu->pc++;

    fprintf(stderr, "%s $%04X,Y", instr.fmt, cpu->addr);
    break;

  case AM_IND:
  case AM_INX:
  case AM_INY:
    TODO;
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

  case AM_ZPG:
  case AM_ZPX:
  case AM_ZPY:
  case AM_ABS:
  case AM_ABX:
  case AM_ABY:
  case AM_INX:
  case AM_INY:
  case AM_REL:
  case AM_IND:
    // address was already fetched and parsed by cpu_fetch, just read from it
    cpu->b = cpu_read(cpu->addr);
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
  case AM_ZPX:
  case AM_ZPY:
  case AM_ABS:
  case AM_ABX:
  case AM_ABY:
  case AM_INX:
  case AM_INY:
  case AM_REL:
  case AM_IND:
    cpu_write(cpu->addr, cpu->b);
    break;
  }
}

// many of the instructions are incomplete or wrong, but the idea is that
// nestest will find those errors, so i don't have to worry for now
void cpu_exec(cpu *cpu) {
  switch (cpu->instr.instruction) {
  default:
    TODO;

  case I_ADC:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->a += cpu->b;
    break;

  case I_AND:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->a &= cpu->b;
    break;

  case I_ASL:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->b <<= 1;
    cpu_write_addr_mode(cpu, cpu->instr.addr_mode);
    break;

  case I_BCC:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    if (!(cpu->p & F_C)) {
      cpu->pc += (int8_t) cpu->b;
    }
    break;

  case I_BCS:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    if (cpu->p & F_C) {
      cpu->pc += (int8_t) cpu->b;
    }
    break;

  case I_BEQ:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    if (cpu->p & F_Z) {
      cpu->pc += (int8_t) cpu->b;
    }
    break;

  case I_BIT:
    TODO;

  case I_BMI:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    if (cpu->p & F_N) {
      cpu->pc += (int8_t) cpu->b;
    }
    break;

  case I_BNE:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    if (cpu->p & F_Z) {
      cpu->pc += (int8_t) cpu->b;
    }
    break;

  case I_BPL:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    if (!(cpu->p & F_N)) {
      cpu->pc += (int8_t) cpu->b;
    }
    break;

  case I_BRK:
    TODO;

  case I_BVC:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    if (!(cpu->p & F_V)) {
      cpu->pc += (int8_t) cpu->b;
    }
    break;

  case I_CLC:
    cpu->p &= ~F_C;
    break;

  case I_CLD:
    cpu->p &= ~F_D;
    break;

  case I_CLI:
    cpu->p &= ~F_I;
    break;

  case I_CLV:
    cpu->p &= ~F_V;
    break;

  case I_CMP:
  case I_CPX:
  case I_CPY:
    TODO;

  case I_DEC:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->b--;
    cpu_write_addr_mode(cpu, cpu->instr.addr_mode);
    break;

  case I_DEX:
    cpu->x--;
    break;

  case I_DEY:
    cpu->y--;
    break;

  case I_EOR:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->a ^= cpu->b;
    break;

  case I_INC:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->b++;
    cpu_write_addr_mode(cpu, cpu->instr.addr_mode);
    break;

  case I_INX:
    cpu->x++;
    break;

  case I_INY:
    cpu->y++;
    break;

  case I_JMP:
    cpu->pc = cpu->addr;
    break;

  case I_JSR:
    stack_push16(cpu, cpu->pc);
    break;

  case I_LDA:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->a = cpu->b;
    cpu->p |= cpu->a & F_N;
    break;

  case I_LDX:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->x = cpu->b;
    cpu->p |= cpu->a & F_N;
    break;

  case I_LDY:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->y = cpu->b;
    cpu->p |= cpu->a & F_N;
    break;

  case I_LSR:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->b >>= 1;
    cpu_write_addr_mode(cpu, cpu->instr.addr_mode);
    break;

  case I_NOP:
    break;

  case I_ORA:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->a |= cpu->b;
    break;

  case I_PHA:
    stack_push(cpu, cpu->a);
    break;

  case I_PHP:
    stack_push(cpu, cpu->p);
    break;

  case I_PLA:
    cpu->a = stack_pop(cpu);
    break;

  case I_PLP:
    cpu->p = stack_pop(cpu);
    break;

  case I_ROR:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->b >>= 1;
    cpu_write_addr_mode(cpu, cpu->instr.addr_mode);
    break;

  case I_RTI:
    cpu->p = stack_pop(cpu);
    cpu->pc = stack_pop16(cpu);
    break;

  case I_RTS:
    cpu->pc = stack_pop16(cpu);
    break;

  case I_SBC:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->a -= cpu->b;
    break;

  case I_SEC:
    cpu->p |= F_C;
    break;

  case I_SED:
    cpu->p |= F_D;
    break;

  case I_SEI:
    cpu->p |= F_I;
    break;

  case I_STA:
    cpu->b = cpu->a;
    cpu_write_addr_mode(cpu, cpu->instr.addr_mode);
    break;

  case I_STX:
    cpu->b = cpu->x;
    cpu_write_addr_mode(cpu, cpu->instr.addr_mode);
    break;

  case I_STY:
    cpu->b = cpu->y;
    cpu_write_addr_mode(cpu, cpu->instr.addr_mode);
    break;

  case I_TAX:
    cpu->x = cpu->a;
    break;

  case I_TAY:
    cpu->y = cpu->a;
    break;

  case I_TSX:
    cpu->x = cpu->s;
    break;
  case I_TXA:
    cpu->a = cpu->x;
    break;

  case I_TXS:
    cpu->s = cpu->x;
    break;

  case I_TYA:
    cpu->a = cpu->y;
    break;

  case I_ROL:
    cpu_read_addr_mode(cpu, cpu->instr.addr_mode);
    cpu->b = (cpu->b << 1) | (cpu->b >> 7);
    cpu_write_addr_mode(cpu, cpu->instr.addr_mode);
    break;
  }
}

void test_lda() {
  printf("--> [%s] %s\n", __FILE__, __func__);
  cpu cpu;
  cpu_init(&cpu);
  cpu.p = 0x26;
  cpu.pc = 0;
  ram[0x00] = 0xA9;
  ram[0x01] = 0x85;
  cpu_fetch(&cpu);
  cpu_exec(&cpu);
  puts("load accumulator - immediate - 0x85 -> N=1");
  assert(cpu.p & F_N);
}

void test_rol() {
  printf("--> [%s] %s\n", __FILE__, __func__);
  cpu cpu;
  cpu_init(&cpu);
  cpu.p = 0x26;
  cpu.pc = 0;
  ram[0x00] = 0x26;
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
}
