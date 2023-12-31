#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#define MEM_SZ 255

typedef struct FlagsState {
  uint8_t s : 1;  // Sign flag (= bit 7 of result)
  uint8_t z : 1;  // Zero flag (if result is 0)
  uint8_t a : 1;  // Auxiliary Carry flag
  uint8_t p : 1;  // Parity flag (if result is even)
  uint8_t c : 1;  // Carry flag
  uint8_t padding : 3; // 3 unused bits in 8080 flags
} FlagsState;

typedef struct State {
  // Registers
  uint8_t b; uint8_t c;
  uint8_t d; uint8_t e;
  uint8_t h; uint8_t l;  // "register M" is the memory location pointed to by HL, i.e. Mem[(h,l)]
  uint8_t a;
  uint16_t sp; // Stack Pointer
  uint16_t pc; // Program Counter

  // Memory
  uint8_t* mem;

  // Flags
  FlagsState flags;
  uint8_t enable;
} State;

State* State_NEW()
{
  State* s = malloc(sizeof(State));
  uint8_t* memory = malloc(MEM_SZ * sizeof(uint8_t));
  FlagsState flags = {0}; // { .s = 1, .z = 1, .a = 1, .p = 1, .c = 1, .padding = 0 };
  s->pc = 0;
  s->sp = 0;
  s->mem = memory;
  s->flags = flags;
  s->enable = 0;
  s->b = 0; s->c = 0;
  s->d = 0; s->e = 0;
  s->h = 0; s->l = 0;
  s->a = 0;
  return s;
}

void State_PRINT(State* s)
{
  printf("=== CPU State ===\n");
  char flagStr[6];
  if (s->flags.s) flagStr[0] = 'S'; else flagStr[0] = ' ';
  if (s->flags.z) flagStr[1] = 'Z'; else flagStr[1] = ' ';
  if (s->flags.a) flagStr[2] = 'A'; else flagStr[2] = ' ';
  if (s->flags.p) flagStr[3] = 'P'; else flagStr[3] = ' ';
  if (s->flags.c) flagStr[4] = 'C'; else flagStr[4] = ' ';
  flagStr[5] = '\0';
  
  printf("PC: %02x\n", s->pc);
  printf("SP: %02x    A: %02x\n", s->sp, s->a);
  printf("Registers:\n\tBC: %02x %02x\n\tDE: %02x %02x\n\tHL: %02x %02x\n", s->b, s->c, s->d, s->e, s->h, s->l);
  printf("Flags: %s\n", flagStr);
}

void State_PRINTMEM(State* s)
{
  printf("=== Memory Dump ===\n");
  printf("   |  00 01 02 03  04 05 06 07  08 09 0a 0b  0c 0d 0e 0f\n");
  printf("---+----------------------------------------------------\n");
  for (int i = 0; i < MEM_SZ; i += 16) {
    printf("%02x |  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x  %02x %02x %02x %02x\n", i, *(s->mem + i), *(s->mem + i + 1), *(s->mem + i + 2), *(s->mem + i + 3), *(s->mem + i + 4), *(s->mem + i + 5), *(s->mem + i + 6), *(s->mem + i + 7), *(s->mem + i + 8), *(s->mem + i + 9), *(s->mem + i + 10), *(s->mem + i + 11), *(s->mem + i + 12), *(s->mem + i + 13), *(s->mem + i + 14), *(s->mem + i + 15));
  }
  
}

uint8_t* getRegisterFromCode(State* s, uint8_t reg)
{
  switch (reg) {
  case 0b000:
    return &(s->b);
  case 0b001:
    return &(s->c);
  case 0b010:
    return &(s->d);
  case 0b011:
    return &(s->e);
  case 0b100:
    return &(s->h);
  case 0b101:
    return &(s->l);
  case 0b110:
    // Memref
    uint8_t addr = (s->h << 8) + s->l;
    return &(s->mem[addr]);
  case 0b111:
    return &(s->a);
  default:
    printf("ERROR: Invalid register code %02x.\n", reg);
    exit(1);
  }
}

void InstructionNotImplemented(State* s)
{
  s->pc--;
  printf("ERROR: Instruction not implemented.\n");
  exit(1);
}

void InstructionMOV(State* s, uint8_t dst, uint8_t src)
{
  uint8_t* dstReg = getRegisterFromCode(s, dst);
  uint8_t* srcReg = getRegisterFromCode(s, src);
  *dstReg = *srcReg;
}

int processInstruction(State* s)
{
  // Extract operation
  unsigned char* opcode = &s->mem[s->pc];

  // Execute
  s->pc++;
  // 1. Process for hardcoded instructions
  switch (*opcode) {
  case 0x00: return s->pc; // NOP
  case 0x27: InstructionNotImplemented(s); // DAA
  case 0x2f: InstructionNotImplemented(s); // CMA
  case 0x76: InstructionNotImplemented(s); // HLT
  case 0xf9: InstructionNotImplemented(s); // SPHL
  case 0xe9: InstructionNotImplemented(s); // PCHL
  case 0xe3: InstructionNotImplemented(s); // XTHL
  case 0xeb: InstructionNotImplemented(s); // XCHG
  }

  // 2. Split opcode
  uint8_t cmd = *opcode >> 6;
  switch (cmd) {
  case 0b00:
    InstructionNotImplemented(s);
  case 0b01:
    // 0b01_110_110 is HLT, so this MUST be MOV
    uint8_t dst = (*opcode & 0b00111000) >> 3;
    uint8_t src = (*opcode & 0b00000111);
    InstructionMOV(s, dst, src);
  }
  
}

int main(int argc, char** argv)
{
  State* cpu = State_NEW();
  State_PRINT(cpu);
  State_PRINTMEM(cpu);
  free(cpu);
}
