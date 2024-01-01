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
  case 0b110: {
    uint8_t addr = (s->h << 8) + s->l;
    return &(s->mem[addr]);
  }
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
  case 0x76: InstructionNotImplemented(s); // HLT
  case 0xf9: InstructionNotImplemented(s); // SPHL
  case 0xe9: InstructionNotImplemented(s); // PCHL
  case 0xe3: InstructionNotImplemented(s); // XTHL
  case 0xeb: InstructionNotImplemented(s); // XCHG
  }

  // 2. Split opcode
  uint8_t cmd = *opcode >> 6;
  
  switch (cmd) {
  case 0b00: {
    // Identify instruction based on final bits

    // 2.1. Filter out Math operations on RP and LXI
    uint8_t finalbits = (*opcode & 0b1111);
    uint8_t rp = (*opcode & 0b00110000) >> 4; // register pair, for use in these instructions
    switch (finalbits) {
    case 0b0001: // LXI
      switch (rp) {
      case 00: // BC
	s->b = opcode[2];
	s->c = opcode[1];
	break;
      case 01: // DE
	s->d = opcode[2];
	s->e = opcode[1];
	break;
      case 10: // HL
	s->h = opcode[2];
	s->l = opcode[1];
	break;
      case 11: // SP
	s->sp = (opcode[2] << 8) + opcode[1];
	break;
      }
      s->pc += 2;
      return s->pc;
    case 0b0011: // INX
      switch (rp) {
      case 00: { // BC
	uint16_t value = (s->b << 8) + s->c + 1;
	s->b = value >> 8;
	s->c = value & 0b1111;
	break;
      }
      case 01: { // DE
	uint16_t value = (s->d << 8) + s->e + 1;
	s->d = value >> 8;
	s->e = value & 0b1111;
	break;
      }
      case 10: { // HL
	uint16_t value = (s->h << 8) + s->l + 1;
	s->h = value >> 8;
	s->l = value & 0b1111;
	break;
      }
      case 11: // SP
        s->sp++;
	break;
      }
      return s->pc;
    case 0b1001: // DAD with rp (TODO)
      InstructionNotImplemented(s);
      return s->pc;
    case 0b1011: // DCX
      switch (rp) {
      case 00: { // BC
	uint16_t value = (s->b << 8) + s->c - 1;
	s->b = value >> 8;
	s->c = value & 0b1111;
	break;
      }
      case 01: { // DE
	uint16_t value = (s->d << 8) + s->e - 1;
	s->d = value >> 8;
	s->e = value & 0b1111;
	break;
      }
      case 10: { // HL
	uint16_t value = (s->h << 8) + s->l - 1;
	s->h = value >> 8;
	s->l = value & 0b1111;
	break;
      }
      case 11: // SP
        s->sp--;
	break;
      }
    }

    // 2.2. Otherwise, identify based on last 3 bits
    finalbits &= 0b111;
    uint8_t middlebits = (*opcode & 0b00111000) >> 3;
    switch (finalbits) {
    case 0b000: InstructionNotImplemented(s);
    case 0b001: InstructionNotImplemented(s);
    case 0b010:
      switch (middlebits) {
      case 0b000: InstructionNotImplemented(s); // STAX BC
      case 0b001: InstructionNotImplemented(s); // STAX DE
      case 0b010: InstructionNotImplemented(s); // LDAX BC
      case 0b011: InstructionNotImplemented(s); // LDAX DE
      case 0b100: InstructionNotImplemented(s); // SHLD
      case 0b101: InstructionNotImplemented(s); // LHLD
      case 0b110: InstructionNotImplemented(s); // STA
      case 0b111: InstructionNotImplemented(s); // LDA
      }
      return s->pc;
    case 0b100: {
      // INR: Increment register/memref
      uint8_t* reg = getRegisterFromCode(s, middlebits);
      InstructionNotImplemented(s);
      return s->pc;
    }
    case 0b101: {
      // DCR: Decrement register/memref
      uint8_t* reg = getRegisterFromCode(s, middlebits);
      InstructionNotImplemented(s);
      return s->pc;
    }
    case 0b110:
      // MVI: Move immediate data to register middlebits
      InstructionNotImplemented(s);
      return s->pc;
    case 0b111:
      switch (middlebits) {
      case 0b000: InstructionNotImplemented(s); // RLC
      case 0b001: InstructionNotImplemented(s); // RRC
      case 0b010: InstructionNotImplemented(s); // RAL
      case 0b011: InstructionNotImplemented(s); // RAR
      case 0b100: InstructionNotImplemented(s); // DAA
      case 0b101: InstructionNotImplemented(s); // CMA
      case 0b110: InstructionNotImplemented(s); // STC
      case 0b111: InstructionNotImplemented(s); // CMC
      }
    }
  }
  case 0b01: {
    // 0b01_110_110 is HLT, so this MUST be MOV
    uint8_t dst = (*opcode & 0b00111000) >> 3;
    uint8_t src = (*opcode & 0b00000111);
    InstructionMOV(s, dst, src);
    return s->pc;
  }
  case 0b10: {
    // Reg/Memref to Accumulator Instructions
    uint8_t opr = (*opcode & 0b00111000) >> 3;
    uint8_t reg = (*opcode & 0b00000111);
    switch (opr) {
    case 0b000: InstructionNotImplemented(s); // ADD
    case 0b001: InstructionNotImplemented(s); // ADC
    case 0b010: InstructionNotImplemented(s); // SUB
    case 0b011: InstructionNotImplemented(s); // SBB
    case 0b100: InstructionNotImplemented(s); // ANA
    case 0b101: InstructionNotImplemented(s); // XRA
    case 0b110: InstructionNotImplemented(s); // ORA
    case 0b111: InstructionNotImplemented(s); // CMP
    default:
      printf("ERROR: Invalid accumulator instruction %02x\n", *opcode);
      exit(1);
    }
  }
  case 0b11: {
    // Identify instruction based on final bits

    // 2.1. Filter out 0001/0101 (Stack Operations)
    uint8_t finalbits = (*opcode & 0b1111);
    uint8_t rp = (*opcode & 0b00110000) >> 4; // register pair, for stack operations
    switch (finalbits) {
    case 0b0001: // POP
      // POP stack to regpair
      InstructionNotImplemented(s);
      return s->pc;
    case 0b0101: // PUSH
      // PUSH regpair to stack
      InstructionNotImplemented(s);
      return s->pc;
    }

    // 2.2. Otherwise, identify based on last 3 bits
    finalbits &= 0b111;
    uint8_t middlebits = (*opcode & 0b00111000) >> 3;
    switch (finalbits) {
    case 0b000:
      // Apply return operation based on middlebits
      switch (middlebits) {
      case 0b000: InstructionNotImplemented(s); // RNZ
      case 0b001: InstructionNotImplemented(s); // RZ
      case 0b010: InstructionNotImplemented(s); // RNC
      case 0b011: InstructionNotImplemented(s); // RC
      case 0b100: InstructionNotImplemented(s); // RPO
      case 0b101: InstructionNotImplemented(s); // RPE
      case 0b110: InstructionNotImplemented(s); // RP
      case 0b111: InstructionNotImplemented(s); // RM
      default: InstructionNotImplemented(s);
      }
      return s->pc;
    case 0b001:
      if (middlebits == 0b001) InstructionNotImplemented(s); // RET
      else if (middlebits == 0b111) InstructionNotImplemented(s); // SPHL
      else if (middlebits == 0b101) InstructionNotImplemented(s); // PCHL
      else InstructionNotImplemented(s);
      return s->pc;
    case 0b010:
      // Apply jump operation based on middlebits
      switch (middlebits) {
      case 0b000: InstructionNotImplemented(s); // JNZ
      case 0b001: InstructionNotImplemented(s); // JZ
      case 0b010: InstructionNotImplemented(s); // JNC
      case 0b011: InstructionNotImplemented(s); // JC
      case 0b100: InstructionNotImplemented(s); // JPO
      case 0b101: InstructionNotImplemented(s); // JPE
      case 0b110: InstructionNotImplemented(s); // JP
      case 0b111: InstructionNotImplemented(s); // JM
      default: InstructionNotImplemented(s);
      }
      return s->pc;
    case 0b011:
      if (middlebits == 0b000) InstructionNotImplemented(s); // JMP
      else if (middlebits == 0b010) InstructionNotImplemented(s); // OUT (device)
      else if (middlebits == 0b011) InstructionNotImplemented(s); // IN  (device)
      else if (middlebits == 0b100) InstructionNotImplemented(s); // XTHL
      else if (middlebits == 0b101) InstructionNotImplemented(s); // XCHG
      else if (middlebits == 0b110) InstructionNotImplemented(s); // DI
      else if (middlebits == 0b111) InstructionNotImplemented(s); // EI
      else InstructionNotImplemented(s);
      return s->pc;
    case 0b100:
      // Apply call operation based on middlebits
      switch (middlebits) {
      case 0b000: InstructionNotImplemented(s); // CNZ
      case 0b001: InstructionNotImplemented(s); // CZ
      case 0b010: InstructionNotImplemented(s); // CNC
      case 0b011: InstructionNotImplemented(s); // CC
      case 0b100: InstructionNotImplemented(s); // CPO
      case 0b101: InstructionNotImplemented(s); // CPE
      case 0b110: InstructionNotImplemented(s); // CP
      case 0b111: InstructionNotImplemented(s); // CM
      default: InstructionNotImplemented(s);
      }
      return s->pc;
    case 0b101:
      // CALL
      if (middlebits != 0b000) InstructionNotImplemented(s);
      InstructionNotImplemented(s); // Implement this
      return s-> pc;
    case 0b110:
      // Apply accumulator operation on A with immediate value
      switch (middlebits) {
      case 0b000: InstructionNotImplemented(s); // ADI
      case 0b001: InstructionNotImplemented(s); // ACI
      case 0b010: InstructionNotImplemented(s); // SUI
      case 0b011: InstructionNotImplemented(s); // SBI
      case 0b100: InstructionNotImplemented(s); // ANI
      case 0b101: InstructionNotImplemented(s); // XRI
      case 0b110: InstructionNotImplemented(s); // ORI
      case 0b111: InstructionNotImplemented(s); // CPI
      default: InstructionNotImplemented(s);	
      }
      return s->pc;
    case 0b111: {
      // RST: Jump to 00ex p000
      uint8_t addr = middlebits << 3;
      InstructionNotImplemented(s);
      return s->pc;
    }
    }
  }

    
  }
  return 0;
}

int main(int argc, char** argv)
{
  State* cpu = State_NEW();
  State_PRINT(cpu);
  
  State_PRINTMEM(cpu);
  free(cpu);
}
