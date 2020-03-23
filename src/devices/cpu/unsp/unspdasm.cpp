// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/**************************\
*
*   SunPlus u'nSP disassembler
*
*    by Ryan Holtz
*
\**************************/

#include "emu.h"
#include <stdarg.h>

/*****************************************************************************/

static const char *reg[] =
{
	"sp", "r1", "r2", "r3", "r4", "bp", "sr", "pc"
};

static const char *jmp[] =
{
	"jb", "jae", "jge", "jl", "jne", "je", "jpl", "jmi",
	"jbe", "ja", "jle", "jg", "jvc", "jvs", "jmp", "<inv>"
};

static const char *alu[] =
{
	"add",  "adc",   "sub",   "sbc",
	"cmp",  "<inv>", "neg",   "<inv>",
	"xor",  "load",  "or",    "and",
	"test", "store", "<inv>", "<inv>"
};

/*****************************************************************************/

#define OP0     (op >> 12)
#define OPA     ((op >> 9) & 7)
#define OP1     ((op >> 6) & 7)
#define OPN     ((op >> 3) & 7)
#define OPB     (op & 7)
#define OPIMM   (op & 0x3f)
#define OP2X    ((OP0 < 14 && OP1 == 4 && (OPN >= 1 && OPN <= 3)) || (OP0 == 15 && (OP1 == 1 || OP1 == 2)))

/*****************************************************************************/

#define UNSP_DASM_OK ((OP2X ? 2 : 1) | DASMFLAG_SUPPORTED)

CPU_DISASSEMBLE(unsp)
{
	uint16_t op = *(uint16_t *)oprom;
	uint16_t imm16 = *(uint16_t *)(oprom + 2);
	op = big_endianize_int16(op);
	imm16 = big_endianize_int16(imm16);

	if(OP0 < 0xf && OPA == 0x7 && OP1 < 2)
	{
		util::stream_format(stream, "%s %04x", jmp[OP0], OP1 ? (pc - OPIMM + 1) : (pc + OPIMM + 1));
		return UNSP_DASM_OK;
	}

	switch((OP1 << 4) | OP0)
	{
		// ALU, Indexed
		case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x06: case 0x08: case 0x09: case 0x0a: case 0x0b: case 0x0c: case 0x0d:
			util::stream_format(stream, "%s %s, [bp+%02x]", alu[OP0], reg[OPA], OPIMM);
			return UNSP_DASM_OK;

		// ALU, Immediate
		case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x16: case 0x18: case 0x19: case 0x1a: case 0x1b: case 0x1c:
			util::stream_format(stream, "%s %s, %02x", alu[OP0], reg[OPA], OPIMM);
			return UNSP_DASM_OK;

		// Pop / Interrupt return
		case 0x29:
			if(op == 0x9a90)
			{
				util::stream_format(stream, "retf");
				return UNSP_DASM_OK;
			}
			else if(op == 0x9a98)
			{
				util::stream_format(stream, "reti");
				return UNSP_DASM_OK;
			}
			else if((OPA + 1) < 8 && ((OPA + OPN) < 8))
			{
				util::stream_format(stream, "pop %s, %s [%s]", reg[OPA+1], reg[OPA+OPN], reg[OPB]);
				return UNSP_DASM_OK;
			}
			break;

		// Push
		case 0x2d:
			if((OPA + 1) >= OPN && OPA < (OPN + 7))
			{
				util::stream_format(stream, "push %s, %s [%s]", reg[(OPA+1)-OPN], reg[OPA], reg[OPB]);
				return UNSP_DASM_OK;
			}
			break;

		// ALU, Indirect
		case 0x30: case 0x31: case 0x32: case 0x33: case 0x34: case 0x36: case 0x38: case 0x39: case 0x3a: case 0x3b: case 0x3c: case 0x3d:
			switch(OPN & 3)
			{
				case 0:
					util::stream_format(stream, "%s %s, [%s%s]", alu[OP0], reg[OPA], (OPN & 4) ? "ds:" : "", reg[OPB]);
					return UNSP_DASM_OK;
				case 1:
					util::stream_format(stream, "%s %s, [%s%s--]", alu[OP0], reg[OPA], (OPN & 4) ? "ds:" : "", reg[OPB]);
					return UNSP_DASM_OK;
				case 2:
					util::stream_format(stream, "%s %s, [%s%s++]", alu[OP0], reg[OPA], (OPN & 4) ? "ds:" : "", reg[OPB]);
					return UNSP_DASM_OK;
				case 3:
					util::stream_format(stream, "%s %s, [%s++%s]", alu[OP0], reg[OPA], (OPN & 4) ? "ds:" : "", reg[OPB]);
					return UNSP_DASM_OK;
			}
			return UNSP_DASM_OK;

		// ALU, 16-bit ops
		case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x46: case 0x48: case 0x49: case 0x4a: case 0x4b: case 0x4c:
			switch(OPN)
			{
				// ALU, Register
				case 0:
					util::stream_format(stream, "%s %s, %s", alu[OP0], reg[OPA], reg[OPB]);
					return UNSP_DASM_OK;

				// ALU, 16-bit Immediate
				case 1:
					if(!((OP0 == 4 || OP0 == 6 || OP0 == 9 || OP0 == 12) && OPA != OPB))
					{
						if(OP0 != 4 && OP0 != 12)
						{
							util::stream_format(stream, "%s %s, %s, %04x", alu[OP0], reg[OPA], reg[OPB], imm16);
							return UNSP_DASM_OK;
						}
						else
						{
							util::stream_format(stream, "%s %s, %04x", alu[OP0], reg[OPB], imm16);
							return UNSP_DASM_OK;
						}
					}
					break;

				// ALU, Direct 16
				case 2:
					util::stream_format(stream, "%s %s, [%04x]", alu[OP0], reg[OPA], imm16);
					return UNSP_DASM_OK;

				// ALU, Direct 16
				case 3:
					util::stream_format(stream, "%s [%04x], %s, %s", alu[OP0], imm16, reg[OPA], reg[OPB]);
					return UNSP_DASM_OK;

				// ALU, Shifted
				default:
					util::stream_format(stream, "%s %s, %s asr %d", alu[OP0], reg[OPA], reg[OPB], (OPN & 3) + 1);
					return UNSP_DASM_OK;
			}
		case 0x4d:
			if((OPN == 3) && (OPA == OPB))
				util::stream_format(stream, "store [%04x], %s", imm16, reg[OPB]);
			else
				util::stream_format(stream, "<inv>");
			return UNSP_DASM_OK;

		// ALU, Shifted
		case 0x50: case 0x51: case 0x52: case 0x53: case 0x54: case 0x56: case 0x58: case 0x59: case 0x5a: case 0x5b: case 0x5c:
			util::stream_format(stream, "%s %s, %s %s %d", alu[OP0], reg[OPA], reg[OPB], (OPN & 4) ? ">>" : "<<", (OPN & 3) + 1);
			return UNSP_DASM_OK;

		// ALU, Rotated
		case 0x60: case 0x61: case 0x62: case 0x63: case 0x64: case 0x66: case 0x68: case 0x69: case 0x6a: case 0x6b: case 0x6c:
			util::stream_format(stream, "%s %s, %s %s %d", alu[OP0], reg[OPA], reg[OPB], (OPN & 4) ? "ror" : "rol", (OPN & 3) + 1);
			return UNSP_DASM_OK;

		// ALU, Direct 8
		case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x76: case 0x78: case 0x79: case 0x7a: case 0x7b: case 0x7c:
			util::stream_format(stream, "%s %s, [%02x]", alu[OP0], reg[OPA], OPIMM);
			return UNSP_DASM_OK;

		// Call
		case 0x1f:
			if(OPA == 0)
				util::stream_format(stream, "call %06x", ((OPIMM << 16) | imm16) << 1);
			else
				util::stream_format(stream, "<inv>");
			return UNSP_DASM_OK;

		// Far Jump
		case 0x2f: case 0x3f: case 0x6f: case 0x7f:
			if (OPA == 7 && OP1 == 2)
				util::stream_format(stream, "goto %06x", ((OPIMM << 16) | imm16) << 1);
			else
				util::stream_format(stream, "<inv>");
			return UNSP_DASM_OK;

		// Multiply, Unsigned * Signed
		case 0x0f:
			if(OPN == 1 && OPA != 7)
				util::stream_format(stream, "mulus %s, %s", reg[OPA], reg[OPB]);
			else
				util::stream_format(stream, "<inv>");
			return UNSP_DASM_OK;

		// Multiply, Signed * Signed
		case 0x4f:
			if(OPN == 1 && OPA != 7)
				util::stream_format(stream, "mulss %s, %s", reg[OPA], reg[OPB]);
			else
				util::stream_format(stream, "<inv>");
			return UNSP_DASM_OK;

		// Interrupt flags
		case 0x5f:
			if(OPA == 0)
			{
				switch(OPIMM)
				{
					case 0:
						util::stream_format(stream, "int off");
						break;
					case 1:
						util::stream_format(stream, "int irq");
						break;
					case 2:
						util::stream_format(stream, "int fiq");
						break;
					case 3:
						util::stream_format(stream, "int irq,fiq");
						break;
					case 8:
						util::stream_format(stream, "irq off");
						break;
					case 9:
						util::stream_format(stream, "irq on");
						break;
					case 12:
						util::stream_format(stream, "fiq off");
						break;
					case 14:
						util::stream_format(stream, "fiq on");
						break;
					case 37:
						util::stream_format(stream, "nop");
						break;
					default:
						util::stream_format(stream, "<inv>");
						break;
				}
			}
			else
				util::stream_format(stream, "<inv>");
			return UNSP_DASM_OK;
	}
	util::stream_format(stream, "<inv>");
	return UNSP_DASM_OK;
}
