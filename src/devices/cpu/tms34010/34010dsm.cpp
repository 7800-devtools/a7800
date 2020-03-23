// license:BSD-3-Clause
// copyright-holders:Zsolt Vasvari
/*
 *   A TMS34010 disassembler
 *
 *   This code written by Zsolt Vasvari for the MAME project
 *
 */

#include "emu.h"

#ifdef STANDALONE
#define PC __pc + (offset << 3)
#define OP_WORD(v) { v = filebuf[_pc>>3]; _pc += 8; v = v | (filebuf[_pc>>3] << 8); _pc += 8;}
#define PARAM_WORD(v) { v = filebuf[_pc>>3]; _pc += 8; v = v | (filebuf[_pc>>3] << 8); _pc += 8;}
#define PARAM_LONG(v) { int v1, v2; PARAM_WORD(v1); PARAM_WORD(v2); v = v1 | (v2 << 16); }
#else
#define PC __pc
#define OP_WORD(v) { v = rombase[(__pc - pcbase) >> 3] | (rombase[(__pc + 8 - pcbase) >> 3] << 8); _pc += 16; }
#define PARAM_WORD(v) { v = rambase[(__pc + 16 - pcbase) >> 3] | (rambase[(__pc + 24 - pcbase) >> 3] << 8); _pc += 16; }
#define PARAM_LONG(v) { v = rambase[(__pc + 16 - pcbase) >> 3] | (rambase[(__pc + 24 - pcbase) >> 3] << 8) | (rambase[(__pc + 32 - pcbase) >> 3] << 16) | (rambase[(__pc + 40 - pcbase) >> 3] << 24); _pc += 32; }
#define PARM2_LONG(v) { v = rambase[(__pc + 48 - pcbase) >> 3] | (rambase[(__pc + 56 - pcbase) >> 3] << 8) | (rambase[(__pc + 64 - pcbase) >> 3] << 16) | (rambase[(__pc + 72 - pcbase) >> 3] << 24); _pc += 32; }
#endif

static uint8_t rf;
static uint32_t __pc, _pc;
static uint16_t op,rs,rd;

static const uint8_t *rombase;
static const uint8_t *rambase;
static offs_t pcbase;


static void print_reg(std::ostream &stream, uint8_t reg)
{
	if (reg != 0x0f)
	{
		util::stream_format(stream, "%c%d", rf, reg);
	}
	else
	{
		stream << "SP";
	}
}

static void print_src_reg(std::ostream &stream)
{
	print_reg(stream, rs);
}

static void print_des_reg(std::ostream &stream)
{
	print_reg(stream, rd);
}

static void print_src_des_reg(std::ostream &stream)
{
	print_src_reg(stream);
	stream << ",";
	print_des_reg(stream);
}

static void print_word_parm(std::ostream &stream)
{
	uint16_t w;

	PARAM_WORD(w);

	util::stream_format(stream, "%Xh", w);
}

static void print_word_parm_1s_comp(std::ostream &stream)
{
	uint16_t w;

	PARAM_WORD(w);
	w = ~w;
	util::stream_format(stream, "%Xh", w);
}

static void print_long_parm(std::ostream &stream)
{
	uint32_t l;

	PARAM_LONG(l);
	util::stream_format(stream, "%Xh", l);
}

static void print_long_parm2(std::ostream &stream)
{
	uint32_t l;

	PARM2_LONG(l);
	util::stream_format(stream, "%Xh", l);
}

static void print_long_parm_1s_comp(std::ostream &stream)
{
	uint32_t l;

	PARAM_LONG(l);
	util::stream_format(stream, "%Xh", ~l);
}

static void print_constant(std::ostream &stream)
{
	uint8_t constant = (op >> 5) & 0x1f;

	util::stream_format(stream, "%Xh", constant);
}

static void print_constant_1_32(std::ostream &stream)
{
	uint8_t constant = (op >> 5) & 0x1f;
	if (!constant) constant = 0x20;

	util::stream_format(stream, "%Xh", constant);
}

static void print_constant_1s_comp(std::ostream &stream)
{
	uint8_t constant = (~op >> 5) & 0x1f;

	util::stream_format(stream, "%Xh", constant);
}

static void print_constant_2s_comp(std::ostream &stream)
{
	uint8_t constant = 32 - ((op >> 5) & 0x1f);

	util::stream_format(stream, "%Xh", constant);
}

static void print_relative(std::ostream &stream)
{
	uint16_t l;
	int16_t ls;

	PARAM_WORD(l);
	ls = (int16_t)l;

	util::stream_format(stream, "%Xh", PC + 32 + (ls << 4));
}

static void print_relative_8bit(std::ostream &stream)
{
	int8_t ls = (int8_t)op;

	util::stream_format(stream, "%Xh", PC + 16 + (ls << 4));
}

static void print_relative_5bit(std::ostream &stream)
{
	int8_t ls = (int8_t)((op >> 5) & 0x1f);
	if (op & 0x0400) ls = -ls;

	util::stream_format(stream, "%Xh", PC + 16 + (ls << 4));
}

static void print_field(std::ostream &stream)
{
	util::stream_format(stream, "%c", (op & 0x200) ? '1' : '0');
}

static void print_condition_code(std::ostream &stream)
{
	switch (op & 0x0f00)
	{
	case 0x0000: stream << "  "; break;  /* This is really UC (Unconditional) */
	case 0x0100: stream << "P "; break;
	case 0x0200: stream << "LS"; break;
	case 0x0300: stream << "HI"; break;
	case 0x0400: stream << "LT"; break;
	case 0x0500: stream << "GE"; break;
	case 0x0600: stream << "LE"; break;
	case 0x0700: stream << "GT"; break;
	case 0x0800: stream << "C "; break;
	case 0x0900: stream << "NC"; break;
	case 0x0a00: stream << "EQ"; break;
	case 0x0b00: stream << "NE"; break;
	case 0x0c00: stream << "V "; break;
	case 0x0d00: stream << "NV"; break;
	case 0x0e00: stream << "N "; break;
	case 0x0f00: stream << "NN"; break;
	}
}

static void print_reg_list_range(std::ostream &stream, int8_t first, int8_t last)
{
	if ((first != -1 ) && (first != last))
	{
		if ((last - first) == 1)
			stream << ",";
		else
			stream << "-";
		print_reg(stream, last);
	}
}

static void print_reg_list(std::ostream &stream, uint16_t rev)
{
	uint16_t l;
	uint8_t i;
	int8_t first = -1, last = 0;

	PARAM_WORD(l);

	for (i = 0; i  < 16; i++)
	{
		int moved;

		if (rev)
		{
			moved = l & 0x8000;
			l <<= 1;
		}
		else
		{
			moved = l & 0x01;
			l >>= 1;
		}

		if (moved)
		{
			if (first == -1)
			{
				stream << ",";
				print_reg(stream, i);
				first = i;
			}
			last = i;
		}
		else
		{
			print_reg_list_range(stream, first, last);
			first = -1;
		}
	}

	print_reg_list_range(stream, first, last);
}


static unsigned Dasm340x0(std::ostream &stream, uint32_t pc, bool is_34020)
{
	int flags = 0;
	uint8_t bad = 0;
	uint16_t subop;

	__pc = _pc = pc;

	OP_WORD(op);

	subop = (op & 0x01e0);
	rs = (op >> 5) & 0x0f;          /* Source register */
	rd =  op & 0x0f;                /* Destination register */
	rf = ((op & 0x10) ? 'B' : 'A'); /* Register file */

	switch (op & 0xfe00)
	{
	case 0x0000:
		switch (subop)
		{
		case 0x0020:
			util::stream_format(stream, "REV    ");
			print_des_reg(stream);
			break;

		case 0x0040:
			if (is_34020)
				util::stream_format(stream, "IDLE   ");
			else
				bad = 1;
			break;

		case 0x0080:
			if (is_34020)
				util::stream_format(stream, "MWAIT  ");
			else
				bad = 1;
			break;

		case 0x00e0:
			if (is_34020)
				util::stream_format(stream, "BLMOVE %d,%d", (op >> 1) & 1, op & 1);
			else
				bad = 1;
			break;

		case 0x0100:
			util::stream_format(stream, "EMU    ");
			break;

		case 0x0120:
			util::stream_format(stream, "EXGPC  ");
			print_des_reg(stream);
			break;

		case 0x0140:
			util::stream_format(stream, "GETPC  ");
			print_des_reg(stream);
			break;

		case 0x0160:
			util::stream_format(stream, "JUMP   ");
			print_des_reg(stream);
			break;

		case 0x0180:
			util::stream_format(stream, "GETST  ");
			print_des_reg(stream);
			break;

		case 0x01a0:
			util::stream_format(stream, "PUTST  ");
			print_des_reg(stream);
			break;

		case 0x01c0:
			util::stream_format(stream, "POPST  ");
			break;

		case 0x01e0:
			util::stream_format(stream, "PUSHST ");
			break;

		default:
			bad = 1;
		}
		break;


	case 0x0200:
		switch (subop)
		{
		case 0x0040:
			if (is_34020)
				util::stream_format(stream, "SETCSP ");
			else
				bad = 1;
			break;

		case 0x0060:
			if (is_34020)
				util::stream_format(stream, "SETCDP ");
			else
				bad = 1;
			break;

		case 0x0080:
			if (is_34020)
			{
				util::stream_format(stream, "RPIX   ");
				print_des_reg(stream);
			}
			else
				bad = 1;
			break;

		case 0x00a0:
			if (is_34020)
			{
				util::stream_format(stream, "EXGPS  ");
				print_des_reg(stream);
			}
			else
				bad = 1;
			break;

		case 0x00c0:
			if (is_34020)
			{
				util::stream_format(stream, "GETPS  ");
				print_des_reg(stream);
			}
			else
				bad = 1;
			break;

		case 0x00e0:
			if (is_34020)
				util::stream_format(stream, "SETCMP ");
			else
				bad = 1;
			break;

		case 0x0100:
			util::stream_format(stream, "NOP    ");
			break;

		case 0x0120:
			util::stream_format(stream, "CLRC   ");
			break;

		case 0x0140:
			util::stream_format(stream, "MOVB   @");
			print_long_parm(stream);
			stream << ",@";
			print_long_parm2(stream);
			break;

		case 0x0160:
			util::stream_format(stream, "DINT   ");
			break;

		case 0x0180:
			util::stream_format(stream, "ABS    ");
			print_des_reg(stream);
			break;

		case 0x01a0:
			util::stream_format(stream, "NEG    ");
			print_des_reg(stream);
			break;

		case 0x01c0:
			util::stream_format(stream, "NEGB   ");
			print_des_reg(stream);
			break;

		case 0x01e0:
			util::stream_format(stream, "NOT    ");
			print_des_reg(stream);
			break;

		default:
			bad = 1;
		}
		break;


	case 0x0400:
	case 0x0600:
		switch (subop)
		{
		case 0x0000:
			if (is_34020 && (op & 0xfe00) == 0x0600)
			{
				uint32_t x;
				PARAM_LONG(x);
				util::stream_format(stream, "CEXEC  %d,%06X,%d", (x >> 7) & 1, (x >> 8) & 0x1fffff, (x >> 29) & 7);
			}
			else
				bad = 1;
			break;

		case 0x0020:
			if (is_34020 && (op & 0xfe00) == 0x0600)
			{
				uint32_t x;
				PARAM_LONG(x);
				util::stream_format(stream, "CMOVGC ");
				print_des_reg(stream);
				util::stream_format(stream, ",%06X,%d", (x >> 8) & 0x1fffff, (x >> 29) & 7);
			}
			else
				bad = 1;
			break;

		case 0x0040:
			if (is_34020 && (op & 0xfe00) == 0x0600)
			{
				uint32_t x;
				PARAM_LONG(x);
				util::stream_format(stream, "CMOVGC ");
				print_des_reg(stream);
				stream << ",";
				rf = (x & 0x10) ? 'B' : 'A';
				print_reg(stream, x & 0x0f);
				util::stream_format(stream, ",%d,%06X,%d", (x >> 7) & 1, (x >> 8) & 0x1fffff, (x >> 29) & 7);
			}
			else
				bad = 1;
			break;

		case 0x0060:
			if (is_34020 && (op & 0xfe00) == 0x0600)
			{
				uint32_t x;
				PARAM_LONG(x);

				if (op == 0x0660 && (x & 0xff) == 0x01)
				{
					util::stream_format(stream, "CMOVCS ");
					util::stream_format(stream, ",%06X,%d", (x >> 8) & 0x1fffff, (x >> 29) & 7);
				}
				else
				{
					util::stream_format(stream, "CMOVCG ");
					print_des_reg(stream);
					stream << ",";
					rf = (x & 0x10) ? 'B' : 'A';
					print_reg(stream, x & 0x0f);
					util::stream_format(stream, ",%d,%06X,%d", (x >> 7) & 1, (x >> 8) & 0x1fffff, (x >> 29) & 7);
				}
			}
			else
				bad = 1;
			break;

		case 0x0080:
			if (is_34020 && (op & 0xfe00) == 0x0600)
			{
				uint32_t x;
				PARAM_LONG(x);
				util::stream_format(stream, "CMOVMC *");
				rf = (x & 0x10) ? 'B' : 'A';
				print_reg(stream, x & 0x0f);
				util::stream_format(stream, "+,%d,%d,%06X,%d", op & 0x1f, (x >> 7) & 1, (x >> 8) & 0x1fffff, (x >> 29) & 7);
			}
			else
				bad = 1;
			break;

		case 0x00a0:
			if (is_34020 && (op & 0xfe00) == 0x0600)
			{
				uint32_t x;
				PARAM_LONG(x);
				util::stream_format(stream, "CMOVCM *");
				print_des_reg(stream);
				util::stream_format(stream, "+,%d,%d,%06X,%d", x & 0x1f, (x >> 7) & 1, (x >> 8) & 0x1fffff, (x >> 29) & 7);
			}
			else
				bad = 1;
			break;

		case 0x00c0:
			if (is_34020 && (op & 0xfe00) == 0x0600)
			{
				uint32_t x;
				PARAM_LONG(x);
				util::stream_format(stream, "CMOVCM *-");
				print_des_reg(stream);
				util::stream_format(stream, ",%d,%d,%06X,%d", x & 0x1f, (x >> 7) & 1, (x >> 8) & 0x1fffff, (x >> 29) & 7);
			}
			else
				bad = 1;
			break;

		case 0x00e0:
			if (is_34020 && (op & 0xfe00) == 0x0600)
			{
				uint32_t x;
				PARAM_LONG(x);
				util::stream_format(stream, "CMOVMC *");
				rf = (x & 0x10) ? 'B' : 'A';
				print_reg(stream, x & 0x0f);
				stream << "+,";
				rf = (op & 0x10) ? 'B' : 'A';
				print_reg(stream, op & 0x0f);
				util::stream_format(stream, ",%d,%06X,%d", (x >> 7) & 1, (x >> 8) & 0x1fffff, (x >> 29) & 7);
			}
			else
				bad = 1;
			break;

		case 0x0100:
			util::stream_format(stream, "SEXT   ");
			print_des_reg(stream);
			stream << ",";
			print_field(stream);
			break;

		case 0x0120:
			util::stream_format(stream, "ZEXT   ");
			print_des_reg(stream);
			stream << ",";
			print_field(stream);
			break;

		case 0x0140:
		case 0x0160:
			util::stream_format(stream, "SETF   %Xh,%X,",
						(op & 0x1f) ? op & 0x1f : 0x20,
						(op >> 5) & 1);
			print_field(stream);
			break;

		case 0x0180:
			util::stream_format(stream, "MOVE   ");
			print_des_reg(stream);
			stream << ",@";
			print_long_parm(stream);
			stream << ",";
			print_field(stream);
			break;

		case 0x01a0:
			util::stream_format(stream, "MOVE   @");
			print_long_parm(stream);
			stream << ",";
			print_des_reg(stream);
			stream << ",";
			print_field(stream);
			break;

		case 0x01c0:
			util::stream_format(stream, "MOVE   @");
			print_long_parm(stream);
			stream << ",@";
			print_long_parm2(stream);
			stream << ",";
			print_field(stream);
			break;

		case 0x01e0:
			if (op & 0x200)
			{
				util::stream_format(stream, "MOVE   @");
				print_long_parm(stream);
				stream << ",";
				print_des_reg(stream);
			}
			else
			{
				util::stream_format(stream, "MOVB   ");
				print_des_reg(stream);
				stream << ",@";
				print_long_parm(stream);
			}
			break;

		default:
			bad = 1;
		}
		break;


	case 0x0800:
		switch (subop)
		{
		case 0x0000:
			if (is_34020)
			{
				util::stream_format(stream, "TRAPL  ");
				flags = DASMFLAG_STEP_OVER;
			}
			else
				bad = 1;
			break;

		case 0x0020:
			if (is_34020)
			{
				uint32_t x;
				PARAM_LONG(x);
				util::stream_format(stream, "CMOVMC *-");
				rf = (x & 0x10) ? 'B' : 'A';
				print_reg(stream, x & 0x0f);
				util::stream_format(stream, ",%d,%d,%06X,%d", op & 0x1f, (x >> 7) & 1, (x >> 8) & 0x1fffff, (x >> 29) & 7);
			}
			else
				bad = 1;
			break;

		case 0x0040:
			if (is_34020)
				util::stream_format(stream, "VBLT   B,L");
			else
				bad = 1;
			break;

		case 0x0060:
			if (is_34020)
			{
				util::stream_format(stream, "RETM   ");
				flags = DASMFLAG_STEP_OUT;
			}
			else
				bad = 1;
			break;

		case 0x00e0:
			if (is_34020)
				util::stream_format(stream, "CLIP   ");
			else
				bad = 1;
			break;

		case 0x0100:
			util::stream_format(stream, "TRAP   %Xh", op & 0x1f);
			flags = DASMFLAG_STEP_OVER;
			break;

		case 0x0120:
			util::stream_format(stream, "CALL   ");
			print_des_reg(stream);
			flags = DASMFLAG_STEP_OVER;
			break;

		case 0x0140:
			util::stream_format(stream, "RETI   ");
			flags = DASMFLAG_STEP_OUT;
			break;

		case 0x0160:
			util::stream_format(stream, "RETS   ");
			flags = DASMFLAG_STEP_OUT;
			if (op & 0x1f)
			{
				util::stream_format(stream, "%Xh", op & 0x1f);
			}
			break;

		case 0x0180:
			util::stream_format(stream, "MMTM   ");
			print_des_reg(stream);
			print_reg_list(stream, 1);
			break;

		case 0x01a0:
			util::stream_format(stream, "MMFM   ");
			print_des_reg(stream);
			print_reg_list(stream, 0);
			break;

		case 0x01c0:
			util::stream_format(stream, "MOVI   ");
			print_word_parm(stream);
			stream << ",";
			print_des_reg(stream);
			break;

		case 0x01e0:
			util::stream_format(stream, "MOVI   ");
			print_long_parm(stream);
			stream << ",";
			print_des_reg(stream);
			break;

		default:
			bad = 1;
		}
		break;


	case 0x0a00:
		switch (subop)
		{
		case 0x0000:
			if (is_34020)
				util::stream_format(stream, "VLCOL  ");
			else
				bad = 1;
			break;

		case 0x0020:
			if (is_34020)
				util::stream_format(stream, "PFILL  XY");
			else
				bad = 1;
			break;

		case 0x0040:
			if (is_34020)
				util::stream_format(stream, "VFILL  L");
			else
				bad = 1;
			break;

		case 0x0060:
			if (is_34020)
			{
				util::stream_format(stream, "CVMXYL ");
				print_des_reg(stream);
			}
			else
				bad = 1;
			break;

		case 0x0080:
			if (is_34020)
			{
				util::stream_format(stream, "CVDXYL ");
				print_des_reg(stream);
			}
			else
				bad = 1;
			break;

		case 0x00a0:
			if (is_34020)
				util::stream_format(stream, "FPIXEQ ");
			else
				bad = 1;
			break;

		case 0x00c0:
			if (is_34020)
				util::stream_format(stream, "FPIXNE ");
			else
				bad = 1;
			break;

		case 0x0100:
			util::stream_format(stream, "ADDI   ");
			print_word_parm(stream);
			stream << ",";
			print_des_reg(stream);
			break;

		case 0x0120:
			util::stream_format(stream, "ADDI   ");
			print_long_parm(stream);
			stream << ",";
			print_des_reg(stream);
			break;

		case 0x0140:
			util::stream_format(stream, "CMPI   ");
			print_word_parm_1s_comp(stream);
			stream << ",";
			print_des_reg(stream);
			break;

		case 0x0160:
			util::stream_format(stream, "CMPI   ");
			print_long_parm_1s_comp(stream);
			stream << ",";
			print_des_reg(stream);
			break;

		case 0x0180:
			util::stream_format(stream, "ANDI   ");
			print_long_parm_1s_comp(stream);
			stream << ",";
			print_des_reg(stream);
			break;

		case 0x01a0:
			util::stream_format(stream, "ORI    ");
			print_long_parm(stream);
			stream << ",";
			print_des_reg(stream);
			break;

		case 0x01c0:
			util::stream_format(stream, "XORI   ");
			print_long_parm(stream);
			stream << ",";
			print_des_reg(stream);
			break;

		case 0x01e0:
			util::stream_format(stream, "SUBI   ");
			print_word_parm_1s_comp(stream);
			stream << ",";
			print_des_reg(stream);
			break;

		default:
			bad = 1;
		}
		break;


	case 0x0c00:
		switch (subop)
		{
		case 0x0000:
			if (is_34020)
			{
				util::stream_format(stream, "ADDXYI ");
				print_long_parm(stream);
				stream << ",";
				print_des_reg(stream);
			}
			else
				bad = 1;
			break;

		case 0x0040:
			if (is_34020)
				util::stream_format(stream, "LINIT  ");
			else
				bad = 1;
			break;

		case 0x0100:
			util::stream_format(stream, "SUBI   ");
			print_long_parm_1s_comp(stream);
			stream << ",";
			print_des_reg(stream);
			break;

		case 0x0120:
			util::stream_format(stream, "CALLR  ");
			print_relative(stream);
			flags = DASMFLAG_STEP_OVER;
			break;

		case 0x0140:
			util::stream_format(stream, "CALLA  ");
			print_long_parm(stream);
			flags = DASMFLAG_STEP_OVER;
			break;

		case 0x0160:
			util::stream_format(stream, "EINT   ");
			break;

		case 0x0180:
			util::stream_format(stream, "DSJ    ");
			print_des_reg(stream);
			stream << ",";
			print_relative(stream);
			flags = DASMFLAG_STEP_OVER;
			break;

		case 0x01a0:
			util::stream_format(stream, "DSJEQ  ");
			print_des_reg(stream);
			stream << ",";
			print_relative(stream);
			flags = DASMFLAG_STEP_OVER;
			break;

		case 0x01c0:
			util::stream_format(stream, "DSJNE  ");
			print_des_reg(stream);
			stream << ",";
			print_relative(stream);
			flags = DASMFLAG_STEP_OVER;
			break;

		case 0x01e0:
			util::stream_format(stream, "SETC   ");
			break;

		default:
			bad = 1;
		}
		break;


	case 0x0e00:
		flags = DASMFLAG_STEP_OVER;
		switch (subop)
		{
		case 0x0000:
			if (is_34020)
				util::stream_format(stream, "PIXBLT L,M,L");
			else
				bad = 1;
			break;

		case 0x00e0:
			if (is_34020)
				util::stream_format(stream, "TFILL  XY");
			else
				bad = 1;
			break;

		case 0x0100:
			util::stream_format(stream, "PIXBLT L,L");
			break;

		case 0x0120:
			util::stream_format(stream, "PIXBLT L,XY");
			break;

		case 0x0140:
			util::stream_format(stream, "PIXBLT XY,L");
			break;

		case 0x0160:
			util::stream_format(stream, "PIXBLT XY,XY");
			break;

		case 0x0180:
			util::stream_format(stream, "PIXBLT B,L");
			break;

		case 0x01a0:
			util::stream_format(stream, "PIXBLT B,XY");
			break;

		case 0x01c0:
			util::stream_format(stream, "FILL   L");
			break;

		case 0x01e0:
			util::stream_format(stream, "FILL   XY");
			break;

		default:
			bad = 1;
		}
		break;


	case 0x1000:
	case 0x1200:
		if ((op & 0x03e0) != 0x0020)
		{
			util::stream_format(stream, "ADDK   ");
			print_constant_1_32(stream);
			stream << ",";
		}
		else
		{
			util::stream_format(stream, "INC    ");
		}
		print_des_reg(stream);

		break;


	case 0x1400:
	case 0x1600:
		if ((op & 0x03e0) != 0x0020)
		{
			util::stream_format(stream, "SUBK   ");
			print_constant_1_32(stream);
			stream << ",";
		}
		else
		{
			util::stream_format(stream, "DEC    ");
		}
		print_des_reg(stream);

		break;


	case 0x1800:
	case 0x1a00:
		util::stream_format(stream, "MOVK   ");
		print_constant_1_32(stream);
		stream << ",";
		print_des_reg(stream);
		break;


	case 0x1c00:
	case 0x1e00:
		util::stream_format(stream, "BTST   ");
		print_constant_1s_comp(stream);
		stream << ",";
		print_des_reg(stream);
		break;


	case 0x2000:
	case 0x2200:
		util::stream_format(stream, "SLA    ");
		print_constant(stream);
		stream << ",";
		print_des_reg(stream);
		break;


	case 0x2400:
	case 0x2600:
		util::stream_format(stream, "SLL    ");
		print_constant(stream);
		stream << ",";
		print_des_reg(stream);
		break;


	case 0x2800:
	case 0x2a00:
		util::stream_format(stream, "SRA    ");
		print_constant_2s_comp(stream);
		stream << ",";
		print_des_reg(stream);
		break;


	case 0x2c00:
	case 0x2e00:
		util::stream_format(stream, "SRL    ");
		print_constant_2s_comp(stream);
		stream << ",";
		print_des_reg(stream);
		break;


	case 0x3000:
	case 0x3200:
		util::stream_format(stream, "RL     ");
		print_constant(stream);
		stream << ",";
		print_des_reg(stream);
		break;

	case 0x3400:
	case 0x3600:
		if (is_34020)
		{
			util::stream_format(stream, "CMPK   ");
			print_constant_1_32(stream);
			stream << ",";
			print_des_reg(stream);
		}
		else
			bad = 1;
		break;

	case 0x3800:
	case 0x3a00:
	case 0x3c00:
	case 0x3e00:
		util::stream_format(stream, "DSJS   ");
		print_des_reg(stream);
		stream << ",";
		print_relative_5bit(stream);
		flags = DASMFLAG_STEP_OVER;
		break;


	case 0x4000:
		util::stream_format(stream, "ADD    ");
		print_src_des_reg(stream);
		break;


	case 0x4200:
		util::stream_format(stream, "ADDC   ");
		print_src_des_reg(stream);
		break;


	case 0x4400:
		util::stream_format(stream, "SUB    ");
		print_src_des_reg(stream);
		break;


	case 0x4600:
		util::stream_format(stream, "SUBB   ");
		print_src_des_reg(stream);
		break;


	case 0x4800:
		util::stream_format(stream, "CMP    ");
		print_src_des_reg(stream);
		break;


	case 0x4a00:
		util::stream_format(stream, "BTST   ");
		print_src_des_reg(stream);
		break;


	case 0x4c00:
	case 0x4e00:
		util::stream_format(stream, "MOVE   ");

		if (!(op & 0x0200))
		{
			print_src_des_reg(stream);
		}
		else
		{
			print_src_reg(stream);
			stream << ",";

			if (rf == 'A')
			{
				rf = 'B';
			}
			else
			{
				rf = 'A';
			}

			print_des_reg(stream);
		}
		break;


	case 0x5000:
		util::stream_format(stream, "AND    ");
		print_src_des_reg(stream);
		break;


	case 0x5200:
		util::stream_format(stream, "ANDN   ");
		print_src_des_reg(stream);
		break;


	case 0x5400:
		util::stream_format(stream, "OR     ");
		print_src_des_reg(stream);
		break;


	case 0x5600:
		if (rs != rd)
		{
			util::stream_format(stream, "XOR    ");
			print_src_des_reg(stream);
		}
		else
		{
			util::stream_format(stream, "CLR    ");
			print_des_reg(stream);
		}
		break;


	case 0x5800:
		util::stream_format(stream, "DIVS   ");
		print_src_des_reg(stream);
		break;


	case 0x5a00:
		util::stream_format(stream, "DIVU   ");
		print_src_des_reg(stream);
		break;


	case 0x5c00:
		util::stream_format(stream, "MPYS   ");
		print_src_des_reg(stream);
		break;


	case 0x5e00:
		util::stream_format(stream, "MPYU   ");
		print_src_des_reg(stream);
		break;


	case 0x6000:
		util::stream_format(stream, "SLA    ");
		print_src_des_reg(stream);
		break;


	case 0x6200:
		util::stream_format(stream, "SLL    ");
		print_src_des_reg(stream);
		break;


	case 0x6400:
		util::stream_format(stream, "SRA    ");
		print_src_des_reg(stream);
		break;


	case 0x6600:
		util::stream_format(stream, "SRL    ");
		print_src_des_reg(stream);
		break;


	case 0x6800:
		util::stream_format(stream, "RL     ");
		print_src_des_reg(stream);
		break;


	case 0x6a00:
		util::stream_format(stream, "LMO    ");
		print_src_des_reg(stream);
		break;


	case 0x6c00:
		util::stream_format(stream, "MODS   ");
		print_src_des_reg(stream);
		break;


	case 0x6e00:
		util::stream_format(stream, "MODU   ");
		print_src_des_reg(stream);
		break;


	case 0x7a00:
		if (is_34020)
		{
			util::stream_format(stream, "RMO    ");
			print_src_des_reg(stream);
		}
		else
			bad = 1;
		break;

	case 0x7e00:
		if (is_34020)
		{
			util::stream_format(stream, "SWAPF  *");
			print_src_des_reg(stream);
			stream << ",0";
		}
		else
			bad = 1;
		break;


	case 0x8000:
	case 0x8200:
		util::stream_format(stream, "MOVE   ");
		print_src_reg(stream);
		stream << ",*";
		print_des_reg(stream);
		stream << ",";
		print_field(stream);
		break;


	case 0x8400:
	case 0x8600:
		util::stream_format(stream, "MOVE   *");
		print_src_des_reg(stream);
		stream << ",";
		print_field(stream);
		break;


	case 0x8800:
	case 0x8a00:
		util::stream_format(stream, "MOVE   *");
		print_src_reg(stream);
		stream << ",*";
		print_des_reg(stream);
		stream << ",";
		print_field(stream);
		break;


	case 0x8c00:
		util::stream_format(stream, "MOVB   ");
		print_src_reg(stream);
		stream << ",*";
		print_des_reg(stream);
		break;


	case 0x8e00:
		util::stream_format(stream, "MOVB   *");
		print_src_des_reg(stream);
		break;


	case 0x9000:
	case 0x9200:
		util::stream_format(stream, "MOVE   ");
		print_src_reg(stream);
		stream << ",*";
		print_des_reg(stream);
		stream << "+,";
		print_field(stream);
		break;


	case 0x9400:
	case 0x9600:
		util::stream_format(stream, "MOVE   *");
		print_src_reg(stream);
		stream << "+,";
		print_des_reg(stream);
		stream << ",";
		print_field(stream);
		break;


	case 0x9800:
	case 0x9a00:
		util::stream_format(stream, "MOVE   *");
		print_src_reg(stream);
		stream << "+,*";
		print_des_reg(stream);
		stream << "+,";
		print_field(stream);
		break;


	case 0x9c00:
		util::stream_format(stream, "MOVB   *");
		print_src_reg(stream);
		stream << ",*";
		print_des_reg(stream);
		break;


	case 0xa000:
	case 0xa200:
		util::stream_format(stream, "MOVE   ");
		print_src_reg(stream);
		stream << ",-*";
		print_des_reg(stream);
		stream << ",";
		print_field(stream);
		break;


	case 0xa400:
	case 0xa600:
		util::stream_format(stream, "MOVE   -*");
		print_src_des_reg(stream);
		stream << ",";
		print_field(stream);
		break;


	case 0xa800:
	case 0xaa00:
		util::stream_format(stream, "MOVE   -*");
		print_src_reg(stream);
		stream << ",-*";
		print_des_reg(stream);
		stream << ",";
		print_field(stream);
		break;


	case 0xac00:
		util::stream_format(stream, "MOVB   ");
		print_src_reg(stream);
		stream << ",*";
		print_des_reg(stream);
		stream << "(";
		print_word_parm(stream);
		stream << ")";
		break;


	case 0xae00:
		util::stream_format(stream, "MOVB   *");
		print_src_reg(stream);
		stream << "(";
		print_word_parm(stream);
		stream << "),";
		print_des_reg(stream);
		break;


	case 0xb000:
	case 0xb200:
		util::stream_format(stream, "MOVE   ");
		print_src_reg(stream);
		stream << ",*";
		print_des_reg(stream);
		stream << "(";
		print_word_parm(stream);
		stream << "),";
		print_field(stream);
		break;


	case 0xb400:
	case 0xb600:
		util::stream_format(stream, "MOVE   *");
		print_src_reg(stream);
		stream << "(";
		print_word_parm(stream);
		stream << "),";
		print_des_reg(stream);
		stream << ",";
		print_field(stream);
		break;


	case 0xb800:
	case 0xba00:
		util::stream_format(stream, "MOVE   *");
		print_src_reg(stream);
		stream << "(";
		print_word_parm(stream);
		stream << "),*";
		print_des_reg(stream);
		stream << "(";
		print_word_parm(stream);
		stream << "),";
		print_field(stream);
		break;


	case 0xbc00:
		util::stream_format(stream, "MOVB   *");
		print_src_reg(stream);
		stream << "(";
		print_word_parm(stream);
		stream << "),*";
		print_des_reg(stream);
		stream << "(";
		print_word_parm(stream);
		stream << ")";
		break;


	case 0xc000:
	case 0xc200:
	case 0xc400:
	case 0xc600:
	case 0xc800:
	case 0xca00:
	case 0xcc00:
	case 0xce00:
		if ((op & 0x00ff) == 0x80)
		{
			util::stream_format(stream, "JA");
		}
		else
		{
			util::stream_format(stream, "JR");
		}

		print_condition_code(stream);
		stream << "   ";

		switch (op & 0x00ff)
		{
		case 0x00:
			print_relative(stream);
			break;

		case 0x80:
			print_long_parm(stream);
			break;

		default:
			print_relative_8bit(stream);
		}
		break;


	case 0xd000:
	case 0xd200:
		util::stream_format(stream, "MOVE   *");
		print_src_reg(stream);
		stream << "(";
		print_word_parm(stream);
		stream << "),*";
		print_des_reg(stream);
		stream << "+,";
		print_field(stream);
		break;


	case 0xd400:
	case 0xd600:
		switch (subop)
		{
		case 0x0000:
			util::stream_format(stream, "MOVE   @");
			print_long_parm(stream);
			stream << ",*";
			print_des_reg(stream);
			stream << "+,";
			print_field(stream);
			break;

		case 0x0100:
			util::stream_format(stream, "EXGF   ");
			print_des_reg(stream);
			stream << ",";
			print_field(stream);
			break;

		default:
			bad = 1;
		}
		break;

	case 0xd800:
		if (is_34020)
		{
			uint32_t x;
			PARAM_WORD(x);
			util::stream_format(stream, "CEXEC  %d,%06X,%d", op & 1, ((x << 5) & 0x1fffe0) | ((op >> 1) & 0x1f), (x >> 13) & 7);
		}
		else
			bad = 1;
		break;


	case 0xde00:
		switch (subop)
		{
		case 0x0000:
			if (is_34020)
				util::stream_format(stream, "FLINE   0");
			else
				bad = 1;
			break;

		case 0x0080:
			if (is_34020)
				util::stream_format(stream, "FLINE   1");
			else
				bad = 1;
			break;

		case 0x0100:
			util::stream_format(stream, "LINE   0");
			break;

		case 0x0180:
			util::stream_format(stream, "LINE   1");
			break;

		default:
			bad = 1;
		}
		break;

	case 0xe000:
		util::stream_format(stream, "ADDXY  ");
		print_src_des_reg(stream);
		break;


	case 0xe200:
		util::stream_format(stream, "SUBXY  ");
		print_src_des_reg(stream);
		break;


	case 0xe400:
		util::stream_format(stream, "CMPXY  ");
		print_src_des_reg(stream);
		break;


	case 0xe600:
		util::stream_format(stream, "CPW    ");
		print_src_des_reg(stream);
		break;


	case 0xe800:
		util::stream_format(stream, "CVXYL  ");
		print_src_des_reg(stream);
		break;


	case 0xea00:
		if (is_34020)
		{
			util::stream_format(stream, "CVSXYL ");
			print_src_des_reg(stream);
		}
		else
			bad = 1;
		break;


	case 0xec00:
		util::stream_format(stream, "MOVX   ");
		print_src_des_reg(stream);
		break;


	case 0xee00:
		util::stream_format(stream, "MOVY   ");
		print_src_des_reg(stream);
		break;


	case 0xf000:
		util::stream_format(stream, "PIXT   ");
		print_src_reg(stream);
		stream << ",*";
		print_des_reg(stream);
		stream << ",XY";
		break;


	case 0xf200:
		util::stream_format(stream, "PIXT   *");
		print_src_reg(stream);
		stream << ",XY,";
		print_des_reg(stream);
		break;


	case 0xf400:
		util::stream_format(stream, "PIXT   *");
		print_src_reg(stream);
		stream << ",XY,*";
		print_des_reg(stream);
		stream << ",XY";
		break;


	case 0xf600:
		util::stream_format(stream, "DRAV   ");
		print_src_des_reg(stream);
		break;


	case 0xf800:
		util::stream_format(stream, "PIXT   ");
		print_src_reg(stream);
		stream << ",*";
		print_des_reg(stream);
		break;


	case 0xfa00:
		util::stream_format(stream, "PIXT   *");
		print_src_des_reg(stream);
		break;


	case 0xfc00:
		util::stream_format(stream, "PIXT   *");
		print_src_reg(stream);
		stream << ",*";
		print_des_reg(stream);
		break;

	default:
		bad = 1;
	}

	if (bad)
	{
		util::stream_format(stream, "DW     %04Xh", op & 0xffff);
	}

	return (_pc - __pc) | flags | DASMFLAG_SUPPORTED;
}

CPU_DISASSEMBLE( tms34010 )
{
	rombase = oprom;
	rambase = opram;
	pcbase = pc;
	return Dasm340x0(stream, pc, false);
}

CPU_DISASSEMBLE( tms34020 )
{
	rombase = oprom;
	rambase = opram;
	pcbase = pc;
	return Dasm340x0(stream, pc, true);
}
