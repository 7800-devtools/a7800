// license:BSD-3-Clause
// copyright-holders:Tony La Porta
	/**************************************************************************\
	*                      Microchip PIC16C5x Emulator                         *
	*                                                                          *
	*                    Copyright Tony La Porta                               *
	*                 Originally written for the MAME project.                 *
	*                                                                          *
	*                                                                          *
	*      Addressing architecture is based on the Harvard addressing scheme.  *
	*                                                                          *
	*         Many thanks to those involved in the i8039 Disassembler          *
	*                        as this was based on it.                          *
	*                                                                          *
	*                                                                          *
	*                                                                          *
	* A Address to jump to.                                                    *
	* B Bit address within an 8-bit file register.                             *
	* D Destination select (0 = store result in W (accumulator))               *
	*                      (1 = store result in file register)                 *
	* F Register file address (00-1F).                                         *
	* K Literal field, constant data.                                          *
	*                                                                          *
	\**************************************************************************/

#include "emu.h"
#include <ctype.h>

static const uint8_t *rombase;
static const uint8_t *rambase;
static offs_t pcbase;
#define READOP16(A)  (rombase[(A) - pcbase] | (rombase[(A) + 1 - pcbase] << 8))
#define READARG16(A) (rambase[(A) - pcbase] | (rambase[(A) + 1 - pcbase] << 8))



typedef unsigned char byte;
typedef unsigned short int word;

#define FMT(a,b) a, b
#define PTRS_PER_FORMAT 2

static const char *const regfile[32] = { "Reg$00 (IND)",    "Reg$01 (TMR)",    "Reg$02 (PCL)",  "Reg$03 (ST)", "Reg$04 (FSR)", "Reg$05 (PTA)", "Reg$06 (PTB)", "Reg$07 (PTC)",
									"Reg$08", "Reg$09", "Reg$0A", "Reg$0B", "Reg$0C", "Reg$0D", "Reg$0E", "Reg$0F",
									"Reg$10", "Reg$11", "Reg$12", "Reg$13", "Reg$14", "Reg$15", "Reg$16", "Reg$17",
									"Reg$18", "Reg$19", "Reg$1A", "Reg$1B", "Reg$1C", "Reg$1D", "Reg$1E", "Reg$1F" };

static const char *const dest[2] = { "W", "Reg" };

static const char *const PIC16C5xFormats[] = {
	FMT("000000000000", "nop"),
	FMT("000000000010", "option"),
	FMT("000000000011", "sleep"),
	FMT("000000000100", "clrwdt"),
	FMT("000000000101", "tris   Port A"),
	FMT("000000000110", "tris   Port B"),
	FMT("000000000111", "tris   Port C"),
	FMT("0000001fffff", "movwf  %F"),
	FMT("000001000000", "clrw"),
	FMT("0000011fffff", "clrf   %F"),
	FMT("000010dfffff", "subwf  %F,%D"),
	FMT("000011dfffff", "decf   %F,%D"),
	FMT("000100dfffff", "iorwf  %F,%D"),
	FMT("000101dfffff", "andwf  %F,%D"),
	FMT("000110dfffff", "xorwf  %F,%D"),
	FMT("000111dfffff", "addwf  %F,%D"),
	FMT("001000dfffff", "movf   %F,%D"),
	FMT("001001dfffff", "comf   %F,%D"),
	FMT("001010dfffff", "incf   %F,%D"),
	FMT("001011dfffff", "decfsz %F,%D"),
	FMT("001100dfffff", "rrf    %F,%D"),
	FMT("001101dfffff", "rlf    %F,%D"),
	FMT("001110dfffff", "swapf  %F,%D"),
	FMT("001111dfffff", "incfsz %F,%D"),
	FMT("0100bbbfffff", "bcf    %F,%B"),
	FMT("0101bbbfffff", "bsf    %F,%B"),
	FMT("0110bbbfffff", "btfsc  %F,%B"),
	FMT("0111bbbfffff", "btfss  %F,%B"),
	FMT("1000kkkkkkkk", "retlw  %K"),
	FMT("1001aaaaaaaa", "call   %A"),
	FMT("101aaaaaaaaa", "goto   %A"),
	FMT("1100kkkkkkkk", "movlw  %K"),
	FMT("1101kkkkkkkk", "iorlw  %K"),
	FMT("1110kkkkkkkk", "andlw  %K"),
	FMT("1111kkkkkkkk", "xorlw  %K"),
	nullptr
};

#define MAX_OPS ((ARRAY_LENGTH(PIC16C5xFormats) - 1) / PTRS_PER_FORMAT)

struct PIC16C5xOpcode  {
	word mask;          /* instruction mask */
	word bits;          /* constant bits */
	word extcode;       /* value that gets extension code */
	const char *parse;  /* how to parse bits */
	const char *fmt;    /* instruction format */
};

static PIC16C5xOpcode Op[MAX_OPS+1];
static int OpInizialized = 0;

static void InitDasm16C5x(void)
{
	const char *p;
	const char *const *ops;
	word mask, bits;
	int bit;
	int i;

	ops = PIC16C5xFormats; i = 0;
	while (*ops)
	{
		p = *ops;
		mask = 0; bits = 0; bit = 11;
		while (*p && bit >= 0)
		{
			switch (*p++)
			{
				case '1': mask |= 1<<bit; bits |= 1<<bit; bit--; break;
				case '0': mask |= 1<<bit; bit--; break;
				case ' ': break;
				case 'a':
				case 'b':
				case 'd':
				case 'f':
				case 'k':
					bit --;
					break;
				default: fatalerror("Invalid instruction encoding '%s %s'\n",
					ops[0],ops[1]);
			}
		}
		if (bit != -1 )
		{
			fatalerror("not enough bits in encoding '%s %s' %d\n",
				ops[0],ops[1],bit);
		}
		while (isspace((uint8_t)*p)) p++;
		if (*p) Op[i].extcode = *p;
		Op[i].bits = bits;
		Op[i].mask = mask;
		Op[i].fmt = ops[1];
		Op[i].parse = ops[0];

		ops += PTRS_PER_FORMAT;
		i++;
	}

	OpInizialized = 1;
}

CPU_DISASSEMBLE(pic16c5x)
{
	int a, b, d, f, k;  /* these can all be filled in by parsing an instruction */
	int i;
	int op;
	int cnt = 1;
	int code;
	int bit;
	//char *buffertmp;
	const char *cp;             /* character pointer in OpFormats */
	uint32_t flags = 0;

	rombase = oprom;
	rambase = opram;
	pcbase = 2*pc;

	if (!OpInizialized) InitDasm16C5x();

	op = -1;                /* no matching opcode */
	code = READOP16(2*pc);
	for ( i = 0; i < MAX_OPS; i++)
	{
		if ((code & Op[i].mask) == Op[i].bits)
		{
			if (op != -1)
			{
				osd_printf_debug("Error: opcode %04Xh matches %d (%s) and %d (%s)\n",
					code,i,Op[i].fmt,op,Op[op].fmt);
			}
			op = i;
		}
	}
	if (op == -1)
	{
		util::stream_format(stream, "???? dw %04Xh",code);
		return cnt;
	}
	//buffertmp = buffer;
	if (Op[op].extcode)     /* Actually, theres no double length opcodes */
	{
		bit = 27;
		code <<= 16;
		code |= READARG16(2*(pc+cnt));
		cnt++;
	}
	else
	{
		bit = 11;
	}

	/* shift out operands */
	cp = Op[op].parse;
	a = b = d = f = k = 0;

	while (bit >= 0)
	{
		/* osd_printf_debug("{%c/%d}",*cp,bit); */
		switch(*cp)
		{
			case 'a': a <<=1; a |= ((code & (1<<bit)) ? 1 : 0); bit--; break;
			case 'b': b <<=1; b |= ((code & (1<<bit)) ? 1 : 0); bit--; break;
			case 'd': d <<=1; d |= ((code & (1<<bit)) ? 1 : 0); bit--; break;
			case 'f': f <<=1; f |= ((code & (1<<bit)) ? 1 : 0); bit--; break;
			case 'k': k <<=1; k |= ((code & (1<<bit)) ? 1 : 0); bit--; break;
			case ' ': break;
			case '1': case '0':  bit--; break;
			case '\0': fatalerror("premature end of parse string, opcode %x, bit = %d\n",code,bit);
		}
		cp++;
	}

	/* now traverse format string */
	cp = Op[op].fmt;
	if (!strncmp(cp, "call", 4))
		flags = DASMFLAG_STEP_OVER;
	else if (!strncmp(cp, "ret", 3))
		flags = DASMFLAG_STEP_OUT;

	while (*cp)
	{
		if (*cp == '%')
		{
			cp++;
			switch (*cp++)
			{
				case 'A': util::stream_format(stream, "$%03X", a); break;
				case 'B': util::stream_format(stream, "%d", b); break;
				case 'D': util::stream_format(stream, "%s", dest[d]); break;
				case 'F': util::stream_format(stream, "%s", regfile[f]); break;
				case 'K': util::stream_format(stream, "%02Xh", k); break;
				default:
					fatalerror("illegal escape character in format '%s'\n",Op[op].fmt);
			}
		}
		else
		{
			stream << *cp++;
		}
	}
	return cnt | flags | DASMFLAG_SUPPORTED;
}
