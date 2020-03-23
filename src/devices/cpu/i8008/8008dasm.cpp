// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/*****************************************************************************
 *
 *   8008dasm.c
 *
 *   Intel 8008 CPU Disassembly
 *
 *****************************************************************************/

#include "emu.h"

#define OP(A)   oprom[(A) - PC]
#define ARG(A)  opram[(A) - PC]

static const char reg[] = { 'a', 'b', 'c', 'd', 'e', 'h', 'l', 'm' };
static const char flag_names[] = { 'c', 'z', 's', 'p' };

CPU_DISASSEMBLE(i8008)
{
	uint32_t flags = 0;
	unsigned PC = pc;
	uint8_t op = OP(pc++);
	switch (op >> 6)
	{
		case 0x03:  // starting with 11
					if (op==0xff) {
						util::stream_format(stream, "hlt");
					} else {
						util::stream_format(stream, "l%c%c",reg[(op >> 3) & 7],reg[op & 7]);
					}
					break;
		case 0x00:  // starting with 00
					switch(op & 7) {
						case 0 :    if(((op >> 3) & 7)==0) {
										util::stream_format(stream, "hlt");
									} else {
										if(((op >> 3) & 7)==7) {
											util::stream_format(stream, "illegal");
										} else {
											util::stream_format(stream, "in%c",reg[(op >> 3) & 7]);
										}
									}
									break;
						case 1 :    if(((op >> 3) & 7)==0) {
										util::stream_format(stream, "hlt");
									} else {
										if(((op >> 3) & 7)==7) {
											util::stream_format(stream, "illegal");
										} else {
											util::stream_format(stream, "dc%c",reg[(op >> 3) & 7]);
										}
									}
									break;
						case 2 :    {
										switch((op >> 3) & 7) {
											case 0 :    util::stream_format(stream, "rlc"); break;
											case 1 :    util::stream_format(stream, "rrc"); break;
											case 2 :    util::stream_format(stream, "ral"); break;
											case 3 :    util::stream_format(stream, "rar"); break;
											default :   util::stream_format(stream, "illegal"); break;
										}
									}
									break;
						case 3 :    util::stream_format(stream, "r%c%c",(BIT(op,5) ? 't' : 'f'),flag_names[(op>>3)&3]); break;
						case 4 :    {
										switch((op >> 3) & 7) {
											case 0 :    util::stream_format(stream, "adi %02x",ARG(pc)); pc++; break;
											case 1 :    util::stream_format(stream, "aci %02x",ARG(pc)); pc++; break;
											case 2 :    util::stream_format(stream, "sui %02x",ARG(pc)); pc++; break;
											case 3 :    util::stream_format(stream, "sbi %02x",ARG(pc)); pc++; break;
											case 4 :    util::stream_format(stream, "ndi %02x",ARG(pc)); pc++; break;
											case 5 :    util::stream_format(stream, "xri %02x",ARG(pc)); pc++; break;
											case 6 :    util::stream_format(stream, "ori %02x",ARG(pc)); pc++; break;
											case 7 :    util::stream_format(stream, "cpi %02x",ARG(pc)); pc++; break;
										}
									}
									break;
						case 5 :    util::stream_format(stream, "rst %02x",(op>>3) & 7); break;
						case 6 :    util::stream_format(stream, "l%ci %02x",reg[(op >> 3) & 7],ARG(pc)); pc++; break;
						case 7 :    util::stream_format(stream, "ret"); break;
					}
					break;
		case 0x01:  // starting with 01
					switch(op & 7) {
						case 0 :    util::stream_format(stream, "j%c%c %02x%02x",(BIT(op,5)? 't' : 'f'),flag_names[(op>>3)&3], ARG(pc+1) & 0x3f,ARG(pc)); pc+=2; break;
						case 2 :    util::stream_format(stream, "c%c%c %02x%02x",(BIT(op,5)? 't' : 'f'),flag_names[(op>>3)&3], ARG(pc+1) & 0x3f,ARG(pc)); pc+=2; break;
						case 4 :    util::stream_format(stream, "jmp %02x%02x",ARG(pc+1) & 0x3f,ARG(pc)); pc+=2; break;
						case 6 :    util::stream_format(stream, "cal %02x%02x",ARG(pc+1) & 0x3f,ARG(pc)); pc+=2; break;
						case 1 :
						case 3 :
						case 5 :
						case 7 :    if (((op>>4)&3)==0) {
										util::stream_format(stream, "inp %02x",(op >> 1) & 0x07);
									} else {
										util::stream_format(stream, "out %02x",(op >> 1) & 0x1f);
									}
									break;
					}
					break;
		case 0x02:  // starting with 10
					switch((op >> 3) & 7) {
						case 0 :    util::stream_format(stream, "ad%c",reg[op & 7]); break;
						case 1 :    util::stream_format(stream, "ac%c",reg[op & 7]); break;
						case 2 :    util::stream_format(stream, "su%c",reg[op & 7]); break;
						case 3 :    util::stream_format(stream, "sb%c",reg[op & 7]); break;
						case 4 :    util::stream_format(stream, "nd%c",reg[op & 7]); break;
						case 5 :    util::stream_format(stream, "xr%c",reg[op & 7]); break;
						case 6 :    util::stream_format(stream, "or%c",reg[op & 7]); break;
						case 7 :    util::stream_format(stream, "cp%c",reg[op & 7]); break;
					}
					break;
	}
	return (pc - PC) | flags | DASMFLAG_SUPPORTED;
}
