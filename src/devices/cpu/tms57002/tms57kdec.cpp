// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    tms57kdec.c

    TMS57002 "DASP" emulator.

***************************************************************************/

#include "emu.h"
#include "debugger.h"
#include "tms57002.h"

inline int tms57002_device::xmode(uint32_t opcode, char type, cstate *cs)
{
	if(((opcode & 0x400) && (type == 'c')) || (!(opcode & 0x400) && (type == 'd'))) {
		if(opcode & 0x100)
			return 0;
		else if(opcode & 0x80)
			cs->inc |= type == 'c' ? INC_CA : INC_ID;

		return 1;
	}
	else if(opcode & 0x200)
		cs->inc |= type == 'c' ? INC_CA : INC_ID;

	return 1;
}

inline int tms57002_device::sfao(uint32_t st1)
{
	return st1 & ST1_SFAO ? 1 : 0;
}

inline int tms57002_device::dbp(uint32_t st1)
{
	return st1 & ST1_DBP ? 1 : 0;
}

inline int tms57002_device::crm(uint32_t st1)
{
	int crm = (st1 & ST1_CRM) >> ST1_CRM_SHIFT;
	return crm <= 2 ? crm : 0;
}

inline int tms57002_device::sfai(uint32_t st1)
{
	return st1 & ST1_SFAI ? 1 : 0;
}

inline int tms57002_device::sfmo(uint32_t st1)
{
	return (st1 & ST1_SFMO) >> ST1_SFMO_SHIFT;
}

inline int tms57002_device::rnd(uint32_t st1)
{
	int rnd = (st1 & ST1_RND) >> ST1_RND_SHIFT;
	return rnd <= 4 ? rnd : 0;
}

inline int tms57002_device::movm(uint32_t st1)
{
	return st1 & ST1_MOVM ? 1 : 0;
}

inline int tms57002_device::sfma(uint32_t st1)
{
	return (st1 & ST1_SFMA) >> ST1_SFMA_SHIFT;
}

void tms57002_device::decode_error(uint32_t opcode)
{
	uint8_t opr[3];
	if(unsupported_inst_warning)
		return;

	unsupported_inst_warning = 1;
	opr[0] = opcode;
	opr[1] = opcode >> 8;
	opr[2] = opcode >> 16;

	std::stringstream stream;
	disasm_disassemble(stream, pc, opr, opr, 0);
	popmessage("tms57002: %s - Contact Mamedev", stream.str());
}

void tms57002_device::decode_cat1(uint32_t opcode, unsigned short *op, cstate *cs)
{
	switch(opcode >> 18) {
	case 0x00: // nop
		break;

#define CDEC1
#include "cpu/tms57002/tms57002.hxx"
#undef CDEC1

	default:
		decode_error(opcode);
		break;
	}
}

void tms57002_device::decode_cat2_pre(uint32_t opcode, unsigned short *op, cstate *cs)
{
	switch((opcode >> 11) & 0x7f) {
	case 0x00: // nop
		break;

#define CDEC2A
#include "cpu/tms57002/tms57002.hxx"
#undef CDEC2A

	default:
		decode_error(opcode);
		break;
	}
}

void tms57002_device::decode_cat2_post(uint32_t opcode, unsigned short *op, cstate *cs)
{
	switch((opcode >> 11) & 0x7f) {
	case 0x00: // nop
		break;

#define CDEC2B
#include "cpu/tms57002/tms57002.hxx"
#undef CDEC2B

	default:
		decode_error(opcode);
		break;
	}
}

void tms57002_device::decode_cat3(uint32_t opcode, unsigned short *op, cstate *cs)
{
	switch((opcode >> 11) & 0x7f) {
	case 0x00: // nop
		break;

#define CDEC3
#include "cpu/tms57002/tms57002.hxx"
#undef CDEC3

	default:
		decode_error(opcode);
		break;
	}
}
