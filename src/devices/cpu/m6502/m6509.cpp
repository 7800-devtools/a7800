// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    m6509.c

    6502 with banking and extended address bus

***************************************************************************/

#include "emu.h"
#include "m6509.h"

DEFINE_DEVICE_TYPE(M6509, m6509_device, "m6509", "M6509")

m6509_device::m6509_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	m6502_device(mconfig, M6509, tag, owner, clock), XPC(0), bank_i(0), bank_y(0)
{
	program_config.m_addrbus_width = 20;
	program_config.m_logaddr_width = 20;
	sprogram_config.m_addrbus_width = 20;
	sprogram_config.m_logaddr_width = 20;
}

void m6509_device::device_start()
{
	if(direct_disabled)
		mintf = new mi_6509_nd(this);
	else
		mintf = new mi_6509_normal(this);

	init();

	state_add(STATE_GENPC, "GENPC", XPC).callexport().noshow();
	state_add(STATE_GENPCBASE, "CURPC", XPC).callexport().noshow();
	state_add(M6509_BI, "BI", bank_i);
	state_add(M6509_BY, "BY", bank_y);
}

void m6509_device::device_reset()
{
	m6502_device::device_reset();
	bank_i = 0x0f;
	bank_y = 0x0f;
}

void m6509_device::state_export(const device_state_entry &entry)
{
	switch(entry.index()) {
	case STATE_GENPC:
	case STATE_GENPCBASE:
		XPC = adr_in_bank_i(NPC);
		break;
	}
}

offs_t m6509_device::disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options)
{
	return disassemble_generic(stream, pc, oprom, opram, options, disasm_entries);
}


m6509_device::mi_6509_normal::mi_6509_normal(m6509_device *_base)
{
	base = _base;
}

uint8_t m6509_device::mi_6509_normal::read(uint16_t adr)
{
	uint8_t res = program->read_byte(base->adr_in_bank_i(adr));
	if(adr == 0x0000)
		res = base->bank_i_r();
	else if(adr == 0x0001)
		res = base->bank_y_r();
	return res;
}

uint8_t m6509_device::mi_6509_normal::read_sync(uint16_t adr)
{
	uint8_t res = sdirect->read_byte(base->adr_in_bank_i(adr));
	if(adr == 0x0000)
		res = base->bank_i_r();
	else if(adr == 0x0001)
		res = base->bank_y_r();
	return res;
}

uint8_t m6509_device::mi_6509_normal::read_arg(uint16_t adr)
{
	uint8_t res = direct->read_byte(base->adr_in_bank_i(adr));
	if(adr == 0x0000)
		res = base->bank_i_r();
	else if(adr == 0x0001)
		res = base->bank_y_r();
	return res;
}

uint8_t m6509_device::mi_6509_normal::read_9(uint16_t adr)
{
	uint8_t res = program->read_byte(base->adr_in_bank_y(adr));
	if(adr == 0x0000)
		res = base->bank_i_r();
	else if(adr == 0x0001)
		res = base->bank_y_r();
	return res;
}

void m6509_device::mi_6509_normal::write(uint16_t adr, uint8_t val)
{
	program->write_byte(base->adr_in_bank_i(adr), val);
	if(adr == 0x0000)
		base->bank_i_w(val);
	else if(adr == 0x0001)
		base->bank_y_w(val);
}

void m6509_device::mi_6509_normal::write_9(uint16_t adr, uint8_t val)
{
	program->write_byte(base->adr_in_bank_y(adr), val);
	if(adr == 0x0000)
		base->bank_i_w(val);
	else if(adr == 0x0001)
		base->bank_y_w(val);
}

m6509_device::mi_6509_nd::mi_6509_nd(m6509_device *_base) : mi_6509_normal(_base)
{
}

uint8_t m6509_device::mi_6509_nd::read_sync(uint16_t adr)
{
	uint8_t res = sprogram->read_byte(base->adr_in_bank_i(adr));
	if(adr == 0x0000)
		res = base->bank_i_r();
	else if(adr == 0x0001)
		res = base->bank_y_r();
	return res;
}

uint8_t m6509_device::mi_6509_nd::read_arg(uint16_t adr)
{
	uint8_t res = program->read_byte(base->adr_in_bank_i(adr));
	if(adr == 0x0000)
		res = base->bank_i_r();
	else if(adr == 0x0001)
		res = base->bank_y_r();
	return res;
}

#include "cpu/m6502/m6509.hxx"
