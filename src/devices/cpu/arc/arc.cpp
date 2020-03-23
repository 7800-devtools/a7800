// license:BSD-3-Clause
// copyright-holders:David Haywood
/*********************************\

 ARCtangent (A4) core
 ARC == Argonaut RISC Core

 (this is a skeleton core)

\*********************************/

#include "emu.h"
#include "debugger.h"
#include "arc.h"


DEFINE_DEVICE_TYPE(ARC, arc_cpu_device, "arc_a4", "ARCtangent A4")


arc_cpu_device::arc_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: cpu_device(mconfig, ARC, tag, owner, clock)
	, m_program_config("program", ENDIANNESS_BIG, 32, 24, 0)
	, m_pc(0), m_program(nullptr), m_icount(0), m_debugger_temp(0)
// some docs describe these as 'middle endian'?!
{
}


offs_t arc_cpu_device::disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options)
{
	extern CPU_DISASSEMBLE( arc );
	return CPU_DISASSEMBLE_NAME(arc)(this, stream, pc, oprom, opram, options);
}


/*****************************************************************************/

/*****************************************************************************/

void arc_cpu_device::unimplemented_opcode(uint16_t op)
{
	fatalerror("arc: unknown opcode %04x at %04x\n", op, m_pc << 2);
}

/*****************************************************************************/

uint32_t arc_cpu_device::READ32(uint32_t address)
{
	return m_program->read_dword(address << 2);
}

void arc_cpu_device::WRITE32(uint32_t address, uint32_t data)
{
	m_program->write_dword(address << 2, data);
}

/*****************************************************************************/

void arc_cpu_device::device_start()
{
	m_pc = 0;

	m_debugger_temp = 0;

	m_program = &space(AS_PROGRAM);

	state_add(ARC_PC,  "PC", m_debugger_temp).callimport().callexport().formatstr("%08X");
	state_add(STATE_GENPCBASE, "CURPC", m_debugger_temp).callimport().callexport().noshow();

	m_icountptr = &m_icount;
}


//-------------------------------------------------
//  state_export - export state from the device,
//  to a known location where it can be read
//-------------------------------------------------

void arc_cpu_device::state_export(const device_state_entry &entry)
{
	switch (entry.index())
	{
		case ARC_PC:
		case STATE_GENPCBASE:
			m_debugger_temp = m_pc << 2;
			break;
	}
}


//-------------------------------------------------
//  state_import - import state into the device,
//  after it has been set
//-------------------------------------------------

void arc_cpu_device::state_import(const device_state_entry &entry)
{
	switch (entry.index())
	{
		case ARC_PC:
		case STATE_GENPCBASE:
			m_pc = (m_debugger_temp & 0xfffffffc) >> 2;
			break;
	}
}

void arc_cpu_device::device_reset()
{
	m_pc = 0x00000000;
}

/*****************************************************************************/

void arc_cpu_device::execute_set_input(int irqline, int state)
{
}


void arc_cpu_device::execute_run()
{
	//uint32_t lres;
	//lres = 0;

	while (m_icount > 0)
	{
		debugger_instruction_hook(this, m_pc<<2);

		//uint32_t op = READ32(m_pc);

		m_pc++;

		m_icount--;
	}

}
