// license:BSD-3-Clause
// copyright-holders:David Haywood
/*********************************\

 ARCompact Core

 The following procesors use the ARCompact instruction set

  - ARCtangent-A5
  - ARC 600
  - ARC 700

 (this is a skeleton core)

 ARCompact is a 32-bit CPU that freely mixes 32-bit and 16-bit instructions
 various user customizations could be made as with the ARC A4 based processors
 these include custom instructions and registers.

\*********************************/

#include "emu.h"
#include "debugger.h"
#include "arcompact.h"
#include "arcompact_common.h"


DEFINE_DEVICE_TYPE(ARCA5, arcompact_device, "arc_a5", "ARCtangent A5")


READ32_MEMBER( arcompact_device::arcompact_auxreg002_LPSTART_r) { return m_LP_START&0xfffffffe; }
WRITE32_MEMBER(arcompact_device::arcompact_auxreg002_LPSTART_w) { m_LP_START = data&0xfffffffe; }
READ32_MEMBER( arcompact_device::arcompact_auxreg003_LPEND_r) { return m_LP_END&0xfffffffe; }
WRITE32_MEMBER(arcompact_device::arcompact_auxreg003_LPEND_w) { m_LP_END = data&0xfffffffe; }

READ32_MEMBER( arcompact_device::arcompact_auxreg00a_STATUS32_r) { return 0xffffdead; /*m_status32;*/ }

READ32_MEMBER( arcompact_device::arcompact_auxreg025_INTVECTORBASE_r) { return m_INTVECTORBASE&0xfffffc00; }
WRITE32_MEMBER(arcompact_device::arcompact_auxreg025_INTVECTORBASE_w) { m_INTVECTORBASE = data&0xfffffc00; }




static ADDRESS_MAP_START( arcompact_auxreg_map, AS_IO, 32, arcompact_device )
	AM_RANGE(0x000000002, 0x000000002) AM_READWRITE(arcompact_auxreg002_LPSTART_r, arcompact_auxreg002_LPSTART_w)
	AM_RANGE(0x000000003, 0x000000003) AM_READWRITE(arcompact_auxreg003_LPEND_r, arcompact_auxreg003_LPEND_w)
	AM_RANGE(0x000000009, 0x000000009) AM_READ(arcompact_auxreg00a_STATUS32_r) // r/o
	AM_RANGE(0x000000025, 0x000000025) AM_READWRITE(arcompact_auxreg025_INTVECTORBASE_r, arcompact_auxreg025_INTVECTORBASE_w)
ADDRESS_MAP_END

#define AUX_SPACE_ADDRESS_WIDTH 32  // IO space is 32 bits of dwords

arcompact_device::arcompact_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: cpu_device(mconfig, ARCA5, tag, owner, clock)
	, m_program_config("program", ENDIANNESS_LITTLE, 32, 32, 0) // some docs describe these as 'middle endian'?!
	, m_io_config( "io", ENDIANNESS_LITTLE, 32, AUX_SPACE_ADDRESS_WIDTH, -2, ADDRESS_MAP_NAME( arcompact_auxreg_map ) )
{
}

device_memory_interface::space_config_vector arcompact_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_program_config),
		std::make_pair(AS_IO,      &m_io_config)
	};
}

offs_t arcompact_device::disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options)
{
	extern CPU_DISASSEMBLE( arcompact );
	return CPU_DISASSEMBLE_NAME(arcompact)(this, stream, pc, oprom, opram, options);
}


/*****************************************************************************/

/*****************************************************************************/

void arcompact_device::unimplemented_opcode(uint16_t op)
{
	fatalerror("ARCOMPACT: unknown opcode %04x at %04x\n", op, m_pc << 2);
}

/*****************************************************************************/


/*****************************************************************************/

void arcompact_device::device_start()
{
	m_pc = 0;

	m_debugger_temp = 0;

	m_program = &space(AS_PROGRAM);
	m_io = &space(AS_IO);

	state_add( ARCOMPACT_PC, "PC", m_debugger_temp).callimport().callexport().formatstr("%08X");

	state_add( ARCOMPACT_STATUS32, "STATUS32", m_debugger_temp).callimport().callexport().formatstr("%08X");
	state_add( ARCOMPACT_LP_START, "LP_START", m_debugger_temp).callimport().callexport().formatstr("%08X");
	state_add( ARCOMPACT_LP_END, "LP_END", m_debugger_temp).callimport().callexport().formatstr("%08X");

	state_add(STATE_GENPCBASE, "CURPC", m_debugger_temp).callimport().callexport().noshow();

	for (int i = 0x100; i < 0x140; i++)
	{
		state_add(i, regnames[i-0x100], m_debugger_temp).callimport().callexport().formatstr("%08X");
	}


	m_icountptr = &m_icount;
}


//-------------------------------------------------
//  state_export - export state from the device,
//  to a known location where it can be read
//-------------------------------------------------

void arcompact_device::state_export(const device_state_entry &entry)
{
	int index = entry.index();

	switch (index)
	{
		case ARCOMPACT_PC:
		case STATE_GENPCBASE:
			m_debugger_temp = m_pc;
			break;

		case ARCOMPACT_STATUS32:
			m_debugger_temp = m_status32;
			break;
		case ARCOMPACT_LP_START:
			m_debugger_temp = m_LP_START;
			break;
		case ARCOMPACT_LP_END:
			m_debugger_temp = m_LP_END;
			break;

		default:
			if ((index >= 0x100) && (index < 0x140))
			{
				m_debugger_temp = m_regs[index - 0x100];
			}
			break;

	}
}


//-------------------------------------------------
//  state_import - import state into the device,
//  after it has been set
//-------------------------------------------------

void arcompact_device::state_import(const device_state_entry &entry)
{
	int index = entry.index();

	switch (index)
	{
		case ARCOMPACT_PC:
		case STATE_GENPCBASE:
			m_pc = (m_debugger_temp & 0xfffffffe);
			break;

		case ARCOMPACT_STATUS32:
			m_status32 = m_debugger_temp;
			break;
		case ARCOMPACT_LP_START:
			m_LP_START = m_debugger_temp;
			break;
		case ARCOMPACT_LP_END:
			m_LP_END = m_debugger_temp;
			break;

		default:
			if ((index >= 0x100) && (index < 0x140))
			{
				m_regs[index - 0x100] = m_debugger_temp;
			}
			break;
	}
}

void arcompact_device::device_reset()
{
	m_pc = 0x00000000;

	m_delayactive = 0;
	m_delayjump = 0x00000000;

	for (auto & elem : m_regs)
		elem = 0;

	m_status32 = 0;
	m_LP_START = 0;
	m_LP_END = 0;
	m_INTVECTORBASE = 0;

}


/*****************************************************************************/


void arcompact_device::execute_set_input(int irqline, int state)
{
}
