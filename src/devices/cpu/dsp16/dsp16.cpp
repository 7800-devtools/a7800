// license:BSD-3-Clause
// copyright-holders:Andrew Gardner
/***************************************************************************

    dsp16.h

    WE|AT&T DSP16 series emulator.

***************************************************************************/

#include "emu.h"
#include "debugger.h"
#include "dsp16.h"

//
// TODO:
//  * Store the cache in 15 unique memory locations as it is on-chip.
//  * Modify cycle counts when running from within the cache
//  * A write to PI resets the pseudoramdom sequence generator)  (page 2-4)
//  * Handle virtual shift addressing using RB & RE (when RE is enabled)  (page 2-6)
//  * The ALU sign-extends 32-bit operands from y or p to 36 bits and produces a 36-bit output
//  * Interrupt lines  (page 2-15)
//

//**************************************************************************
//  DEVICE INTERFACE
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(DSP16, dsp16_device, "dsp16", "DSP16")


//-------------------------------------------------
//  dsp16_device - constructor
//-------------------------------------------------

dsp16_device::dsp16_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: cpu_device(mconfig, DSP16, tag, owner, clock),
		m_program_config("program", ENDIANNESS_LITTLE, 16, 16, -1),
		m_data_config("data", ENDIANNESS_LITTLE, 16, 16, -1),
		m_i(0),
		m_pc(0),
		m_pt(0),
		m_pr(0),
		m_pi(0),
		m_j(0),
		m_k(0),
		m_rb(0),
		m_re(0),
		m_r0(0),
		m_r1(0),
		m_r2(0),
		m_r3(0),
		m_x(0),
		m_y(0),
		m_p(0),
		m_a0(0),
		m_a1(0),
		m_auc(0),
		m_psw(0),
		m_c0(0),
		m_c1(0),
		m_c2(0),
		m_sioc(0),
		m_srta(0),
		m_sdx(0),
		m_pioc(0),
		m_pdx0(0),
		m_pdx1(0),
		m_ppc(0),
		m_cacheStart(CACHE_INVALID),
		m_cacheEnd(CACHE_INVALID),
		m_cacheRedoNextPC(CACHE_INVALID),
		m_cacheIterations(0),
		m_program(nullptr),
		m_data(nullptr),
		m_direct(nullptr),
		m_icount(0)
{
	// Allocate & setup
}



//-------------------------------------------------
//  device_start - start up the device
//-------------------------------------------------

void dsp16_device::device_start()
{
	// register state with the debugger
	state_add(STATE_GENPC,    "GENPC",     m_pc).noshow();
	state_add(STATE_GENPCBASE, "CURPC",    m_ppc).noshow();
	state_add(STATE_GENFLAGS, "GENFLAGS",  m_psw).callimport().callexport().formatstr("%10s").noshow();
	state_add(DSP16_PC,       "PC",        m_pc);
	state_add(DSP16_I,        "I",         m_i);
	state_add(DSP16_PT,       "PT",        m_pt);
	state_add(DSP16_PR,       "PR",        m_pr);
	state_add(DSP16_PI,       "PI",        m_pi);
	state_add(DSP16_J,        "J",         m_j);
	state_add(DSP16_K,        "K",         m_k);
	state_add(DSP16_RB,       "RB",        m_rb);
	state_add(DSP16_RE,       "RE",        m_re);
	state_add(DSP16_R0,       "R0",        m_r0);
	state_add(DSP16_R1,       "R1",        m_r1);
	state_add(DSP16_R2,       "R2",        m_r2);
	state_add(DSP16_R3,       "R3",        m_r3);
	state_add(DSP16_X,        "X",         m_x);
	state_add(DSP16_Y,        "Y",         m_y);
	state_add(DSP16_P,        "P",         m_p);
	state_add(DSP16_A0,       "A0",        m_a0).mask(0xfffffffffU);
	state_add(DSP16_A1,       "A1",        m_a1).mask(0xfffffffffU);
	state_add(DSP16_AUC,      "AUC",       m_auc).formatstr("%8s");
	state_add(DSP16_PSW,      "PSW",       m_psw).formatstr("%16s");
	state_add(DSP16_C0,       "C0",        m_c0);
	state_add(DSP16_C1,       "C1",        m_c1);
	state_add(DSP16_C2,       "C2",        m_c2);
	state_add(DSP16_SIOC,     "SIOC",      m_sioc).formatstr("%10s");
	state_add(DSP16_SRTA,     "SRTA",      m_srta);
	state_add(DSP16_SDX,      "SDX",       m_sdx);
	state_add(DSP16_PIOC,     "PIOC",      m_pioc).formatstr("%16s");
	state_add(DSP16_PDX0,     "PDX0",      m_pdx0);
	state_add(DSP16_PDX1,     "PDX1",      m_pdx1);

	// register our state for saving
	save_item(NAME(m_i));
	save_item(NAME(m_pc));
	save_item(NAME(m_pt));
	save_item(NAME(m_pr));
	save_item(NAME(m_pi));
	save_item(NAME(m_j));
	save_item(NAME(m_k));
	save_item(NAME(m_rb));
	save_item(NAME(m_re));
	save_item(NAME(m_r0));
	save_item(NAME(m_r1));
	save_item(NAME(m_r2));
	save_item(NAME(m_r3));
	save_item(NAME(m_x));
	save_item(NAME(m_y));
	save_item(NAME(m_p));
	save_item(NAME(m_a0));
	save_item(NAME(m_a1));
	save_item(NAME(m_auc));
	save_item(NAME(m_psw));
	save_item(NAME(m_c0));
	save_item(NAME(m_c1));
	save_item(NAME(m_c2));
	save_item(NAME(m_sioc));
	save_item(NAME(m_srta));
	save_item(NAME(m_sdx));
	save_item(NAME(m_pioc));
	save_item(NAME(m_pdx0));
	save_item(NAME(m_pdx1));
	save_item(NAME(m_ppc));
	save_item(NAME(m_cacheStart));
	save_item(NAME(m_cacheEnd));
	save_item(NAME(m_cacheRedoNextPC));
	save_item(NAME(m_cacheIterations));

	// get our address spaces
	m_program = &space(AS_PROGRAM);
	m_data = &space(AS_DATA);
	m_direct = &m_program->direct();

	// set our instruction counter
	m_icountptr = &m_icount;
}


//-------------------------------------------------
//  device_reset - reset the device
//-------------------------------------------------

void dsp16_device::device_reset()
{
	// Page 7-5
	m_pc = 0x0000;
	m_pi = 0x0000;
	m_sioc = 0x0000;    // (page 5-4)

	// SRTA is unaltered by reset
	m_pioc = 0x0008;
	m_rb = 0x0000;
	m_re = 0x0000;

	// AUC is not affected by reset
	m_ppc = m_pc;

	// Hacky cache emulation.
	m_cacheStart = CACHE_INVALID;
	m_cacheEnd = CACHE_INVALID;
	m_cacheRedoNextPC = CACHE_INVALID;
	m_cacheIterations = 0;
}


//-------------------------------------------------
//  memory_space_config - return the configuration
//  of the specified address space, or nullptr if
//  the space doesn't exist
//-------------------------------------------------

device_memory_interface::space_config_vector dsp16_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_program_config),
		std::make_pair(AS_DATA,    &m_data_config)
	};
}


//-------------------------------------------------
//  state_string_export - export state as a string
//  for the debugger
//-------------------------------------------------

void dsp16_device::state_string_export(const device_state_entry &entry, std::string &str) const
{
	switch (entry.index())
	{
	case STATE_GENFLAGS:
		str = "(below)";
		break;

	case DSP16_AUC:
		{
			std::string alignString;
			const uint8_t align = m_auc & 0x03;
			switch (align)
			{
			case 0x00: alignString = "xy"; break;
			case 0x01: alignString = "/4"; break;
			case 0x02: alignString = "x4"; break;
			case 0x03: alignString = ",,"; break;
			}
			str = string_format("%c%c%c%c%c%s",
					m_auc & 0x40 ? 'Y':'.',
					m_auc & 0x20 ? '1':'.',
					m_auc & 0x10 ? '0':'.',
					m_auc & 0x08 ? '1':'.',
					m_auc & 0x04 ? '0':'.',
					alignString);
		}
		break;

	case DSP16_PSW:
		str = string_format("%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
				m_psw & 0x8000 ? 'M':'.',
				m_psw & 0x4000 ? 'E':'.',
				m_psw & 0x2000 ? 'L':'.',
				m_psw & 0x1000 ? 'V':'.',
				m_psw & 0x0800 ? ',':',',
				m_psw & 0x0400 ? ',':',',
				m_psw & 0x0200 ? 'O':'.',
				m_psw & 0x0100 ? '1':'.',
				m_psw & 0x0080 ? '1':'.',
				m_psw & 0x0040 ? '1':'.',
				m_psw & 0x0020 ? '1':'.',
				m_psw & 0x0010 ? 'O':'.',
				m_psw & 0x0008 ? '1':'.',
				m_psw & 0x0004 ? '1':'.',
				m_psw & 0x0002 ? '1':'.',
				m_psw & 0x0001 ? '1':'.');
		break;

	case DSP16_PIOC:
		{
			std::string strobeString;
			const uint8_t strobe = (m_pioc & 0x6000) >> 13;
			switch (strobe)
			{
			case 0x00: strobeString = "1T"; break;
			case 0x01: strobeString = "2T"; break;
			case 0x02: strobeString = "3T"; break;
			case 0x03: strobeString = "4T"; break;
			}
			str = string_format("%c%s%c%c%c%c%c%c%c%c%c%c%c%c%c",
					m_pioc & 0x8000 ? 'I':'.',
					strobeString,
					m_pioc & 0x1000 ? 'O':'I',
					m_pioc & 0x0800 ? 'O':'I',
					m_pioc & 0x0400 ? 'S':'.',
					m_pioc & 0x0200 ? 'I':'.',
					m_pioc & 0x0100 ? 'O':'.',
					m_pioc & 0x0080 ? 'P':'.',
					m_pioc & 0x0040 ? 'P':'.',
					m_pioc & 0x0020 ? 'I':'.',
					m_pioc & 0x0010 ? 'I':'.',
					m_pioc & 0x0008 ? 'O':'.',
					m_pioc & 0x0004 ? 'P':'.',
					m_pioc & 0x0002 ? 'P':'.',
					m_pioc & 0x0001 ? 'I':'.');
		}
		break;

	// Placeholder for a better view later (TODO)
	case DSP16_SIOC:
		{
			std::string clkString;
			const uint8_t clk = (m_sioc & 0x0180) >> 7;
			switch (clk)
			{
				case 0x00: clkString = "/4"; break;
				case 0x01: clkString = "12"; break;
				case 0x02: clkString = "16"; break;
				case 0x03: clkString = "20"; break;
			}
			str = string_format("%c%s%c%c%c%c%c%c%c",
					m_sioc & 0x0200 ? 'I':'O',
					clkString,
					m_sioc & 0x0040 ? 'L':'M',
					m_sioc & 0x0020 ? 'I':'O',
					m_sioc & 0x0010 ? 'I':'O',
					m_sioc & 0x0008 ? 'I':'O',
					m_sioc & 0x0004 ? 'I':'O',
					m_sioc & 0x0002 ? '2':'1',
					m_sioc & 0x0001 ? '2':'1');
		}
		break;
	}
}


//-------------------------------------------------
//  disasm_min_opcode_bytes - return the length
//  of the shortest instruction, in bytes
//-------------------------------------------------

uint32_t dsp16_device::disasm_min_opcode_bytes() const
{
	return 2;
}


//-------------------------------------------------
//  disasm_max_opcode_bytes - return the length
//  of the longest instruction, in bytes
//-------------------------------------------------

uint32_t dsp16_device::disasm_max_opcode_bytes() const
{
	return 4;
}


//-------------------------------------------------
//  disasm_disassemble - call the disassembly
//  helper function
//-------------------------------------------------

offs_t dsp16_device::disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options)
{
	extern CPU_DISASSEMBLE( dsp16a );
	return CPU_DISASSEMBLE_NAME(dsp16a)(this, stream, pc, oprom, opram, options);
}



/***************************************************************************
    MEMORY ACCESSORS
***************************************************************************/

inline uint32_t dsp16_device::data_read(const uint16_t& addr)
{
	return m_data->read_word(addr << 1);
}

inline void dsp16_device::data_write(const uint16_t& addr, const uint16_t& data)
{
	m_data->write_word(addr << 1, data & 0xffff);
}

inline uint32_t dsp16_device::opcode_read(const uint8_t pcOffset)
{
	const uint16_t readPC = m_pc + pcOffset;
	return m_direct->read_dword(readPC << 1);
}


/***************************************************************************
    CORE EXECUTION LOOP
***************************************************************************/

//-------------------------------------------------
//  execute_min_cycles - return minimum number of
//  cycles it takes for one instruction to execute
//-------------------------------------------------

uint32_t dsp16_device::execute_min_cycles() const
{
	return 1;
}


//-------------------------------------------------
//  execute_max_cycles - return maximum number of
//  cycles it takes for one instruction to execute
//-------------------------------------------------

uint32_t dsp16_device::execute_max_cycles() const
{
	return 1;
}


//-------------------------------------------------
//  execute_input_lines - return the number of
//  input/interrupt lines
//-------------------------------------------------

uint32_t dsp16_device::execute_input_lines() const
{
	return 1;
}


void dsp16_device::execute_set_input(int inputnum, int state)
{
	// Only has one external IRQ line
}


void dsp16_device::execute_run()
{
	// HACK TO MAKE CPU DO NOTHING.
	// REMOVE IF DEVELOPING CPU CORE.
	m_icount = 0;
	return;

	do
	{
		// debugging
		m_ppc = m_pc;   // copy PC to previous PC
		debugger_instruction_hook(this, m_pc);

		// instruction fetch & execute
		uint8_t cycles;
		uint8_t pcAdvance;
		const uint16_t op = opcode_read();
		execute_one(op, cycles, pcAdvance);

		// step
		m_pc += pcAdvance;
		m_icount -= cycles;

		// The 16 bit PI "shadow" register gets set to PC on each instruction except
		// when an interrupt service routine is active (TODO: Interrupt check)  (page 2-4)
		m_pi = m_pc;

	} while (m_icount > 0);
}

#include "dsp16ops.hxx"
