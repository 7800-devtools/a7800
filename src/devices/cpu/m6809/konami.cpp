// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/*********************************************************************

    konami.c

    Portable Konami cpu emulator

    Based on M6809 cpu core copyright John Butler

    References:

        6809 Simulator V09, By L.C. Benschop, Eindhoven The Netherlands.

        m6809: Portable 6809 emulator, DS (6809 code in MAME, derived from
            the 6809 Simulator V09)

        6809 Microcomputer Programming & Interfacing with Experiments"
            by Andrew C. Staugaard, Jr.; Howard W. Sams & Co., Inc.

    System dependencies:    uint16_t must be 16 bit unsigned int
                            uint8_t must be 8 bit unsigned int
                            uint32_t must be more than 16 bits
                            arrays up to 65536 bytes must be supported
                            machine must be twos complement

    History:

March 2013 NPW:
    Rewrite of 6809/6309/Konami CPU; overall core is now unified and
    supports mid-instruction timings.

    Some of the instruction timings have changed with the new core; the
    old core had some nonsensical timings.  For example (from scontra):

        819A    3A 07 1F 8C        STA $1f8C

    Under the old core, this took four clock cycles, which is dubious
    because this instruction would have to do four opcode reads and one
    write.  OGalibert says that the current timings are just a guess and
    nobody has done precise readings, so I'm replacing the old guesses
    with new guesses.

991022 HJB:
    Tried to improve speed: Using bit7 of cycles1 as flag for multi
    byte opcodes is gone, those opcodes now instead go through opcode2().
    Inlined fetch_effective_address() into that function as well.
    Got rid of the slow/fast flags for stack (S and U) memory accesses.
    Minor changes to use 32 bit values as arguments to memory functions
    and added defines for that purpose (e.g. X = 16bit XD = 32bit).

990720 EHC:
    Created this file

*****************************************************************************/

#include "emu.h"
#include "debugger.h"
#include "konami.h"
#include "m6809inl.h"


//**************************************************************************
//  PARAMETERS
//**************************************************************************

// turn off 'unreferenced label' errors
#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-label"
#endif
#ifdef _MSC_VER
#pragma warning( disable : 4102 )
#endif


//**************************************************************************
//  DEVICE INTERFACE
//**************************************************************************

DEFINE_DEVICE_TYPE(KONAMI, konami_cpu_device, "konami_cpu", "KONAMI CPU")


//-------------------------------------------------
//  konami_cpu_device - constructor
//-------------------------------------------------

konami_cpu_device::konami_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: m6809_base_device(mconfig, tag, owner, clock, KONAMI, 1)
	, m_set_lines(*this)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void konami_cpu_device::device_start()
{
	super::device_start();

	// resolve callbacks
	m_set_lines.resolve();
}


//-------------------------------------------------
//  disasm_disassemble - call the disassembly
//  helper function
//-------------------------------------------------

offs_t konami_cpu_device::disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options)
{
	extern CPU_DISASSEMBLE( konami );
	return CPU_DISASSEMBLE_NAME(konami)(this, stream, pc, oprom, opram, options);
}


//-------------------------------------------------
//  read_operand
//-------------------------------------------------

inline uint8_t konami_cpu_device::read_operand()
{
	return super::read_operand();
}


//-------------------------------------------------
//  read_operand
//-------------------------------------------------

inline uint8_t konami_cpu_device::read_operand(int ordinal)
{
	switch(m_addressing_mode)
	{
		case ADDRESSING_MODE_EA:            return read_memory(m_ea.w + ordinal);
		case ADDRESSING_MODE_IMMEDIATE:     return read_opcode_arg();
		case ADDRESSING_MODE_REGISTER_D:    return (ordinal & 1) ? m_q.r.b : m_q.r.a;
		default:                            fatalerror("Unexpected");
	}
}


//-------------------------------------------------
//  write_operand
//-------------------------------------------------

inline void konami_cpu_device::write_operand(uint8_t data)
{
	super::write_operand(data);
}



//-------------------------------------------------
//  write_operand
//-------------------------------------------------

inline void konami_cpu_device::write_operand(int ordinal, uint8_t data)
{
	switch(m_addressing_mode)
	{
		case ADDRESSING_MODE_IMMEDIATE:     /* do nothing */                                break;
		case ADDRESSING_MODE_EA:            write_memory(m_ea.w + ordinal, data);           break;
		case ADDRESSING_MODE_REGISTER_D:    *((ordinal & 1) ? &m_q.r.b : &m_q.r.a) = data;  break;
		default:                            fatalerror("Unexpected");
	}
}


//-------------------------------------------------
//  ireg
//-------------------------------------------------

inline uint16_t &konami_cpu_device::ireg()
{
	switch(m_opcode & 0x70)
	{
		case 0x20:  return m_x.w;
		case 0x30:  return m_y.w;
		case 0x50:  return m_u.w;
		case 0x60:  return m_s.w;
		case 0x70:  return m_pc.w;
		default:
			fatalerror("Should not get here");
			// never executed
			//return m_x.w;
	}
}


//-------------------------------------------------
//  read_exgtfr_register
//-------------------------------------------------

inline m6809_base_device::exgtfr_register konami_cpu_device::read_exgtfr_register(uint8_t reg)
{
	exgtfr_register result;
	result.word_value = 0x00FF;

	switch(reg & 0x07)
	{
		case  0: result.word_value = m_q.r.a;   break;  // A
		case  1: result.word_value = m_q.r.b;   break;  // B
		case  2: result.word_value = m_x.w;     break;  // X
		case  3: result.word_value = m_y.w;     break;  // Y
		case  4: result.word_value = m_s.w;     break;  // S
		case  5: result.word_value = m_u.w;     break;  // U
	}
	result.byte_value = (uint8_t) result.word_value;
	return result;
}


//-------------------------------------------------
//  write_exgtfr_register
//-------------------------------------------------

inline void konami_cpu_device::write_exgtfr_register(uint8_t reg, m6809_base_device::exgtfr_register value)
{
	switch(reg & 0x07)
	{
		case  0: m_q.r.a = value.byte_value;    break;  // A
		case  1: m_q.r.b = value.byte_value;    break;  // B
		case  2: m_x.w   = value.word_value;    break;  // X
		case  3: m_y.w   = value.word_value;    break;  // Y
		case  4: m_s.w   = value.word_value;    break;  // S
		case  5: m_u.w   = value.word_value;    break;  // U
	}
}


//-------------------------------------------------
//  safe_shift_right
//-------------------------------------------------

template<class T> T konami_cpu_device::safe_shift_right(T value, uint32_t shift)
{
	T result;

	if (shift < (sizeof(T) * 8))
		result = value >> shift;
	else if (value < 0)
		result = (T) -1;
	else
		result = 0;

	return result;
}


//-------------------------------------------------
//  safe_shift_right_unsigned
//-------------------------------------------------

template<class T> T konami_cpu_device::safe_shift_right_unsigned(T value, uint32_t shift)
{
	T result;

	if (shift < (sizeof(T) * 8))
		result = value >> shift;
	else
		result = 0;

	return result;
}

//-------------------------------------------------
//  safe_shift_left
//-------------------------------------------------

template<class T> T konami_cpu_device::safe_shift_left(T value, uint32_t shift)
{
	T result;

	if (shift < (sizeof(T) * 8))
		result = value << shift;
	else
		result = 0;

	return result;
}


//-------------------------------------------------
//  lmul
//-------------------------------------------------

inline void konami_cpu_device::lmul()
{
	PAIR result;

	// do the multiply
	result.d = (uint32_t)m_x.w * m_y.w;

	// set the result registers
	m_x.w = result.w.h;
	m_y.w = result.w.l;

	// set Z flag
	set_flags<uint32_t>(CC_Z, result.d);

	// set C flag
	if (result.d & 0x8000)
		m_cc |= CC_C;
	else
		m_cc &= ~CC_C;
}


//-------------------------------------------------
//  divx
//-------------------------------------------------

inline void konami_cpu_device::divx()
{
	uint16_t result;
	uint8_t remainder;

	if (m_q.r.b != 0)
	{
		result = m_x.w / m_q.r.b;
		remainder = m_x.w % m_q.r.b;
	}
	else
	{
		// divide by zero; not sure what happens
		result = 0;
		remainder = 0;
	}

	// set results and Z flag
	m_x.w = set_flags<uint16_t>(CC_Z, result);
	m_q.r.b = remainder;

	// set C flag
	if (result & 0x0080)
		m_cc |= CC_C;
	else
		m_cc &= ~CC_C;
}


//-------------------------------------------------
//  set_lines
//-------------------------------------------------

void konami_cpu_device::set_lines(uint8_t data)
{
	if (!m_set_lines.isnull())
		m_set_lines((offs_t)0, data);
}


//-------------------------------------------------
//  execute_one - try to execute a single instruction
//-------------------------------------------------

inline void konami_cpu_device::execute_one()
{
	switch(pop_state())
	{
#include "cpu/m6809/konami.hxx"
	}
}


//-------------------------------------------------
//  execute_run - execute a timeslice's worth of
//  opcodes
//-------------------------------------------------

void konami_cpu_device::execute_run()
{
	do
	{
		execute_one();
	} while(m_icount > 0);
}
