// license:BSD-3-Clause
// copyright-holders:hap
/*

  Sharp SM511 MCU core implementation

  TODO:
  - undocumented/guessed opcodes:
    * $01 is guessed as DIV to ACC transfer, unknown which bits
    * $5d is certainly CEND
    * $65 is certainly divider reset, but not sure if it behaves same as on SM510
    * $6036 and $6037 may be instruction timing? (16kHz and 8kHz), mnemonics unknown

*/

#include "emu.h"
#include "sm510.h"
#include "debugger.h"


// MCU types
DEFINE_DEVICE_TYPE(SM511, sm511_device, "sm511", "SM511") // 4Kx8 ROM, 128x4 RAM(32x4 for LCD), melody controller
DEFINE_DEVICE_TYPE(SM512, sm512_device, "sm512", "SM512") // 4Kx8 ROM, 128x4 RAM(48x4 for LCD), melody controller


// internal memory maps
static ADDRESS_MAP_START(program_4k, AS_PROGRAM, 8, sm510_base_device)
	AM_RANGE(0x0000, 0x0fff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START(data_96_32x4, AS_DATA, 8, sm510_base_device)
	AM_RANGE(0x00, 0x5f) AM_RAM
	AM_RANGE(0x60, 0x6f) AM_RAM AM_SHARE("lcd_ram_a")
	AM_RANGE(0x70, 0x7f) AM_RAM AM_SHARE("lcd_ram_b")
ADDRESS_MAP_END

static ADDRESS_MAP_START(data_80_48x4, AS_DATA, 8, sm510_base_device)
	AM_RANGE(0x00, 0x4f) AM_RAM
	AM_RANGE(0x50, 0x5f) AM_RAM AM_SHARE("lcd_ram_c")
	AM_RANGE(0x60, 0x6f) AM_RAM AM_SHARE("lcd_ram_a")
	AM_RANGE(0x70, 0x7f) AM_RAM AM_SHARE("lcd_ram_b")
ADDRESS_MAP_END


// disasm
offs_t sm511_device::disasm_disassemble(std::ostream &stream, offs_t pc, const u8 *oprom, const u8 *opram, u32 options)
{
	extern CPU_DISASSEMBLE(sm511);
	return CPU_DISASSEMBLE_NAME(sm511)(this, stream, pc, oprom, opram, options);
}


// device definitions
sm511_device::sm511_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: sm511_device(mconfig, SM511, tag, owner, clock, 2 /* stack levels */, 12 /* prg width */, ADDRESS_MAP_NAME(program_4k), 7 /* data width */, ADDRESS_MAP_NAME(data_96_32x4))
{
}

sm511_device::sm511_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, int stack_levels, int prgwidth, address_map_constructor program, int datawidth, address_map_constructor data)
	: sm510_base_device(mconfig, type, tag, owner, clock, stack_levels, prgwidth, program, datawidth, data)
{
}

sm512_device::sm512_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: sm511_device(mconfig, SM512, tag, owner, clock, 2, 12, ADDRESS_MAP_NAME(program_4k), 7, ADDRESS_MAP_NAME(data_80_48x4))
{
}



//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void sm511_device::device_reset()
{
	sm510_base_device::device_reset();

	m_melody_rd &= ~1;
	m_clk_div = 4; // 8kHz
	notify_clock_changed();
}



//-------------------------------------------------
//  melody controller
//-------------------------------------------------

void sm511_device::clock_melody()
{
	if (!m_melody_rom)
		return;

	// tone cycle table (SM511/SM512 datasheet fig.5)
	// cmd 0 = cmd, 1 = stop, > 13 = illegal(unknown)
	static const u8 lut_tone_cycles[4*16] =
	{
		0, 0, 7, 8, 8, 9, 9, 10,11,11,12,13,14,14, 7*2, 8*2,
		0, 0, 8, 8, 9, 9, 10,11,11,12,13,13,14,15, 8*2, 8*2,
		0, 0, 8, 8, 9, 9, 10,10,11,12,12,13,14,15, 8*2, 8*2,
		0, 0, 8, 9, 9, 10,10,11,11,12,13,14,14,15, 8*2, 9*2
	};

	u8 cmd = m_melody_rom[m_melody_address] & 0x3f;
	u8 out = 0;

	// clock duty cycle if tone is active
	if ((cmd & 0xf) > 1)
	{
		out = m_melody_duty_index & m_melody_rd & 1;
		m_melody_duty_count++;
		int index = m_melody_duty_index << 4 | (cmd & 0xf);
		int shift = ~cmd >> 4 & 1; // OCT

		if (m_melody_duty_count >= (lut_tone_cycles[index] << shift))
		{
			m_melody_duty_count = 0;
			m_melody_duty_index = (m_melody_duty_index + 1) & 3;
		}
	}
	else if ((cmd & 0xf) == 1)
	{
		// rest tell signal
		m_melody_rd |= 2;
	}

	// clock time base on F8(d7)
	if ((m_div & 0x7f) == 0)
	{
		u8 mask = (cmd & 0x20) ? 0x1f : 0x0f;
		m_melody_step_count = (m_melody_step_count + 1) & mask;

		if (m_melody_step_count == 0)
			m_melody_address++;
	}

	// output to R pin
	if (out != m_r_out)
	{
		m_write_r(0, out, 0xff);
		m_r_out = out;
	}
}

void sm511_device::init_melody()
{
	if (!m_melody_rom)
		return;

	// verify melody rom
	for (int i = 0; i < 0x100; i++)
	{
		u8 data = m_melody_rom[i];
		if (data & 0xc0 || (data & 0x0f) > 13)
			logerror("%s unknown melody ROM data $%02X at $%02X\n", tag(), data, i);
	}
}



//-------------------------------------------------
//  execute
//-------------------------------------------------

void sm511_device::get_opcode_param()
{
	// LBL, PRE, TL, TML and prefix opcodes are 2 bytes
	if ((m_op >= 0x5f && m_op <= 0x61) || (m_op & 0xf0) == 0x70 || (m_op & 0xfc) == 0x68)
	{
		m_icount--;
		m_param = m_program->read_byte(m_pc);
		increment_pc();
	}
}

void sm511_device::execute_one()
{
	switch (m_op & 0xf0)
	{
		case 0x20: op_lax(); break;
		case 0x30: op_adx(); break;
		case 0x40: op_lb(); break;
		case 0x70: op_tl(); break;

		case 0x80: case 0x90: case 0xa0: case 0xb0:
			op_t(); break;
		case 0xc0: case 0xd0: case 0xe0: case 0xf0:
			op_tm(); break;

		default:
			switch (m_op & 0xfc)
			{
		case 0x04: op_rm(); break;
		case 0x0c: op_sm(); break;
		case 0x10: op_exc(); break;
		case 0x14: op_exci(); break;
		case 0x18: op_lda(); break;
		case 0x1c: op_excd(); break;
		case 0x54: op_tmi(); break;
		case 0x68: op_tml(); break;

		default:
			switch (m_op)
			{
		case 0x00: op_rot(); break;
		case 0x01: op_dta(); break;
		case 0x02: op_sbm(); break;
		case 0x03: op_atpl(); break;
		case 0x08: op_add(); break;
		case 0x09: op_add11(); break;
		case 0x0a: op_coma(); break;
		case 0x0b: op_exbla(); break;

		case 0x50: op_kta(); break;
		case 0x51: op_tb(); break;
		case 0x52: op_tc(); break;
		case 0x53: op_tam(); break;
		case 0x58: op_tis(); break;
		case 0x59: op_atl(); break;
		case 0x5a: op_ta0(); break;
		case 0x5b: op_tabl(); break;
		case 0x5c: op_atx(); break;
		case 0x5d: op_cend(); break;
		case 0x5e: op_tal(); break;
		case 0x5f: op_lbl(); break;

		case 0x61: op_pre(); break;
		case 0x62: op_wr(); break;
		case 0x63: op_ws(); break;
		case 0x64: op_incb(); break;
		case 0x65: op_dr(); break;
		case 0x66: op_rc(); break;
		case 0x67: op_sc(); break;
		case 0x6c: op_decb(); break;
		case 0x6d: op_ptw(); break;
		case 0x6e: op_rtn0(); break;
		case 0x6f: op_rtn1(); break;

		// extended opcodes
		case 0x60:
			m_op = m_op << 8 | m_param;
			switch (m_param)
			{
		case 0x30: op_rme(); break;
		case 0x31: op_sme(); break;
		case 0x32: op_tmel(); break;
		case 0x33: op_atfc(); break;
		case 0x34: op_bdc(); break;
		case 0x35: op_atbp(); break;
		case 0x36: op_clkhi(); break;
		case 0x37: op_clklo(); break;

		default: op_illegal(); break;
			}
			break; // 0x60

		default: op_illegal(); break;
			}
			break; // 0xff

			}
			break; // 0xfc

	} // big switch

	// BM high bit is only valid for 1 step
	m_sbm = (m_op == 0x02);
}
