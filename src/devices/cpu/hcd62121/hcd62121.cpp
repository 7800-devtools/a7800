// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    Hitachi hcd62121 cpu core emulation.

The Hitachi hcd62121 is the custom cpu which was used in the Casio
CFX-9850 (and maybe some other things too).

This CPU core is based on the information provided by Martin Poupe.
Martin Poupe's site can be found at http://martin.poupe.org/casio/

TODO:
 - instruction timings
 - unknown instructions

**********************************************************************/

#include "emu.h"
#include "hcd62121.h"

#include "debugger.h"


enum
{
	HCD62121_IP=1, HCD62121_SP, HCD62121_F, HCD62121_LAR,
	HCD62121_CS, HCD62121_DS, HCD62121_SS, HCD62121_DSIZE,
	/* 128 byte register file */
	HCD62121_R00, HCD62121_R04, HCD62121_R08, HCD62121_R0C,
	HCD62121_R10, HCD62121_R14, HCD62121_R18, HCD62121_R1C,
	HCD62121_R20, HCD62121_R24, HCD62121_R28, HCD62121_R2C,
	HCD62121_R30, HCD62121_R34, HCD62121_R38, HCD62121_R3C,
	HCD62121_R40, HCD62121_R44, HCD62121_R48, HCD62121_R4C,
	HCD62121_R50, HCD62121_R54, HCD62121_R58, HCD62121_R5C,
	HCD62121_R60, HCD62121_R64, HCD62121_R68, HCD62121_R6C,
	HCD62121_R70, HCD62121_R74, HCD62121_R78, HCD62121_R7C
};


constexpr u8 FLAG_Z = 0x08;
constexpr u8 FLAG_C = 0x02;
constexpr u8 FLAG_ZL = 0x04;
constexpr u8 FLAG_CL = 0x01;
constexpr u8 FLAG_ZH = 0x10;


DEFINE_DEVICE_TYPE(HCD62121, hcd62121_cpu_device, "hcd62121_cpu_device", "Hitachi HCD62121")


hcd62121_cpu_device::hcd62121_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: cpu_device(mconfig, HCD62121, tag, owner, clock)
	, m_program_config("program", ENDIANNESS_BIG, 8, 24, 0)
	, m_prev_pc(0)
	, m_sp(0)
	, m_ip(0)
	, m_dsize(0)
	, m_cseg(0)
	, m_dseg(0)
	, m_sseg(0)
	, m_f(0)
	, m_lar(0)
	, m_opt(0)
	, m_port(0)
	, m_program(nullptr)
	, m_icount(0)
	, m_kol_cb(*this)
	, m_koh_cb(*this)
	, m_port_cb(*this)
	, m_opt_cb(*this)
	, m_ki_cb(*this)
	, m_in0_cb(*this)
{
}

device_memory_interface::space_config_vector hcd62121_cpu_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_program_config)
	};
}

u8 hcd62121_cpu_device::read_op()
{
	u8 d = m_program->read_byte( ( m_cseg << 16 ) | m_ip );
	m_ip++;
	return d;
}


u8 hcd62121_cpu_device::datasize(u8 op)
{
	switch (op & 0x03)
	{
	case 0:
		return 1;
	case 1:
		return 2;
	case 2:
		return (m_dsize >> 4) + 1;
	case 3:
		return (m_dsize & 0x0f) + 1;
	}
	return 1;
}


void hcd62121_cpu_device::read_reg(int size, u8 op1)
{
	if (op1 & 0x80)
	{
		for (int i = 0; i < size; i++)
			m_temp1[i] = m_reg[(op1 - i) & 0x7f];
	}
	else
	{
		for (int i = 0; i < size; i++)
			m_temp1[i] = m_reg[(op1 + i) & 0x7f];
	}
}


void hcd62121_cpu_device::write_reg(int size, u8 op1)
{
	if (op1 & 0x80)
	{
		for (int i = 0; i < size; i++)
			m_reg[(op1 - i) & 0x7f] = m_temp1[i];
	}
	else
	{
		for (int i = 0; i < size; i++)
			m_reg[(op1 + i) & 0x7f] = m_temp1[i];
	}
}


void hcd62121_cpu_device::read_regreg(int size, u8 op1, u8 op2, bool op_is_logical)
{
	for (int i = 0; i < size; i++)
		m_temp1[i] = m_reg[(op1 + i) & 0x7f];

	if (op1 & 0x80)
	{
		/* Second operand is an immediate value */
		m_temp2[0] = op2;
		for (int i = 1; i < size; i++)
			m_temp2[i] = op_is_logical ? op2 : 0;
	}
	else
	{
		/* Second operand is a register */
		for (int i = 0; i < size; i++)
			m_temp2[i] = m_reg[(op2 + i) & 0x7f];
	}

	if (!(op1 & 0x80) && !(op2 & 0x80))
	{
		/* We need to swap parameters */
		for (int i = 0; i < size; i++)
		{
			u8 v = m_temp1[i];
			m_temp1[i] = m_temp2[i];
			m_temp2[i] = v;
		}
	}
}


void hcd62121_cpu_device::write_regreg(int size, u8 op1, u8 op2)
{
	if ((op1 & 0x80) || (op2 & 0x80))
	{
		/* store in reg1 */
		for (int i = 0; i < size; i++)
			m_reg[(op1 + i) & 0x7f] = m_temp1[i];
	}
	else
	{
		/* store in reg2 */
		for (int i = 0; i < size; i++)
			m_reg[(op2 + i) & 0x7f] = m_temp1[i];
	}
}


void hcd62121_cpu_device::read_iregreg(int size, u8 op1, u8 op2)
{
	u16 ad = m_reg[(0x40 | op1) & 0x7f ] | (m_reg[(0x40 | (op1 + 1)) & 0x7f] << 8);

	for (int i = 0; i < size; i++)
	{
		m_temp1[i] = m_program->read_byte((m_dseg << 16) | ad);
		ad += (op1 & 0x40) ? -1 : 1;
	}
	m_lar = ad;

	if (op1 & 0x80)
	{
		m_temp2[0] = op2;
		for (int i = 1; i < size; i++)
			m_temp2[i] = 0;
	}
	else
	{
		for (int i = 0; i < size; i++)
			m_temp2[i] = m_reg[(op2 + i) & 0x7f];
	}

	if (!(op1 & 0x80) && !(op2 & 0x80))
	{
		/* We need to swap parameters */
		for (int i = 0; i < size; i++)
		{
			u8 v = m_temp1[i];
			m_temp1[i] = m_temp2[i];
			m_temp2[i] = v;
		}
	}
}


void hcd62121_cpu_device::write_iregreg(int size, u8 op1, u8 op2)
{
	if ((op1 & 0x80) || (op2 & 0x80))
	{
		/* store in (reg1) */
		u16 ad = m_reg[(0x40 | op1) & 0x7f] | (m_reg[(0x40 | (op1 + 1)) & 0x7f] << 8);

		for (int i = 0; i < size; i++)
		{
			m_program->write_byte((m_dseg << 16) | ad, m_temp1[i]);
			ad += (op1 & 0x40) ? -1 : 1;
		}
		m_lar = ad;
	}
	else
	{
		/* store in reg2 */
		for (int i = 0; i < size; i++)
			m_reg[(op2 + i) & 0x7f] = m_temp1[i];
	}
}


void hcd62121_cpu_device::write_iregreg2(int size, u8 op1, u8 op2)
{
	if ((op1 & 0x80) || (op2 & 0x80))
	{
		/* store in reg2 */
		for (int i = 0; i < size; i++)
			m_reg[(op2 + i) & 0x7f] = m_temp2[i];
	}
	else
	{
		/* store in (reg1) */
		u16 ad = m_reg[(0x40 | op1) & 0x7f] | (m_reg[(0x40 | (op1 + 1)) & 0x7f] << 8);

		for (int i = 0; i < size; i++)
		{
			m_program->write_byte((m_dseg << 16) | ad, m_temp2[i]);
			ad += (op1 & 0x40) ? -1 : 1;
		}
		m_lar = ad;
	}
}


bool hcd62121_cpu_device::check_cond(u8 op)
{
	switch (op & 0x07)
	{
	case 0x00:  /* ZH set */
		return (m_f & FLAG_ZH);

	case 0x01:  /* ZL set */
		return (m_f & FLAG_ZL);

	case 0x02:  /* C set */
		return (m_f & FLAG_C);

	case 0x03:  /* Z set */
		return (m_f & FLAG_Z);

	case 0x04:  /* Z or C set */
		return (m_f & (FLAG_Z | FLAG_C));

	case 0x05:  /* CL set */
		return (m_f & FLAG_CL);

	case 0x06:  /* C clear */
		return (!(m_f & FLAG_C));

	case 0x07:  /* Z clear */
		return (!(m_f & FLAG_Z));
	}

	return false;
}


void hcd62121_cpu_device::device_start()
{
	m_program = &space(AS_PROGRAM);

	m_kol_cb.resolve_safe();
	m_koh_cb.resolve_safe();
	m_port_cb.resolve_safe();
	m_opt_cb.resolve_safe();
	m_ki_cb.resolve_safe(0);
	m_in0_cb.resolve_safe(0);

	save_item(NAME(m_prev_pc));
	save_item(NAME(m_sp));
	save_item(NAME(m_ip));
	save_item(NAME(m_dsize));
	save_item(NAME(m_cseg));
	save_item(NAME(m_dseg));
	save_item(NAME(m_sseg));
	save_item(NAME(m_f));
	save_item(NAME(m_lar));
	save_item(NAME(m_reg));
	save_item(NAME(m_temp1));
	save_item(NAME(m_temp2));
	save_item(NAME(m_opt));
	save_item(NAME(m_port));

	// Register state for debugger
	state_add(STATE_GENPC,    "GENPC",    m_rtemp).callexport().formatstr("%8s");
	state_add(STATE_GENPCBASE,"CURPC",    m_rtemp).callexport().formatstr("%8s");
	state_add(STATE_GENFLAGS, "GENFLAGS", m_f ).callimport().callexport().formatstr("%12s").noshow();

	state_add(HCD62121_IP,    "IP",    m_ip   ).callimport().callexport().formatstr("%04X");
	state_add(HCD62121_SP,    "SP",    m_sp   ).callimport().callexport().formatstr("%04X");
	state_add(HCD62121_LAR,   "LAR",   m_lar  ).callimport().callexport().formatstr("%04X");
	state_add(HCD62121_CS,    "CS",    m_cseg ).callimport().callexport().formatstr("%02X");
	state_add(HCD62121_DS,    "DS",    m_dseg ).callimport().callexport().formatstr("%02X");
	state_add(HCD62121_SS,    "SS",    m_sseg ).callimport().callexport().formatstr("%02X");
	state_add(HCD62121_DSIZE, "DSIZE", m_dsize).callimport().callexport().formatstr("%02X");
	state_add(HCD62121_F,     "F",     m_f    ).callimport().callexport().formatstr("%02X");

	state_add(HCD62121_R00, "R00", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R04, "R04", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R08, "R08", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R0C, "R0C", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R10, "R10", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R14, "R14", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R18, "R18", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R1C, "R1C", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R20, "R20", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R24, "R24", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R28, "R28", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R2C, "R2C", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R30, "R30", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R34, "R34", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R38, "R38", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R3C, "R3C", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R40, "R40", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R44, "R44", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R48, "R48", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R4C, "R4C", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R50, "R50", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R54, "R54", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R58, "R58", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R5C, "R5C", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R60, "R60", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R64, "R64", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R68, "R68", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R6C, "R6C", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R70, "R70", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R74, "R74", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R78, "R78", m_reg[0x00]).callimport().callexport().formatstr("%8s");
	state_add(HCD62121_R7C, "R7C", m_reg[0x00]).callimport().callexport().formatstr("%8s");

	m_icountptr = &m_icount;
}


void hcd62121_cpu_device::state_export(const device_state_entry &entry)
{
	switch (entry.index())
	{
		case STATE_GENPC:
		case STATE_GENPCBASE:
			m_rtemp = (m_cseg << 16) | m_ip;
			break;
	}
}


void hcd62121_cpu_device::state_string_export(const device_state_entry &entry, std::string &str) const
{
	switch (entry.index())
	{
		case STATE_GENPC:
		case STATE_GENPCBASE:
			str = string_format("%06X", (m_cseg << 16) | m_ip);
			break;

		case STATE_GENFLAGS:
			str = string_format("%s-%s-%s-%c-%c",
				m_f & FLAG_ZH ? "ZH":"__",
				m_f & FLAG_CL ? "CL":"__",
				m_f & FLAG_ZL ? "ZL":"__",
				m_f & FLAG_C ? 'C':'_',
				m_f & FLAG_Z ? 'Z':'_'
			);

			break;

		case HCD62121_R00:
			str = string_format("%02X%02X%02X%02X", m_reg[0x00], m_reg[0x01], m_reg[0x02], m_reg[0x03]);
			break;
		case HCD62121_R04:
			str = string_format("%02X%02X%02X%02X", m_reg[0x04], m_reg[0x05], m_reg[0x06], m_reg[0x07]);
			break;
		case HCD62121_R08:
			str = string_format("%02X%02X%02X%02X", m_reg[0x08], m_reg[0x09], m_reg[0x0A], m_reg[0x0B]);
			break;
		case HCD62121_R0C:
			str = string_format("%02X%02X%02X%02X", m_reg[0x0C], m_reg[0x0D], m_reg[0x0E], m_reg[0x0F]);
			break;
		case HCD62121_R10:
			str = string_format("%02X%02X%02X%02X", m_reg[0x10], m_reg[0x11], m_reg[0x12], m_reg[0x13]);
			break;
		case HCD62121_R14:
			str = string_format("%02X%02X%02X%02X", m_reg[0x14], m_reg[0x15], m_reg[0x16], m_reg[0x17]);
			break;
		case HCD62121_R18:
			str = string_format("%02X%02X%02X%02X", m_reg[0x18], m_reg[0x19], m_reg[0x1A], m_reg[0x1B]);
			break;
		case HCD62121_R1C:
			str = string_format("%02X%02X%02X%02X", m_reg[0x1C], m_reg[0x1D], m_reg[0x1E], m_reg[0x1F]);
			break;
		case HCD62121_R20:
			str = string_format("%02X%02X%02X%02X", m_reg[0x20], m_reg[0x21], m_reg[0x22], m_reg[0x23]);
			break;
		case HCD62121_R24:
			str = string_format("%02X%02X%02X%02X", m_reg[0x24], m_reg[0x25], m_reg[0x26], m_reg[0x27]);
			break;
		case HCD62121_R28:
			str = string_format("%02X%02X%02X%02X", m_reg[0x28], m_reg[0x29], m_reg[0x2A], m_reg[0x2B]);
			break;
		case HCD62121_R2C:
			str = string_format("%02X%02X%02X%02X", m_reg[0x2C], m_reg[0x2D], m_reg[0x2E], m_reg[0x2F]);
			break;
		case HCD62121_R30:
			str = string_format("%02X%02X%02X%02X", m_reg[0x30], m_reg[0x31], m_reg[0x32], m_reg[0x33]);
			break;
		case HCD62121_R34:
			str = string_format("%02X%02X%02X%02X", m_reg[0x34], m_reg[0x35], m_reg[0x36], m_reg[0x37]);
			break;
		case HCD62121_R38:
			str = string_format("%02X%02X%02X%02X", m_reg[0x38], m_reg[0x39], m_reg[0x3A], m_reg[0x3B]);
			break;
		case HCD62121_R3C:
			str = string_format("%02X%02X%02X%02X", m_reg[0x3C], m_reg[0x3D], m_reg[0x3E], m_reg[0x3F]);
			break;
		case HCD62121_R40:
			str = string_format("%02X%02X%02X%02X", m_reg[0x40], m_reg[0x41], m_reg[0x42], m_reg[0x43]);
			break;
		case HCD62121_R44:
			str = string_format("%02X%02X%02X%02X", m_reg[0x44], m_reg[0x45], m_reg[0x46], m_reg[0x47]);
			break;
		case HCD62121_R48:
			str = string_format("%02X%02X%02X%02X", m_reg[0x48], m_reg[0x49], m_reg[0x4A], m_reg[0x4B]);
			break;
		case HCD62121_R4C:
			str = string_format("%02X%02X%02X%02X", m_reg[0x4C], m_reg[0x4D], m_reg[0x4E], m_reg[0x4F]);
			break;
		case HCD62121_R50:
			str = string_format("%02X%02X%02X%02X", m_reg[0x50], m_reg[0x51], m_reg[0x52], m_reg[0x53]);
			break;
		case HCD62121_R54:
			str = string_format("%02X%02X%02X%02X", m_reg[0x54], m_reg[0x55], m_reg[0x56], m_reg[0x57]);
			break;
		case HCD62121_R58:
			str = string_format("%02X%02X%02X%02X", m_reg[0x58], m_reg[0x59], m_reg[0x5A], m_reg[0x5B]);
			break;
		case HCD62121_R5C:
			str = string_format("%02X%02X%02X%02X", m_reg[0x5C], m_reg[0x5D], m_reg[0x5E], m_reg[0x5F]);
			break;
		case HCD62121_R60:
			str = string_format("%02X%02X%02X%02X", m_reg[0x60], m_reg[0x61], m_reg[0x62], m_reg[0x63]);
			break;
		case HCD62121_R64:
			str = string_format("%02X%02X%02X%02X", m_reg[0x64], m_reg[0x65], m_reg[0x66], m_reg[0x67]);
			break;
		case HCD62121_R68:
			str = string_format("%02X%02X%02X%02X", m_reg[0x68], m_reg[0x69], m_reg[0x6A], m_reg[0x6B]);
			break;
		case HCD62121_R6C:
			str = string_format("%02X%02X%02X%02X", m_reg[0x6C], m_reg[0x6D], m_reg[0x6E], m_reg[0x6F]);
			break;
		case HCD62121_R70:
			str = string_format("%02X%02X%02X%02X", m_reg[0x70], m_reg[0x71], m_reg[0x72], m_reg[0x73]);
			break;
		case HCD62121_R74:
			str = string_format("%02X%02X%02X%02X", m_reg[0x74], m_reg[0x75], m_reg[0x76], m_reg[0x77]);
			break;
		case HCD62121_R78:
			str = string_format("%02X%02X%02X%02X", m_reg[0x78], m_reg[0x79], m_reg[0x7A], m_reg[0x7B]);
			break;
		case HCD62121_R7C:
			str = string_format("%02X%02X%02X%02X", m_reg[0x7C], m_reg[0x7D], m_reg[0x7E], m_reg[0x7F]);
			break;
	}
}


void hcd62121_cpu_device::device_reset()
{
	m_sp = 0x0000;
	m_ip = 0x0000;
	m_cseg = 0;
	m_dseg = 0;
	m_sseg = 0;
	m_lar = 0;
	m_f = 0;
	m_dsize = 0;
	m_opt = 0;
	m_port = 0;

	for (auto & elem : m_reg)
	{
		elem = 0;
	}
}


inline void hcd62121_cpu_device::set_zero_flag(bool is_zero)
{
	if (is_zero)
		m_f |= FLAG_Z;
	else
		m_f &= ~FLAG_Z;
}


inline void hcd62121_cpu_device::set_carry_flag(bool is_carry)
{
	if (is_carry)
		m_f |= FLAG_C;
	else
		m_f &= ~FLAG_C;
}


inline void hcd62121_cpu_device::set_zl_flag(bool is_zl)
{
	if (is_zl)
		m_f |= FLAG_ZL;
	else
		m_f &= ~FLAG_ZL;
}


inline void hcd62121_cpu_device::set_zh_flag(bool is_zh)
{
	if (is_zh)
		m_f |= FLAG_ZH;
	else
		m_f &= ~FLAG_ZH;
}


inline void hcd62121_cpu_device::set_cl_flag(bool is_cl)
{
	if (is_cl)
		m_f |= FLAG_CL;
	else
		m_f &= ~FLAG_CL;
}


inline void hcd62121_cpu_device::op_msk(int size)
{
	bool mskres = true;

	for (int i = 0; i < size; i++)
	{
		if ((m_temp1[i] & m_temp2[i]) != m_temp2[i])
			mskres = false;
	}

	set_zero_flag(!mskres);
}


inline void hcd62121_cpu_device::op_imsk(int size)
{
	bool mskres = true;
	bool set_zero = false;

	for (int i = 0; i < size; i++)
	{
		if ((m_temp1[i] & ~m_temp2[i]) != ~m_temp2[i])
			mskres = false;
		if (m_temp1[i] | m_temp2[i])
			set_zero = true;
	}

	set_zero_flag(set_zero);
	set_carry_flag(!mskres);
}


inline void hcd62121_cpu_device::op_and(int size)
{
	bool is_zero = true;

	for (int i = 0; i < size; i++)
	{
		m_temp1[i] = m_temp1[i] & m_temp2[i];
		if (m_temp1[i])
			is_zero = false;
	}

	set_zero_flag(is_zero);
	set_zl_flag((m_temp1[0] & 0x0f) == 0);
	set_zh_flag((m_temp1[0] & 0xf0) == 0);
}


inline void hcd62121_cpu_device::op_or(int size)
{
	bool is_zero = true;

	for (int i = 0; i < size; i++)
	{
		m_temp1[i] = m_temp1[i] | m_temp2[i];
		if (m_temp1[i])
			is_zero = false;
	}

	set_zero_flag(is_zero);
	set_zl_flag((m_temp1[0] & 0x0f) == 0);
	set_zh_flag((m_temp1[0] & 0xf0) == 0);
}


inline void hcd62121_cpu_device::op_xor(int size)
{
	bool is_zero = true;

	for (int i = 0; i < size; i++)
	{
		m_temp1[i] = m_temp1[i] ^ m_temp2[i];
		if (m_temp1[i])
			is_zero = false;
	}

	set_zero_flag(is_zero);
	set_zl_flag((m_temp1[0] & 0x0f) == 0);
	set_zh_flag((m_temp1[0] & 0xf0) == 0);
}


inline void hcd62121_cpu_device::op_add(int size)
{
	bool is_zero = true;
	u8 carry = 0;

	set_cl_flag((m_temp1[0] & 0x0f) + (m_temp2[0] & 0x0f) > 15);

	for (int i = 0; i < size; i++)
	{
		u16 res = m_temp1[i] + m_temp2[i] + carry;

		m_temp1[i] = res & 0xff;
		if (m_temp1[i])
			is_zero = false;

		carry = (res & 0xff00) ? 1 : 0;
	}

	set_zero_flag(is_zero);
	set_carry_flag(carry);
	set_zl_flag((m_temp1[0] & 0x0f) == 0);
	set_zh_flag((m_temp1[0] & 0xf0) == 0);
}


// BCD ADD
inline void hcd62121_cpu_device::op_addb(int size)
{
	bool is_zero = true;
	u8 carry = 0;

	set_cl_flag((m_temp1[0] & 0x0f) + (m_temp2[0] & 0x0f) > 9);

	for (int i = 0; i < size; i++)
	{
		u16 res = (m_temp1[i] & 0x0f) + (m_temp2[i] & 0x0f) + carry;

		if (res > 9)
		{
			res += 6;
		}
		res += (m_temp1[i] & 0xf0) + (m_temp2[i] & 0xf0);
		if (res > 0x9f)
		{
			res += 0x60;
		}
		m_temp1[i] = res & 0xff;
		if (m_temp1[i])
			is_zero = false;

		carry = (res & 0xff00) ? 1 : 0;
	}

	set_zero_flag(is_zero);
	set_carry_flag(carry);
	set_zl_flag((m_temp1[0] & 0x0f) == 0);
	set_zh_flag((m_temp1[0] & 0xf0) == 0);
}


inline void hcd62121_cpu_device::op_sub(int size)
{
	bool is_zero = true;
	u8 carry = 0;

	set_cl_flag((m_temp1[0] & 0x0f) < (m_temp2[0] & 0x0f));

	for (int i = 0; i < size; i++)
	{
		u16 res = m_temp1[i] - m_temp2[i] - carry;

		m_temp1[i] = res & 0xff;
		if (m_temp1[i])
			is_zero = false;

		carry = ( res & 0xff00 ) ? 1 : 0;
	}

	set_zero_flag(is_zero);
	set_carry_flag(carry);
	set_zl_flag((m_temp1[0] & 0x0f) == 0);
	set_zh_flag((m_temp1[0] & 0xf0) == 0);
}


inline void hcd62121_cpu_device::op_pushw(u16 source)
{
	m_program->write_byte(( m_sseg << 16) | m_sp, source & 0xff);
	m_sp--;
	m_program->write_byte(( m_sseg << 16) | m_sp, source >> 8);
	m_sp--;
}


inline u16 hcd62121_cpu_device::op_popw()
{
	m_sp++;
	u16 res = m_program->read_byte((m_sseg << 16) | m_sp) << 8;
	m_sp++;
	res |= m_program->read_byte((m_sseg << 16) | m_sp);

	return res;
}


void hcd62121_cpu_device::execute_run()
{
	do
	{
		offs_t pc = (m_cseg << 16) | m_ip;

		debugger_instruction_hook(this, pc);
		m_prev_pc = pc;

		u8 op = read_op();

		// actual instruction timings unknown
		m_icount -= 4;

		switch (op)
		{
		case 0x00:      /* rorb/rolb r1,4 */
		case 0x01:      /* rorw/rolw r1,4 */
		case 0x02:      /* rorq/rolq r1,4 */
		case 0x03:      /* rort/rolt r1,4 */
			/* Nibble rotate */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 d1 = 0, d2 = 0;

				read_reg(size, reg1);

				if (reg1 & 0x80)
				{
					// rotate right
					d2 = (m_temp1[size-1] & 0x0f) << 4;
					for (int i = 0; i < size; i++)
					{
						d1 = (m_temp1[i] & 0x0f) << 4;
						m_temp1[i] = (m_temp1[i] >> 4) | d2;
						d2 = d1;
					}
				}
				else
				{
					// rotate left
					d2 = (m_temp1[size-1] & 0xf0) >> 4;
					for (int i = 0; i < size; i++)
					{
						d1 = (m_temp1[i] & 0xf0) >> 4;
						m_temp1[i] = (m_temp1[i] << 4) | d2;
						d2 = d1;
					}
				}

				write_reg(size, reg1);
			}
			break;

		case 0x04:      /* mskb r1,r2 */
		case 0x05:      /* mskw r1,r2 */
		case 0x06:      /* mskq r1,r2 */
		case 0x07:      /* mskt r1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_regreg(size, reg1, reg2, false);

				op_msk(size);
			}
			break;

		case 0x08:      /* shrb/shlb r1,4 */
		case 0x09:      /* shrw/shlw r1,4 */
		case 0x0A:      /* shrq/shlq r1,4 */
		case 0x0B:      /* shrt/shlt r1,4 */
			/* Nibble shift */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 d1 = 0, d2 = 0;

				read_reg(size, reg1);

				if (reg1 & 0x80)
				{
					// shift right
					for (int i = 0; i < size; i++)
					{
						d1 = (m_temp1[i] & 0x0f) << 4;
						m_temp1[i] = (m_temp1[i] >> 4) | d2;
						d2 = d1;
					}
				}
				else
				{
					// shift left
					for (int i = 0; i < size; i++)
					{
						d1 = (m_temp1[i] & 0xf0) >> 4;
						m_temp1[i] = (m_temp1[i] << 4) | d2;
						d2 = d1;
					}
				}

				write_reg(size, reg1);
			}
			break;

		case 0x0C:      /* testb r1,r2 */
		case 0x0D:      /* testw r1,r2 */
		case 0x0E:      /* testq r1,r2 */
		case 0x0F:      /* testt r1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_regreg(size, reg1, reg2, false);

				op_and(size);
			}
			break;

		case 0x10:      /* xorb r1,r2 */
		case 0x11:      /* xorw r1,r2 */
		case 0x12:      /* xorq r1,r2 */
		case 0x13:      /* xort r1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_regreg(size, reg1, reg2, false);

				op_xor(size);

				write_regreg(size, reg1, reg2);
			}
			break;

		case 0x14:      /* cmpb r1,r2 */
		case 0x15:      /* cmpw r1,r2 */
		case 0x16:      /* cmpq r1,r2 */
		case 0x17:      /* cmpt r1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_regreg(size, reg1, reg2, false);

				op_sub(size);
			}
			break;

		case 0x18:      /* movb r1,r2 */
		case 0x19:      /* movw r1,r2 */
		case 0x1A:      /* movq r1,r2 */
		case 0x1B:      /* movt r1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_regreg(size, reg1, reg2, false);

				for (int i = 0; i < size; i++)
					m_temp1[i] = m_temp2[i];

				write_regreg(size, reg1, reg2);
			}
			break;

		case 0x1C:      /* imskb r1,r2 */
		case 0x1D:      /* imskw r1,r2 */
		case 0x1E:      /* imskq r1,r2 */
		case 0x1F:      /* imskt r1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_regreg(size, reg1, reg2, false);

				op_imsk(size);
			}
			break;

		case 0x20:      /* rorb/rolb r1 */
		case 0x21:      /* rorw/rolw r1 */
		case 0x22:      /* rorq/rolq r1 */
		case 0x23:      /* rort/rolt r1 */
			/* Single bit rotate */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 d1 = 0, d2 = 0;

				read_reg(size, reg1);

				if (reg1 & 0x80)
				{
					// rotate right
					d2 = (m_temp1[size-1] & 0x01) << 7;
					for (int i = 0; i < size; i++)
					{
						d1 = (m_temp1[i] & 0x01) << 7;
						m_temp1[i] = (m_temp1[i] >> 1) | d2;
						d2 = d1;
					}
				}
				else
				{
					// rotate left
					d2 = (m_temp1[size-1] & 0x80) >> 7;
					for (int i = 0; i < size; i++)
					{
						d1 = (m_temp1[i] & 0x80) >> 7;
						m_temp1[i] = (m_temp1[i] << 1) | d2;
						d2 = d1;
					}
				}

				write_reg(size, reg1);
			}
			break;

		case 0x24:      /* orb r1,r2 */
		case 0x25:      /* orw r1,r2 */
		case 0x26:      /* orq r1,r2 */
		case 0x27:      /* ort r1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_regreg(size, reg1, reg2, false);

				op_or(size);

				write_regreg(size, reg1, reg2);
			}
			break;

		case 0x28:      /* shrb/shlb r1 */
		case 0x29:      /* shrw/shlw r1 */
		case 0x2A:      /* shrq/shlq r1 */
		case 0x2B:      /* shrt/shlt r1 */
			/* Single bit shift */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 d1 = 0, d2 = 0;

				read_reg(size, reg1);

				if (reg1 & 0x80)
				{
					// shift right
					for (int i = 0; i < size; i++)
					{
						d1 = (m_temp1[i] & 0x01) << 7;
						m_temp1[i] = (m_temp1[i] >> 1) | d2;
						d2 = d1;
					}
				}
				else
				{
					// shift left
					for (int i = 0; i < size; i++)
					{
						d1 = (m_temp1[i] & 0x80) >> 7;
						m_temp1[i] = (m_temp1[i] << 1) | d2;
						d2 = d1;
					}
				}

				write_reg(size, reg1);
			}
			break;

		case 0x2C:      /* andb r1,r2 */
		case 0x2D:      /* andw r1,r2 */
		case 0x2E:      /* andq r1,r2 */
		case 0x2F:      /* andt r1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_regreg(size, reg1, reg2, true);

				op_and(size);

				write_regreg(size, reg1, reg2);
			}
			break;

		case 0x34:      /* subb r1,r2 */
		case 0x35:      /* subw r1,r2 */
		case 0x36:      /* subq r1,r2 */
		case 0x37:      /* subt r1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_regreg(size, reg1, reg2, false);

				op_sub(size);

				write_regreg(size, reg1, reg2);
			}
			break;

		case 0x38:      /* adbb r1,r2 */
		case 0x39:      /* adbw r1,r2 */
		case 0x3A:      /* adbq r1,r2 */
		case 0x3B:      /* adbt r1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_regreg(size, reg1, reg2, false);

				op_addb(size);

				write_regreg(size, reg1, reg2);
			}
			break;

		case 0x3C:      /* addb r1,r2 */
		case 0x3D:      /* addw r1,r2 */
		case 0x3E:      /* addq r1,r2 */
		case 0x3F:      /* addt r1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_regreg(size, reg1, reg2, false);

				op_add(size);

				write_regreg(size, reg1, reg2);
			}
			break;

		case 0x4C:      /* testb ir1,r2 */
		case 0x4D:      /* testw ir1,r2 */
		case 0x4E:      /* testq ir1,r2 */
		case 0x4F:      /* testt ir1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_iregreg(size, reg1, reg2);

				op_and(size);
			}
			break;

		case 0x54:      /* cmpb ir1,r2 */
		case 0x55:      /* cmpw ir1,r2 */
		case 0x56:      /* cmpq ir1,r2 */
		case 0x57:      /* cmpt ir1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_iregreg(size, reg1, reg2);

				op_sub(size);
			}
			break;

		case 0x58:      /* movb ir1,r2 */
		case 0x59:      /* movw ir1,r2 */
		case 0x5A:      /* movq ir1,r2 */
		case 0x5B:      /* movt ir1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_iregreg(size, reg1, reg2);

				for (int i = 0; i < size; i++)
					m_temp1[i] = m_temp2[i];

				write_iregreg(size, reg1, reg2);
			}
			break;

		case 0x64:      /* orb ir1,r2 */
		case 0x65:      /* orb ir1,r2 */
		case 0x66:      /* orb ir1,r2 */
		case 0x67:      /* orb ir1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_iregreg(size, reg1, reg2);

				op_or(size);

				write_iregreg(size, reg1, reg2);
			}
			break;

		case 0x6C:      /* andb ir1,r2 */
		case 0x6D:      /* andw ir1,r2 */
		case 0x6E:      /* andq ir1,r2 */
		case 0x6F:      /* andt ir1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_iregreg(size, reg1, reg2);

				op_and(size);

				write_iregreg(size, reg1, reg2);
			}
			break;

		case 0x7C:      /* addb ir1,r2 */
		case 0x7D:      /* addw ir1,r2 */
		case 0x7E:      /* addq ir1,r2 */
		case 0x7F:      /* addt ir1,r2 */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_iregreg(size, reg1, reg2);

				op_add(size);

				write_iregreg(size, reg1, reg2);
			}
			break;

		case 0x88:      /* jump _a16 */
			m_ip = (read_op() << 8) | read_op();
			break;

		case 0x89:      /* jumpf cs:a16 */
			{
				u8 cs = read_op();
				u8 a1 = read_op();
				u8 a2 = read_op();

				m_cseg = cs;
				m_ip = (a1 << 8) | a2;
			}
			break;

		case 0x8A:      /* call a16 */
			{
				u8 a1 = read_op();
				u8 a2 = read_op();

				op_pushw(m_ip);

				m_ip = (a1 << 8) | a2;
			}
			break;

		case 0x8C:      /* unk_8C */
		case 0x8D:      /* unk_8D */
		case 0x8E:      /* unk_8E */
			logerror("%02x:%04x: unimplemented instruction %02x encountered\n", m_cseg, m_ip-1, op);
			break;

		case 0x90:      /* retzh */
		case 0x91:      /* retzl */
		case 0x92:      /* retc */
		case 0x93:      /* retz */
		case 0x94:      /* retzc */
		case 0x95:      /* retcl */
		case 0x96:      /* retnc */
		case 0x97:      /* retnz */
			if (check_cond(op))
				m_ip = op_popw();
			break;

		case 0x98:      /* jump (r1) */
			{
				u8 reg1 = read_op();
				u16 ad = m_reg[(reg1 | 0x40) & 0x7f] << 8;

				if (reg1 & 0x40)
					ad |= m_reg[((reg1 - 1) | 0x40) & 0x7f];
				else
					ad |= m_reg[((reg1 + 1) | 0x40) & 0x7f];

				m_ip = ad;
			}
			break;

		case 0x9F:      /* ret */
			m_ip = op_popw();
			break;

		case 0xA0:      /* jmpzh a16 */
		case 0xA1:      /* jmpzl a16 */
		case 0xA2:      /* jmpc a16 */
		case 0xA3:      /* jmpz a16 */
		case 0xA4:      /* jmpzc a16 */
		case 0xA5:      /* jmpcl a16 */
		case 0xA6:      /* jmpnc a16 */
		case 0xA7:      /* jmpnz a16 */
			{
				u8 a1 = read_op();
				u8 a2 = read_op();

				if (check_cond(op))
					m_ip = (a1 << 8) | a2;
			}
			break;

		case 0xA8:      /* callzh a16 */
		case 0xA9:      /* callzl a16 */
		case 0xAA:      /* callc a16 */
		case 0xAB:      /* callz a16 */
		case 0xAC:      /* callzc a16 */
		case 0xAD:      /* callcl a16 */
		case 0xAE:      /* callnc a16 */
		case 0xAF:      /* callnz a16 */
			{
				u8 a1 = read_op();
				u8 a2 = read_op();

				if (check_cond(op))
				{
					op_pushw(m_ip);

					m_ip = (a1 << 8) | a2;
				}
			}
			break;

		case 0xB1:      /* unk_B1 reg/i8 - PORTx control/direction? */
		case 0xB3:      /* unk_B3 reg/i8 - timer/irq related? */
			logerror("%02x:%04x: unimplemented instruction %02x encountered\n", m_cseg, m_ip-1, op);
			read_op();
			break;

		case 0xB4:      /* out koh,reg */
			m_koh_cb(m_reg[read_op() & 0x7f]);
			break;

		case 0xB5:      /* out koh,i8 */
			m_koh_cb(read_op());
			break;

		case 0xB6:      /* out kol,reg */
			m_kol_cb(m_reg[read_op() & 0x7f]);
			break;

		case 0xB7:      /* out kol,i8 */
			m_kol_cb(read_op());
			break;

		case 0xB9:      /* unk_B9 reg/i8 - timer/irq related? */
			logerror("%02x:%04x: unimplemented instruction %02x encountered\n", m_cseg, m_ip-1, op);
			read_op();
			break;

		case 0xBB:      /* jmpcl? a16 */
			logerror("%02x:%04x: unimplemented instruction %02x encountered\n", m_cseg, m_ip-1, op);
			{
				u8 a1 = read_op();
				u8 a2 = read_op();

				if (m_f & FLAG_CL)
					m_ip = ( a1 << 8) | a2;
			}
			break;

		case 0xBC:      /* unk_BC reg/i8 */
			logerror("%02x:%04x: unimplemented instruction %02x encountered\n", m_cseg, m_ip-1, op);
			read_op();
			break;

		case 0xBF:      /* jmpncl? a16 */
			logerror("%02x:%04x: unimplemented instruction %02x encountered\n", m_cseg, m_ip-1, op);
			{
				u8 a1 = read_op();
				u8 a2 = read_op();

				if (!(m_f & FLAG_CL))
					m_ip = (a1 << 8) | a2;
			}
			break;

		case 0xC0:      /* movb reg,i8 */
		case 0xC1:      /* movw reg,i16 */
		case 0xC2:      /* movw reg,i64 */
		case 0xC3:      /* movw reg,i80 */
			{
				int size = datasize(op);
				u8 reg = read_op();

				for (int i = 0; i < size; i++)
				{
					m_reg[(reg + i) & 0x7f] = read_op();
				}
			}
			break;

		case 0xC4:      /* movb (lar),r1 / r1,(lar) */
		case 0xC5:      /* movw (lar),r1 / r1,(lar) */
		case 0xC6:      /* movq (lar),r1 / r1,(lar) */
		case 0xC7:      /* movt (lar),r1 / r1,(lar) */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();
				int pre_inc = 0;
				int post_inc = 1;

				switch (reg1 & 0x60)
				{
				case 0x00:
					pre_inc = 0;
					post_inc = 1;
					break;
				case 0x20:
					pre_inc = 1;
					post_inc = 0;
					break;
				case 0x40:
					pre_inc = 0;
					post_inc = -1;
					break;
				case 0x60:
					pre_inc = -1;
					post_inc = 0;
					break;
				}

				if ((reg1 & 0x80) || (reg2 & 0x80))
				{
					/* (lar) <- r1 */
					for (int i = 0; i < size; i++)
					{
						m_lar += pre_inc;
						m_program->write_byte((m_dseg << 16) | m_lar, m_reg[(reg2 + i) & 0x7f]);
						m_lar += post_inc;
					}
				}
				else
				{
					/* r1 <- (lar) */
					for (int i = 0; i < size; i++)
					{
						m_lar += pre_inc;
						m_reg[(reg2 + i) & 0x7f] = m_program->read_byte((m_dseg << 16) | m_lar);
						m_lar += post_inc;
					}
				}
			}
			break;

		case 0xCC:      /* swapb ir1,r2 */
		case 0xCD:      /* swapw ir1,r2 */
		case 0xCE:      /* swapq ir1,r2 */
		case 0xCF:      /* swapt ir1,r2? */
			{
				int size = datasize(op);
				u8 reg1 = read_op();
				u8 reg2 = read_op();

				read_iregreg(size, reg1, reg2);

				for (int i = 0; i < size; i++)
				{
					u8 d = m_temp1[i];
					m_temp1[i] = m_temp2[i];
					m_temp2[i] = d;
				}

				write_iregreg(size, reg1, reg2);
				write_iregreg2(size, reg1, reg2);
			}
			break;

		case 0xD0:      /* movb cs,reg */
			m_cseg = m_reg[read_op() & 0x7f];
			break;

		case 0xD1:      /* movb cs,i8 */
			m_cseg = read_op();
			break;

		case 0xD2:      /* movb dsize,reg */
			m_dsize = m_reg[read_op() & 0x7f];
			break;

		case 0xD3:      /* movb dsize,i8 */
			m_dsize = read_op();
			break;

		case 0xD4:      /* movb ss,reg */
			m_sseg = m_reg[read_op() & 0x7f];
			break;

		case 0xD5:      /* movb ss,i8 */
			m_sseg = read_op();
			break;

		case 0xD6:      /* movw sp,reg */
			{
				u8 reg1 = read_op();

				m_sp = m_reg[reg1 & 0x7f] | (m_reg[(reg1 + 1) & 0x7f] << 8);
			}
			break;

		case 0xD7:      /* movw sp,i16 */
			m_sp = read_op() << 8;
			m_sp |= read_op();
			break;

		case 0xD8:      /* movb f,reg */
			m_f = m_reg[read_op() & 0x7f];
			break;

		case 0xD9:      /* movb f,i8 */
			m_f = read_op();
			break;

		case 0xDC:      /* movb ds,reg */
			m_dseg = m_reg[read_op() & 0x7f];
			break;

		case 0xDD:      /* movb ds,i8 */
			m_dseg = read_op();
			break;

		case 0xDE:      /* movw lar,reg */
			{
				u8 reg1 = read_op();

				m_lar = m_reg[reg1 & 0x7f] | (m_reg[( reg1 + 1) & 0x7f] << 8);
			}
			break;

		case 0xE0:      /* in0 reg */
			{
				logerror("%06x: in0 read\n", (m_cseg << 16) | m_ip);
				u8 reg1 = read_op();

				m_reg[reg1 & 0x7f] = m_in0_cb();
			}
			break;

		case 0xE1:      /* movb reg,OPT */
			m_reg[read_op() & 0x7f] = m_opt;
			break;

		case 0xE2:      /* in kb, reg */
			m_reg[read_op() & 0x7f] = m_ki_cb();
			break;

		case 0xE6:      /* movb reg,PORT */
			m_reg[read_op() & 0x7f] = m_port;
			break;

		case 0xE3:      /* unk_e3 reg/i8 (in?) */
		case 0xE4:      /* unk_e4 reg/i8 (in?) */
		case 0xE5:      /* unk_e5 reg/i8 (in?) */
		case 0xE7:      /* unk_e7 reg/i8 (in?) */
			logerror("%02x:%04x: unimplemented instruction %02x encountered\n", m_cseg, m_ip-1, op);
			read_op();
			break;

		case 0xE8:      /* movw r1,lar */
			{
				u8 reg1 = read_op();

				m_reg[reg1 & 0x7f] = m_lar & 0xff;
				m_reg[(reg1 + 1) & 0x7f] = m_lar >> 8;
			}
			break;

		case 0xEB:      /* movw reg,ss */
			{
				u8 reg1 = read_op();

				m_reg[reg1 & 0x7f] = m_sp & 0xff;
				m_reg[(reg1 + 1) & 0x7f] = m_sp >> 8;
			}
			break;

		case 0xEF:      /* movb reg,ss */
			m_reg[read_op() & 0x7f] = m_sseg;
			break;

		case 0xF0:      /* movb OPT,reg */
			m_opt = m_reg[read_op() & 0x7f];
			m_opt_cb(m_opt);
			break;

		case 0xF2:      /* movb PORT,reg */
			m_port = m_reg[read_op() & 0x7f];
			m_port_cb(m_port);
			break;

		case 0xF1:      /* unk_F1 reg/i8 (out?) */
		case 0xF3:      /* unk_F3 reg/i8 (out?) */
		case 0xF4:      /* unk_F4 reg/i8 (out?) */
		case 0xF5:      /* unk_F5 reg/i8 (out?) */
		case 0xF6:      /* unk_F6 reg/i8 (out?) */
		case 0xF7:      /* unk_F7 reg/i8 (out?) */
			logerror("%02x:%04x: unimplemented instruction %02x encountered\n", m_cseg, m_ip-1, op);
			read_op();
			break;

		case 0xFC:      /* unk_FC - disable interrupts/stop timer?? */
		case 0xFD:      /* unk_FD */
		case 0xFE:      /* unk_FE - wait for/start timer */
			logerror("%02x:%04x: unimplemented instruction %02x encountered\n", m_cseg, m_ip-1, op);
			break;

		case 0xFF:      /* nop */
			break;

		default:
			fatalerror("%02x:%04x: unimplemented instruction %02x encountered\n", m_cseg, m_ip-1, op);
		};

	} while (m_icount > 0);
}


offs_t hcd62121_cpu_device::disasm_disassemble(std::ostream &stream, offs_t pc, const u8 *oprom, const u8 *opram, uint32_t options)
{
	extern CPU_DISASSEMBLE(hcd62121);
	return CPU_DISASSEMBLE_NAME(hcd62121)(this, stream, pc, oprom, opram, options);
}
