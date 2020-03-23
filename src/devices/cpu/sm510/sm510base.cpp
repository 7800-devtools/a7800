// license:BSD-3-Clause
// copyright-holders:hap
/*

  Sharp SM5xx family, using SM510 as parent device

  References:
  - 1986 Sharp Semiconductor Data Book
  - 1990 Sharp Microcomputers Data Book
  - 1996 Sharp Microcomputer Databook
  - KB1013VK1-2/KB1013VK4-2 manual

  TODO:
  - source organiziation between files is a mess
  - proper support for LFSR program counter in debugger
  - LCD bs pin blink mode via Y register (0.5s off, 0.5s on)
  - wake up after CEND doesn't work right

  for more, see the *core.cpp file notes

*/

#include "emu.h"
#include "sm510.h"
#include "debugger.h"


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

enum
{
	SM510_PC=1, SM510_ACC, SM510_X, SM510_BL, SM510_BM,
	SM510_C, SM510_W
};

void sm510_base_device::device_start()
{
	m_program = &space(AS_PROGRAM);
	m_data = &space(AS_DATA);
	m_prgmask = (1 << m_prgwidth) - 1;
	m_datamask = (1 << m_datawidth) - 1;

	// resolve callbacks
	m_read_k.resolve_safe(0);
	m_read_ba.resolve_safe(1);
	m_read_b.resolve_safe(1);
	m_write_s.resolve_safe();
	m_write_r.resolve_safe();

	m_write_sega.resolve_safe();
	m_write_segb.resolve_safe();
	m_write_segbs.resolve_safe();
	m_write_segc.resolve_safe();

	// init/zerofill
	memset(m_stack, 0, sizeof(m_stack));
	m_pc = 0;
	m_prev_pc = 0;
	m_op = 0;
	m_prev_op = 0;
	m_param = 0;
	m_acc = 0;
	m_bl = 0;
	m_bm = 0;
	m_sbm = false;
	m_c = 0;
	m_skip = false;
	m_w = 0;
	m_r = 0;
	m_r_out = 0;
	m_div = 0;
	m_1s = false;
	m_k_active = false;
	m_l = 0;
	m_x = 0;
	m_y = 0;
	m_bp = false;
	m_bc = false;
	m_halt = false;
	m_melody_rd = 0;
	m_melody_step_count = 0;
	m_melody_duty_count = 0;
	m_melody_duty_index = 0;
	m_melody_address = 0;
	m_clk_div = 2; // 16kHz

	// register for savestates
	save_item(NAME(m_stack));
	save_item(NAME(m_pc));
	save_item(NAME(m_prev_pc));
	save_item(NAME(m_op));
	save_item(NAME(m_prev_op));
	save_item(NAME(m_param));
	save_item(NAME(m_acc));
	save_item(NAME(m_bl));
	save_item(NAME(m_bm));
	save_item(NAME(m_sbm));
	save_item(NAME(m_c));
	save_item(NAME(m_skip));
	save_item(NAME(m_w));
	save_item(NAME(m_r));
	save_item(NAME(m_r_out));
	save_item(NAME(m_div));
	save_item(NAME(m_1s));
	save_item(NAME(m_k_active));
	save_item(NAME(m_l));
	save_item(NAME(m_x));
	save_item(NAME(m_y));
	save_item(NAME(m_bp));
	save_item(NAME(m_bc));
	save_item(NAME(m_halt));
	save_item(NAME(m_melody_rd));
	save_item(NAME(m_melody_step_count));
	save_item(NAME(m_melody_duty_count));
	save_item(NAME(m_melody_duty_index));
	save_item(NAME(m_melody_address));
	save_item(NAME(m_clk_div));

	// register state for debugger
	state_add(SM510_PC,  "PC",  m_pc).formatstr("%04X");
	state_add(SM510_ACC, "ACC", m_acc).formatstr("%01X");
	state_add(SM510_X,   "X",   m_x).formatstr("%01X");
	state_add(SM510_BL,  "BL",  m_bl).formatstr("%01X");
	state_add(SM510_BM,  "BM",  m_bm).formatstr("%01X");
	state_add(SM510_C,   "C",   m_c).formatstr("%01X");
	state_add(SM510_W,   "W",   m_w).formatstr("%02X");

	state_add(STATE_GENPC, "GENPC", m_pc).formatstr("%04X").noshow();
	state_add(STATE_GENPCBASE, "CURPC", m_pc).formatstr("%04X").noshow();
	state_add(STATE_GENFLAGS, "GENFLAGS", m_c).formatstr("%1s").noshow();

	m_icountptr = &m_icount;

	// init peripherals
	init_divider();
	init_lcd_driver();
	init_melody();
}



//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void sm510_base_device::device_reset()
{
	// ACL
	m_skip = false;
	m_halt = false;
	m_sbm = false;
	m_op = m_prev_op = 0;
	reset_vector();
	m_prev_pc = m_pc;

	// lcd is on (Bp on, BC off, bs(y) off)
	m_bp = true;
	m_bc = false;
	m_y = 0;

	m_r = m_r_out = 0;
	m_write_r(0, 0, 0xff);
}



//-------------------------------------------------
//  lcd driver
//-------------------------------------------------

inline u16 sm510_base_device::get_lcd_row(int column, u8* ram)
{
	// output 0 if lcd blackpate/bleeder is off, or in case row doesn't exist
	if (ram == nullptr || m_bc || !m_bp)
		return 0;

	u16 rowdata = 0;
	for (int i = 0; i < 0x10; i++)
		rowdata |= (ram[i] >> column & 1) << i;

	return rowdata;
}

device_memory_interface::space_config_vector sm510_base_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_program_config),
		std::make_pair(AS_DATA,    &m_data_config)
	};
}

void sm510_base_device::lcd_update()
{
	// 4 columns
	for (int h = 0; h < 4; h++)
	{
		// 16 segments per row from upper part of RAM
		m_write_sega(h | SM510_PORT_SEGA, get_lcd_row(h, m_lcd_ram_a), 0xffff);
		m_write_segb(h | SM510_PORT_SEGB, get_lcd_row(h, m_lcd_ram_b), 0xffff);
		m_write_segc(h | SM510_PORT_SEGC, get_lcd_row(h, m_lcd_ram_c), 0xffff);

		// bs output from L/X and Y regs
		u8 bs = (m_l >> h & 1) | ((m_x*2) >> h & 2);
		m_write_segbs(h | SM510_PORT_SEGBS, (m_bc || !m_bp) ? 0 : bs, 0xffff);
	}
}

TIMER_CALLBACK_MEMBER(sm510_base_device::lcd_timer_cb)
{
	lcd_update();
}

void sm510_base_device::init_lcd_driver()
{
	// note: in reality, this timer runs at high frequency off the main divider, strobing one segment at a time
	m_lcd_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(sm510_base_device::lcd_timer_cb), this));
	attotime period = attotime::from_ticks(0x200, unscaled_clock()); // 64hz default
	m_lcd_timer->adjust(period, 0, period);
}



//-------------------------------------------------
//  interrupt/divider
//-------------------------------------------------

bool sm510_base_device::wake_me_up()
{
	// in halt mode, wake up after 1S signal or K input
	if (m_k_active || m_1s)
	{
		// note: official doc warns that Bl/Bm and the stack are undefined
		// after waking up, but we leave it unchanged
		m_halt = false;
		wakeup_vector();

		standard_irq_callback(0);
		return true;
	}
	else
		return false;
}

void sm510_base_device::execute_set_input(int line, int state)
{
	if (line != SM510_INPUT_LINE_K)
		return;

	// set K input lines active state
	m_k_active = (state != 0);
}

TIMER_CALLBACK_MEMBER(sm510_base_device::div_timer_cb)
{
	m_div = (m_div + 1) & 0x7fff;

	// 1S signal on overflow(falling edge of F1)
	if (m_div == 0)
		m_1s = true;

	clock_melody();
}

void sm510_base_device::init_divider()
{
	m_div_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(sm510_base_device::div_timer_cb), this));
	m_div_timer->adjust(attotime::from_ticks(1, unscaled_clock()), 0, attotime::from_ticks(1, unscaled_clock()));
}



//-------------------------------------------------
//  execute
//-------------------------------------------------

void sm510_base_device::increment_pc()
{
	// PL(program counter low 6 bits) is a simple LFSR: newbit = (bit0==bit1)
	// PU,PM(high bits) specify page, PL specifies steps within page
	int feed = ((m_pc >> 1 ^ m_pc) & 1) ? 0 : 0x20;
	m_pc = feed | (m_pc >> 1 & 0x1f) | (m_pc & ~0x3f);
}

void sm510_base_device::execute_run()
{
	while (m_icount > 0)
	{
		m_icount--;

		if (m_halt && !wake_me_up())
		{
			// got nothing to do
			m_icount = 0;
			return;
		}

		// remember previous state
		m_prev_op = m_op;
		m_prev_pc = m_pc;

		// fetch next opcode
		debugger_instruction_hook(this, m_pc);
		m_op = m_program->read_byte(m_pc);
		increment_pc();
		get_opcode_param();

		// handle opcode if it's not skipped
		if (m_skip)
		{
			m_skip = false;
			m_op = 0; // fake nop
		}
		else
			execute_one();
	}
}
