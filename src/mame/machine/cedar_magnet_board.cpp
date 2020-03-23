// license:BSD-3-Clause
// copyright-holders:David Haywood

#include "emu.h"
#include "cedar_magnet_board.h"

cedar_magnet_board_interface::cedar_magnet_board_interface(const machine_config &mconfig, device_t &device, const char *cputag, const char *ramtag)
	: device_interface(device, "cedmag_board"),
	m_cpu(device, cputag),
	m_ram(device, ramtag)
{
}

void cedar_magnet_board_interface::write_cpu_bus(int offset, u8 data)
{
	address_space &ap = m_cpu->space(AS_PROGRAM);
	ap.write_byte(offset, data);
}

uint8_t cedar_magnet_board_interface::read_cpu_bus(int offset)
{
	address_space &ap = m_cpu->space(AS_PROGRAM);
	return ap.read_byte(offset);
}

bool cedar_magnet_board_interface::is_running() const
{
	return m_is_running;
}

TIMER_CALLBACK_MEMBER(cedar_magnet_board_interface::reset_assert_callback)
{
	m_cpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
}

TIMER_CALLBACK_MEMBER(cedar_magnet_board_interface::reset_clear_callback)
{
	m_cpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
}

TIMER_CALLBACK_MEMBER(cedar_magnet_board_interface::halt_assert_callback)
{
	m_cpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
	m_is_running = false;
}

TIMER_CALLBACK_MEMBER(cedar_magnet_board_interface::halt_clear_callback)
{
	m_cpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
	m_is_running = true;
}

void cedar_magnet_board_interface::irq_hold()
{
	m_cpu->set_input_line(INPUT_LINE_IRQ0, HOLD_LINE);
}

void cedar_magnet_board_interface::halt_assert()
{
	device().machine().scheduler().timer_set(attotime::from_usec(2), timer_expired_delegate(FUNC(cedar_magnet_board_interface::halt_assert_callback),this));
}

void cedar_magnet_board_interface::halt_clear()
{
	device().machine().scheduler().timer_set(attotime::from_usec(2), timer_expired_delegate(FUNC(cedar_magnet_board_interface::halt_clear_callback),this));
}

void cedar_magnet_board_interface::reset_assert()
{
	device().machine().scheduler().timer_set(attotime::from_usec(1), timer_expired_delegate(FUNC(cedar_magnet_board_interface::reset_assert_callback),this));
}

void cedar_magnet_board_interface::reset_clear()
{
	device().machine().scheduler().timer_set(attotime::from_usec(1), timer_expired_delegate(FUNC(cedar_magnet_board_interface::reset_clear_callback),this));
}

void cedar_magnet_board_interface::interface_pre_reset()
{
	halt_assert();
}
