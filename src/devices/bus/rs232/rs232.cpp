// license:BSD-3-Clause
// copyright-holders:smf
#include "emu.h"
#include "rs232.h"

DEFINE_DEVICE_TYPE(RS232_PORT, rs232_port_device, "rs232", "RS232 Port")

rs232_port_device::rs232_port_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	rs232_port_device(mconfig, RS232_PORT, tag, owner, clock)
{
}

rs232_port_device::rs232_port_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	m_rxd(0),
	m_dcd(0),
	m_dsr(0),
	m_ri(0),
	m_cts(0),
	m_rxd_handler(*this),
	m_dcd_handler(*this),
	m_dsr_handler(*this),
	m_ri_handler(*this),
	m_cts_handler(*this),
	m_dev(nullptr)
{
}

rs232_port_device::~rs232_port_device()
{
}

void rs232_port_device::device_config_complete()
{
	m_dev = dynamic_cast<device_rs232_port_interface *>(get_card_device());
}

void rs232_port_device::device_start()
{
	m_rxd_handler.resolve_safe();
	m_dcd_handler.resolve_safe();
	m_dsr_handler.resolve_safe();
	m_ri_handler.resolve_safe();
	m_cts_handler.resolve_safe();

	save_item(NAME(m_rxd));
	save_item(NAME(m_dcd));
	save_item(NAME(m_dsr));
	save_item(NAME(m_ri));
	save_item(NAME(m_cts));

	m_rxd = 1;
	m_dcd = 1;
	m_dsr = 1;
	m_ri = 1;
	m_cts = 1;

	m_rxd_handler(1);
	m_dcd_handler(1);
	m_dsr_handler(1);
	m_ri_handler(1);
	m_cts_handler(1);
}

WRITE_LINE_MEMBER( rs232_port_device::write_txd )
{
	if(m_dev)
		m_dev->input_txd(state);
}

WRITE_LINE_MEMBER( rs232_port_device::write_dtr )
{
	if(m_dev)
		m_dev->input_dtr(state);
}

WRITE_LINE_MEMBER( rs232_port_device::write_rts )
{
	if(m_dev)
		m_dev->input_rts(state);
}

WRITE_LINE_MEMBER( rs232_port_device::write_etc )
{
	if(m_dev)
		m_dev->input_etc(state);
}

device_rs232_port_interface::device_rs232_port_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig, device)
{
	m_port = dynamic_cast<rs232_port_device *>(device.owner());
}

device_rs232_port_interface::~device_rs232_port_interface()
{
}

#include "keyboard.h"
#include "loopback.h"
#include "null_modem.h"
#include "printer.h"
#include "pty.h"
#include "sun_kbd.h"
#include "terminal.h"
#include "ie15.h"

SLOT_INTERFACE_START( default_rs232_devices )
	SLOT_INTERFACE("keyboard", SERIAL_KEYBOARD)
	SLOT_INTERFACE("loopback", RS232_LOOPBACK)
	SLOT_INTERFACE("null_modem", NULL_MODEM)
	SLOT_INTERFACE("printer", SERIAL_PRINTER)
	SLOT_INTERFACE("terminal", SERIAL_TERMINAL)
	SLOT_INTERFACE("pty", PSEUDO_TERMINAL)
	SLOT_INTERFACE("sunkbd", SUN_KBD_ADAPTOR)
	SLOT_INTERFACE("ie15", SERIAL_TERMINAL_IE15)
SLOT_INTERFACE_END
