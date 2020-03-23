// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/**********************************************************************

    Nintendo Super Famicom & SNES Joypad

**********************************************************************/

#include "emu.h"
#include "joypad.h"

//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(SNES_JOYPAD, snes_joypad_device, "snes_joypad", "Nintendo SNES / SFC Control Pad")


static INPUT_PORTS_START( snes_joypad )
	PORT_START("JOYPAD")
	PORT_BIT( 0x0001, IP_ACTIVE_HIGH, IPT_BUTTON2 ) PORT_NAME("B")
	PORT_BIT( 0x0002, IP_ACTIVE_HIGH, IPT_BUTTON1 ) PORT_NAME("Y")
	PORT_BIT( 0x0004, IP_ACTIVE_HIGH, IPT_SELECT ) PORT_NAME("Select")
	PORT_BIT( 0x0008, IP_ACTIVE_HIGH, IPT_START ) PORT_NAME("Start")
	PORT_BIT( 0x0010, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP )
	PORT_BIT( 0x0020, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN )
	PORT_BIT( 0x0040, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT )
	PORT_BIT( 0x0080, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT )
	PORT_BIT( 0x0100, IP_ACTIVE_HIGH, IPT_BUTTON3 ) PORT_NAME("A")
	PORT_BIT( 0x0200, IP_ACTIVE_HIGH, IPT_BUTTON4 ) PORT_NAME("X")
	PORT_BIT( 0x0400, IP_ACTIVE_HIGH, IPT_BUTTON6 ) PORT_NAME("L")
	PORT_BIT( 0x0800, IP_ACTIVE_HIGH, IPT_BUTTON5 ) PORT_NAME("R")
	PORT_BIT( 0xf000, IP_ACTIVE_HIGH, IPT_UNUSED )
INPUT_PORTS_END


//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

ioport_constructor snes_joypad_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( snes_joypad );
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  snes_joypad_device - constructor
//-------------------------------------------------

snes_joypad_device::snes_joypad_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, SNES_JOYPAD, tag, owner, clock),
	device_snes_control_port_interface(mconfig, *this),
	m_joypad(*this, "JOYPAD"),
	m_strobe(0), m_latch(0)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void snes_joypad_device::device_start()
{
	save_item(NAME(m_latch));
	save_item(NAME(m_strobe));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void snes_joypad_device::device_reset()
{
	m_latch = 0;
	m_strobe = 0;
}


//-------------------------------------------------
//  poll
//-------------------------------------------------

void snes_joypad_device::port_poll()
{
	uint16_t temp = m_joypad->read();
	// avoid sending signals that could crash games
	// if left, no right
	if (temp & 0x40)
		temp &= ~0x80;
	// if up, no down
	if (temp & 0x10)
		temp &= ~0x20;

	m_latch = temp | 0xffff0000;
}

//-------------------------------------------------
//  read
//-------------------------------------------------

uint8_t snes_joypad_device::read_pin4()
{
	uint8_t ret = m_latch & 1;
	m_latch >>= 1;
	return ret;
}

//-------------------------------------------------
//  write
//-------------------------------------------------

void snes_joypad_device::write_strobe(uint8_t data)
{
	int old = m_strobe;
	m_strobe = data & 0x01;

	if (m_strobe < old) // 1 -> 0 transition
		port_poll();
}
