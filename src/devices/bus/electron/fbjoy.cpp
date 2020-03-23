// license:BSD-3-Clause
// copyright-holders:Nigel Barnes
/**********************************************************************

    First Byte Switched Joystick Interface

    http://chrisacorns.computinghistory.org.uk/8bit_Upgrades/FirstByte_JoystickIF.html

**********************************************************************/

#include "emu.h"
#include "fbjoy.h"

//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(ELECTRON_FBJOY, electron_fbjoy_device, "electron_fbjoy", "First Byte Joystick Interface")


static INPUT_PORTS_START( fbjoy )
	PORT_START("JOY")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP) PORT_8WAY
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN) PORT_8WAY
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT) PORT_8WAY
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_8WAY
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_BUTTON1) PORT_NAME("Fire")
INPUT_PORTS_END


//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

ioport_constructor electron_fbjoy_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( fbjoy );
}


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  electron_fbjoy_device - constructor
//-------------------------------------------------

electron_fbjoy_device::electron_fbjoy_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, ELECTRON_FBJOY, tag, owner, clock)
	, device_electron_expansion_interface(mconfig, *this)
	, m_joy(*this, "JOY")
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void electron_fbjoy_device::device_start()
{
	address_space& space = machine().device("maincpu")->memory().space(AS_PROGRAM);
	m_slot = dynamic_cast<electron_expansion_slot_device *>(owner());

	space.install_read_handler(0xfcc0, 0xfcc0, READ8_DELEGATE(electron_fbjoy_device, joystick_r));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void electron_fbjoy_device::device_reset()
{
}


//**************************************************************************
//  IMPLEMENTATION
//**************************************************************************

READ8_MEMBER(electron_fbjoy_device::joystick_r)
{
	return m_joy->read() | 0xe0;
}
