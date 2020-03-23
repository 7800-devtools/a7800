// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/**********************************************************************

    Sega Mark III "Joypad" / Master System "Control Pad" emulation


Release data from the Sega Retro project:

- Joypad:

  Year: 1985    Country/region: JP    Model code: SJ-152

- Control Pad:

  Year: 1986    Country/region: US    Model code: 3020
  Year: 1987    Country/region: JP    Model code: 3020
  Year: 1987    Country/region: EU    Model code: ?
  Year: 1989    Country/region: BR    Model code: 011770
  Year: 1989    Country/region: KR    Model code: ?

**********************************************************************/

#include "emu.h"
#include "joypad.h"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(SMS_JOYPAD, sms_joypad_device, "sms_joypad", "Sega SMS Control Pad")


static INPUT_PORTS_START( sms_joypad )
	PORT_START("JOYPAD")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNUSED )   // Vcc
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON1 )  // TL
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNUSED )   // TH
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON2 )  // TR
INPUT_PORTS_END


//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

ioport_constructor sms_joypad_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( sms_joypad );
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  sms_joypad_device - constructor
//-------------------------------------------------

sms_joypad_device::sms_joypad_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, SMS_JOYPAD, tag, owner, clock),
	device_sms_control_port_interface(mconfig, *this),
	m_joypad(*this, "JOYPAD")
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void sms_joypad_device::device_start()
{
}


//-------------------------------------------------
//  sms_peripheral_r - joypad read
//-------------------------------------------------

uint8_t sms_joypad_device::peripheral_r()
{
	return m_joypad->read();
}
