// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/**********************************************************************

    SNK Neo Geo Mahjong controller emulation

**********************************************************************/

#include "emu.h"
#include "mahjong.h"

//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(NEOGEO_MJCTRL_AC, neogeo_mjctrl_ac_device, "neogeo_mj_ac", "SNK Neo Geo Arcade Mahjong panel")
DEFINE_DEVICE_TYPE(NEOGEO_MJCTRL,    neogeo_mjctrl_device,    "neogeo_mj",    "SNK Neo Geo Mahjong controller")


static INPUT_PORTS_START( neogeo_mj_ac )
	PORT_START("MJ.0")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_MAHJONG_A )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_MAHJONG_B )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_MAHJONG_C )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_MAHJONG_D )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_MAHJONG_E )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_MAHJONG_F )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_MAHJONG_G )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("MJ.1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_MAHJONG_H )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_MAHJONG_I )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_MAHJONG_J )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_MAHJONG_K )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_MAHJONG_L )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_MAHJONG_M )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_MAHJONG_N )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON6 )

	// is this actually connected?
	PORT_START("MJ.2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON3 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON4 )

	PORT_START("MJ.3")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_MAHJONG_PON )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_MAHJONG_CHI )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_MAHJONG_KAN )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_MAHJONG_RON )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_MAHJONG_REACH )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )
INPUT_PORTS_END


static INPUT_PORTS_START( neogeo_mj )
	PORT_INCLUDE( neogeo_mj_ac )

	PORT_START("START_SELECT")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_START )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_SELECT )
INPUT_PORTS_END


//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

ioport_constructor neogeo_mjctrl_ac_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( neogeo_mj_ac );
}

ioport_constructor neogeo_mjctrl_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( neogeo_mj );
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  neogeo_joystick_device - constructor
//-------------------------------------------------

neogeo_mjctrl_ac_device::neogeo_mjctrl_ac_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_neogeo_control_port_interface(mconfig, *this)
	, m_mjpanel(*this, "MJ.%u", 0)
{
}

neogeo_mjctrl_ac_device::neogeo_mjctrl_ac_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: neogeo_mjctrl_ac_device(mconfig, NEOGEO_MJCTRL_AC, tag, owner, clock)
{
}

neogeo_mjctrl_device::neogeo_mjctrl_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: neogeo_mjctrl_ac_device(mconfig, NEOGEO_MJCTRL, tag, owner, clock)
	, m_ss(*this, "START_SELECT")
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void neogeo_mjctrl_ac_device::device_start()
{
	save_item(NAME(m_ctrl_sel));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void neogeo_mjctrl_ac_device::device_reset()
{
	m_ctrl_sel = 0;
}


//-------------------------------------------------
//  read_ctrl
//-------------------------------------------------

uint8_t neogeo_mjctrl_ac_device::read_ctrl()
{
	uint8_t res = 0;
	switch (m_ctrl_sel)
	{
		default:
		case 0x00: res = 0xff; break;
		case 0x09: res = m_mjpanel[0]->read(); break;
		case 0x12: res = m_mjpanel[1]->read(); break;
		case 0x1b: res = m_mjpanel[2]->read(); break;
		case 0x24: res = m_mjpanel[3]->read(); break;
	}

	return res;
}

//-------------------------------------------------
//  write_ctrlsel
//-------------------------------------------------

void neogeo_mjctrl_ac_device::write_ctrlsel(uint8_t data)
{
	m_ctrl_sel = data;
}

//-------------------------------------------------
//  read_start_sel
//-------------------------------------------------

uint8_t neogeo_mjctrl_device::read_start_sel()
{
	return m_ss->read();
}
