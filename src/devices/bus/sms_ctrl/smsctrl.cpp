// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/**********************************************************************

    Sega Master System controller port emulation

**********************************************************************/

#include "emu.h"
#include "smsctrl.h"

// slot devices
#include "joypad.h"
#include "lphaser.h"
#include "paddle.h"
#include "sports.h"
#include "sportsjp.h"
#include "rfu.h"
#include "multitap.h"
#include "graphic.h"



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(SMS_CONTROL_PORT, sms_control_port_device, "sms_control_port", "Sega SMS controller port")



//**************************************************************************
//  CARD INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_sms_control_port_interface - constructor
//-------------------------------------------------

device_sms_control_port_interface::device_sms_control_port_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig,device)
{
	m_port = dynamic_cast<sms_control_port_device *>(device.owner());
}


//-------------------------------------------------
//  ~device_sms_control_port_interface - destructor
//-------------------------------------------------

device_sms_control_port_interface::~device_sms_control_port_interface()
{
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  sms_control_port_device - constructor
//-------------------------------------------------

sms_control_port_device::sms_control_port_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, SMS_CONTROL_PORT, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	m_device(nullptr),
	m_th_pin_handler(*this),
	m_pixel_handler(*this)
{
}


//-------------------------------------------------
//  sms_control_port_device - destructor
//-------------------------------------------------

sms_control_port_device::~sms_control_port_device()
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void sms_control_port_device::device_start()
{
	m_device = dynamic_cast<device_sms_control_port_interface *>(get_card_device());

	m_th_pin_handler.resolve_safe();
	m_pixel_handler.resolve_safe(0);
}


uint8_t sms_control_port_device::port_r()
{
	uint8_t data = 0xff;
	if (m_device)
		data = m_device->peripheral_r();
	return data;
}

void sms_control_port_device::port_w( uint8_t data )
{
	if (m_device)
		m_device->peripheral_w(data);
}


void sms_control_port_device::th_pin_w(int state)
{
	m_th_pin_handler(state);
}

uint32_t sms_control_port_device::pixel_r()
{
	return m_pixel_handler();
}


//-------------------------------------------------
//  SLOT_INTERFACE( sms_control_port_devices )
//-------------------------------------------------

SLOT_INTERFACE_START( sms_control_port_devices )
	SLOT_INTERFACE("joypad", SMS_JOYPAD)
	SLOT_INTERFACE("lphaser", SMS_LIGHT_PHASER)
	SLOT_INTERFACE("paddle", SMS_PADDLE)
	SLOT_INTERFACE("sportspad", SMS_SPORTS_PAD)
	SLOT_INTERFACE("sportspadjp", SMS_SPORTS_PAD_JP)
	SLOT_INTERFACE("rapidfire", SMS_RAPID_FIRE)
	SLOT_INTERFACE("multitap", SMS_MULTITAP)
	SLOT_INTERFACE("graphic", SMS_GRAPHIC)
SLOT_INTERFACE_END
