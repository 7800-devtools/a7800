// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    RCA Cosmac VIP Byte Input/Output port emulation

**********************************************************************/

#include "emu.h"
#include "byteio.h"



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(VIP_BYTEIO_PORT, vip_byteio_port_device, "vip_byteio_port", "VIP byte I/O port")



//**************************************************************************
//  CARD INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_vip_byteio_port_interface - constructor
//-------------------------------------------------

device_vip_byteio_port_interface::device_vip_byteio_port_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig,device)
{
	m_slot = dynamic_cast<vip_byteio_port_device *>(device.owner());
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  vip_byteio_port_device - constructor
//-------------------------------------------------

vip_byteio_port_device::vip_byteio_port_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, VIP_BYTEIO_PORT, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	m_write_inst(*this),
	m_cart(nullptr)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void vip_byteio_port_device::device_start()
{
	m_cart = dynamic_cast<device_vip_byteio_port_interface *>(get_card_device());

	// resolve callbacks
	m_write_inst.resolve_safe();
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void vip_byteio_port_device::device_reset()
{
}


uint8_t vip_byteio_port_device::in_r() { uint8_t data = 0xff; if (m_cart != nullptr) data = m_cart->vip_in_r(); return data; }
void vip_byteio_port_device::out_w(uint8_t data) { if (m_cart != nullptr) m_cart->vip_out_w(data); }
READ_LINE_MEMBER( vip_byteio_port_device::ef3_r ) { int state = CLEAR_LINE; if (m_cart != nullptr) state = m_cart->vip_ef3_r(); return state; }
READ_LINE_MEMBER( vip_byteio_port_device::ef4_r ) { int state = CLEAR_LINE; if (m_cart != nullptr) state = m_cart->vip_ef4_r(); return state; }
WRITE_LINE_MEMBER( vip_byteio_port_device::q_w ) { if (m_cart != nullptr) m_cart->vip_q_w(state); }


//-------------------------------------------------
//  SLOT_INTERFACE( vip_byteio_cards )
//-------------------------------------------------

SLOT_INTERFACE_START( vip_byteio_cards )
	//SLOT_INTERFACE("exp2", VP576_BYTEIO)
	SLOT_INTERFACE("ascii", VP620)
SLOT_INTERFACE_END
