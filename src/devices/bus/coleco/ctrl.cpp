// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    ColecoVision controller port emulation

**********************************************************************/

#include "emu.h"
#include "ctrl.h"



//**************************************************************************
//  DEVICE DEFINITION
//**************************************************************************

DEFINE_DEVICE_TYPE(COLECOVISION_CONTROL_PORT, colecovision_control_port_device, "colecovision_control_port", "ColecoVision control port")



//**************************************************************************
//  CARD INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_colecovision_control_port_interface - constructor
//-------------------------------------------------

device_colecovision_control_port_interface::device_colecovision_control_port_interface(const machine_config &mconfig, device_t &device) :
	device_slot_card_interface(mconfig, device),
	m_common0(1),
	m_common1(1)
{
	m_port = dynamic_cast<colecovision_control_port_device *>(device.owner());
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  colecovision_control_port_device - constructor
//-------------------------------------------------

colecovision_control_port_device::colecovision_control_port_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, COLECOVISION_CONTROL_PORT, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	m_device(nullptr),
	m_write_irq(*this)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void colecovision_control_port_device::device_start()
{
	m_device = dynamic_cast<device_colecovision_control_port_interface *>(get_card_device());

	m_write_irq.resolve_safe();
}


//-------------------------------------------------
//  SLOT_INTERFACE( colecovision_control_port_devices )
//-------------------------------------------------

#include "hand.h"
#include "sac.h"

SLOT_INTERFACE_START( colecovision_control_port_devices )
	SLOT_INTERFACE("hand", COLECO_HAND_CONTROLLER)
	SLOT_INTERFACE("sac", COLECO_SUPER_ACTION_CONTROLLER)
SLOT_INTERFACE_END
