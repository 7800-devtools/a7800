// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Intel Multibus I/O Expansion Bus IEEE-P959 (iSBX) emulation

**********************************************************************/

#include "emu.h"
#include "isbx.h"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(ISBX_SLOT, isbx_slot_device, "isbx_slot", "iSBX bus slot")



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  device_isbx_card_interface - constructor
//-------------------------------------------------

device_isbx_card_interface::device_isbx_card_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig, device)
{
	m_slot = dynamic_cast<isbx_slot_device *>(device.owner());
}


//-------------------------------------------------
//  isbx_slot_device - constructor
//-------------------------------------------------

isbx_slot_device::isbx_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, ISBX_SLOT, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	m_write_mintr0(*this),
	m_write_mintr1(*this),
	m_write_mdrqt(*this),
	m_write_mwait(*this), m_card(nullptr)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void isbx_slot_device::device_start()
{
	m_card = dynamic_cast<device_isbx_card_interface *>(get_card_device());

	// resolve callbacks
	m_write_mintr0.resolve_safe();
	m_write_mintr1.resolve_safe();
	m_write_mdrqt.resolve_safe();
	m_write_mwait.resolve_safe();
}


//-------------------------------------------------
//  SLOT_INTERFACE( isbx_cards )
//-------------------------------------------------

// slot devices
#include "compis_fdc.h"
#include "isbc_218a.h"

SLOT_INTERFACE_START( isbx_cards )
	SLOT_INTERFACE("fdc", COMPIS_FDC)
	SLOT_INTERFACE("fdc_218a", ISBC_218A)
SLOT_INTERFACE_END
