// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Grundy NewBrain Expansion Port emulation

**********************************************************************/

#include "emu.h"
#include "exp.h"



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(NEWBRAIN_EXPANSION_SLOT, newbrain_expansion_slot_device, "newbrain_expansion_slot", "NewBrain expansion port")



//**************************************************************************
//  CARD INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_newbrain_expansion_slot_interface - constructor
//-------------------------------------------------

device_newbrain_expansion_slot_interface::device_newbrain_expansion_slot_interface(const machine_config &mconfig, device_t &device) :
	device_slot_card_interface(mconfig,device)
{
	m_slot = dynamic_cast<newbrain_expansion_slot_device *>(device.owner());
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  newbrain_expansion_slot_device - constructor
//-------------------------------------------------

newbrain_expansion_slot_device::newbrain_expansion_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, NEWBRAIN_EXPANSION_SLOT, tag, owner, clock),
	device_slot_interface(mconfig, *this), m_card(nullptr)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void newbrain_expansion_slot_device::device_start()
{
	m_card = dynamic_cast<device_newbrain_expansion_slot_interface *>(get_card_device());
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void newbrain_expansion_slot_device::device_reset()
{
	if (m_card != nullptr)
	{
		m_card->device().reset();
	}
}


//-------------------------------------------------
//  mreq_r - memory request read
//-------------------------------------------------

uint8_t newbrain_expansion_slot_device::mreq_r(address_space &space, offs_t offset, uint8_t data, bool &romov, int &exrm, bool &raminh)
{
	if (m_card != nullptr)
	{
		data = m_card->mreq_r(space, offset, data, romov, exrm, raminh);
	}

	return data;
}


//-------------------------------------------------
//  mreq_w - memory request write
//-------------------------------------------------

void newbrain_expansion_slot_device::mreq_w(address_space &space, offs_t offset, uint8_t data, bool &romov, int &exrm, bool &raminh)
{
	if (m_card != nullptr)
	{
		m_card->mreq_w(space, offset, data, romov, exrm, raminh);
	}
}


//-------------------------------------------------
//  iorq_r - I/O request read
//-------------------------------------------------

uint8_t newbrain_expansion_slot_device::iorq_r(address_space &space, offs_t offset, uint8_t data, bool &prtov)
{
	if (m_card != nullptr)
	{
		data = m_card->iorq_r(space, offset, data, prtov);
	}

	return data;
}


//-------------------------------------------------
//  iorq_w - I/O request write
//-------------------------------------------------

void newbrain_expansion_slot_device::iorq_w(address_space &space, offs_t offset, uint8_t data, bool &prtov)
{
	if (m_card != nullptr)
	{
		m_card->iorq_w(space, offset, data, prtov);
	}
}


//-------------------------------------------------
//  SLOT_INTERFACE( newbrain_expansion_cards )
//-------------------------------------------------

// slot devices
#include "eim.h"
#include "fdc.h"

SLOT_INTERFACE_START( newbrain_expansion_cards )
	SLOT_INTERFACE("eim", NEWBRAIN_EIM)
	SLOT_INTERFACE("fdc", NEWBRAIN_FDC)
SLOT_INTERFACE_END
