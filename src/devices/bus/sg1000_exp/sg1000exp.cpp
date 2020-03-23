// license:BSD-3-Clause
// copyright-holders:Enik Land
/**********************************************************************

    Sega SG-1000 expansion slot emulation

**********************************************************************/

#include "emu.h"
#include "sg1000exp.h"

// slot devices
#include "sk1100.h"
#include "fm_unit.h"



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(SG1000_EXPANSION_SLOT, sg1000_expansion_slot_device, "sg1000_expansion_slot", "Sega SG-1000 expansion slot")



//**************************************************************************
//  CARD INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_sg1000_expansion_slot_interface - constructor
//-------------------------------------------------

device_sg1000_expansion_slot_interface::device_sg1000_expansion_slot_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig,device)
{
}


//-------------------------------------------------
//  ~device_sg1000_expansion_slot_interface - destructor
//-------------------------------------------------

device_sg1000_expansion_slot_interface::~device_sg1000_expansion_slot_interface()
{
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  sg1000_expansion_slot_device - constructor
//-------------------------------------------------

sg1000_expansion_slot_device::sg1000_expansion_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, SG1000_EXPANSION_SLOT, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	m_device(nullptr)
{
}


//-------------------------------------------------
//  sg1000_expansion_slot_device - destructor
//-------------------------------------------------

sg1000_expansion_slot_device::~sg1000_expansion_slot_device()
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void sg1000_expansion_slot_device::device_start()
{
	m_device = dynamic_cast<device_sg1000_expansion_slot_interface *>(get_card_device());
}


// Address offsets are masked with 0x07 because the SG-1000 expansion slot
// has only 3 address lines (A0, A1, A2).


READ8_MEMBER(sg1000_expansion_slot_device::read)
{
	uint8_t data = 0xff;
	if (m_device)
		data = m_device->peripheral_r(space, offset & 0x07);
	return data;
}

WRITE8_MEMBER(sg1000_expansion_slot_device::write)
{
	if (m_device)
		m_device->peripheral_w(space, offset & 0x07, data);
}


bool sg1000_expansion_slot_device::is_readable(uint8_t offset)
{
	if (m_device)
		return m_device->is_readable(offset & 0x07);
	return false;
}


bool sg1000_expansion_slot_device::is_writeable(uint8_t offset)
{
	if (m_device)
		return m_device->is_writeable(offset & 0x07);
	return false;
}


//-------------------------------------------------
//  SLOT_INTERFACE( sg1000_expansion_devices )
//-------------------------------------------------

SLOT_INTERFACE_START( sg1000_expansion_devices )
	SLOT_INTERFACE("sk1100", SEGA_SK1100)
	SLOT_INTERFACE("fm", SEGA_FM_UNIT)
SLOT_INTERFACE_END
