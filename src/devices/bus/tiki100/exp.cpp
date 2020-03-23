// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    TIKI-100 expansion bus emulation

**********************************************************************/

#include "emu.h"
#include "exp.h"


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(TIKI100_BUS,      tiki100_bus_device,      "tiki100bus",      "TIKI-100 expansion bus")
DEFINE_DEVICE_TYPE(TIKI100_BUS_SLOT, tiki100_bus_slot_device, "tiki100bus_slot", "TIKI-100 expansion bus slot")



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  tiki100_bus_slot_device - constructor
//-------------------------------------------------

tiki100_bus_slot_device::tiki100_bus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, TIKI100_BUS_SLOT, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	device_z80daisy_interface(mconfig, *this),
	m_bus(nullptr),
	m_card(nullptr)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void tiki100_bus_slot_device::device_start()
{
	m_bus = machine().device<tiki100_bus_device>(TIKI100_BUS_TAG);
	device_tiki100bus_card_interface *dev = dynamic_cast<device_tiki100bus_card_interface *>(get_card_device());
	if (dev)
	{
		m_bus->add_card(dev);
		m_card = dev;
	}
}


//-------------------------------------------------
//  tiki100_bus_device - constructor
//-------------------------------------------------

tiki100_bus_device::tiki100_bus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, TIKI100_BUS, tag, owner, clock),
	m_irq_cb(*this),
	m_nmi_cb(*this),
	m_busrq_cb(*this),
	m_in_mrq_cb(*this),
	m_out_mrq_cb(*this)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void tiki100_bus_device::device_start()
{
	// resolve callbacks
	m_irq_cb.resolve_safe();
	m_nmi_cb.resolve_safe();
	m_busrq_cb.resolve_safe();
	m_in_mrq_cb.resolve();
	m_out_mrq_cb.resolve();
}


//-------------------------------------------------
//  add_card - add card
//-------------------------------------------------

void tiki100_bus_device::add_card(device_tiki100bus_card_interface *card)
{
	m_device_list.append(*card);

	card->m_bus = this;
}


//-------------------------------------------------
//  mrq_r - memory read
//-------------------------------------------------

uint8_t tiki100_bus_device::mrq_r(address_space &space, offs_t offset, uint8_t data, bool &mdis)
{
	device_tiki100bus_card_interface *entry = m_device_list.first();

	while (entry)
	{
		data &= entry->mrq_r(space, offset, data, mdis);
		entry = entry->next();
	}

	return data;
}


//-------------------------------------------------
//  mrq_w - memory write
//-------------------------------------------------

WRITE8_MEMBER( tiki100_bus_device::mrq_w )
{
	device_tiki100bus_card_interface *entry = m_device_list.first();

	while (entry)
	{
		entry->mrq_w(space, offset, data);
		entry = entry->next();
	}
}


//-------------------------------------------------
//  iorq_r - I/O read
//-------------------------------------------------

uint8_t tiki100_bus_device::iorq_r(address_space &space, offs_t offset, uint8_t data)
{
	device_tiki100bus_card_interface *entry = m_device_list.first();

	while (entry)
	{
		data &= entry->iorq_r(space, offset, data);
		entry = entry->next();
	}

	return data;
}


//-------------------------------------------------
//  iorq_w - I/O write
//-------------------------------------------------

WRITE8_MEMBER( tiki100_bus_device::iorq_w )
{
	device_tiki100bus_card_interface *entry = m_device_list.first();

	while (entry)
	{
		entry->iorq_w(space, offset, data);
		entry = entry->next();
	}
}


//-------------------------------------------------
//  busak_w - bus acknowledge write
//-------------------------------------------------

WRITE_LINE_MEMBER( tiki100_bus_device::busak_w )
{
	device_tiki100bus_card_interface *entry = m_device_list.first();

	while (entry)
	{
		entry->busak_w(state);
		entry = entry->next();
	}
}



//**************************************************************************
//  DEVICE TIKI-100 BUS CARD INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_tiki100bus_card_interface - constructor
//-------------------------------------------------

device_tiki100bus_card_interface::device_tiki100bus_card_interface(const machine_config &mconfig, device_t &device) :
	device_slot_card_interface(mconfig, device), m_bus(nullptr),
	m_busak(CLEAR_LINE), m_next(nullptr)
{
	m_slot = dynamic_cast<tiki100_bus_slot_device *>(device.owner());
}


//-------------------------------------------------
//  SLOT_INTERFACE( tiki100_cards )
//-------------------------------------------------

// slot devices
#include "8088.h"
#include "hdc.h"

SLOT_INTERFACE_START( tiki100_cards )
	SLOT_INTERFACE("8088", TIKI100_8088)
	SLOT_INTERFACE("hdc", TIKI100_HDC)
SLOT_INTERFACE_END
