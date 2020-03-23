// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Commodore CBM-II Expansion Port emulation

**********************************************************************/

#include "emu.h"
#include "exp.h"



//**************************************************************************
//  MACROS/CONSTANTS
//**************************************************************************

#define LOG 0



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(CBM2_EXPANSION_SLOT, cbm2_expansion_slot_device, "cbm2_expansion_slot", "CBM-II expansion port")



//**************************************************************************
//  DEVICE CBM2_EXPANSION CARD INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_cbm2_expansion_card_interface - constructor
//-------------------------------------------------

device_cbm2_expansion_card_interface::device_cbm2_expansion_card_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig, device),
		m_bank1(*this, "bank1"),
		m_bank2(*this, "bank2"),
		m_bank3(*this, "bank3")
{
	m_slot = dynamic_cast<cbm2_expansion_slot_device *>(device.owner());
}


//-------------------------------------------------
//  ~device_cbm2_expansion_card_interface - destructor
//-------------------------------------------------

device_cbm2_expansion_card_interface::~device_cbm2_expansion_card_interface()
{
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  cbm2_expansion_slot_device - constructor
//-------------------------------------------------

cbm2_expansion_slot_device::cbm2_expansion_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, CBM2_EXPANSION_SLOT, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	device_image_interface(mconfig, *this),
	m_card(nullptr)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void cbm2_expansion_slot_device::device_start()
{
	m_card = dynamic_cast<device_cbm2_expansion_card_interface *>(get_card_device());

	// inherit bus clock
	if (clock() == 0)
	{
		cbm2_expansion_slot_device *root = machine().device<cbm2_expansion_slot_device>(CBM2_EXPANSION_SLOT_TAG);
		assert(root);
		set_unscaled_clock(root->clock());
	}
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void cbm2_expansion_slot_device::device_reset()
{
}


//-------------------------------------------------
//  call_load -
//-------------------------------------------------

image_init_result cbm2_expansion_slot_device::call_load()
{
	size_t size;

	if (m_card)
	{
		if (!loaded_through_softlist())
		{
			size = length();

			if (is_filetype("20"))
			{
				m_card->m_bank1.allocate(size);
				fread(m_card->m_bank1, size);
			}
			else if (is_filetype("40"))
			{
				m_card->m_bank2.allocate(size);
				fread(m_card->m_bank2, size);
			}
			else if (is_filetype("60"))
			{
				m_card->m_bank3.allocate(size);
				fread(m_card->m_bank3, size);
			}
		}
		else
		{
			load_software_region("bank1", m_card->m_bank1);
			load_software_region("bank2", m_card->m_bank2);
			load_software_region("bank3", m_card->m_bank3);
		}
	}

	return image_init_result::PASS;
}


//-------------------------------------------------
//  get_default_card_software -
//-------------------------------------------------

std::string cbm2_expansion_slot_device::get_default_card_software(get_default_card_software_hook &hook) const
{
	return software_get_default_slot("standard");
}


//-------------------------------------------------
//  read - cartridge data read
//-------------------------------------------------

uint8_t cbm2_expansion_slot_device::read(address_space &space, offs_t offset, uint8_t data, int csbank1, int csbank2, int csbank3)
{
	if (m_card != nullptr)
	{
		data = m_card->cbm2_bd_r(space, offset, data, csbank1, csbank2, csbank3);
	}

	return data;
}


//-------------------------------------------------
//  write - cartridge data write
//-------------------------------------------------

void cbm2_expansion_slot_device::write(address_space &space, offs_t offset, uint8_t data, int csbank1, int csbank2, int csbank3)
{
	if (m_card != nullptr)
	{
		m_card->cbm2_bd_w(space, offset, data, csbank1, csbank2, csbank3);
	}
}


//-------------------------------------------------
//  SLOT_INTERFACE( cbm2_expansion_cards )
//-------------------------------------------------

// slot devices
#include "24k.h"
#include "hrg.h"
#include "std.h"

SLOT_INTERFACE_START( cbm2_expansion_cards )
	SLOT_INTERFACE("24k", CBM2_24K)
	SLOT_INTERFACE("hrga", CBM2_HRG_A)
	SLOT_INTERFACE("hrgb", CBM2_HRG_B)
	SLOT_INTERFACE_INTERNAL("standard", CBM2_STD)
SLOT_INTERFACE_END
