// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    ColecoVision cartridge port emulation

**********************************************************************/

#include "emu.h"
#include "exp.h"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(COLECOVISION_CARTRIDGE_SLOT, colecovision_cartridge_slot_device, "coleco_cartridge_port", "ColecoVision cartridge port")



//**************************************************************************
//  DEVICE C64_EXPANSION CARD INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_colecovision_cartridge_interface - constructor
//-------------------------------------------------

device_colecovision_cartridge_interface::device_colecovision_cartridge_interface(const machine_config &mconfig, device_t &device) :
	device_slot_card_interface(mconfig, device),
	m_rom(nullptr),
	m_rom_size(0)
{
	m_slot = dynamic_cast<colecovision_cartridge_slot_device *>(device.owner());
}

void device_colecovision_cartridge_interface::rom_alloc(size_t size)
{
	if (m_rom == nullptr)
	{
		m_rom = device().machine().memory().region_alloc(":coleco_cart:rom", size, 1, ENDIANNESS_LITTLE)->base();
		m_rom_size = size;
	}
}


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  colecovision_cartridge_slot_device - constructor
//-------------------------------------------------

colecovision_cartridge_slot_device::colecovision_cartridge_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, COLECOVISION_CARTRIDGE_SLOT, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	device_image_interface(mconfig, *this),
	m_card(nullptr)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void colecovision_cartridge_slot_device::device_start()
{
	m_card = dynamic_cast<device_colecovision_cartridge_interface *>(get_card_device());
}


//-------------------------------------------------
//  call_load -
//-------------------------------------------------

image_init_result colecovision_cartridge_slot_device::call_load()
{
	if (m_card)
	{
		size_t size = !loaded_through_softlist() ? length() : get_software_region_length("rom");
		m_card->rom_alloc(size);

		if (!loaded_through_softlist())
		{
			fread(m_card->m_rom, size);
		}
		else
		{
			// TODO 8000/a000/c000/e000
			memcpy(m_card->m_rom, get_software_region("rom"), size);
		}
	}

	return image_init_result::PASS;
}


//-------------------------------------------------
//  get_default_card_software -
//-------------------------------------------------

std::string colecovision_cartridge_slot_device::get_default_card_software(get_default_card_software_hook &hook) const
{
	if (hook.image_file())
	{
		uint32_t length = hook.image_file()->size();
		if (length == 0x100000 || length == 0x200000)
			return software_get_default_slot("xin1");
	}
	return software_get_default_slot("standard");
}


//-------------------------------------------------
//  bd_r - cartridge data read
//-------------------------------------------------

uint8_t colecovision_cartridge_slot_device::bd_r(address_space &space, offs_t offset, uint8_t data, int _8000, int _a000, int _c000, int _e000)
{
	if (m_card != nullptr)
	{
		data = m_card->bd_r(space, offset, data, _8000, _a000, _c000, _e000);
	}

	return data;
}


//-------------------------------------------------
//  SLOT_INTERFACE( colecovision_cartridges )
//-------------------------------------------------

#include "std.h"
#include "xin1.h"

SLOT_INTERFACE_START( colecovision_cartridges )
	// the following need ROMs from the software list
	SLOT_INTERFACE_INTERNAL("standard", COLECOVISION_STANDARD)
	SLOT_INTERFACE_INTERNAL("xin1", COLECOVISION_XIN1)
SLOT_INTERFACE_END
