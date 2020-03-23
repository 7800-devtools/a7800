// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
#include "emu.h"
#include "rom.h"


DEFINE_DEVICE_TYPE(MSX_SLOT_ROM, msx_slot_rom_device, "msx_slot_rom", "MSX Internal ROM")


msx_slot_rom_device::msx_slot_rom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: msx_slot_rom_device(mconfig, MSX_SLOT_ROM, tag, owner, clock)
{
}


msx_slot_rom_device::msx_slot_rom_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, msx_internal_slot_interface()
	, m_rom_region(*this, finder_base::DUMMY_TAG)
	, m_region_offset(0)
	, m_rom(nullptr)
{
}


void msx_slot_rom_device::set_rom_start(device_t &device, const char *region, uint32_t offset)
{
	msx_slot_rom_device &dev = downcast<msx_slot_rom_device &>(device);

	dev.m_rom_region.set_tag(region);
	dev.m_region_offset = offset;
}


void msx_slot_rom_device::device_start()
{
	// Sanity checks
	if (m_rom_region->bytes() < m_region_offset + m_size)
	{
		fatalerror("Memory region '%s' is too small for rom slot '%s'\n", m_rom_region.finder_tag(), tag());
	}

	m_rom = m_rom_region->base() + m_region_offset;
}


READ8_MEMBER(msx_slot_rom_device::read)
{
	if ( offset >= m_start_address && offset < m_end_address )
	{
		return m_rom[ offset - m_start_address ];
	}
	return 0xFF;
}
