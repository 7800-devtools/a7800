// license:BSD-3-Clause
// copyright-holders:Wilbert Pol

#include "emu.h"
#include "nomapper.h"

DEFINE_DEVICE_TYPE(MSX_CART_NOMAPPER, msx_cart_nomapper_device, "msx_cart_nomapper", "MSX Cartridge - ROM")


msx_cart_nomapper_device::msx_cart_nomapper_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, MSX_CART_NOMAPPER, tag, owner, clock)
	, msx_cart_interface(mconfig, *this)
	, m_start_address(0)
	, m_end_address(0)
{
}

void msx_cart_nomapper_device::device_start()
{
}

void msx_cart_nomapper_device::initialize_cartridge()
{
	uint32_t size = get_rom_size();
	uint8_t *rom = get_rom_base();

	// determine start address
	// default to $4000
	m_start_address = 0x4000;

	switch (size)
	{
		/* 8KB/16KB */
		case 0x2000: case 0x4000:
		{
			uint16_t start = rom[3] << 8 | rom[2];

			// start address of $0000: call address in the $4000 region: $4000, else $8000
			if (start == 0)
			{
				if ((rom[5] & 0xc0) == 0x40)
					m_start_address = 0x4000;
				else
					m_start_address = 0x8000;
			}

			// start address in the $8000 region: $8000, else default
			else if ((start & 0xc000) == 0x8000)
				m_start_address = 0x8000;

			break;
		}

		/* 32KB */
		case 0x8000:
			// take default, check when no "AB" at $0000, but "AB" at $4000
			if (rom[0] != 'A' && rom[1] != 'B' && rom[0x4000] == 'A' && rom[0x4001] == 'B')
			{
				uint16_t start = rom[0x4003] << 8 | rom[0x4002];

				// start address of $0000 and call address in the $4000 region, or start address outside the $8000 region: $0000, else default
				if ((start == 0 && (rom[0x4005] & 0xc0) == 0x40) || start < 0x8000 || start >= 0xc000)
					m_start_address = 0;
			}

			break;

		/* 48KB */
		case 0xc000:
			// "AB" at $0000, but no "AB" at $4000, not "AB": $0000
			if (rom[0] == 'A' && rom[1] == 'B' && rom[0x4000] != 'A' && rom[0x4001] != 'B')
				m_start_address = 0x4000;
			else
				m_start_address = 0;

			break;

		/* 64KB */
		default:
			m_start_address = 0;
			break;
	}

	m_end_address = std::min<uint32_t>(m_start_address + size, 0x10000);
}

READ8_MEMBER(msx_cart_nomapper_device::read_cart)
{
	if ( offset >= m_start_address && offset < m_end_address )
	{
		return get_rom_base()[offset - m_start_address];
	}
	return 0xff;
}
