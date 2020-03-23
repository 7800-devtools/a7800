// license:BSD-3-Clause
// copyright-holders:David Haywood
#include "emu.h"
#include "k001006.h"

/*****************************************************************************/
/* Konami K001006 Texel Unit (KS10081) */

/***************************************************************************/
/*                                                                         */
/*                                  001006                                 */
/*                                                                         */
/***************************************************************************/

DEFINE_DEVICE_TYPE(K001006, k001006_device, "k001006", "K001006 Texel Unit")

k001006_device::k001006_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, K001006, tag, owner, clock),
	m_pal_ram(nullptr),
	m_unknown_ram(nullptr),
	m_addr(0),
	m_device_sel(0),
	m_palette(nullptr), m_gfx_region(nullptr), m_gfxrom(nullptr),
	m_tex_layout(0)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void k001006_device::device_start()
{
	m_pal_ram = make_unique_clear<uint16_t[]>(0x800);
	m_unknown_ram = make_unique_clear<uint16_t[]>(0x1000);
	m_palette = make_unique_clear<uint32_t[]>(0x800);

	m_gfxrom = machine().root_device().memregion(m_gfx_region)->base();
	m_texrom = std::make_unique<uint8_t[]>(0x800000);

	preprocess_texture_data(m_texrom.get(), m_gfxrom, 0x800000, m_tex_layout);

	save_pointer(NAME(m_pal_ram.get()), 0x800*sizeof(uint16_t));
	save_pointer(NAME(m_unknown_ram.get()), 0x1000*sizeof(uint16_t));
	save_pointer(NAME(m_palette.get()), 0x800*sizeof(uint32_t));
	save_item(NAME(m_device_sel));
	save_item(NAME(m_addr));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void k001006_device::device_reset()
{
	m_addr = 0;
	m_device_sel = 0;
	memset(m_palette.get(), 0, 0x800*sizeof(uint32_t));
}

/*****************************************************************************
    DEVICE HANDLERS
*****************************************************************************/

READ32_MEMBER( k001006_device::read )
{
	if (offset == 1)
	{
		switch (m_device_sel)
		{
			case 0x0b:      // CG Board ROM read
			{
				uint16_t *rom = (uint16_t*)space.machine().root_device().memregion(m_gfx_region)->base();
				return rom[m_addr / 2] << 16;
			}
			case 0x0d:      // Palette RAM read
			{
				uint32_t addr = m_addr;

				m_addr += 2;
				return m_pal_ram[addr >> 1];
			}
			case 0x0f:      // Unknown RAM read
			{
				return m_unknown_ram[m_addr++];
			}
			default:
			{
				fatalerror("k001006_r, unknown device %02X\n", m_device_sel);
			}
		}
	}
	return 0;
}

WRITE32_MEMBER( k001006_device::write )
{
	if (offset == 0)
	{
		COMBINE_DATA(&m_addr);
	}
	else if (offset == 1)
	{
		switch (m_device_sel)
		{
			case 0xd:   // Palette RAM write
			{
				int r, g, b, a;
				uint32_t index = m_addr;

				m_pal_ram[index >> 1] = data & 0xffff;

				a = (data & 0x8000) ? 0x00 : 0xff;
				b = ((data >> 10) & 0x1f) << 3;
				g = ((data >>  5) & 0x1f) << 3;
				r = ((data >>  0) & 0x1f) << 3;
				b |= (b >> 5);
				g |= (g >> 5);
				r |= (r >> 5);
				m_palette[index >> 1] = rgb_t(a, r, g, b);

				m_addr += 2;
				break;
			}
			case 0xf:   // Unknown RAM write
			{
			//  osd_printf_debug("Unknown RAM %08X = %04X\n", m_addr, data & 0xffff);
				m_unknown_ram[m_addr++] = data & 0xffff;
				break;
			}
			default:
			{
				osd_printf_debug("k001006_w: device %02X, write %04X to %08X\n", m_device_sel, data & 0xffff, m_addr++);
			}
		}
	}
	else if (offset == 2)
	{
		if (ACCESSING_BITS_16_31)
		{
			m_device_sel = (data >> 16) & 0xf;
		}
	}
}

uint32_t k001006_device::fetch_texel(int page, int pal_index, int u, int v)
{
	uint8_t *tex = m_texrom.get() + page;
	int texel = tex[((v & 0x1ff) * 512) + (u & 0x1ff)];
	return m_palette[pal_index + texel];
}


void k001006_device::preprocess_texture_data(uint8_t *dst, uint8_t *src, int length, int layout)
{
	static const int decode_x_gti[8] = {  0, 16, 2, 18, 4, 20, 6, 22 };
	static const int decode_y_gti[16] = {  0, 8, 32, 40, 1, 9, 33, 41, 64, 72, 96, 104, 65, 73, 97, 105 };

	static const int decode_x_zr107[8] = {  0, 16, 1, 17, 2, 18, 3, 19 };
	static const int decode_y_zr107[16] = {  0, 8, 32, 40, 4, 12, 36, 44, 64, 72, 96, 104, 68, 76, 100, 108 };

	int index;
	int i, x, y;
	uint8_t temp[0x40000];

	const int *decode_x;
	const int *decode_y;

	if (layout == 1)
	{
		decode_x = decode_x_gti;
		decode_y = decode_y_gti;
	}
	else
	{
		decode_x = decode_x_zr107;
		decode_y = decode_y_zr107;
	}

	for (index=0; index < length; index += 0x40000)
	{
		int offset = index;

		memset(temp, 0, 0x40000);

		for (i=0; i < 0x800; i++)
		{
			int tx = ((i & 0x400) >> 5) | ((i & 0x100) >> 4) | ((i & 0x40) >> 3) | ((i & 0x10) >> 2) | ((i & 0x4) >> 1) | (i & 0x1);
			int ty = ((i & 0x200) >> 5) | ((i & 0x80) >> 4) | ((i & 0x20) >> 3) | ((i & 0x8) >> 2) | ((i & 0x2) >> 1);

			tx <<= 3;
			ty <<= 4;

			for (y=0; y < 16; y++)
			{
				for (x=0; x < 8; x++)
				{
					uint8_t pixel = src[offset + decode_y[y] + decode_x[x]];

					temp[((ty+y) * 512) + (tx+x)] = pixel;
				}
			}

			offset += 128;
		}

		memcpy(&dst[index], temp, 0x40000);
	}
}
