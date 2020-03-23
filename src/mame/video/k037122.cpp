// license:BSD-3-Clause
// copyright-holders:Fabio Priuli,Acho A. Tang, R. Belmont
/*
Konami 037122
*/

#include "emu.h"
#include "k037122.h"
#include "konami_helper.h"
#include "screen.h"

#define VERBOSE 0
#define LOG(x) do { if (VERBOSE) logerror x; } while (0)

#define K037122_NUM_TILES       16384

DEFINE_DEVICE_TYPE(K037122, k037122_device, "k037122", "K037122 2D Tilemap")

k037122_device::k037122_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, K037122, tag, owner, clock),
	device_video_interface(mconfig, *this),
	device_gfx_interface(mconfig, *this, nullptr),
	m_tile_ram(nullptr),
	m_char_ram(nullptr),
	m_reg(nullptr),
	m_gfx_index(0)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void k037122_device::device_start()
{
	if (!palette().device().started())
		throw device_missing_dependencies();

	static const gfx_layout k037122_char_layout =
	{
	8, 8,
	K037122_NUM_TILES,
	8,
	{ 0,1,2,3,4,5,6,7 },
	{ 1*16, 0*16, 3*16, 2*16, 5*16, 4*16, 7*16, 6*16 },
	{ 0*128, 1*128, 2*128, 3*128, 4*128, 5*128, 6*128, 7*128 },
	8*128
	};

	m_char_ram = make_unique_clear<uint32_t[]>(0x200000 / 4);
	m_tile_ram = make_unique_clear<uint32_t[]>(0x20000 / 4);
	m_reg = make_unique_clear<uint32_t[]>(0x400 / 4);

	m_layer[0] = &machine().tilemap().create(*this, tilemap_get_info_delegate(FUNC(k037122_device::tile_info_layer0),this), TILEMAP_SCAN_ROWS, 8, 8, 256, 64);
	m_layer[1] = &machine().tilemap().create(*this, tilemap_get_info_delegate(FUNC(k037122_device::tile_info_layer1),this), TILEMAP_SCAN_ROWS, 8, 8, 128, 64);

	m_layer[0]->set_transparent_pen(0);
	m_layer[1]->set_transparent_pen(0);

	set_gfx(m_gfx_index,std::make_unique<gfx_element>(&palette(), k037122_char_layout, (uint8_t*)m_char_ram.get(), 0, palette().entries() / 16, 0));

	save_pointer(NAME(m_reg.get()), 0x400 / 4);
	save_pointer(NAME(m_char_ram.get()), 0x200000 / 4);
	save_pointer(NAME(m_tile_ram.get()), 0x20000 / 4);

}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void k037122_device::device_reset()
{
	memset(m_char_ram.get(), 0, 0x200000);
	memset(m_tile_ram.get(), 0, 0x20000);
	memset(m_reg.get(), 0, 0x400);
}

/*****************************************************************************
    DEVICE HANDLERS
*****************************************************************************/

TILE_GET_INFO_MEMBER(k037122_device::tile_info_layer0)
{
	uint32_t val = m_tile_ram[tile_index + (0x8000/4)];
	int color = (val >> 17) & 0x1f;
	int tile = val & 0x3fff;
	int flags = 0;

	if (val & 0x400000)
		flags |= TILE_FLIPX;
	if (val & 0x800000)
		flags |= TILE_FLIPY;

	SET_TILE_INFO_MEMBER(m_gfx_index, tile, color, flags);
}

TILE_GET_INFO_MEMBER(k037122_device::tile_info_layer1)
{
	uint32_t val = m_tile_ram[tile_index];
	int color = (val >> 17) & 0x1f;
	int tile = val & 0x3fff;
	int flags = 0;

	if (val & 0x400000)
		flags |= TILE_FLIPX;
	if (val & 0x800000)
		flags |= TILE_FLIPY;

	SET_TILE_INFO_MEMBER(m_gfx_index, tile, color, flags);
}


void k037122_device::tile_draw( screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect )
{
	const rectangle &visarea = m_screen->visible_area();

	if (m_reg[0xc] & 0x10000)
	{
		m_layer[1]->set_scrolldx(visarea.min_x, visarea.min_x);
		m_layer[1]->set_scrolldy(visarea.min_y, visarea.min_y);
		m_layer[1]->draw(screen, bitmap, cliprect, 0, 0);
	}
	else
	{
		m_layer[0]->set_scrolldx(visarea.min_x, visarea.min_x);
		m_layer[0]->set_scrolldy(visarea.min_y, visarea.min_y);
		m_layer[0]->draw(screen, bitmap, cliprect, 0, 0);
	}
}

void k037122_device::update_palette_color( uint32_t palette_base, int color )
{
	uint32_t data = m_tile_ram[(palette_base / 4) + color];

	palette().set_pen_color(color, pal5bit(data >> 6), pal6bit(data >> 0), pal5bit(data >> 11));
}

READ32_MEMBER( k037122_device::sram_r )
{
	return m_tile_ram[offset];
}

WRITE32_MEMBER( k037122_device::sram_w )
{
	COMBINE_DATA(m_tile_ram.get() + offset);

	if (m_reg[0xc] & 0x10000)
	{
		if (offset < 0x8000 / 4)
		{
			m_layer[1]->mark_tile_dirty(offset);
		}
		else if (offset >= 0x8000 / 4 && offset < 0x18000 / 4)
		{
			m_layer[0]->mark_tile_dirty(offset - (0x8000 / 4));
		}
		else if (offset >= 0x18000 / 4)
		{
			update_palette_color(0x18000, offset - (0x18000 / 4));
		}
	}
	else
	{
		if (offset < 0x8000 / 4)
		{
			update_palette_color(0, offset);
		}
		else if (offset >= 0x8000 / 4 && offset < 0x18000 / 4)
		{
			m_layer[0]->mark_tile_dirty(offset - (0x8000 / 4));
		}
		else if (offset >= 0x18000 / 4)
		{
			m_layer[1]->mark_tile_dirty(offset - (0x18000 / 4));
		}
	}
}


READ32_MEMBER( k037122_device::char_r )
{
	int bank = m_reg[0x30 / 4] & 0x7;

	return m_char_ram[offset + (bank * (0x40000 / 4))];
}

WRITE32_MEMBER( k037122_device::char_w )
{
	int bank = m_reg[0x30 / 4] & 0x7;
	uint32_t addr = offset + (bank * (0x40000/4));

	COMBINE_DATA(m_char_ram.get() + addr);
	gfx(m_gfx_index)->mark_dirty(addr / 32);
}

READ32_MEMBER( k037122_device::reg_r )
{
	switch (offset)
	{
		case 0x14/4:
		{
			return 0x000003fa;
		}
	}
	return m_reg[offset];
}

WRITE32_MEMBER( k037122_device::reg_w )
{
	COMBINE_DATA(m_reg.get() + offset);
}
