// license:BSD-3-Clause
// copyright-holders:Uki
/******************************************************************************

Himeshikibu (C) 1989 Hi-Soft

Video hardware
    driver by Uki

******************************************************************************/

#include "emu.h"
#include "includes/himesiki.h"

TILE_GET_INFO_MEMBER(himesiki_state::get_bg_tile_info)
{
	int code = m_bg_ram[tile_index * 2] + m_bg_ram[tile_index * 2 + 1] * 0x100 ;
	int col = code >> 12;

	code &= 0xfff;

	SET_TILE_INFO_MEMBER(0, code, col, 0);
}

void himesiki_state::video_start()
{
	m_bg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(himesiki_state::get_bg_tile_info),this), TILEMAP_SCAN_ROWS, 8, 8, 64, 32);
}

WRITE8_MEMBER(himesiki_state::himesiki_bg_ram_w)
{
	m_bg_ram[offset] = data;
	m_bg_tilemap->mark_tile_dirty(offset / 2);
}

WRITE8_MEMBER(himesiki_state::himesiki_scrollx_w)
{
	m_scrollx[offset] = data;
}

WRITE8_MEMBER(himesiki_state::himesiki_scrolly_w)
{
	m_scrolly = data;
}

void himesiki_state::himesiki_draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect )
{
	uint8_t *spriteram;
	int offs;

	// these sprites are from the ET-P103A board (himesiki only)
	spriteram = m_spriteram_p103a;
	for (offs = 0x00; offs < 0x60; offs += 4)
	{
		int attr = spriteram[offs + 1];
		int code = spriteram[offs + 0] | (attr & 3) << 8;
		int x = spriteram[offs + 3] | (attr & 8) << 5;
		int y = spriteram[offs + 2];

		int col = (attr & 0xf0) >> 4;
		int fx = attr & 4;
		int fy = 0;

		if (x > 0x1e0)
			x -= 0x200;

		if (m_flipscreen)
		{
			y = (y - 31) & 0xff;
			x = 224 - x;
			fx ^= 4;
			fy = 1;
		}
		else
		{
			y = 257 - y;
			if (y > 0xc0)
				y -= 0x100;
		}

		m_gfxdecode->gfx(2)->transpen(bitmap,cliprect, code, col, fx, fy, x, y, 15);
	}

	// 0xc0 - 0xff unused
	spriteram = m_spriteram;
	for (offs = 0; offs < 0x100; offs += 4)
	{
		// not sure about this, but you sometimes get a garbage sprite in the corner otherwise.
		if ((spriteram[offs + 0] == 0x00) &&
			(spriteram[offs + 1] == 0x00) &&
			(spriteram[offs + 2] == 0x00) &&
			(spriteram[offs + 3] == 0x00))
				continue;

		int attr = spriteram[offs + 1];
		int code = spriteram[offs + 0] | (attr & 7) << 8;
		int x = spriteram[offs + 3] | (attr & 8) << 5;
		int y = spriteram[offs + 2];

		int col = (attr & 0xf0) >> 4;
		int f = 0;

		if (x > 0x1e0)
			x -= 0x200;

		if (m_flipscreen)
		{
			y = (y - 15) &0xff;
			x = 240 - x;
			f = 1;
		}
		else
			y = 257 - y;

		y &= 0xff;
		if (y > 0xf0)
			y -= 0x100;

		m_gfxdecode->gfx(1)->transpen(bitmap,cliprect, code, col, f, f, x, y, 15);
	}
}

uint32_t himesiki_state::screen_update_himesiki(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	int x = -(m_scrollx[0] << 8 | m_scrollx[1]) & 0x1ff;
	m_bg_tilemap->set_scrolldx(x, x);
	m_bg_tilemap->set_scrolldy(-m_scrolly, -m_scrolly-64);

	m_bg_tilemap->draw(screen, bitmap, cliprect, TILEMAP_DRAW_OPAQUE, 0);
	himesiki_draw_sprites(bitmap, cliprect);

	return 0;
}
