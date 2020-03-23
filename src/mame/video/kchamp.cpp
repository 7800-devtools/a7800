// license:BSD-3-Clause
// copyright-holders:Ernesto Corvi
/***************************************************************************

  video.c

  Functions to emulate the video hardware of the machine.

***************************************************************************/

#include "emu.h"
#include "includes/kchamp.h"


PALETTE_INIT_MEMBER(kchamp_state, kchamp)
{
	const uint8_t *color_prom = memregion("proms")->base();
	int i, red, green, blue;

	for (i = 0; i < palette.entries(); i++)
	{
		red = color_prom[i];
		green = color_prom[palette.entries() + i];
		blue = color_prom[2 * palette.entries() + i];

		palette.set_pen_color(i, pal4bit(red), pal4bit(green), pal4bit(blue));
	}
}

WRITE8_MEMBER(kchamp_state::kchamp_videoram_w)
{
	m_videoram[offset] = data;
	m_bg_tilemap->mark_tile_dirty(offset);
}

WRITE8_MEMBER(kchamp_state::kchamp_colorram_w)
{
	m_colorram[offset] = data;
	m_bg_tilemap->mark_tile_dirty(offset);
}

WRITE8_MEMBER(kchamp_state::kchamp_flipscreen_w)
{
	flip_screen_set(data & 0x01);
}

TILE_GET_INFO_MEMBER(kchamp_state::get_bg_tile_info)
{
	int code = m_videoram[tile_index] + ((m_colorram[tile_index] & 7) << 8);
	int color = (m_colorram[tile_index] >> 3) & 0x1f;

	SET_TILE_INFO_MEMBER(0, code, color, 0);
}

void kchamp_state::video_start()
{
	m_bg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(kchamp_state::get_bg_tile_info),this), TILEMAP_SCAN_ROWS, 8, 8, 32, 32);
}

/*
        Sprites
        -------
        Offset        Encoding
             0        YYYYYYYY
             1        TTTTTTTT
             2        FGGTCCCC
             3        XXXXXXXX
*/

void kchamp_state::kchamp_draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect )
{
	uint8_t *spriteram = m_spriteram;
	int offs;

	for (offs = 0; offs < 0x100; offs += 4)
	{
		int attr = spriteram[offs + 2];
		int bank = 1 + ((attr & 0x60) >> 5);
		int code = spriteram[offs + 1] + ((attr & 0x10) << 4);
		int color = attr & 0x0f;
		int flipx = 0;
		int flipy = attr & 0x80;
		int sx = spriteram[offs + 3] - 8;
		int sy = 247 - spriteram[offs];

		if (flip_screen())
		{
			sx = 240 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}

		m_gfxdecode->gfx(bank)->transpen(bitmap,cliprect, code, color, flipx, flipy, sx, sy, 0);
	}
}

void kchamp_state::kchampvs_draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect )
{
	uint8_t *spriteram = m_spriteram;
	int offs;

	for (offs = 0; offs < 0x100; offs += 4)
	{
		int attr = spriteram[offs + 2];
		int bank = 1 + ((attr & 0x60) >> 5);
		int code = spriteram[offs + 1] + ((attr & 0x10) << 4);
		int color = attr & 0x0f;
		int flipx = 0;
		int flipy = attr & 0x80;
		int sx = spriteram[offs + 3];
		int sy = 240 - spriteram[offs];

		if (flip_screen())
		{
			sx = 240 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}

		m_gfxdecode->gfx(bank)->transpen(bitmap,cliprect, code, color, flipx, flipy, sx, sy, 0);
	}
}


uint32_t kchamp_state::screen_update_kchamp(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_bg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	kchamp_draw_sprites(bitmap, cliprect);
	return 0;
}

uint32_t kchamp_state::screen_update_kchampvs(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_bg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	kchampvs_draw_sprites(bitmap, cliprect);
	return 0;
}
