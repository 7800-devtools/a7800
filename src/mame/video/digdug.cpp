// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria
#include "emu.h"
#include "includes/galaga.h"
#include "includes/digdug.h"


/***************************************************************************

  Convert the color PROMs.

  digdug has one 32x8 palette PROM and two 256x4 color lookup table PROMs
  (one for characters, one for sprites).
  The palette PROM is connected to the RGB output this way:

  bit 7 -- 220 ohm resistor  -- BLUE
        -- 470 ohm resistor  -- BLUE
        -- 220 ohm resistor  -- GREEN
        -- 470 ohm resistor  -- GREEN
        -- 1  kohm resistor  -- GREEN
        -- 220 ohm resistor  -- RED
        -- 470 ohm resistor  -- RED
  bit 0 -- 1  kohm resistor  -- RED

***************************************************************************/

PALETTE_INIT_MEMBER(digdug_state,digdug)
{
	const uint8_t *color_prom = memregion("proms")->base();
	int i;

	for (i = 0;i < 32;i++)
	{
		int bit0,bit1,bit2,r,g,b;

		bit0 = (*color_prom >> 0) & 0x01;
		bit1 = (*color_prom >> 1) & 0x01;
		bit2 = (*color_prom >> 2) & 0x01;
		r = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;
		bit0 = (*color_prom >> 3) & 0x01;
		bit1 = (*color_prom >> 4) & 0x01;
		bit2 = (*color_prom >> 5) & 0x01;
		g = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;
		bit0 = 0;
		bit1 = (*color_prom >> 6) & 0x01;
		bit2 = (*color_prom >> 7) & 0x01;
		b = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;
		palette.set_indirect_color(i,rgb_t(r,g,b));
		color_prom++;
	}

	/* characters - direct mapping */
	for (i = 0; i < 16; i++)
	{
		palette.set_pen_indirect(i*2+0, 0);
		palette.set_pen_indirect(i*2+1, i);
	}

	/* sprites */
	for (i = 0;i < 0x100;i++)
		palette.set_pen_indirect(16*2+i, (*color_prom++ & 0x0f) + 0x10);

	/* bg_select */
	for (i = 0;i < 0x100;i++)
		palette.set_pen_indirect(16*2+256+i, *color_prom++ & 0x0f);
}



/***************************************************************************

  Callbacks for the TileMap code

***************************************************************************/

/* convert from 32x32 to 36x28 */
TILEMAP_MAPPER_MEMBER(digdug_state::tilemap_scan)
{
	int offs;

	row += 2;
	col -= 2;
	if (col & 0x20)
		offs = row + ((col & 0x1f) << 5);
	else
		offs = col + (row << 5);

	return offs;
}


TILE_GET_INFO_MEMBER(digdug_state::bg_get_tile_info)
{
	uint8_t *rom = memregion("gfx4")->base();

	int code = rom[tile_index | (m_bg_select << 10)];
	/* when the background is "disabled", it is actually still drawn, but using
	   a color code that makes all pixels black. There are pullups setting the
	   code to 0xf, but also solder pads that optionally connect the lines with
	   tilemap RAM, therefore allowing to pick some bits of the color code from
	   the top 4 bits of alpha code. This feature is not used by Dig Dug. */
	int color = m_bg_disable ? 0xf : (code >> 4);
	SET_TILE_INFO_MEMBER(2,
			code,
			color | m_bg_color_bank,
			0);
}

TILE_GET_INFO_MEMBER(digdug_state::tx_get_tile_info)
{
	uint8_t code = m_videoram[tile_index];
	int color;

	/* the hardware has two ways to pick the color, either straight from the
	   bottom 4 bits of the character code, or from the top 4 bits through a
	   formula. The former method isnot used by Dig Dug and seems kind of
	   useless (I don't know what use they were thinking of when they added
	   it), anyway here it is reproduced faithfully. */
	if (m_tx_color_mode)
		color = code & 0x0f;
	else
		color = ((code >> 4) & 0x0e) | ((code >> 3) & 2);

	/* the hardware has two character sets, one normal and one x-flipped. When
	   screen is flipped, character y flip is done by the hardware inverting the
	   timing signals, while x flip is done by selecting the 2nd character set.
	   We reproduce this here, but since the tilemap system automatically flips
	   characters when screen is flipped, we have to flip them back. */
	SET_TILE_INFO_MEMBER(0,
			(code & 0x7f) | (flip_screen() ? 0x80 : 0),
			color,
			flip_screen() ? TILE_FLIPX : 0);
}



/***************************************************************************

  Start the video hardware emulation.

***************************************************************************/

VIDEO_START_MEMBER(digdug_state,digdug)
{
	m_bg_select = 0;
	m_tx_color_mode = 0;
	m_bg_disable = 0;
	m_bg_color_bank = 0;

	m_bg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(digdug_state::bg_get_tile_info),this),tilemap_mapper_delegate(FUNC(digdug_state::tilemap_scan),this),8,8,36,28);
	m_fg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(digdug_state::tx_get_tile_info),this),tilemap_mapper_delegate(FUNC(digdug_state::tilemap_scan),this),8,8,36,28);

	m_fg_tilemap->set_transparent_pen(0);

	save_item(NAME(m_bg_select));
	save_item(NAME(m_tx_color_mode));
	save_item(NAME(m_bg_disable));
	save_item(NAME(m_bg_color_bank));
}



/***************************************************************************

  Memory handlers

***************************************************************************/

WRITE8_MEMBER( digdug_state::digdug_videoram_w )
{
	m_videoram[offset] = data;
	m_fg_tilemap->mark_tile_dirty(offset & 0x3ff);
}

WRITE8_MEMBER( digdug_state::digdug_PORT_w )
{
	switch (offset)
	{
		case 0: /* select background picture */
		case 1:
			{
				int shift = offset;
				int mask = 1 << shift;

				if ((m_bg_select & mask) != ((data & 1) << shift))
				{
					m_bg_select = (m_bg_select & ~mask) | ((data & 1) << shift);
					m_bg_tilemap->mark_all_dirty();
				}
			}
			break;

		case 2: /* select alpha layer color mode (see tx_get_tile_info) */
			if (m_tx_color_mode != (data & 1))
			{
				m_tx_color_mode = data & 1;
				m_fg_tilemap->mark_all_dirty();
			}
			break;

		case 3: /* "disable" background (see bg_get_tile_info) */
			if (m_bg_disable != (data & 1))
			{
				m_bg_disable = data & 1;
				m_bg_tilemap->mark_all_dirty();
			}
			break;

		case 4: /* background color bank */
		case 5:
			{
				int shift = offset;
				int mask = 1 << shift;

				if ((m_bg_color_bank & mask) != ((data & 1) << shift))
				{
					m_bg_color_bank = (m_bg_color_bank & ~mask) | ((data & 1) << shift);
					m_bg_tilemap->mark_all_dirty();
				}
			}
			break;

		case 6: /* n.c. */
			break;

		case 7: /* FLIP */
			flip_screen_set(data & 1);
			break;
	}
}



/***************************************************************************

  Display refresh

***************************************************************************/

void digdug_state::draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect )
{
	uint8_t *spriteram = m_digdug_objram + 0x380;
	uint8_t *spriteram_2 = m_digdug_posram + 0x380;
	uint8_t *spriteram_3 = m_digdug_flpram + 0x380;
	int offs;

	// mask upper and lower columns
	rectangle visarea = cliprect;
	visarea.min_x = 2*8;
	visarea.max_x = 34*8-1;

	for (offs = 0;offs < 0x80;offs += 2)
	{
		static const int gfx_offs[2][2] =
		{
			{ 0, 1 },
			{ 2, 3 }
		};
		int sprite = spriteram[offs];
		int color = spriteram[offs+1] & 0x3f;
		int sx = spriteram_2[offs+1] - 40+1;
		int sy = 256 - spriteram_2[offs] + 1;   // sprites are buffered and delayed by one scanline
		int flipx = (spriteram_3[offs] & 0x01);
		int flipy = (spriteram_3[offs] & 0x02) >> 1;
		int size  = (sprite & 0x80) >> 7;
		int x,y;

		if (size)
			sprite = (sprite & 0xc0) | ((sprite & ~0xc0) << 2);

		sy -= 16 * size;
		sy = (sy & 0xff) - 32;  // fix wraparound

		if (flip_screen())
		{
			flipx ^= 1;
			flipy ^= 1;
		}

		for (y = 0;y <= size;y++)
		{
			for (x = 0;x <= size;x++)
			{
				uint32_t transmask =  m_palette->transpen_mask(*m_gfxdecode->gfx(1), color, 0x1f);
				m_gfxdecode->gfx(1)->transmask(bitmap,visarea,
					sprite + gfx_offs[y ^ (size * flipy)][x ^ (size * flipx)],
					color,
					flipx,flipy,
					((sx + 16*x) & 0xff), sy + 16*y,transmask);
				/* wraparound */
				m_gfxdecode->gfx(1)->transmask(bitmap,visarea,
					sprite + gfx_offs[y ^ (size * flipy)][x ^ (size * flipx)],
					color,
					flipx,flipy,
					((sx + 16*x) & 0xff) + 0x100, sy + 16*y,transmask);
			}
		}
	}
}


uint32_t digdug_state::screen_update_digdug(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_bg_tilemap->draw(screen, bitmap, cliprect, 0,0);
	m_fg_tilemap->draw(screen, bitmap, cliprect, 0,0);
	draw_sprites(bitmap,cliprect);
	return 0;
}
