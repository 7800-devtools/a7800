// license:BSD-3-Clause
// copyright-holders:Luca Elia
/***************************************************************************

                          -= Yun Sung 8 Bit Games =-

                    driver by   Luca Elia (l.elia@tin.it)


Note:   if MAME_DEBUG is defined, pressing Z with:

        Q       shows the background layer
        W       shows the foreground layer

        [ 2 Fixed Layers ]

            [ Background ]

            Layer Size:             512 x 256
            Tiles:                  8 x 8 x 8

            [ Foreground ]

            Layer Size:             512 x 256
            Tiles:                  8 x 8 x 4


        There are no sprites.

***************************************************************************/

#include "emu.h"
#include "includes/yunsung8.h"


/***************************************************************************

                                Memory Handlers

***************************************************************************/

WRITE8_MEMBER(yunsung8_state::videobank_w)
{
	m_videobank = data;
}


READ8_MEMBER(yunsung8_state::videoram_r)
{
	int bank;

	/*  Bit 1 of the bankswitching register controls the c000-c7ff
	    area (Palette). Bit 0 controls the c800-dfff area (Tiles) */

	if (offset < 0x0800)
		bank = m_videobank & 2;
	else
		bank = m_videobank & 1;

	if (bank)
		return m_bg_vram[offset];
	else
		return m_fg_vram[offset];
}


WRITE8_MEMBER(yunsung8_state::videoram_w)
{
	if (offset < 0x0800)        // c000-c7ff    Banked Palette RAM
	{
		int bank = m_videobank & 2;
		uint8_t *RAM;
		int color;

		if (bank)
			RAM = m_bg_vram;
		else
			RAM = m_fg_vram;

		RAM[offset] = data;
		color = RAM[offset & ~1] | (RAM[offset | 1] << 8);

		/* BBBBBGGGGGRRRRRx */
		m_palette->set_pen_color(offset / 2 + (bank ? 0x400 : 0), pal5bit(color >> 0), pal5bit(color >> 5), pal5bit(color >> 10));
	}
	else
	{
		int tile;
		int bank = m_videobank & 1;

		if (offset < 0x1000)
			tile = (offset - 0x0800);       // c800-cfff: Banked Color RAM
		else
			tile = (offset - 0x1000) / 2;   // d000-dfff: Banked Tiles RAM

		if (bank)
		{
			m_bg_vram[offset] = data;
			m_bg_tilemap->mark_tile_dirty(tile);
		}
		else
		{
			m_fg_vram[offset] = data;
			m_fg_tilemap->mark_tile_dirty(tile);
		}
	}
}


WRITE8_MEMBER(yunsung8_state::flipscreen_w)
{
	machine().tilemap().set_flip_all((data & 1) ? (TILEMAP_FLIPX | TILEMAP_FLIPY) : 0);
}


/***************************************************************************

                              [ Tiles Format ]

    Offset:

    Video RAM + 0000.b      Code (Low  Bits)
    Video RAM + 0001.b      Code (High Bits)

    Color RAM + 0000.b      Color


***************************************************************************/

/* Background */

#define DIM_NX_0            (0x40)
#define DIM_NY_0            (0x20)

TILE_GET_INFO_MEMBER(yunsung8_state::get_bg_tile_info)
{
	int code  =  m_bg_vram[0x1000 + tile_index * 2 + 0] + m_bg_vram[0x1000 + tile_index * 2 + 1] * 256;
	int color =  m_bg_vram[0x0800 + tile_index] & 0x07;

	SET_TILE_INFO_MEMBER(0,
			code,
			color,
			0);
}

/* Text Plane */

#define DIM_NX_1            (0x40)
#define DIM_NY_1            (0x20)

TILE_GET_INFO_MEMBER(yunsung8_state::get_fg_tile_info)
{
	int code  =  m_fg_vram[0x1000 + tile_index * 2 + 0] + m_fg_vram[0x1000 + tile_index * 2 + 1] * 256;
	int color =  m_fg_vram[0x0800 + tile_index] & 0x3f;

	SET_TILE_INFO_MEMBER(1,
			code,
			color,
			0);
}




/***************************************************************************


                            Video Hardware Init


***************************************************************************/

void yunsung8_state::video_start()
{
	m_bg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(yunsung8_state::get_bg_tile_info),this), TILEMAP_SCAN_ROWS, 8, 8, DIM_NX_0, DIM_NY_0 );
	m_fg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(yunsung8_state::get_fg_tile_info),this), TILEMAP_SCAN_ROWS, 8, 8, DIM_NX_1, DIM_NY_1 );

	m_fg_tilemap->set_transparent_pen(0);
}



/***************************************************************************


                                Screen Drawing


***************************************************************************/

uint32_t yunsung8_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	int layers_ctrl = (~m_layers_ctrl) >> 4;

#ifdef MAME_DEBUG
if (machine().input().code_pressed(KEYCODE_Z))
{
	int msk = 0;
	if (machine().input().code_pressed(KEYCODE_Q))  msk |= 1;
	if (machine().input().code_pressed(KEYCODE_W))  msk |= 2;
	if (msk != 0) layers_ctrl &= msk;
}
#endif

	if (layers_ctrl & 1)
		m_bg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	else
		bitmap.fill(0, cliprect);

	if (layers_ctrl & 2)
		m_fg_tilemap->draw(screen, bitmap, cliprect, 0, 0);

	return 0;
}
