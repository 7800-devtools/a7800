// license:BSD-3-Clause
// copyright-holders:Eugene Sandulenko
/***************************************************************************

  TIA-MC1 video hardware

  driver by Eugene Sandulenko
  special thanks to Shiru for his standalone emulator and documentation

***************************************************************************/

#include "emu.h"
#include "includes/tiamc1.h"


WRITE8_MEMBER(tiamc1_state::tiamc1_videoram_w)
{
	if(!(m_layers_ctrl & 2))
		m_charram[offset + 0x0000] = data;
	if(!(m_layers_ctrl & 4))
		m_charram[offset + 0x0800] = data;
	if(!(m_layers_ctrl & 8))
		m_charram[offset + 0x1000] = data;
	if(!(m_layers_ctrl & 16))
		m_charram[offset + 0x1800] = data;

	if ((m_layers_ctrl & (16|8|4|2)) != (16|8|4|2))
		m_gfxdecode->gfx(0)->mark_dirty((offset / 8) & 0xff);

	if(!(m_layers_ctrl & 1)) {
		m_tileram[offset] = data;
		if (offset < 1024)
			m_bg_tilemap1->mark_tile_dirty(offset & 0x3ff);
		else
			m_bg_tilemap2->mark_tile_dirty(offset & 0x3ff);
	}
}

WRITE8_MEMBER(tiamc1_state::kot_videoram_w)
{
	if ((m_layers_ctrl & 1) == 0)
	{
		m_tileram[offset] = data;
		m_bg_tilemap1->mark_tile_dirty(offset);
	}
}

WRITE8_MEMBER(tiamc1_state::tiamc1_bankswitch_w)
{
	if ((data & 128) != (m_layers_ctrl & 128))
		machine().tilemap().mark_all_dirty();

	m_layers_ctrl = data;
}

WRITE8_MEMBER(tiamc1_state::kot_bankswitch_w)
{
	m_gfxdecode->gfx(0)->set_source(m_charram + (data >> 1) * 0x100);
	m_layers_ctrl = data;
}

WRITE8_MEMBER(tiamc1_state::tiamc1_sprite_x_w)
{
	m_spriteram_x[offset] = data;
}

WRITE8_MEMBER(tiamc1_state::tiamc1_sprite_y_w)
{
	m_spriteram_y[offset] = data;
}

WRITE8_MEMBER(tiamc1_state::tiamc1_sprite_a_w)
{
	m_spriteram_a[offset] = data;
}

WRITE8_MEMBER(tiamc1_state::tiamc1_sprite_n_w)
{
	m_spriteram_n[offset] = data;
}

WRITE8_MEMBER(tiamc1_state::tiamc1_bg_vshift_w)
{
	m_bg_vshift = data;
}

WRITE8_MEMBER(tiamc1_state::tiamc1_bg_hshift_w)
{
	m_bg_hshift = data;
}

WRITE8_MEMBER(tiamc1_state::tiamc1_bg_bplctrl_w)
{
	m_bg_bplctrl = data;
	update_bg_palette();
}

WRITE8_MEMBER(tiamc1_state::tiamc1_palette_w)
{
	m_paletteram[offset] = data;
	m_palette->set_pen_color(offset, m_palette_ptr[data]);
	update_bg_palette();
}

void tiamc1_state::update_bg_palette()
{
	uint8_t bplmask = ((m_bg_bplctrl >> 0) & 1) | ((m_bg_bplctrl >> 1) & 2) | ((m_bg_bplctrl >> 2) & 4) | ((m_bg_bplctrl >> 3) & 8);
	for (int i = 0; i < 16; i++)
		m_palette->set_pen_color(i + 16, m_palette_ptr[m_paletteram[i | bplmask]]);
}

PALETTE_INIT_MEMBER(tiamc1_state, tiamc1)
{
	// Voltage computed by Proteus
	//static const float g_v[8]={1.05f,0.87f,0.81f,0.62f,0.44f,0.25f,0.19f,0.00f};
	//static const float r_v[8]={1.37f,1.13f,1.00f,0.75f,0.63f,0.38f,0.25f,0.00f};
	//static const float b_v[4]={1.16f,0.75f,0.42f,0.00f};

	// Voltage adjusted by Shiru
	static const float g_v[8] = { 1.2071f,0.9971f,0.9259f,0.7159f,0.4912f,0.2812f,0.2100f,0.0000f};
	static const float r_v[8] = { 1.5937f,1.3125f,1.1562f,0.8750f,0.7187f,0.4375f,0.2812f,0.0000f};
	static const float b_v[4] = { 1.3523f,0.8750f,0.4773f,0.0000f};

	int col;
	int r, g, b, ir, ig, ib;
	float tcol;

	m_palette_ptr = std::make_unique<rgb_t[]>(256);

	for (col = 0; col < 256; col++) {
		ir = (col >> 3) & 7;
		ig = col & 7;
		ib = (col >> 6) & 3;
		tcol = 255.0f * r_v[ir] / r_v[0];
		r = 255 - (((int)tcol) & 255);
		tcol = 255.0f * g_v[ig] / g_v[0];
		g = 255 - (((int)tcol) & 255);
		tcol = 255.0f * b_v[ib] / b_v[0];
		b = 255 - (((int)tcol) & 255);

		m_palette_ptr[col] = rgb_t(r,g,b);
	}
}

TILE_GET_INFO_MEMBER(tiamc1_state::get_bg1_tile_info)
{
	SET_TILE_INFO_MEMBER(0, m_tileram[tile_index], 0, 0);
}

TILE_GET_INFO_MEMBER(tiamc1_state::get_bg2_tile_info)
{
	SET_TILE_INFO_MEMBER(0, m_tileram[tile_index + 1024], 0, 0);
}

void tiamc1_state::video_start()
{
	m_videoram = make_unique_clear<uint8_t[]>(0x3050);

		m_charram = m_videoram.get() + 0x0800;     /* Ram is banked */
		m_tileram = m_videoram.get() + 0x0000;

	m_spriteram_y = m_videoram.get() + 0x3000;
	m_spriteram_x = m_videoram.get() + 0x3010;
	m_spriteram_n = m_videoram.get() + 0x3020;
	m_spriteram_a = m_videoram.get() + 0x3030;
	m_paletteram  = m_videoram.get() + 0x3040;

	save_pointer(NAME(m_videoram.get()), 0x3050);

	m_bg_tilemap1 = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(tiamc1_state::get_bg1_tile_info),this), TILEMAP_SCAN_ROWS,
			8, 8, 32, 32);

	m_bg_tilemap2 = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(tiamc1_state::get_bg2_tile_info),this), TILEMAP_SCAN_ROWS,
			8, 8, 32, 32);
	m_bg_tilemap1->set_scrolldx(4, 4);
	m_bg_tilemap2->set_scrolldx(4, 4);
	m_bg_tilemap1->set_palette_offset(16);
	m_bg_tilemap2->set_palette_offset(16);

	m_bg_vshift = 0;
	m_bg_hshift = 0;
	m_bg_bplctrl = 0;

	save_item(NAME(m_layers_ctrl));
	save_item(NAME(m_bg_vshift));
	save_item(NAME(m_bg_hshift));

	m_gfxdecode->gfx(0)->set_source(m_charram);
}

VIDEO_START_MEMBER(tiamc1_state, kot)
{
	m_charram = memregion("gfx2")->base();

	m_videoram    = make_unique_clear<uint8_t[]>(0x450);
	m_tileram     = m_videoram.get() + 0x000;
	m_spriteram_y = m_videoram.get() + 0x400;
	m_spriteram_x = m_videoram.get() + 0x410;
	m_spriteram_n = m_videoram.get() + 0x420;
	m_spriteram_a = m_videoram.get() + 0x430;
	m_paletteram  = m_videoram.get() + 0x440;

	save_pointer(NAME(m_videoram.get()), 0x450);

	m_bg_tilemap1 = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(tiamc1_state::get_bg1_tile_info), this), TILEMAP_SCAN_ROWS,
		8, 8, 32, 32);

	m_bg_tilemap1->set_scrolldx(4, 4);

	m_bg_vshift = 0;
	m_bg_hshift = 0;

	save_item(NAME(m_layers_ctrl));
	save_item(NAME(m_bg_vshift));
	save_item(NAME(m_bg_hshift));

	m_gfxdecode->gfx(0)->set_source(m_charram);
}
void tiamc1_state::draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	int offs;

	for (offs = 0; offs < 16; offs++)
	{
		int flipx, flipy, sx, sy, spritecode;

		sx = m_spriteram_x[offs] ^ 0xff;
		sy = m_spriteram_y[offs] ^ 0xff;
		flipx = !(m_spriteram_a[offs] & 0x08);
		flipy = !(m_spriteram_a[offs] & 0x02);
		spritecode = m_spriteram_n[offs] ^ 0xff;

		if (!(m_spriteram_a[offs] & 0x01))
			m_gfxdecode->gfx(1)->transpen(bitmap,cliprect,
				spritecode,
				0,
				flipx, flipy,
				sx, sy, 15);
	}
}

uint32_t tiamc1_state::screen_update_tiamc1(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	int i;

	for (i = 0; i < 32; i++)
	{
		m_bg_tilemap1->set_scrolly(i, m_bg_vshift);
		m_bg_tilemap2->set_scrolly(i, m_bg_vshift);
	}

	for (i = 0; i < 32; i++)
	{
		m_bg_tilemap1->set_scrollx(i, m_bg_hshift);
		m_bg_tilemap2->set_scrollx(i, m_bg_hshift);
	}

	if (m_layers_ctrl & 0x80)
		m_bg_tilemap2->draw(screen, bitmap, cliprect, 0, 0);
	else
		m_bg_tilemap1->draw(screen, bitmap, cliprect, 0, 0);


	draw_sprites(bitmap, cliprect);

	return 0;
}

uint32_t tiamc1_state::screen_update_kot(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	for (int i = 0; i < 32; i++)
		m_bg_tilemap1->set_scrolly(i, m_bg_vshift);

	for (int i = 0; i < 32; i++)
		m_bg_tilemap1->set_scrollx(i, m_bg_hshift);

	m_bg_tilemap1->draw(screen, bitmap, cliprect, 0, 0);

	draw_sprites(bitmap, cliprect);

	return 0;
}
