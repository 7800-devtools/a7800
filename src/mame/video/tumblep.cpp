// license:BSD-3-Clause
// copyright-holders:David Haywood,Bryan McPhail
/***************************************************************************

   Tumblepop Video emulation - Bryan McPhail, mish@tendril.co.uk

*********************************************************************

Uses Data East custom chip 55 for backgrounds, custom chip 52 for sprites.

See Dark Seal & Caveman Ninja drivers for info on these chips.

Tumblepop is one of few games to take advantage of the playfields ability
to switch between 8*8 tiles and 16*16 tiles.

***************************************************************************/

#include "emu.h"
#include "includes/tumblep.h"


uint32_t tumblep_state::screen_update_tumblep(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	address_space &space = machine().dummy_space();
	uint16_t flip = m_deco_tilegen1->pf_control_r(space, 0, 0xffff);

	flip_screen_set(BIT(flip, 7));
	m_deco_tilegen1->pf_update(m_pf1_rowscroll, m_pf2_rowscroll);

	bitmap.fill(256, cliprect); /* not verified */

	m_deco_tilegen1->tilemap_2_draw(screen, bitmap, cliprect, TILEMAP_DRAW_OPAQUE, 0);
	m_deco_tilegen1->tilemap_1_draw(screen, bitmap, cliprect, 0, 0);

	m_sprgen->draw_sprites(bitmap, cliprect, m_spriteram, 0x400);
	return 0;
}
