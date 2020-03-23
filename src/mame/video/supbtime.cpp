// license:BSD-3-Clause
// copyright-holders:Bryan McPhail
/***************************************************************************

   Super Burger Time Video emulation - Bryan McPhail, mish@tendril.co.uk

*********************************************************************

Uses Data East custom chip 55 for backgrounds, custom chip 52 for sprites.

See Dark Seal & Caveman Ninja drivers for info on these chips.

End sequence uses rowscroll '98 c0' on pf1 (jmp to 1d61a on supbtimj)

***************************************************************************/

#include "emu.h"
#include "includes/supbtime.h"


/******************************************************************************/

/******************************************************************************/

uint32_t supbtime_state::screen_update_supbtime(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	address_space &space = machine().dummy_space();
	uint16_t flip = m_deco_tilegen1->pf_control_r(space, 0, 0xffff);

	flip_screen_set(BIT(flip, 7));
	m_deco_tilegen1->pf_update(m_pf1_rowscroll, m_pf2_rowscroll);

	bitmap.fill(768, cliprect);

	m_deco_tilegen1->tilemap_2_draw(screen, bitmap, cliprect, 0, 0);
	m_sprgen->draw_sprites(bitmap, cliprect, m_spriteram, 0x400);
	m_deco_tilegen1->tilemap_1_draw(screen, bitmap, cliprect, 0, 0);
	return 0;
}
