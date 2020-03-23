// license:BSD-3-Clause
// copyright-holders:Manuel Abadia
#include "emu.h"
#include "includes/aliens.h"

/***************************************************************************

  Callbacks for the K052109

***************************************************************************/

K052109_CB_MEMBER(aliens_state::tile_callback)
{
	*code |= ((*color & 0x3f) << 8) | (bank << 14);
	*color = layer * 4 + ((*color & 0xc0) >> 6);
}


/***************************************************************************

  Callbacks for the K051960

***************************************************************************/

K051960_CB_MEMBER(aliens_state::sprite_callback)
{
	enum { sprite_colorbase = 256 / 16 };

	/* The PROM allows for mixed priorities, where sprites would have */
	/* priority over text but not on one or both of the other two planes. */
	switch (*color & 0x70)
	{
		case 0x10: *priority = 0x00; break;            /* over ABF */
		case 0x00: *priority = GFX_PMASK_4          ; break;  /* over AB, not F */
		case 0x40: *priority = GFX_PMASK_4|GFX_PMASK_2     ; break;  /* over A, not BF */
		case 0x20:
		case 0x60: *priority = GFX_PMASK_4|GFX_PMASK_2|GFX_PMASK_1; break;  /* over -, not ABF */
		case 0x50: *priority = GFX_PMASK_2     ; break;  /* over AF, not B */
		case 0x30:
		case 0x70: *priority = GFX_PMASK_2|GFX_PMASK_1; break;  /* over F, not AB */
	}
	*code |= (*color & 0x80) << 6;
	*color = sprite_colorbase + (*color & 0x0f);
	*shadow = 0;    /* shadows are not used by this game */
}


/***************************************************************************

  Display refresh

***************************************************************************/

uint32_t aliens_state::screen_update_aliens(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_k052109->tilemap_update();

	screen.priority().fill(0, cliprect);
	/* The background color is always from layer 1 */
	m_k052109->tilemap_draw(screen, bitmap, cliprect, 1, TILEMAP_DRAW_OPAQUE, 0);

	m_k052109->tilemap_draw(screen, bitmap, cliprect, 1, 0, 1);
	m_k052109->tilemap_draw(screen, bitmap, cliprect, 2, 0, 2);
	m_k052109->tilemap_draw(screen, bitmap, cliprect, 0, 0, 4);

	m_k051960->k051960_sprites_draw(bitmap, cliprect, screen.priority(), -1, -1);
	return 0;
}
