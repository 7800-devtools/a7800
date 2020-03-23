// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria
#ifndef MAME_INCLUDES_BOSCO_H
#define MAME_INCLUDES_BOSCO_H

#pragma once

#include "galaga.h"

class bosco_state : public galaga_state
{
public:
	bosco_state(const machine_config &mconfig, device_type type, const char *tag)
		: galaga_state(mconfig, type, tag)
		, m_bosco_radarattr(*this, "bosco_radarattr")
		, m_bosco_starcontrol(*this, "starcontrol")
		, m_bosco_starblink(*this, "bosco_starblink")
	{
	}

	required_shared_ptr<uint8_t> m_bosco_radarattr;

	required_shared_ptr<uint8_t> m_bosco_starcontrol;
	required_shared_ptr<uint8_t> m_bosco_starblink;

	uint8_t *m_bosco_radarx;
	uint8_t *m_bosco_radary;

	uint8_t *m_spriteram;
	uint8_t *m_spriteram2;
	uint32_t m_spriteram_size;
	DECLARE_WRITE8_MEMBER(bosco_flip_screen_w);
	TILEMAP_MAPPER_MEMBER(fg_tilemap_scan);
	TILE_GET_INFO_MEMBER(bg_get_tile_info);
	TILE_GET_INFO_MEMBER(fg_get_tile_info);
	DECLARE_VIDEO_START(bosco);
	DECLARE_PALETTE_INIT(bosco);
	uint32_t screen_update_bosco(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(screen_vblank_bosco);

	inline void get_tile_info_bosco(tile_data &tileinfo,int tile_index,int ram_offs);
	void draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect, int flip);
	void draw_bullets(bitmap_ind16 &bitmap, const rectangle &cliprect, int flip);
	void draw_stars(bitmap_ind16 &bitmap, const rectangle &cliprect, int flip);
	DECLARE_WRITE8_MEMBER( bosco_videoram_w );
	DECLARE_WRITE8_MEMBER( bosco_scrollx_w );
	DECLARE_WRITE8_MEMBER( bosco_scrolly_w );
	DECLARE_WRITE8_MEMBER( bosco_starclr_w );
};

#endif // MAME_INCLUDES_BOSCO_H
