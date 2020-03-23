// license:BSD-3-Clause
// copyright-holders:David Haywood
/*************************************************************************

    Super Slams

*************************************************************************/
#ifndef MAME_INCLUDES_SUPRSLAM_H
#define MAME_INCLUDES_SUPRSLAM_H

#include "video/vsystem_spr.h"
#include "machine/gen_latch.h"
#include "video/k053936.h"

class suprslam_state : public driver_device
{
public:
	suprslam_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_screen_videoram(*this, "screen_videoram"),
		m_bg_videoram(*this, "bg_videoram"),
		m_sp_videoram(*this, "sp_videoram"),
		m_spriteram(*this, "spriteram"),
		m_screen_vregs(*this, "screen_vregs"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_k053936(*this, "k053936"),
		m_spr(*this, "vsystem_spr"),
		m_palette(*this, "palette"),
		m_gfxdecode(*this, "gfxdecode"),
		m_soundlatch(*this, "soundlatch") { }

	/* memory pointers */
	required_shared_ptr<uint16_t> m_screen_videoram;
	required_shared_ptr<uint16_t> m_bg_videoram;
	required_shared_ptr<uint16_t> m_sp_videoram;
	required_shared_ptr<uint16_t> m_spriteram;
	required_shared_ptr<uint16_t> m_screen_vregs;

	/* video-related */
	tilemap_t     *m_screen_tilemap;
	tilemap_t     *m_bg_tilemap;
	uint16_t      m_screen_bank;
	uint16_t      m_bg_bank;
	uint32_t  suprslam_tile_callback( uint32_t code );
	uint8_t       m_spr_ctrl;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<k053936_device> m_k053936;
	required_device<vsystem_spr_device> m_spr;
	required_device<palette_device> m_palette;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_WRITE8_MEMBER(suprslam_sh_bankswitch_w);
	DECLARE_WRITE16_MEMBER(suprslam_screen_videoram_w);
	DECLARE_WRITE16_MEMBER(suprslam_bg_videoram_w);
	DECLARE_WRITE16_MEMBER(suprslam_bank_w);
	DECLARE_WRITE8_MEMBER(spr_ctrl_w);
	TILE_GET_INFO_MEMBER(get_suprslam_tile_info);
	TILE_GET_INFO_MEMBER(get_suprslam_bg_tile_info);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_suprslam(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
};

#endif // MAME_INCLUDES_SUPRSLAM_H
