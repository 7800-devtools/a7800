// license:BSD-3-Clause
// copyright-holders:Brad Oliver

/***************************************************************************

 Espial hardware games (drivers: espial.cpp)

***************************************************************************/

#include "machine/gen_latch.h"

class espial_state : public driver_device
{
public:
	espial_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_videoram(*this, "videoram"),
		m_attributeram(*this, "attributeram"),
		m_scrollram(*this, "scrollram"),
		m_spriteram_1(*this, "spriteram_1"),
		m_spriteram_2(*this, "spriteram_2"),
		m_spriteram_3(*this, "spriteram_3"),
		m_colorram(*this, "colorram"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch") { }

	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_attributeram;
	required_shared_ptr<uint8_t> m_scrollram;
	required_shared_ptr<uint8_t> m_spriteram_1;
	required_shared_ptr<uint8_t> m_spriteram_2;
	required_shared_ptr<uint8_t> m_spriteram_3;
	required_shared_ptr<uint8_t> m_colorram;

	/* video-related */
	tilemap_t   *m_bg_tilemap;
	tilemap_t   *m_fg_tilemap;
	int       m_flipscreen;

	/* sound-related */
	uint8_t     m_main_nmi_enabled;
	uint8_t     m_sound_nmi_enabled;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_WRITE8_MEMBER(espial_master_interrupt_mask_w);
	DECLARE_WRITE8_MEMBER(espial_master_soundlatch_w);
	DECLARE_WRITE8_MEMBER(espial_sound_nmi_mask_w);
	DECLARE_WRITE8_MEMBER(espial_videoram_w);
	DECLARE_WRITE8_MEMBER(espial_colorram_w);
	DECLARE_WRITE8_MEMBER(espial_attributeram_w);
	DECLARE_WRITE8_MEMBER(espial_scrollram_w);
	DECLARE_WRITE8_MEMBER(espial_flipscreen_w);
	TILE_GET_INFO_MEMBER(get_tile_info);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	DECLARE_PALETTE_INIT(espial);
	DECLARE_VIDEO_START(netwars);
	uint32_t screen_update_espial(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(espial_sound_nmi_gen);
	TIMER_DEVICE_CALLBACK_MEMBER(espial_scanline);
	void draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
};
