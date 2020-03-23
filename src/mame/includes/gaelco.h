// license:BSD-3-Clause
// copyright-holders:Manuel Abadia
/***************************************************************************

    Gaelco game hardware from 1991-1996

***************************************************************************/

#include "machine/gen_latch.h"

class gaelco_state : public driver_device
{
public:
	gaelco_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_audiocpu(*this, "audiocpu"),
		m_soundlatch(*this, "soundlatch"),
		m_videoram(*this, "videoram"),
		m_vregs(*this, "vregs"),
		m_spriteram(*this, "spriteram"),
		m_screenram(*this, "screenram") { }

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	optional_device<cpu_device> m_audiocpu;
	optional_device<generic_latch_8_device> m_soundlatch;

	/* memory pointers */
	required_shared_ptr<uint16_t> m_videoram;
	required_shared_ptr<uint16_t> m_vregs;
	required_shared_ptr<uint16_t> m_spriteram;
	optional_shared_ptr<uint16_t> m_screenram;

	/* video-related */
	tilemap_t      *m_tilemap[2];

	DECLARE_WRITE8_MEMBER(bigkarnk_sound_command_w);
	DECLARE_WRITE8_MEMBER(bigkarnk_coin_w);
	DECLARE_WRITE8_MEMBER(OKIM6295_bankswitch_w);
	DECLARE_WRITE16_MEMBER(gaelco_vram_encrypted_w);
	DECLARE_WRITE16_MEMBER(gaelco_encrypted_w);
	DECLARE_WRITE16_MEMBER(thoop_vram_encrypted_w);
	DECLARE_WRITE16_MEMBER(thoop_encrypted_w);
	DECLARE_WRITE16_MEMBER(gaelco_vram_w);

	TILE_GET_INFO_MEMBER(get_tile_info_gaelco_screen0);
	TILE_GET_INFO_MEMBER(get_tile_info_gaelco_screen1);

	virtual void machine_start() override;
	DECLARE_VIDEO_START(bigkarnk);
	DECLARE_VIDEO_START(maniacsq);

	uint32_t screen_update_bigkarnk(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_maniacsq(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void draw_sprites( screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect );
};
