// license:BSD-3-Clause
// copyright-holders:Paul Leaman, Couriersud
/***************************************************************************

    1942

***************************************************************************/

#include "machine/gen_latch.h"

class _1942_state : public driver_device
{
public:
	_1942_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_spriteram(*this, "spriteram"),
		m_fg_videoram(*this, "fg_videoram"),
		m_bg_videoram(*this, "bg_videoram"),
		m_protopal(*this, "protopal"),
		m_audiocpu(*this, "audiocpu"),
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch") { }

	/* memory pointers */
	required_shared_ptr<uint8_t> m_spriteram;
	required_shared_ptr<uint8_t> m_fg_videoram;
	required_shared_ptr<uint8_t> m_bg_videoram;
	optional_shared_ptr<uint8_t> m_protopal;

	/* video-related */
	tilemap_t *m_fg_tilemap;
	tilemap_t *m_bg_tilemap;
	int m_palette_bank;
	uint8_t m_scroll[2];
	void create_palette();
	DECLARE_PALETTE_INIT(1942);
	DECLARE_PALETTE_INIT(1942p);
	DECLARE_WRITE8_MEMBER(c1942p_palette_w);

	/* devices */
	required_device<cpu_device> m_audiocpu;
	DECLARE_WRITE8_MEMBER(c1942_bankswitch_w);
	DECLARE_WRITE8_MEMBER(c1942_fgvideoram_w);
	DECLARE_WRITE8_MEMBER(c1942_bgvideoram_w);
	DECLARE_WRITE8_MEMBER(c1942_palette_bank_w);
	DECLARE_WRITE8_MEMBER(c1942_scroll_w);
	DECLARE_WRITE8_MEMBER(c1942_c804_w);
	DECLARE_WRITE8_MEMBER(c1942p_f600_w);
	DECLARE_WRITE8_MEMBER(c1942p_soundlatch_w);
	DECLARE_DRIVER_INIT(1942);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);
	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	void video_start_c1942p();
	uint32_t screen_update_1942(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_1942p(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(c1942_scanline);
	void draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
	void draw_sprites_p( bitmap_ind16 &bitmap, const rectangle &cliprect );
	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;
};
