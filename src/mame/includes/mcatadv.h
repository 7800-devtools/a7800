// license:BSD-3-Clause
// copyright-holders:Paul Priest, David Haywood

#include "machine/gen_latch.h"
#include "machine/watchdog.h"

class mcatadv_state : public driver_device
{
public:
	mcatadv_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_videoram1(*this, "videoram1"),
		m_videoram2(*this, "videoram2"),
		m_scroll1(*this, "scroll1"),
		m_scroll2(*this, "scroll2"),
		m_spriteram(*this, "spriteram"),
		m_vidregs(*this, "vidregs"),
		m_maincpu(*this, "maincpu"),
		m_soundcpu(*this, "soundcpu"),
		m_watchdog(*this, "watchdog"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch") { }

	/* memory pointers */
	required_shared_ptr<uint16_t> m_videoram1;
	required_shared_ptr<uint16_t> m_videoram2;
	required_shared_ptr<uint16_t> m_scroll1;
	required_shared_ptr<uint16_t> m_scroll2;
	required_shared_ptr<uint16_t> m_spriteram;
	std::unique_ptr<uint16_t[]>     m_spriteram_old;
	required_shared_ptr<uint16_t> m_vidregs;
	std::unique_ptr<uint16_t[]>     m_vidregs_old;

	/* video-related */
	tilemap_t    *m_tilemap1;
	tilemap_t    *m_tilemap2;
	int m_palette_bank1;
	int m_palette_bank2;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_soundcpu;
	required_device<watchdog_timer_device> m_watchdog;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_WRITE16_MEMBER(mcat_soundlatch_w);
	DECLARE_READ16_MEMBER(mcat_wd_r);
	DECLARE_WRITE8_MEMBER(mcatadv_sound_bw_w);
	DECLARE_WRITE16_MEMBER(mcatadv_videoram1_w);
	DECLARE_WRITE16_MEMBER(mcatadv_videoram2_w);
	TILE_GET_INFO_MEMBER(get_mcatadv_tile_info1);
	TILE_GET_INFO_MEMBER(get_mcatadv_tile_info2);
	virtual void machine_start() override;
	virtual void video_start() override;
	uint32_t screen_update_mcatadv(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(screen_vblank_mcatadv);
	void draw_sprites( screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect );
	void mcatadv_draw_tilemap_part( screen_device &screen, uint16_t* current_scroll, uint16_t* current_videoram1, int i, tilemap_t* current_tilemap, bitmap_ind16 &bitmap, const rectangle &cliprect );
};
