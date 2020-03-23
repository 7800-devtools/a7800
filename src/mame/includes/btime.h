// license:BSD-3-Clause
// copyright-holders:Zsolt Vasvari, Couriersud
/***************************************************************************

    Burger Time hardware

***************************************************************************/

#include "machine/gen_latch.h"
#include "screen.h"

class btime_state : public driver_device
{
public:
	btime_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_rambase(*this, "rambase")
		, m_videoram(*this, "videoram")
		, m_colorram(*this, "colorram")
		, m_bnj_backgroundram(*this, "bnj_bgram")
		, m_zoar_scrollram(*this, "zoar_scrollram")
		, m_lnc_charbank(*this, "lnc_charbank")
		, m_deco_charram(*this, "deco_charram")
		, m_spriteram(*this, "spriteram")
		, m_audio_rambase(*this, "audio_rambase")
		, m_maincpu(*this, "maincpu")
		, m_audiocpu(*this, "audiocpu")
		, m_gfxdecode(*this, "gfxdecode")
		, m_screen(*this, "screen")
		, m_palette(*this, "palette")
		, m_soundlatch(*this, "soundlatch")
		, m_prom_region(*this, "proms")
	{
	}

	/* memory pointers */
	optional_shared_ptr<uint8_t> m_rambase;
	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_colorram;
	optional_shared_ptr<uint8_t> m_bnj_backgroundram;
	optional_shared_ptr<uint8_t> m_zoar_scrollram;
	optional_shared_ptr<uint8_t> m_lnc_charbank;
	optional_shared_ptr<uint8_t> m_deco_charram;
	optional_shared_ptr<uint8_t> m_spriteram;     // used by disco
//  uint8_t *  m_decrypted;
	optional_shared_ptr<uint8_t> m_audio_rambase;

	/* video-related */
	std::unique_ptr<bitmap_ind16> m_background_bitmap;
	uint8_t    m_btime_palette;
	uint8_t    m_bnj_scroll1;
	uint8_t    m_bnj_scroll2;
	uint8_t    m_btime_tilemap[4];

	/* audio-related */
	uint8_t    m_audio_nmi_enable_type;
	uint8_t    m_audio_nmi_enabled;
	uint8_t    m_audio_nmi_state;

	/* protection-related (for mmonkey) */
	int      m_protection_command;
	int      m_protection_status;
	int      m_protection_value;
	int      m_protection_ret;

	/* devices */
	required_device<cpu_device> m_maincpu;
	optional_device<cpu_device> m_audiocpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	optional_device<generic_latch_8_device> m_soundlatch;
	optional_memory_region m_prom_region;

	DECLARE_WRITE8_MEMBER(audio_nmi_enable_w);
	DECLARE_WRITE8_MEMBER(audio_command_w);
	DECLARE_READ8_MEMBER(audio_command_r);
	DECLARE_READ8_MEMBER(zoar_dsw1_read);
	DECLARE_READ8_MEMBER(wtennis_reset_hack_r);
	DECLARE_READ8_MEMBER(mmonkey_protection_r);
	DECLARE_WRITE8_MEMBER(mmonkey_protection_w);
	DECLARE_WRITE8_MEMBER(lnc_videoram_w);
	DECLARE_READ8_MEMBER(btime_mirrorvideoram_r);
	DECLARE_READ8_MEMBER(btime_mirrorcolorram_r);
	DECLARE_WRITE8_MEMBER(btime_mirrorvideoram_w);
	DECLARE_WRITE8_MEMBER(lnc_mirrorvideoram_w);
	DECLARE_WRITE8_MEMBER(btime_mirrorcolorram_w);
	DECLARE_WRITE8_MEMBER(deco_charram_w);
	DECLARE_WRITE8_MEMBER(bnj_background_w);
	DECLARE_WRITE8_MEMBER(bnj_scroll1_w);
	DECLARE_WRITE8_MEMBER(bnj_scroll2_w);
	DECLARE_WRITE8_MEMBER(btime_video_control_w);
	DECLARE_WRITE8_MEMBER(bnj_video_control_w);
	DECLARE_WRITE8_MEMBER(zoar_video_control_w);
	DECLARE_WRITE8_MEMBER(disco_video_control_w);
	DECLARE_INPUT_CHANGED_MEMBER(coin_inserted_irq_hi);
	DECLARE_INPUT_CHANGED_MEMBER(coin_inserted_irq_lo);
	DECLARE_INPUT_CHANGED_MEMBER(coin_inserted_nmi_lo);
	DECLARE_WRITE8_MEMBER(ay_audio_nmi_enable_w);

	DECLARE_DRIVER_INIT(btime);
	DECLARE_DRIVER_INIT(tisland);
	DECLARE_DRIVER_INIT(cookrace);
	DECLARE_DRIVER_INIT(zoar);
	DECLARE_DRIVER_INIT(sdtennis);
	DECLARE_DRIVER_INIT(wtennis);
	DECLARE_DRIVER_INIT(bnj);
	DECLARE_DRIVER_INIT(protennb);
	DECLARE_DRIVER_INIT(disco);
	DECLARE_DRIVER_INIT(lnc);
	DECLARE_MACHINE_START(btime);
	DECLARE_MACHINE_RESET(btime);
	DECLARE_PALETTE_INIT(btime);
	DECLARE_MACHINE_RESET(lnc);
	DECLARE_PALETTE_INIT(lnc);
	DECLARE_MACHINE_START(mmonkey);
	DECLARE_MACHINE_RESET(mmonkey);
	DECLARE_VIDEO_START(bnj);
	DECLARE_VIDEO_START(disco);
	uint32_t screen_update_btime(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_cookrace(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_lnc(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_eggs(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_bnj(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_zoar(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_disco(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(audio_nmi_gen);
	void draw_chars( bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t transparency, uint8_t color, int priority );
	void draw_background( bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t* tmap, uint8_t color );
	void draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t color,
							uint8_t sprite_y_adjust, uint8_t sprite_y_adjust_flip_screen,
							uint8_t *sprite_ram, offs_t interleave );

};
