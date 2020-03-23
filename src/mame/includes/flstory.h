// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria

#include "machine/gen_latch.h"
#include "sound/msm5232.h"
#include "machine/taito68705interface.h"

class flstory_state : public driver_device
{
public:
	flstory_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_videoram(*this, "videoram"),
		m_spriteram(*this, "spriteram"),
		m_scrlram(*this, "scrlram"),
		m_workram(*this, "workram"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_bmcu(*this, "bmcu"),
		m_msm(*this, "msm"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch") { }

	/* memory pointers */
	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_spriteram;
	required_shared_ptr<uint8_t> m_scrlram;
	optional_shared_ptr<uint8_t> m_workram;

	/* video-related */
	tilemap_t  *m_bg_tilemap;
	std::vector<uint8_t> m_paletteram;
	std::vector<uint8_t> m_paletteram_ext;
	uint8_t    m_gfxctrl;
	uint8_t    m_char_bank;
	uint8_t    m_palette_bank;

	/* sound-related */
	uint8_t    m_snd_data;
	uint8_t    m_snd_flag;
	int      m_sound_nmi_enable;
	int      m_pending_nmi;
	int      m_vol_ctrl[16];
	uint8_t    m_snd_ctrl0;
	uint8_t    m_snd_ctrl1;
	uint8_t    m_snd_ctrl2;
	uint8_t    m_snd_ctrl3;

	/* protection sims */
	uint8_t m_from_mcu;
	int      m_mcu_select;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	optional_device<taito68705_mcu_device> m_bmcu;
	required_device<msm5232_device> m_msm;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_READ8_MEMBER(from_snd_r);
	DECLARE_READ8_MEMBER(snd_flag_r);
	DECLARE_WRITE8_MEMBER(to_main_w);
	DECLARE_WRITE8_MEMBER(sound_command_w);
	DECLARE_WRITE8_MEMBER(nmi_disable_w);
	DECLARE_WRITE8_MEMBER(nmi_enable_w);
	DECLARE_READ8_MEMBER(flstory_mcu_status_r);
	DECLARE_WRITE8_MEMBER(victnine_mcu_w);
	DECLARE_READ8_MEMBER(victnine_mcu_r);
	DECLARE_READ8_MEMBER(victnine_mcu_status_r);
	DECLARE_WRITE8_MEMBER(flstory_videoram_w);
	DECLARE_WRITE8_MEMBER(flstory_palette_w);
	DECLARE_READ8_MEMBER(flstory_palette_r);
	DECLARE_WRITE8_MEMBER(flstory_gfxctrl_w);
	DECLARE_READ8_MEMBER(victnine_gfxctrl_r);
	DECLARE_WRITE8_MEMBER(victnine_gfxctrl_w);
	DECLARE_WRITE8_MEMBER(flstory_scrlram_w);
	DECLARE_CUSTOM_INPUT_MEMBER(victnine_mcu_status_bit01_r);
	DECLARE_WRITE8_MEMBER(sound_control_0_w);
	DECLARE_WRITE8_MEMBER(sound_control_1_w);
	DECLARE_WRITE8_MEMBER(sound_control_2_w);
	DECLARE_WRITE8_MEMBER(sound_control_3_w);
	TILE_GET_INFO_MEMBER(get_tile_info);
	TILE_GET_INFO_MEMBER(victnine_get_tile_info);
	TILE_GET_INFO_MEMBER(get_rumba_tile_info);
	virtual void machine_start() override;
	DECLARE_MACHINE_RESET(flstory);
	DECLARE_VIDEO_START(flstory);
	DECLARE_VIDEO_START(victnine);
	DECLARE_MACHINE_RESET(rumba);
	DECLARE_VIDEO_START(rumba);
	DECLARE_MACHINE_RESET(ta7630);
	uint32_t screen_update_flstory(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_victnine(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_rumba(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_CALLBACK_MEMBER(nmi_callback);
	void flstory_draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect, int pri );
	void victnine_draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
};
