// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria
/*************************************************************************

    Mitchell hardware

*************************************************************************/

#include "machine/74157.h"
#include "machine/nvram.h"
#include "machine/eepromser.h"
#include "machine/gen_latch.h"
#include "sound/msm5205.h"
#include "sound/okim6295.h"

class mitchell_state : public driver_device
{
public:
	mitchell_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_oki(*this, "oki"),
		m_nvram(*this, "nvram"),
		m_eeprom(*this, "eeprom"),
		m_msm(*this, "msm"),
		m_adpcm_select(*this, "adpcm_select"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch"),
		m_colorram(*this, "colorram"),
		m_videoram(*this, "videoram"),
		m_bank1(*this, "bank1"),
		m_bank0d(*this, "bank0d"),
		m_bank1d(*this, "bank1d"),
		m_soundbank(*this, "soundbank") { }

	/* devices */
	required_device<cpu_device> m_maincpu;
	optional_device<cpu_device> m_audiocpu;
	optional_device<okim6295_device> m_oki;
	optional_device<nvram_device> m_nvram;
	optional_device<eeprom_serial_93cxx_device> m_eeprom;
	optional_device<msm5205_device> m_msm;
	optional_device<ls157_device> m_adpcm_select;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	optional_device<generic_latch_8_device> m_soundlatch;

	/* memory pointers */
	required_shared_ptr<uint8_t> m_colorram;
	required_shared_ptr<uint8_t> m_videoram;
	required_memory_bank m_bank1;
	optional_memory_bank m_bank0d;
	optional_memory_bank m_bank1d;
	optional_memory_bank m_soundbank;

	/* video-related */
	tilemap_t    *m_bg_tilemap;
	std::vector<uint8_t> m_objram;           /* Sprite RAM */
	int        m_flipscreen;
	int        m_video_bank;
	int        m_paletteram_bank;
	std::vector<uint8_t> m_paletteram;

	/* sound-related */
	bool       m_sample_select;

	/* misc */
	int        m_input_type;
	int        m_dial[2];
	int        m_dial_selected;
	int        m_dir[2];
	int        m_keymatrix;

	uint8_t m_irq_source;
	DECLARE_READ8_MEMBER(pang_port5_r);
	DECLARE_WRITE8_MEMBER(pang_bankswitch_w);
	DECLARE_READ8_MEMBER(block_input_r);
	DECLARE_WRITE8_MEMBER(block_dial_control_w);
	DECLARE_READ8_MEMBER(mahjong_input_r);
	DECLARE_WRITE8_MEMBER(mahjong_input_select_w);
	DECLARE_READ8_MEMBER(input_r);
	DECLARE_WRITE8_MEMBER(input_w);
	DECLARE_WRITE8_MEMBER(mstworld_sound_w);
	DECLARE_WRITE8_MEMBER(pang_video_bank_w);
	DECLARE_WRITE8_MEMBER(mstworld_video_bank_w);
	DECLARE_WRITE8_MEMBER(mgakuen_videoram_w);
	DECLARE_READ8_MEMBER(mgakuen_videoram_r);
	DECLARE_WRITE8_MEMBER(mgakuen_objram_w);
	DECLARE_READ8_MEMBER(mgakuen_objram_r);
	DECLARE_WRITE8_MEMBER(pang_videoram_w);
	DECLARE_READ8_MEMBER(pang_videoram_r);
	DECLARE_WRITE8_MEMBER(pang_colorram_w);
	DECLARE_READ8_MEMBER(pang_colorram_r);
	DECLARE_WRITE8_MEMBER(pang_gfxctrl_w);
	DECLARE_WRITE8_MEMBER(pangbl_gfxctrl_w);
	DECLARE_WRITE8_MEMBER(mstworld_gfxctrl_w);
	DECLARE_WRITE8_MEMBER(pang_paletteram_w);
	DECLARE_READ8_MEMBER(pang_paletteram_r);
	DECLARE_WRITE8_MEMBER(eeprom_cs_w);
	DECLARE_WRITE8_MEMBER(eeprom_clock_w);
	DECLARE_WRITE8_MEMBER(eeprom_serial_w);
	DECLARE_WRITE8_MEMBER(oki_banking_w);
	DECLARE_DRIVER_INIT(mgakuen2);
	DECLARE_DRIVER_INIT(block);
	DECLARE_DRIVER_INIT(pangb);
	DECLARE_DRIVER_INIT(qtono1);
	DECLARE_DRIVER_INIT(mgakuen);
	DECLARE_DRIVER_INIT(hatena);
	DECLARE_DRIVER_INIT(mstworld);
	DECLARE_DRIVER_INIT(spangbl);
	DECLARE_DRIVER_INIT(pkladiesbl);
	DECLARE_DRIVER_INIT(spang);
	DECLARE_DRIVER_INIT(cworld);
	DECLARE_DRIVER_INIT(spangj);
	DECLARE_DRIVER_INIT(qsangoku);
	DECLARE_DRIVER_INIT(marukin);
	DECLARE_DRIVER_INIT(pang);
	DECLARE_DRIVER_INIT(sbbros);
	DECLARE_DRIVER_INIT(pkladies);
	DECLARE_DRIVER_INIT(blockbl);
	DECLARE_DRIVER_INIT(dokaben);
	TILE_GET_INFO_MEMBER(get_tile_info);
	DECLARE_MACHINE_START(mitchell);
	DECLARE_MACHINE_RESET(mitchell);
	DECLARE_VIDEO_START(pang);
	uint32_t screen_update_pang(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(mitchell_irq);
	void draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
	void bootleg_decode();
	void configure_banks(void (*decode)(uint8_t *src, uint8_t *dst, int size));
	DECLARE_WRITE8_MEMBER(sound_bankswitch_w);
	DECLARE_WRITE_LINE_MEMBER(spangbl_adpcm_int);
};
