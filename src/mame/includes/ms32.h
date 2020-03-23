// license:BSD-3-Clause
// copyright-holders:David Haywood,Paul Priest

#include "machine/gen_latch.h"
#include "screen.h"

class ms32_state : public driver_device
{
public:
	ms32_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_mainram(*this, "mainram"),
		m_roz_ctrl(*this, "roz_ctrl"),
		m_tx_scroll(*this, "tx_scroll"),
		m_bg_scroll(*this, "bg_scroll"),
		m_mahjong_input_select(*this, "mahjong_select"),
		m_priram(*this, "priram", 32),
		m_palram(*this, "palram", 32),
		m_rozram(*this, "rozram", 32),
		m_lineram(*this, "lineram", 32),
		m_sprram(*this, "sprram", 32),
		m_txram(*this, "txram", 32),
		m_bgram(*this, "bgram", 32),
		m_f1superb_extraram(*this, "f1sb_extraram", 32),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch") { }

	optional_shared_ptr<uint32_t> m_mainram;
	optional_shared_ptr<uint32_t> m_roz_ctrl;
	optional_shared_ptr<uint32_t> m_tx_scroll;
	optional_shared_ptr<uint32_t> m_bg_scroll;
	optional_shared_ptr<uint32_t> m_mahjong_input_select;
	optional_shared_ptr<uint8_t> m_priram;
	optional_shared_ptr<uint16_t> m_palram;
	optional_shared_ptr<uint16_t> m_rozram;
	optional_shared_ptr<uint16_t> m_lineram;
	optional_shared_ptr<uint16_t> m_sprram;
	optional_shared_ptr<uint16_t> m_txram;
	optional_shared_ptr<uint16_t> m_bgram;
	optional_shared_ptr<uint16_t> m_f1superb_extraram;
	std::unique_ptr<uint8_t[]> m_nvram_8;
	uint32_t m_to_main;
	uint16_t m_irqreq;
	tilemap_t *m_tx_tilemap;
	tilemap_t *m_roz_tilemap;
	tilemap_t *m_bg_tilemap;
	tilemap_t *m_bg_tilemap_alt;
	uint32_t m_tilemaplayoutcontrol;
	tilemap_t* m_extra_tilemap;
	bitmap_ind16 m_temp_bitmap_tilemaps;
	bitmap_ind16 m_temp_bitmap_sprites;
	bitmap_ind8 m_temp_bitmap_sprites_pri;
	int m_reverse_sprite_order;
	int m_flipscreen;
	uint32_t m_brt[4];
	int m_brt_r;
	int m_brt_g;
	int m_brt_b;
	DECLARE_READ32_MEMBER(ms32_read_inputs3);
	DECLARE_WRITE32_MEMBER(ms32_sound_w);
	DECLARE_READ32_MEMBER(ms32_sound_r);
	DECLARE_WRITE32_MEMBER(reset_sub_w);
	DECLARE_READ8_MEMBER(ms32_nvram_r8);
	DECLARE_WRITE8_MEMBER(ms32_nvram_w8);
	DECLARE_READ8_MEMBER(ms32_priram_r8);
	DECLARE_WRITE8_MEMBER(ms32_priram_w8);
	DECLARE_READ16_MEMBER(ms32_palram_r16);
	DECLARE_WRITE16_MEMBER(ms32_palram_w16);
	DECLARE_READ16_MEMBER(ms32_rozram_r16);
	DECLARE_WRITE16_MEMBER(ms32_rozram_w16);
	DECLARE_READ16_MEMBER(ms32_lineram_r16);
	DECLARE_WRITE16_MEMBER(ms32_lineram_w16);
	DECLARE_READ16_MEMBER(ms32_sprram_r16);
	DECLARE_WRITE16_MEMBER(ms32_sprram_w16);
	DECLARE_READ16_MEMBER(ms32_txram_r16);
	DECLARE_WRITE16_MEMBER(ms32_txram_w16);
	DECLARE_READ16_MEMBER(ms32_bgram_r16);
	DECLARE_WRITE16_MEMBER(ms32_bgram_w16);
	DECLARE_WRITE32_MEMBER(pip_w);
	DECLARE_WRITE16_MEMBER(ms32_extra_w16);
	DECLARE_READ16_MEMBER(ms32_extra_r16);
	DECLARE_WRITE32_MEMBER(ms32_irq2_guess_w);
	DECLARE_WRITE32_MEMBER(ms32_irq5_guess_w);
	DECLARE_READ8_MEMBER(latch_r);
	DECLARE_WRITE8_MEMBER(ms32_snd_bank_w);
	DECLARE_WRITE8_MEMBER(to_main_w);
	DECLARE_WRITE32_MEMBER(ms32_brightness_w);
	DECLARE_WRITE32_MEMBER(ms32_gfxctrl_w);
	DECLARE_CUSTOM_INPUT_MEMBER(mahjong_ctrl_r);
	DECLARE_DRIVER_INIT(ms32_common);
	DECLARE_DRIVER_INIT(ss92047_01);
	DECLARE_DRIVER_INIT(ss91022_10);
	DECLARE_DRIVER_INIT(kirarast);
	DECLARE_DRIVER_INIT(47pie2);
	DECLARE_DRIVER_INIT(ss92048_01);
	DECLARE_DRIVER_INIT(bnstars);
	DECLARE_DRIVER_INIT(f1superb);
	DECLARE_DRIVER_INIT(ss92046_01);
	TILE_GET_INFO_MEMBER(get_ms32_tx_tile_info);
	TILE_GET_INFO_MEMBER(get_ms32_roz_tile_info);
	TILE_GET_INFO_MEMBER(get_ms32_bg_tile_info);
	TILE_GET_INFO_MEMBER(get_ms32_extra_tile_info);
	virtual void machine_reset() override;
	virtual void video_start() override;
	DECLARE_VIDEO_START(f1superb);
	uint32_t screen_update_ms32(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(ms32_interrupt);
	IRQ_CALLBACK_MEMBER(irq_callback);
	void irq_init();
	void irq_raise(int level);
	void update_color(int color);
	void draw_sprites(bitmap_ind16 &bitmap, bitmap_ind8 &bitmap_pri, const rectangle &cliprect, uint16_t *sprram_top, size_t sprram_size, int gfxnum, int reverseorder);
	void draw_roz(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect,int priority);
	void configure_banks();
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<gfxdecode_device> m_gfxdecode;
	optional_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	optional_device<generic_latch_8_device> m_soundlatch; //not for bnstars.cpp
};
