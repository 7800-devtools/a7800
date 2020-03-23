// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria
/*************************************************************************

    Atari Centipede hardware

*************************************************************************/

#include "machine/eepromser.h"
#include "sound/ay8910.h"
#include "screen.h"

class centiped_state : public driver_device
{
public:
	centiped_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_rambase(*this, "rambase"),
		m_videoram(*this, "videoram"),
		m_spriteram(*this, "spriteram"),
		m_paletteram(*this, "paletteram"),
		m_bullsdrt_tiles_bankram(*this, "bullsdrt_bank"),
		m_maincpu(*this, "maincpu"),
		m_eeprom(*this, "eeprom"),
		m_gfxdecode(*this, "gfxdecode"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_aysnd(*this, "aysnd") { }

	optional_shared_ptr<uint8_t> m_rambase;
	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_spriteram;
	optional_shared_ptr<uint8_t> m_paletteram;
	optional_shared_ptr<uint8_t> m_bullsdrt_tiles_bankram;

	required_device<cpu_device> m_maincpu;
	optional_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	optional_device<ay8910_device> m_aysnd;

	uint8_t m_oldpos[4];
	uint8_t m_sign[4];
	uint8_t m_dsw_select;
	uint8_t m_control_select;
	uint8_t m_flipscreen;
	uint8_t m_prg_bank;
	uint8_t m_gfx_bank;
	uint8_t m_bullsdrt_sprites_bank;
	uint8_t m_penmask[64];
	tilemap_t *m_bg_tilemap;

	// drivers/centiped.c
	DECLARE_WRITE8_MEMBER(irq_ack_w);
	DECLARE_READ8_MEMBER(centiped_IN0_r);
	DECLARE_READ8_MEMBER(centiped_IN2_r);
	DECLARE_READ8_MEMBER(milliped_IN1_r);
	DECLARE_READ8_MEMBER(milliped_IN2_r);
	DECLARE_WRITE8_MEMBER(input_select_w);
	DECLARE_WRITE8_MEMBER(control_select_w);
	DECLARE_READ8_MEMBER(mazeinv_input_r);
	DECLARE_WRITE8_MEMBER(mazeinv_input_select_w);
	DECLARE_READ8_MEMBER(bullsdrt_data_port_r);
	DECLARE_WRITE8_MEMBER(led_w);
	DECLARE_WRITE8_MEMBER(coin_count_w);
	DECLARE_WRITE8_MEMBER(bullsdrt_coin_count_w);
	DECLARE_READ8_MEMBER(caterplr_unknown_r);
	DECLARE_WRITE8_MEMBER(caterplr_AY8910_w);
	DECLARE_READ8_MEMBER(caterplr_AY8910_r);
	DECLARE_READ8_MEMBER(multiped_eeprom_r);
	DECLARE_WRITE8_MEMBER(multiped_eeprom_w);
	DECLARE_WRITE8_MEMBER(multiped_prgbank_w);

	// video/centiped.c
	DECLARE_WRITE8_MEMBER(centiped_videoram_w);
	DECLARE_WRITE8_MEMBER(centiped_flip_screen_w);
	DECLARE_WRITE8_MEMBER(multiped_gfxbank_w);
	DECLARE_WRITE8_MEMBER(bullsdrt_tilesbank_w);
	DECLARE_WRITE8_MEMBER(bullsdrt_sprites_bank_w);
	DECLARE_WRITE8_MEMBER(centiped_paletteram_w);
	DECLARE_WRITE8_MEMBER(milliped_paletteram_w);
	DECLARE_WRITE8_MEMBER(mazeinv_paletteram_w);
	DECLARE_DRIVER_INIT(multiped);
	DECLARE_DRIVER_INIT(bullsdrt);
	TILE_GET_INFO_MEMBER(centiped_get_tile_info);
	TILE_GET_INFO_MEMBER(warlords_get_tile_info);
	TILE_GET_INFO_MEMBER(milliped_get_tile_info);
	TILE_GET_INFO_MEMBER(bullsdrt_get_tile_info);
	DECLARE_MACHINE_START(centiped);
	DECLARE_MACHINE_RESET(centiped);
	DECLARE_VIDEO_START(centiped);
	DECLARE_VIDEO_START(bullsdrt);
	DECLARE_MACHINE_RESET(magworm);
	DECLARE_VIDEO_START(milliped);
	DECLARE_VIDEO_START(warlords);
	DECLARE_PALETTE_INIT(warlords);
	uint32_t screen_update_centiped(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_bullsdrt(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_milliped(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_warlords(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(generate_interrupt);
	void init_penmask();
	void init_common();
	void milliped_set_color(offs_t offset, uint8_t data);
	inline int read_trackball(int idx, int switch_port);
};
