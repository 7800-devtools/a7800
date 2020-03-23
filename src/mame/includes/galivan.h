// license:BSD-3-Clause
// copyright-holders:Luca Elia, Olivier Galibert
/***************************************************************************

    Galivan - Cosmo Police

***************************************************************************/

#include "includes/nb1414m4.h"
#include "machine/gen_latch.h"
#include "video/bufsprite.h"

class galivan_state : public driver_device
{
public:
	galivan_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_videoram(*this, "videoram"),
		m_spriteram(*this, "spriteram"),
		m_maincpu(*this, "maincpu"),
		m_nb1414m4(*this, "nb1414m4"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch") { }

	/* memory pointers */
	required_shared_ptr<uint8_t> m_videoram;
	required_device<buffered_spriteram8_device> m_spriteram;

	/* video-related */
	tilemap_t     *m_bg_tilemap;
	tilemap_t     *m_tx_tilemap;
	uint16_t       m_scrollx;
	uint16_t       m_scrolly;
	uint8_t       m_galivan_scrollx[2],m_galivan_scrolly[2];
	uint8_t       m_write_layers;
	uint8_t       m_layers;
	uint8_t       m_ninjemak_dispdisable;

	uint8_t       m_shift_scroll; //youmab
	uint32_t      m_shift_val;
	DECLARE_WRITE8_MEMBER(galivan_sound_command_w);
	DECLARE_READ8_MEMBER(soundlatch_clear_r);
	DECLARE_READ8_MEMBER(IO_port_c0_r);
	DECLARE_WRITE8_MEMBER(blit_trigger_w);
	DECLARE_WRITE8_MEMBER(youmab_extra_bank_w);
	DECLARE_READ8_MEMBER(youmab_8a_r);
	DECLARE_WRITE8_MEMBER(youmab_81_w);
	DECLARE_WRITE8_MEMBER(youmab_84_w);
	DECLARE_WRITE8_MEMBER(youmab_86_w);
	DECLARE_WRITE8_MEMBER(galivan_videoram_w);
	DECLARE_WRITE8_MEMBER(galivan_gfxbank_w);
	DECLARE_WRITE8_MEMBER(ninjemak_gfxbank_w);
	DECLARE_WRITE8_MEMBER(galivan_scrollx_w);
	DECLARE_WRITE8_MEMBER(galivan_scrolly_w);
	DECLARE_DRIVER_INIT(youmab);
	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	TILE_GET_INFO_MEMBER(get_tx_tile_info);
	TILE_GET_INFO_MEMBER(ninjemak_get_bg_tile_info);
	TILE_GET_INFO_MEMBER(ninjemak_get_tx_tile_info);
	DECLARE_PALETTE_INIT(galivan);
	DECLARE_MACHINE_START(galivan);
	DECLARE_MACHINE_RESET(galivan);
	DECLARE_VIDEO_START(galivan);
	DECLARE_MACHINE_START(ninjemak);
	DECLARE_MACHINE_RESET(ninjemak);
	DECLARE_VIDEO_START(ninjemak);
	uint32_t screen_update_galivan(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_ninjemak(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
	required_device<cpu_device> m_maincpu;
	optional_device<nb1414m4_device> m_nb1414m4;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;
};
