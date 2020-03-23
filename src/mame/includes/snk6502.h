// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Dan Boris
/*************************************************************************

    rokola hardware

*************************************************************************/
#ifndef MAME_INCLUDES_SNK6502_H
#define MAME_INCLUDES_SNK6502_H

#pragma once


class snk6502_sound_device;

class snk6502_state : public driver_device
{
public:
	snk6502_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_sound(*this, "snk6502"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_videoram(*this, "videoram"),
		m_videoram2(*this, "videoram2"),
		m_colorram(*this, "colorram"),
		m_charram(*this, "charram") { }

	required_device<cpu_device> m_maincpu;
	required_device<snk6502_sound_device> m_sound;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;

	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_videoram2;
	required_shared_ptr<uint8_t> m_colorram;
	required_shared_ptr<uint8_t> m_charram;

	uint8_t m_sasuke_counter;
	int m_charbank;
	int m_backcolor;
	tilemap_t *m_bg_tilemap;
	tilemap_t *m_fg_tilemap;
	rgb_t m_palette_val[64];
	uint8_t m_irq_mask;

	// common
	DECLARE_WRITE8_MEMBER(videoram_w);
	DECLARE_WRITE8_MEMBER(videoram2_w);
	DECLARE_WRITE8_MEMBER(colorram_w);
	DECLARE_WRITE8_MEMBER(charram_w);

	DECLARE_WRITE8_MEMBER(scrollx_w);
	DECLARE_WRITE8_MEMBER(scrolly_w);
	DECLARE_WRITE8_MEMBER(flipscreen_w);
	DECLARE_WRITE8_MEMBER(fantasy_flipscreen_w);
	DECLARE_WRITE8_MEMBER(satansat_b002_w);
	DECLARE_WRITE8_MEMBER(satansat_backcolor_w);

	DECLARE_CUSTOM_INPUT_MEMBER(snk6502_music0_r);
	DECLARE_CUSTOM_INPUT_MEMBER(sasuke_count_r);
	DECLARE_INPUT_CHANGED_MEMBER(coin_inserted);

	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);
	TILE_GET_INFO_MEMBER(satansat_get_bg_tile_info);
	TILE_GET_INFO_MEMBER(satansat_get_fg_tile_info);

	virtual void machine_start() override;
	DECLARE_MACHINE_RESET(sasuke);
	DECLARE_VIDEO_START(satansat);
	DECLARE_PALETTE_INIT(satansat);
	DECLARE_MACHINE_RESET(vanguard);
	DECLARE_VIDEO_START(snk6502);
	DECLARE_PALETTE_INIT(snk6502);
	DECLARE_MACHINE_RESET(satansat);
	DECLARE_MACHINE_RESET(pballoon);
	DECLARE_VIDEO_START(pballoon);

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	INTERRUPT_GEN_MEMBER(satansat_interrupt);
	INTERRUPT_GEN_MEMBER(snk6502_interrupt);
	TIMER_DEVICE_CALLBACK_MEMBER(sasuke_update_counter);

	void sasuke_start_counter();
	void postload();
};

#endif // MAME_INCLUDES_SNK6502_H
