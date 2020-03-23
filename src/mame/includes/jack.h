// license:BSD-3-Clause
// copyright-holders:Brad Oliver

/*************************************************************************

    Jack the Giant Killer

*************************************************************************/

#include "machine/gen_latch.h"

class jack_state : public driver_device
{
public:
	jack_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_spriteram(*this, "spriteram"),
		m_scrollram(*this, "scrollram"),
		m_videoram(*this, "videoram"),
		m_colorram(*this, "colorram"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch"),
		m_decrypted_opcodes(*this, "decrypted_opcodes")
	{ }

	/* device- and memory pointers */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	optional_shared_ptr<uint8_t> m_spriteram;
	optional_shared_ptr<uint8_t> m_scrollram;
	required_shared_ptr<uint8_t> m_videoram;
	required_shared_ptr<uint8_t> m_colorram;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;
	optional_shared_ptr<uint8_t> m_decrypted_opcodes;

	/* video-related */
	tilemap_t    *m_bg_tilemap;

	/* misc */
	int m_timer_rate;
	uint8_t m_joinem_nmi_enable;
	uint8_t m_joinem_palette_bank;
	int m_question_address;
	int m_question_rom;
	int m_remap_address[16];

	IRQ_CALLBACK_MEMBER(jack_sh_irq_ack);
	DECLARE_WRITE8_MEMBER(joinem_control_w);
	DECLARE_WRITE8_MEMBER(joinem_scroll_w);
	DECLARE_READ8_MEMBER(striv_question_r);
	DECLARE_WRITE8_MEMBER(jack_videoram_w);
	DECLARE_WRITE8_MEMBER(jack_colorram_w);
	DECLARE_READ8_MEMBER(jack_flipscreen_r);
	DECLARE_WRITE8_MEMBER(jack_flipscreen_w);
	DECLARE_READ8_MEMBER(timer_r);

	DECLARE_DRIVER_INIT(zzyzzyxx);
	DECLARE_DRIVER_INIT(striv);
	DECLARE_DRIVER_INIT(treahunt);
	DECLARE_DRIVER_INIT(loverboy);
	DECLARE_DRIVER_INIT(jack);

	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	TILEMAP_MAPPER_MEMBER(tilemap_scan_cols_flipy);
	TILE_GET_INFO_MEMBER(joinem_get_bg_tile_info);
	DECLARE_VIDEO_START(joinem);
	DECLARE_PALETTE_INIT(joinem);
	DECLARE_MACHINE_START(striv);
	DECLARE_MACHINE_RESET(striv);
	DECLARE_MACHINE_START(joinem);
	DECLARE_MACHINE_RESET(joinem);

	uint32_t screen_update_jack(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_striv(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_joinem(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;

	INTERRUPT_GEN_MEMBER(joinem_vblank_irq);
	void jack_draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
	void joinem_draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
	void treahunt_decode(  );
};
