// license:BSD-3-Clause
// copyright-holders:Carlos A. Lozano
/*************************************************************************

    Cops 01

*************************************************************************/

#include "machine/gen_latch.h"

class cop01_state : public driver_device
{
public:
	cop01_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_bgvideoram(*this, "bgvideoram"),
		m_spriteram(*this, "spriteram"),
		m_fgvideoram(*this, "fgvideoram"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch") { }

	/* memory pointers */
	required_shared_ptr<uint8_t> m_bgvideoram;
	required_shared_ptr<uint8_t> m_spriteram;
	required_shared_ptr<uint8_t> m_fgvideoram;

	/* video-related */
	tilemap_t        *m_bg_tilemap;
	tilemap_t        *m_fg_tilemap;
	uint8_t          m_vreg[4];

	/* sound-related */
	int            m_pulse;
	int            m_timer; // kludge for ym3526 in mightguy
	uint8_t        m_prot_command;
	uint8_t        m_prot_reg[6];

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_WRITE8_MEMBER(cop01_sound_command_w);
	DECLARE_READ8_MEMBER(cop01_sound_command_r);
	DECLARE_WRITE8_MEMBER(cop01_irq_ack_w);
	DECLARE_READ8_MEMBER(cop01_sound_irq_ack_w);
	DECLARE_READ8_MEMBER(kludge);
	DECLARE_WRITE8_MEMBER(cop01_background_w);
	DECLARE_WRITE8_MEMBER(cop01_foreground_w);
	DECLARE_WRITE8_MEMBER(cop01_vreg_w);
	DECLARE_WRITE8_MEMBER(prot_address_w);
	DECLARE_WRITE8_MEMBER(prot_data_w);
	DECLARE_READ8_MEMBER(prot_data_r);
	DECLARE_CUSTOM_INPUT_MEMBER(mightguy_area_r);
	DECLARE_DRIVER_INIT(mightguy);
	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	DECLARE_PALETTE_INIT(cop01);
	uint32_t screen_update_cop01(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect );
};
