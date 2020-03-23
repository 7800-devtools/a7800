// license:BSD-3-Clause
// copyright-holders:Acho A. Tang, Nicola Salmoria
/*************************************************************************

    Equites, Splendor Blast driver

*************************************************************************/

#include "machine/alpha8201.h"
#include "machine/gen_latch.h"
#include "sound/samples.h"
#include "sound/msm5232.h"
#include "sound/dac.h"
#include "screen.h"


class equites_state : public driver_device
{
public:
	equites_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_bg_videoram(*this, "bg_videoram"),
		m_spriteram(*this, "spriteram"),
		m_spriteram_2(*this, "spriteram_2"),
		m_mcuram(*this, "mcuram"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_samples(*this, "samples"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_screen(*this, "screen"),
		m_alpha_8201(*this, "alpha_8201"),
		m_fakemcu(*this, "mcu"),
		m_msm(*this, "msm"),
		m_dac_1(*this, "dac1"),
		m_dac_2(*this, "dac2"),
		m_soundlatch(*this, "soundlatch")
	{ }

	/* memory pointers */
	required_shared_ptr<uint16_t> m_bg_videoram;
	std::unique_ptr<uint8_t[]> m_fg_videoram;    // 8bits
	required_shared_ptr<uint16_t> m_spriteram;
	optional_shared_ptr<uint16_t> m_spriteram_2;
	optional_shared_ptr<uint8_t> m_mcuram;

	/* video-related */
	tilemap_t *m_fg_tilemap;
	tilemap_t *m_bg_tilemap;
	int       m_fg_char_bank;
	uint8_t     m_bgcolor;
	uint16_t    m_splndrbt_bg_scrollx;
	uint16_t    m_splndrbt_bg_scrolly;

	/* misc */
	int       m_sound_prom_address;
	uint8_t     m_dac_latch;
	uint8_t     m_eq8155_port_b;
	uint8_t     m_eq8155_port_a;
	uint8_t     m_eq8155_port_c;
	uint8_t     m_ay_port_a;
	uint8_t     m_ay_port_b;
	uint8_t     m_eq_cymbal_ctrl;
	emu_timer *m_adjuster_timer;
	float     m_cymvol;
	float     m_hihatvol;
	int       m_timer_count;
	int       m_gekisou_unknown_bit;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<samples_device> m_samples;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<screen_device> m_screen;
	required_device<alpha_8201_device> m_alpha_8201;
	optional_device<cpu_device> m_fakemcu;
	required_device<msm5232_device> m_msm;
	required_device<dac_byte_interface> m_dac_1;
	required_device<dac_byte_interface> m_dac_2;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_WRITE8_MEMBER(equites_c0f8_w);
	DECLARE_WRITE8_MEMBER(equites_cymbal_ctrl_w);
	DECLARE_WRITE8_MEMBER(equites_dac_latch_w);
	DECLARE_WRITE8_MEMBER(equites_8155_porta_w);
	DECLARE_WRITE8_MEMBER(equites_8155_portb_w);
	DECLARE_WRITE8_MEMBER(equites_8155_portc_w);
	DECLARE_WRITE16_MEMBER(gekisou_unknown_bit_w);
	DECLARE_READ16_MEMBER(equites_spriteram_kludge_r);
	DECLARE_READ8_MEMBER(mcu_ram_r);
	DECLARE_WRITE8_MEMBER(mcu_ram_w);
	DECLARE_WRITE16_MEMBER(mcu_start_w);
	DECLARE_WRITE16_MEMBER(mcu_switch_w);
	DECLARE_READ8_MEMBER(equites_fg_videoram_r);
	DECLARE_WRITE8_MEMBER(equites_fg_videoram_w);
	DECLARE_WRITE16_MEMBER(equites_bg_videoram_w);
	DECLARE_WRITE8_MEMBER(equites_bgcolor_w);
	DECLARE_WRITE16_MEMBER(equites_scrollreg_w);
	DECLARE_WRITE16_MEMBER(splndrbt_selchar_w);
	DECLARE_WRITE16_MEMBER(equites_flipw_w);
	DECLARE_WRITE8_MEMBER(equites_flipb_w);
	DECLARE_WRITE16_MEMBER(splndrbt_bg_scrollx_w);
	DECLARE_WRITE16_MEMBER(splndrbt_bg_scrolly_w);
	DECLARE_CUSTOM_INPUT_MEMBER(gekisou_unknown_bit_r);
	DECLARE_WRITE8_MEMBER(equites_8910porta_w);
	DECLARE_WRITE8_MEMBER(equites_8910portb_w);
	DECLARE_DRIVER_INIT(splndrbt);
	DECLARE_DRIVER_INIT(equites);
	TILE_GET_INFO_MEMBER(equites_fg_info);
	TILE_GET_INFO_MEMBER(splndrbt_fg_info);
	TILE_GET_INFO_MEMBER(equites_bg_info);
	TILE_GET_INFO_MEMBER(splndrbt_bg_info);
	DECLARE_VIDEO_START(equites);
	DECLARE_PALETTE_INIT(equites);
	DECLARE_VIDEO_START(splndrbt);
	DECLARE_PALETTE_INIT(splndrbt);
	uint32_t screen_update_equites(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_splndrbt(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(equites_8155_timer_pulse);
	TIMER_CALLBACK_MEMBER(equites_frq_adjuster_callback);
	TIMER_DEVICE_CALLBACK_MEMBER(equites_scanline);
	TIMER_DEVICE_CALLBACK_MEMBER(splndrbt_scanline);
	DECLARE_WRITE_LINE_MEMBER(equites_msm5232_gate);
	void equites_draw_sprites_block(bitmap_ind16 &bitmap, const rectangle &cliprect, int start, int end);
	void equites_draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect);
	void splndrbt_draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect);
	void splndrbt_copy_bg(bitmap_ind16 &dst_bitmap, const rectangle &cliprect);
	void equites_update_dac();
	void unpack_block(const char *region, int offset, int size);
	void unpack_region(const char *region);

	virtual void machine_start() override;
	virtual void machine_reset() override;
};
