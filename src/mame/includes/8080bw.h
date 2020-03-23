// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Tormod Tjaberg, Mirko Buffoni,Lee Taylor, Valerio Verrando, Zsolt Vasvari
// thanks-to:Michael Strutts, Marco Cassili
/***************************************************************************

    8080-based black and white hardware

****************************************************************************/
#ifndef MAME_INCLUDES_8080BW_H
#define MAME_INCLUDES_8080BW_H

#pragma once

#include "includes/mw8080bw.h"

#include "machine/eepromser.h"
#include "sound/sn76477.h"
#include "sound/spkrdev.h"

#include "screen.h"


/* for games in 8080bw.c */
#define CABINET_PORT_TAG                  "CAB"


class _8080bw_state : public mw8080bw_state
{
public:
	_8080bw_state(const machine_config &mconfig, device_type type, const char *tag)
		: mw8080bw_state(mconfig, type, tag),
		m_schaser_effect_555_timer(*this, "schaser_sh_555"),
		m_claybust_gun_on(*this, "claybust_gun"),
		m_discrete(*this, "discrete"),
		m_speaker(*this, "speaker"),
		m_eeprom(*this, "eeprom"),
		m_sn(*this, "snsnd"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_gunx(*this, "GUNX"),
		m_guny(*this, "GUNY")
	{ }

	/* devices/memory pointers */
	optional_device<timer_device> m_schaser_effect_555_timer;
	optional_device<timer_device> m_claybust_gun_on;
	optional_device<discrete_device> m_discrete;
	optional_device<speaker_sound_device> m_speaker;
	optional_device<eeprom_serial_93cxx_device> m_eeprom;
	optional_device<sn76477_device> m_sn;
	required_device<screen_device> m_screen;
	optional_device<palette_device> m_palette;

	/* misc game specific */
	optional_ioport m_gunx;
	optional_ioport m_guny;
	uint8_t m_color_map;
	uint8_t m_screen_red;
	uint8_t m_fleet_step;

	attotime m_schaser_effect_555_time_remain;
	int32_t m_schaser_effect_555_time_remain_savable;
	int m_schaser_effect_555_is_low;
	int m_schaser_explosion;
	int m_schaser_last_effect;
	uint8_t m_polaris_cloud_speed;
	uint8_t m_polaris_cloud_pos;
	uint8_t m_schaser_background_disable;
	uint8_t m_schaser_background_select;
	uint16_t m_claybust_gun_pos;

	DECLARE_CUSTOM_INPUT_MEMBER(sflush_80_r);
	DECLARE_INPUT_CHANGED_MEMBER(claybust_gun_trigger);
	DECLARE_CUSTOM_INPUT_MEMBER(claybust_gun_on_r);

	DECLARE_READ8_MEMBER(indianbt_r);
	DECLARE_READ8_MEMBER(polaris_port00_r);
	DECLARE_WRITE8_MEMBER(steelwkr_sh_port_3_w);
	DECLARE_WRITE8_MEMBER(invadpt2_sh_port_1_w);
	DECLARE_WRITE8_MEMBER(invadpt2_sh_port_2_w);
	DECLARE_WRITE8_MEMBER(spacerng_sh_port_2_w);
	DECLARE_WRITE8_MEMBER(spcewars_sh_port_w);
	DECLARE_WRITE8_MEMBER(lrescue_sh_port_1_w);
	DECLARE_WRITE8_MEMBER(lrescue_sh_port_2_w);
	DECLARE_WRITE8_MEMBER(cosmicmo_05_w);
	DECLARE_WRITE8_MEMBER(cosmo_sh_port_2_w);
	DECLARE_READ8_MEMBER(darthvdr_01_r);
	DECLARE_WRITE8_MEMBER(darthvdr_00_w);
	DECLARE_WRITE8_MEMBER(darthvdr_08_w);
	DECLARE_WRITE8_MEMBER(ballbomb_01_w);
	DECLARE_WRITE8_MEMBER(ballbomb_sh_port_1_w);
	DECLARE_WRITE8_MEMBER(ballbomb_sh_port_2_w);
	DECLARE_WRITE8_MEMBER(indianbt_sh_port_1_w);
	DECLARE_WRITE8_MEMBER(indianbt_sh_port_2_w);
	DECLARE_WRITE8_MEMBER(indianbtbr_sh_port_1_w);
	DECLARE_WRITE8_MEMBER(indianbtbr_sh_port_2_w);
	DECLARE_READ8_MEMBER(indianbtbr_01_r);
	DECLARE_WRITE8_MEMBER(schaser_sh_port_1_w);
	DECLARE_WRITE8_MEMBER(schaser_sh_port_2_w);
	DECLARE_WRITE8_MEMBER(rollingc_sh_port_w);
	DECLARE_READ8_MEMBER(invrvnge_02_r);
	DECLARE_WRITE8_MEMBER(invrvnge_sh_port_1_w);
	DECLARE_WRITE8_MEMBER(invrvnge_sh_port_2_w);
	DECLARE_WRITE8_MEMBER(lupin3_00_w);
	DECLARE_WRITE8_MEMBER(lupin3_sh_port_1_w);
	DECLARE_WRITE8_MEMBER(lupin3_sh_port_2_w);
	DECLARE_READ8_MEMBER(schasercv_02_r);
	DECLARE_WRITE8_MEMBER(schasercv_sh_port_1_w);
	DECLARE_WRITE8_MEMBER(schasercv_sh_port_2_w);
	DECLARE_WRITE8_MEMBER(yosakdon_sh_port_1_w);
	DECLARE_WRITE8_MEMBER(yosakdon_sh_port_2_w);
	DECLARE_READ8_MEMBER(shuttlei_ff_r);
	DECLARE_WRITE8_MEMBER(shuttlei_ff_w);
	DECLARE_WRITE8_MEMBER(shuttlei_sh_port_1_w);
	DECLARE_WRITE8_MEMBER(shuttlei_sh_port_2_w);
	DECLARE_READ8_MEMBER(claybust_gun_lo_r);
	DECLARE_READ8_MEMBER(claybust_gun_hi_r);
	DECLARE_READ8_MEMBER(invmulti_eeprom_r);
	DECLARE_WRITE8_MEMBER(invmulti_eeprom_w);
	DECLARE_WRITE8_MEMBER(invmulti_bank_w);

	DECLARE_READ8_MEMBER(rollingc_scattered_colorram_r);
	DECLARE_WRITE8_MEMBER(rollingc_scattered_colorram_w);
	DECLARE_READ8_MEMBER(rollingc_scattered_colorram2_r);
	DECLARE_WRITE8_MEMBER(rollingc_scattered_colorram2_w);
	DECLARE_READ8_MEMBER(schaser_scattered_colorram_r);
	DECLARE_WRITE8_MEMBER(schaser_scattered_colorram_w);

	DECLARE_DRIVER_INIT(invmulti);
	DECLARE_DRIVER_INIT(spacecom);
	DECLARE_DRIVER_INIT(vortex);
	DECLARE_DRIVER_INIT(attackfc);

	DECLARE_MACHINE_START(extra_8080bw);
	DECLARE_MACHINE_START(rollingc);
	DECLARE_MACHINE_START(sflush);
	DECLARE_MACHINE_START(schaser);
	DECLARE_MACHINE_START(schasercv);
	DECLARE_MACHINE_RESET(schaser);
	DECLARE_MACHINE_START(polaris);
	DECLARE_MACHINE_START(darthvdr);
	DECLARE_MACHINE_RESET(darthvdr);
	DECLARE_MACHINE_START(extra_8080bw_sh);
	DECLARE_MACHINE_START(extra_8080bw_vh);
	DECLARE_MACHINE_START(schaser_sh);
	DECLARE_MACHINE_RESET(schaser_sh);
	DECLARE_MACHINE_START(claybust);

	DECLARE_PALETTE_INIT(rollingc);
	DECLARE_PALETTE_INIT(sflush);

	uint32_t screen_update_invadpt2(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_cosmo(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_rollingc(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_schaser(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_schasercv(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_sflush(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_indianbt(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_lupin3(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_polaris(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_ballbomb(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_shuttlei(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	uint32_t screen_update_spacecom(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	INTERRUPT_GEN_MEMBER(polaris_interrupt);
	TIMER_DEVICE_CALLBACK_MEMBER(claybust_gun_callback);
	TIMER_DEVICE_CALLBACK_MEMBER(schaser_effect_555_cb);
	DECLARE_WRITE8_MEMBER(indianbt_sh_port_3_w);
	DECLARE_WRITE8_MEMBER(polaris_sh_port_1_w);
	DECLARE_WRITE8_MEMBER(polaris_sh_port_2_w);
	DECLARE_WRITE8_MEMBER(polaris_sh_port_3_w);

	void schaser_reinit_555_time_remain();
	inline void set_pixel( bitmap_rgb32 &bitmap, uint8_t y, uint8_t x, int color );
	inline void set_8_pixels( bitmap_rgb32 &bitmap, uint8_t y, uint8_t x, uint8_t data, int fore_color, int back_color );
	void clear_extra_columns( bitmap_rgb32 &bitmap, int color );
};


/*----------- defined in audio/8080bw.c -----------*/
extern const char *const lrescue_sample_names[];
extern const char *const lupin3_sample_names[];

DISCRETE_SOUND_EXTERN( ballbomb );
DISCRETE_SOUND_EXTERN( indianbt );
DISCRETE_SOUND_EXTERN( polaris );
DISCRETE_SOUND_EXTERN( schaser );

#endif // MAME_INCLUDES_8080BW_H
