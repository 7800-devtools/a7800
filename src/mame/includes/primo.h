// license:BSD-3-Clause
// copyright-holders:Krzysztof Strzecha
/*****************************************************************************
 *
 * includes/primo.h
 *
 ****************************************************************************/

#ifndef MAME_INCLUDES_PRIMO_H
#define MAME_INCLUDES_PRIMO_H

#pragma once

#include "imagedev/snapquik.h"
#include "imagedev/cassette.h"
#include "sound/spkrdev.h"

#include "bus/cbmiec/cbmiec.h"
#include "bus/generic/slot.h"
#include "bus/generic/carts.h"

class primo_state : public driver_device
{
public:
	primo_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_iec(*this, CBM_IEC_TAG),
		m_speaker(*this, "speaker"),
		m_cassette(*this, "cassette"),
		m_cart1(*this, "cartslot1"),
		m_cart2(*this, "cartslot2")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<cbm_iec_device> m_iec;
	required_device<speaker_sound_device> m_speaker;
	required_device<cassette_image_device> m_cassette;
	required_device<generic_slot_device> m_cart1;
	required_device<generic_slot_device> m_cart2;

	memory_region *m_cart1_rom;
	memory_region *m_cart2_rom;

	uint16_t m_video_memory_base;
	uint8_t m_port_FD;
	int m_nmi;
	DECLARE_READ8_MEMBER(primo_be_1_r);
	DECLARE_READ8_MEMBER(primo_be_2_r);
	DECLARE_WRITE8_MEMBER(primo_ki_1_w);
	DECLARE_WRITE8_MEMBER(primo_ki_2_w);
	DECLARE_WRITE8_MEMBER(primo_FD_w);
	DECLARE_DRIVER_INIT(primo48);
	DECLARE_DRIVER_INIT(primo64);
	DECLARE_DRIVER_INIT(primo32);
	virtual void machine_reset() override;
	virtual void machine_start() override;
	DECLARE_MACHINE_RESET(primob);
	uint32_t screen_update_primo(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(primo_vblank_interrupt);
	void primo_draw_scanline(bitmap_ind16 &bitmap, int primo_scanline);
	void primo_update_memory();
	void primo_common_driver_init (primo_state *state);
	void primo_common_machine_init ();
	void primo_setup_pss (uint8_t* snapshot_data, uint32_t snapshot_size);
	void primo_setup_pp (uint8_t* quickload_data, uint32_t quickload_size);
	DECLARE_SNAPSHOT_LOAD_MEMBER( primo );
	DECLARE_QUICKLOAD_LOAD_MEMBER( primo );
};


#endif // MAME_INCLUDES_PRIMO_H
