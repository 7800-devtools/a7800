// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/*****************************************************************************
 *
 * includes/pp01.h
 *
 ****************************************************************************/

#ifndef MAME_INCLUDES_PP01_H
#define MAME_INCLUDES_PP01_H

#pragma once

#include "cpu/i8085/i8085.h"
#include "machine/ram.h"
#include "machine/i8251.h"
#include "machine/pit8253.h"
#include "machine/i8255.h"
#include "sound/spkrdev.h"
//#include "sound/wave.h"
//#include "imagedev/cassette.h"

class pp01_state : public driver_device
{
public:
	pp01_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_pit(*this, "pit8253"),
		m_speaker(*this, "speaker"),
		m_ram(*this, RAM_TAG) { }

	required_device<cpu_device> m_maincpu;
	required_device<pit8253_device> m_pit;
	required_device<speaker_sound_device> m_speaker;
	required_device<ram_device> m_ram;
	uint8_t m_video_scroll;
	uint8_t m_memory_block[16];
	uint8_t m_video_write_mode;
	uint8_t m_key_line;
	DECLARE_WRITE8_MEMBER(pp01_video_write_mode_w);
	DECLARE_WRITE8_MEMBER(pp01_video_r_1_w);
	DECLARE_WRITE8_MEMBER(pp01_video_g_1_w);
	DECLARE_WRITE8_MEMBER(pp01_video_b_1_w);
	DECLARE_WRITE8_MEMBER(pp01_video_r_2_w);
	DECLARE_WRITE8_MEMBER(pp01_video_g_2_w);
	DECLARE_WRITE8_MEMBER(pp01_video_b_2_w);
	DECLARE_WRITE8_MEMBER(pp01_mem_block_w);
	DECLARE_READ8_MEMBER(pp01_mem_block_r);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	DECLARE_PALETTE_INIT(pp01);
	uint32_t screen_update_pp01(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(pp01_pit_out0);
	DECLARE_WRITE_LINE_MEMBER(pp01_pit_out1);
	DECLARE_READ8_MEMBER(pp01_8255_porta_r);
	DECLARE_WRITE8_MEMBER(pp01_8255_porta_w);
	DECLARE_READ8_MEMBER(pp01_8255_portb_r);
	DECLARE_WRITE8_MEMBER(pp01_8255_portb_w);
	DECLARE_WRITE8_MEMBER(pp01_8255_portc_w);
	DECLARE_READ8_MEMBER(pp01_8255_portc_r);
	void pp01_video_w(uint8_t block,uint16_t offset,uint8_t data,uint8_t part);
	void pp01_set_memory(uint8_t block, uint8_t data);
};

#endif // MAME_INCLUDES_PP01_H
