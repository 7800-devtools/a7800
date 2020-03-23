// license:BSD-3-Clause
// copyright-holders:Paul Daniels
/*****************************************************************************
 *
 * includes/p2000t.h
 *
 ****************************************************************************/

#ifndef MAME_INCLUDES_P2000T_H
#define MAME_INCLUDES_P2000T_H

#pragma once

#include "cpu/z80/z80.h"
#include "sound/spkrdev.h"
#include "video/saa5050.h"


class p2000t_state : public driver_device
{
public:
	p2000t_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_speaker(*this, "speaker")
		, m_gfxdecode(*this, "gfxdecode")
		, m_palette(*this, "palette")
		, m_videoram(*this, "videoram")
		, m_keyboard(*this, "KEY.%u", 0)
	{
	}

	required_device<cpu_device> m_maincpu;
	required_device<speaker_sound_device> m_speaker;
	optional_device<gfxdecode_device> m_gfxdecode;
	optional_device<palette_device> m_palette;
	required_shared_ptr<uint8_t> m_videoram;
	required_ioport_array<10> m_keyboard;
	DECLARE_READ8_MEMBER(p2000t_port_000f_r);
	DECLARE_READ8_MEMBER(p2000t_port_202f_r);
	DECLARE_WRITE8_MEMBER(p2000t_port_101f_w);
	DECLARE_WRITE8_MEMBER(p2000t_port_303f_w);
	DECLARE_WRITE8_MEMBER(p2000t_port_505f_w);
	DECLARE_WRITE8_MEMBER(p2000t_port_707f_w);
	DECLARE_WRITE8_MEMBER(p2000t_port_888b_w);
	DECLARE_WRITE8_MEMBER(p2000t_port_8c90_w);
	DECLARE_WRITE8_MEMBER(p2000t_port_9494_w);
	DECLARE_READ8_MEMBER(videoram_r);
	uint8_t m_port_101f;
	uint8_t m_port_202f;
	uint8_t m_port_303f;
	uint8_t m_port_707f;
	int8_t m_frame_count;
	DECLARE_VIDEO_START(p2000m);
	DECLARE_PALETTE_INIT(p2000m);
	uint32_t screen_update_p2000m(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(p2000_interrupt);
};

#endif // MAME_INCLUDES_P2000T_H
