// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/*****************************************************************************
 *
 * includes/orion.h
 *
 ****************************************************************************/

#ifndef MAME_INCLUDES_ORION_H
#define MAME_INCLUDES_ORION_H

#pragma once

#include "includes/radio86.h"

#include "cpu/i8085/i8085.h"
#include "cpu/z80/z80.h"

#include "machine/i8255.h"
#include "machine/mc146818.h"
#include "machine/ram.h"
#include "machine/wd_fdc.h"

#include "sound/ay8910.h"
#include "sound/spkrdev.h"
#include "sound/wave.h"


class orion_state : public radio86_state
{
public:
	orion_state(const machine_config &mconfig, device_type type, const char *tag)
		: radio86_state(mconfig, type, tag)
		, m_fdc(*this, "fd1793")
		, m_ram(*this, RAM_TAG)
		, m_fd0(*this, "fd0")
		, m_fd1(*this, "fd1")
		, m_fd2(*this, "fd2")
		, m_fd3(*this, "fd3")
		, m_rtc(*this, "rtc")
		, m_speaker(*this, "speaker")
		, m_ay8912(*this, "ay8912")
		, m_bank2(*this, "bank2")
		, m_bank3(*this, "bank3")
		, m_bank4(*this, "bank4")
		, m_bank5(*this, "bank5")
		, m_bank6(*this, "bank6")
		, m_bank7(*this, "bank7")
		, m_bank8(*this, "bank8")
	{ }

	DECLARE_READ8_MEMBER(orion128_system_r);
	DECLARE_WRITE8_MEMBER(orion128_system_w);
	DECLARE_READ8_MEMBER(orion128_romdisk_r);
	DECLARE_WRITE8_MEMBER(orion128_romdisk_w);
	DECLARE_WRITE8_MEMBER(orion128_video_mode_w);
	DECLARE_WRITE8_MEMBER(orion128_video_page_w);
	DECLARE_WRITE8_MEMBER(orion128_memory_page_w);
	DECLARE_WRITE8_MEMBER(orion_disk_control_w);
	DECLARE_READ8_MEMBER(orion128_floppy_r);
	DECLARE_WRITE8_MEMBER(orion128_floppy_w);
	DECLARE_READ8_MEMBER(orionz80_floppy_rtc_r);
	DECLARE_WRITE8_MEMBER(orionz80_floppy_rtc_w);
	DECLARE_WRITE8_MEMBER(orionz80_sound_w);
	DECLARE_WRITE8_MEMBER(orionz80_sound_fe_w);
	DECLARE_WRITE8_MEMBER(orionz80_memory_page_w);
	DECLARE_WRITE8_MEMBER(orionz80_dispatcher_w);
	DECLARE_READ8_MEMBER(orionz80_io_r);
	DECLARE_WRITE8_MEMBER(orionz80_io_w);
	DECLARE_WRITE8_MEMBER(orionpro_memory_page_w);
	DECLARE_READ8_MEMBER(orionpro_io_r);
	DECLARE_WRITE8_MEMBER(orionpro_io_w);
	DECLARE_MACHINE_START(orion128);
	DECLARE_MACHINE_RESET(orion128);
	DECLARE_VIDEO_START(orion128);
	DECLARE_PALETTE_INIT(orion128);
	DECLARE_MACHINE_START(orionz80);
	DECLARE_MACHINE_RESET(orionz80);
	DECLARE_MACHINE_RESET(orionpro);
	uint32_t screen_update_orion128(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(orionz80_interrupt);
	DECLARE_READ8_MEMBER(orion_romdisk_porta_r);
	DECLARE_WRITE8_MEMBER(orion_romdisk_portb_w);
	DECLARE_WRITE8_MEMBER(orion_romdisk_portc_w);
	DECLARE_FLOPPY_FORMATS( orion_floppy_formats );

protected:
	uint8_t m_orion128_video_mode;
	uint8_t m_orion128_video_page;
	uint8_t m_orion128_video_width;
	uint8_t m_video_mode_mask;
	uint8_t m_orionpro_pseudo_color;
	uint8_t m_romdisk_lsb;
	uint8_t m_romdisk_msb;
	uint8_t m_orion128_memory_page;
	uint8_t m_orionz80_memory_page;
	uint8_t m_orionz80_dispatcher;
	uint8_t m_speaker_data;
	uint8_t m_orionpro_ram0_segment;
	uint8_t m_orionpro_ram1_segment;
	uint8_t m_orionpro_ram2_segment;
	uint8_t m_orionpro_page;
	uint8_t m_orionpro_128_page;
	uint8_t m_orionpro_rom2_segment;
	uint8_t m_orionpro_dispatcher;

	required_device<fd1793_device> m_fdc;

	required_device<ram_device> m_ram;
	required_device<floppy_connector> m_fd0;
	required_device<floppy_connector> m_fd1;
	required_device<floppy_connector> m_fd2;
	required_device<floppy_connector> m_fd3;
	optional_device<mc146818_device> m_rtc;
	optional_device<speaker_sound_device> m_speaker;
	optional_device<ay8910_device> m_ay8912;
	required_memory_bank m_bank2;
	optional_memory_bank m_bank3;
	optional_memory_bank m_bank4;
	optional_memory_bank m_bank5;
	optional_memory_bank m_bank6;
	optional_memory_bank m_bank7;
	optional_memory_bank m_bank8;

	void orionz80_switch_bank();
	void orion_set_video_mode(int width);
	void orionpro_bank_switch();
};

#endif // MAME_INCLUDES_ORION_H
