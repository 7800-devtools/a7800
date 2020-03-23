// license:BSD-3-Clause
// copyright-holders:Curt Coder, Robbbert, Wilbert Pol
#pragma once

#ifndef __OSI__
#define __OSI__


#include "cpu/m6502/m6502.h"
#include "formats/basicdsk.h"
#include "imagedev/cassette.h"
#include "imagedev/floppy.h"
#include "machine/6850acia.h"
#include "machine/6821pia.h"
#include "machine/ram.h"
#include "sound/discrete.h"
#include "sound/beep.h"

#define SCREEN_TAG      "screen"
#define M6502_TAG       "m6502"
#define DISCRETE_TAG    "discrete"

#define X1          3932160
#define UK101_X1    XTAL_8MHz

#define OSI600_VIDEORAM_SIZE    0x400
#define OSI630_COLORRAM_SIZE    0x400

class sb2m600_state : public driver_device
{
public:
	sb2m600_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, M6502_TAG)
		, m_acia_0(*this, "acia_0")
		, m_cassette(*this, "cassette")
		, m_discrete(*this, DISCRETE_TAG)
		, m_ram(*this, RAM_TAG)
		, m_video_ram(*this, "video_ram")
		, m_color_ram(*this, "color_ram")
		, m_p_chargen(*this, "chargen")
		, m_io_keyboard(*this, "ROW%u", 0)
		, m_io_sound(*this, "Sound")
		, m_io_reset(*this, "Reset")
		, m_beeper(*this, "beeper")
		{ }

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_READ8_MEMBER( keyboard_r );
	DECLARE_WRITE8_MEMBER( keyboard_w );
	DECLARE_WRITE8_MEMBER( ctrl_w );
	DECLARE_WRITE_LINE_MEMBER( cassette_tx );
	DECLARE_WRITE_LINE_MEMBER( write_cassette_clock );

	void floppy_index_callback(floppy_image_device *floppy, int state);

	DECLARE_PALETTE_INIT(osi630);

protected:
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual void machine_start() override;
	virtual void video_start() override;

	enum
	{
		TIMER_SETUP_BEEP
	};

	required_device<cpu_device> m_maincpu;
	required_device<acia6850_device> m_acia_0;
	required_device<cassette_image_device> m_cassette;
	optional_device<discrete_sound_device> m_discrete;
	required_device<ram_device> m_ram;
	required_shared_ptr<uint8_t> m_video_ram;
	optional_shared_ptr<uint8_t> m_color_ram;
	required_region_ptr<u8> m_p_chargen;
	required_ioport_array<8> m_io_keyboard;
	required_ioport m_io_sound;
	required_ioport m_io_reset;
	optional_device<beep_device> m_beeper;

	/* floppy state */
	int m_fdc_index;

	/* keyboard state */
	uint8_t m_keylatch;

	/* video state */
	int m_32;
	int m_coloren;
};

class c1p_state : public sb2m600_state
{
public:
	c1p_state(const machine_config &mconfig, device_type type, const char *tag)
		: sb2m600_state(mconfig, type, tag)
		, m_beep(*this, "beeper")
	{ }

	required_device<beep_device> m_beep;

	virtual void machine_start() override;

	DECLARE_WRITE8_MEMBER( osi630_ctrl_w );
	DECLARE_WRITE8_MEMBER( osi630_sound_w );
	DECLARE_DRIVER_INIT(c1p);
};

class c1pmf_state : public c1p_state
{
public:
	c1pmf_state(const machine_config &mconfig, device_type type, const char *tag)
		: c1p_state(mconfig, type, tag)
		, m_floppy0(*this, "floppy0")
		, m_floppy1(*this, "floppy1")
		{ }

	DECLARE_READ8_MEMBER( osi470_pia_pa_r );
	DECLARE_WRITE8_MEMBER( osi470_pia_pa_w );
	DECLARE_WRITE8_MEMBER( osi470_pia_pb_w );
	DECLARE_WRITE_LINE_MEMBER( osi470_pia_cb2_w );

protected:
	virtual void machine_start() override;

private:
	required_device<floppy_connector> m_floppy0;
	required_device<floppy_connector> m_floppy1;
};

class uk101_state : public sb2m600_state
{
public:
	uk101_state(const machine_config &mconfig, device_type type, const char *tag)
		: sb2m600_state(mconfig, type, tag)
		{ }

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	DECLARE_WRITE8_MEMBER( keyboard_w );
};

/* ---------- defined in video/osi.c ---------- */

MACHINE_CONFIG_EXTERN( osi600_video );
MACHINE_CONFIG_EXTERN( uk101_video );
MACHINE_CONFIG_EXTERN( osi630_video );

#endif
