// license:BSD-3-Clause
// copyright-holders:hap
/*

  TMS1000 MCU series tabletops/handhelds or other simple devices.

*/

#ifndef MAME_INCLUDES_HH_TMS1K_H
#define MAME_INCLUDES_HH_TMS1K_H

#pragma once

#include "cpu/tms1000/tms1000.h"
#include "cpu/tms1000/tms1100.h"
#include "cpu/tms1000/tms1400.h"
#include "cpu/tms1000/tms0970.h"
#include "cpu/tms1000/tms0980.h"
#include "cpu/tms1000/tms0270.h"
#include "cpu/tms1000/tp0320.h"
#include "sound/spkrdev.h"


class hh_tms1k_state : public driver_device
{
public:
	hh_tms1k_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_inp_matrix(*this, "IN.%u", 0),
		m_speaker(*this, "speaker"),
		m_display_wait(33),
		m_display_maxy(1),
		m_display_maxx(0)
	{ }

	// devices
	required_device<tms1k_base_device> m_maincpu;
	optional_ioport_array<18> m_inp_matrix; // max 18
	optional_device<speaker_sound_device> m_speaker;

	// misc common
	u16 m_r;                        // MCU R-pins data
	u16 m_o;                        // MCU O-pins data
	u32 m_inp_mux;                  // multiplexed inputs mask
	bool m_power_on;
	bool m_power_led;

	u8 read_inputs(int columns);
	u8 read_rotated_inputs(int columns, u8 rowmask = 0xf);
	virtual DECLARE_INPUT_CHANGED_MEMBER(power_button);
	virtual DECLARE_WRITE_LINE_MEMBER(auto_power_off);
	virtual void power_off();

	// display common
	int m_display_wait;             // led/lamp off-delay in milliseconds (default 33ms)
	int m_display_maxy;             // display matrix number of rows
	int m_display_maxx;             // display matrix number of columns (max 31 for now)

	u32 m_grid;                     // VFD/LED current row data
	u32 m_plate;                    // VFD/LED current column data

	u32 m_display_state[0x20];      // display matrix rows data (last bit is used for always-on)
	u16 m_display_segmask[0x20];    // if not 0, display matrix row is a digit, mask indicates connected segments
	u32 m_display_cache[0x20];      // (internal use)
	u8 m_display_decay[0x20][0x20]; // (internal use)

	TIMER_DEVICE_CALLBACK_MEMBER(display_decay_tick);
	void display_update();
	void set_display_size(int maxx, int maxy);
	void set_display_segmask(u32 digits, u32 mask);
	void display_matrix(int maxx, int maxy, u32 setx, u32 sety, bool update = true);

protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;
};


// LED segments
enum
{
	lA = 0x01,
	lB = 0x02,
	lC = 0x04,
	lD = 0x08,
	lE = 0x10,
	lF = 0x20,
	lG = 0x40,
	lDP = 0x80
};

#endif // MAME_INCLUDES_HH_TMS1K_H
