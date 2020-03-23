// license:BSD-3-Clause
// copyright-holders:hap
/******************************************************************************
*
*  Novag chess machines base class
*  main driver is novag6502.cpp
*
******************************************************************************/

#pragma once

#ifndef DRIVERS_NOVAGBASE_H
#define DRIVERS_NOVAGBASE_H

#include "sound/dac.h"
#include "sound/beep.h"
#include "video/hd44780.h"

class novagbase_state : public driver_device
{
public:
	novagbase_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_dac(*this, "dac"),
		m_beeper(*this, "beeper"),
		m_lcd(*this, "hd44780"),
		m_inp_matrix(*this, "IN.%u", 0),
		m_display_wait(33),
		m_display_maxy(1),
		m_display_maxx(0)
	{ }

	// devices/pointers
	required_device<cpu_device> m_maincpu;
	optional_device<dac_bit_interface> m_dac;
	optional_device<beep_device> m_beeper;
	optional_device<hd44780_device> m_lcd;
	optional_ioport_array<9> m_inp_matrix; // max 9

	// misc common
	u16 m_inp_mux;                  // multiplexed keypad mask
	u16 m_led_select;
	u16 m_led_data;
	u8 m_lcd_control;

	u16 read_inputs(int columns);

	// display common
	int m_display_wait;             // led/lamp off-delay in milliseconds (default 33ms)
	int m_display_maxy;             // display matrix number of rows
	int m_display_maxx;             // display matrix number of columns (max 31 for now)

	u32 m_display_state[0x20];      // display matrix rows data (last bit is used for always-on)
	u16 m_display_segmask[0x20];    // if not 0, display matrix row is a digit, mask indicates connected segments
	u32 m_display_cache[0x20];      // (internal use)
	u8 m_display_decay[0x20][0x20]; // (internal use)

	TIMER_DEVICE_CALLBACK_MEMBER(display_decay_tick);
	void display_update();
	void set_display_size(int maxx, int maxy);
	void set_display_segmask(u32 digits, u32 mask);
	void display_matrix(int maxx, int maxy, u32 setx, u32 sety, bool update = true);

	DECLARE_PALETTE_INIT(novag_lcd);
	HD44780_PIXEL_UPDATE(novag_lcd_pixel_update);

protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;
};


INPUT_PORTS_EXTERN( novag_cb_buttons );
INPUT_PORTS_EXTERN( novag_cb_magnets );

#endif // DRIVERS_NOVAGBASE_H
