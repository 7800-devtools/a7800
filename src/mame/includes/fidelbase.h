// license:BSD-3-Clause
// copyright-holders:hap
/******************************************************************************
*
*  Fidelity Electronics chess machines base class
*  main driver is fidelz80.cpp
*
******************************************************************************/

#pragma once

#ifndef DRIVERS_FIDELBASE_H
#define DRIVERS_FIDELBASE_H

#include "sound/dac.h"
#include "sound/s14001a.h"
#include "bus/generic/slot.h"
#include "bus/generic/carts.h"
#include "softlist.h"

class fidelbase_state : public driver_device
{
public:
	fidelbase_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_inp_matrix(*this, "IN.%u", 0),
		m_speech(*this, "speech"),
		m_speech_rom(*this, "speech"),
		m_dac(*this, "dac"),
		m_cart(*this, "cartslot"),
		m_display_wait(33),
		m_display_maxy(1),
		m_display_maxx(0)
	{ }

	// devices/pointers
	required_device<cpu_device> m_maincpu;
	optional_ioport_array<11> m_inp_matrix; // max 11
	optional_device<s14001a_device> m_speech;
	optional_region_ptr<u8> m_speech_rom;
	optional_device<dac_bit_interface> m_dac;
	optional_device<generic_slot_device> m_cart;

	// misc common
	u16 m_inp_mux;                  // multiplexed keypad/leds mask
	u16 m_led_select;
	u32 m_7seg_data;                // data for seg leds
	u16 m_led_data;
	u8 m_speech_data;
	u8 m_speech_bank;               // speech rom higher address bits

	u16 read_inputs(int columns);
	DECLARE_DEVICE_IMAGE_LOAD_MEMBER(scc_cartridge);

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

protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;
};


INPUT_PORTS_EXTERN( fidel_cb_buttons );
INPUT_PORTS_EXTERN( fidel_cb_magnets );

#endif // DRIVERS_FIDELBASE_H
