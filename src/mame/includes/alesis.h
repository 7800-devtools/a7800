// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/***************************************************************************

    Alesis HR-16 and SR-16 drum machines

****************************************************************************/

#ifndef MAME_INCLUDES_ALESIS_H
#define MAME_INCLUDES_ALESIS_H

#pragma once

#include "cpu/mcs51/mcs51.h"
#include "machine/nvram.h"
#include "sound/dac.h"
#include "video/hd44780.h"
#include "imagedev/cassette.h"
#include "rendlay.h"

#define MCFG_ALESIS_DM3AG_ADD(_tag,_clock) \
	MCFG_DEVICE_ADD( _tag, ALESIS_DM3AG, _clock )


// ======================> alesis_dm3ag_device

class alesis_dm3ag_device : public device_t
{
public:
	// construction/destruction
	alesis_dm3ag_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device interface
	DECLARE_WRITE8_MEMBER(write);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	static const device_timer_id TIMER_DAC_UPDATE = 1;
	required_device<dac_word_interface> m_dac;
	required_region_ptr<int8_t> m_samples;

	emu_timer * m_dac_update_timer;
	bool        m_output_active;
	int         m_count;
	int         m_shift;
	uint32_t      m_cur_sample;
	uint8_t       m_cmd[5];
};


// ======================> alesis_state

class alesis_state : public driver_device
{
public:
	alesis_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_lcdc(*this, "hd44780"),
			m_cassette(*this, "cassette"),
			m_maincpu(*this, "maincpu"),
			m_col1(*this, "COL1"),
			m_col2(*this, "COL2"),
			m_col3(*this, "COL3"),
			m_col4(*this, "COL4"),
			m_col5(*this, "COL5"),
			m_col6(*this, "COL6"),
			m_select(*this, "SELECT")
	{ }

	required_device<hd44780_device> m_lcdc;
	optional_device<cassette_image_device> m_cassette;

	DECLARE_PALETTE_INIT(alesis);
	virtual void machine_reset() override;

	void update_lcd_symbols(bitmap_ind16 &bitmap, uint8_t pos, uint8_t y, uint8_t x, int state);
	DECLARE_DRIVER_INIT(hr16);
	DECLARE_WRITE8_MEMBER( led_w );
	DECLARE_WRITE8_MEMBER( mmt8_led_w );
	DECLARE_READ8_MEMBER( mmt8_led_r );
	DECLARE_WRITE8_MEMBER( track_led_w );
	DECLARE_WRITE8_MEMBER( kb_matrix_w );
	DECLARE_READ8_MEMBER( kb_r );
	DECLARE_READ8_MEMBER( p3_r );
	DECLARE_WRITE8_MEMBER( p3_w );
	DECLARE_READ8_MEMBER( mmt8_p3_r );
	DECLARE_WRITE8_MEMBER( mmt8_p3_w );
	DECLARE_WRITE8_MEMBER( sr16_lcd_w );
	HD44780_PIXEL_UPDATE(sr16_pixel_update);

private:
	uint8_t       m_kb_matrix;
	uint8_t       m_leds;
	uint8_t       m_lcd_digits[5];
	required_device<cpu_device> m_maincpu;
	required_ioport m_col1;
	required_ioport m_col2;
	required_ioport m_col3;
	required_ioport m_col4;
	required_ioport m_col5;
	required_ioport m_col6;
	optional_ioport m_select;
};

// device type definition
DECLARE_DEVICE_TYPE(ALESIS_DM3AG, alesis_dm3ag_device)

#endif  // MAME_INCLUDES_ALESIS_H
