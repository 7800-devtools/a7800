// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/***************************************************************************

        VTech Laser PC4

****************************************************************************/

#pragma once

#ifndef _PC4_H_
#define _PC4_H_


#include "sound/beep.h"

class pc4_state : public driver_device
{
public:
	pc4_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_beep(*this, "beeper"),
		m_region_charset(*this, "charset"),
		m_rombank(*this, "rombank") { }

	required_device<cpu_device> m_maincpu;
	required_device<beep_device> m_beep;

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	virtual void machine_start() override;

	DECLARE_WRITE8_MEMBER( beep_w );
	DECLARE_WRITE8_MEMBER( bank_w );
	DECLARE_READ8_MEMBER( kb_r );

	//LCD controller
	void update_ac(void);
	void set_busy_flag(uint16_t usec);
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	DECLARE_WRITE8_MEMBER(lcd_control_w);
	DECLARE_READ8_MEMBER(lcd_control_r);
	DECLARE_WRITE8_MEMBER(lcd_data_w);
	DECLARE_READ8_MEMBER(lcd_data_r);
	DECLARE_WRITE8_MEMBER( lcd_offset_w );

	static const device_timer_id BUSY_TIMER = 0;
	static const device_timer_id BLINKING_TIMER = 1;

	emu_timer *m_blink_timer;
	emu_timer *m_busy_timer;

	uint8_t m_busy_flag;
	uint8_t m_ddram[0xa0];
	uint8_t m_cgram[0x40];
	int16_t m_ac;
	uint8_t m_ac_mode;
	uint8_t m_data_bus_flag;
	int16_t m_cursor_pos;
	uint8_t m_display_on;
	uint8_t m_cursor_on;
	uint8_t m_blink_on;
	uint8_t m_shift_on;
	int8_t m_disp_shift;
	int8_t m_direction;
	uint8_t m_blink;
	DECLARE_PALETTE_INIT(pc4);

protected:
	required_memory_region m_region_charset;
	required_memory_bank m_rombank;
	ioport_port *io_port[8];
};

#endif  // _PC4_H_
