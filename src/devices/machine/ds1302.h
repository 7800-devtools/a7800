// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Dallas DS1302 Trickle-Charge Timekeeping Chip emulation

**********************************************************************
                            _____   _____
                  Vcc2   1 |*    \_/     | 8   Vcc1
                    X1   2 |             | 7   SCLK
                    X2   3 |             | 6   I/O
                   GND   4 |_____________| 5   CE

**********************************************************************/

#ifndef MAME_MACHINE_DS1302_H
#define MAME_MACHINE_DS1302_H

#pragma once

#include "dirtc.h"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_DS1302_ADD(_tag, _clock) \
	MCFG_DEVICE_ADD(_tag, DS1302, _clock)



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> ds1302_device

class ds1302_device :  public device_t,
						public device_rtc_interface,
						public device_nvram_interface
{
public:
	// construction/destruction
	ds1302_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_WRITE_LINE_MEMBER( ce_w );
	DECLARE_WRITE_LINE_MEMBER( sclk_w );
	DECLARE_WRITE_LINE_MEMBER( io_w );
	DECLARE_READ_LINE_MEMBER( io_r );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// device_nvram_interface overrides
	virtual void nvram_default() override;
	virtual void nvram_read(emu_file &file) override;
	virtual void nvram_write(emu_file &file) override;

	// device_rtc_interface overrides
	virtual void rtc_clock_updated(int year, int month, int day, int day_of_week, int hour, int minute, int second) override;
	virtual bool rtc_feature_leap_year() const override { return true; }

private:
	void load_shift_register();
	void input_bit();
	void output_bit();

	int m_ce;
	int m_clk;
	int m_io;
	int m_state;
	int m_bits;
	uint8_t m_cmd;
	uint8_t m_data;
	int m_addr;

	uint8_t m_reg[9];
	uint8_t m_user[9];
	uint8_t m_ram[0x20];

	// timers
	emu_timer *m_clock_timer;
};


// device type definition
DECLARE_DEVICE_TYPE(DS1302, ds1302_device)

#endif // MAME_MACHINE_DS1302_H
