// license:BSD-3-Clause
// copyright-holders:R. Belmont
/**********************************************************************

    rtc4543.h - Epson R4543 real-time clock emulation
    by R. Belmont

**********************************************************************/

#ifndef MAME_MACHINE_RTC4543_H
#define MAME_MACHINE_RTC4543_H

#pragma once

#include "dirtc.h"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_RTC4543_ADD(tag, clock) \
		MCFG_DEVICE_ADD((tag), RTC4543, (clock))

#define MCFG_RTC4543_DATA_CALLBACK(cb) \
		devcb = &rtc4543_device::set_data_cb(*device, DEVCB_##cb);

#define MCFG_JRC6355E_ADD(tag, clock) \
		MCFG_DEVICE_ADD((tag), JRC6355E, (clock))



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> rtc4543_device

class rtc4543_device :  public device_t,
						public device_rtc_interface
{
	static const char *s_reg_names[7];

public:
	// construction/destruction
	rtc4543_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_WRITE_LINE_MEMBER( ce_w );
	DECLARE_WRITE_LINE_MEMBER( wr_w );
	DECLARE_WRITE_LINE_MEMBER( clk_w );
	DECLARE_READ_LINE_MEMBER( data_r );
	DECLARE_WRITE_LINE_MEMBER( data_w );

	template <class Object> static devcb_base &set_data_cb(device_t &device, Object &&cb) { return downcast<rtc4543_device &>(device).data_cb.set_callback(std::forward<Object>(cb)); }

protected:
	rtc4543_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// device_rtc_interface overrides
	virtual void rtc_clock_updated(int year, int month, int day, int day_of_week, int hour, int minute, int second) override;
	virtual bool rtc_feature_leap_year() const override { return true; }

	// helpers
	virtual void ce_rising();
	virtual void ce_falling();
	virtual void clk_rising();
	virtual void clk_falling();
	void load_bit(int reg);
	void store_bit(int reg);
	void advance_bit();
	void update_effective();

	devcb_write_line data_cb;

	int m_ce;
	int m_clk;
	int m_wr;
	int m_data;
	int m_regs[7];
	int m_curbit;

	// timers
	emu_timer *m_clock_timer;
};



// ======================> jrc6355e_device

class jrc6355e_device : public rtc4543_device
{
public:
	// construction/destruction
	jrc6355e_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// rtc4543 overrides
	virtual void ce_rising() override;
	virtual void ce_falling() override;
	virtual void clk_rising() override;
	virtual void clk_falling() override;
};


// device type definition
DECLARE_DEVICE_TYPE(RTC4543,  rtc4543_device)
DECLARE_DEVICE_TYPE(JRC6355E, jrc6355e_device)

#endif // MAME_MACHINE_RTC4543_H
