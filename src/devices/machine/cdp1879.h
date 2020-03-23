// license:BSD-3-Clause
// copyright-holders:R. Belmont,Carl
/**********************************************************************

    cdp1879.h - RCA CDP1879 real-time clock emulation

**********************************************************************/

#ifndef MAME_MACHINE_CDP1879_H
#define MAME_MACHINE_CDP1879_H

#pragma once

#include "dirtc.h"


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_CDP1879_IRQ_CALLBACK(_cb) \
	devcb = &cdp1879_device::set_irq_cb(*device, DEVCB_##_cb);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> cdp1879_device

class cdp1879_device :  public device_t,
						public device_rtc_interface
{
public:
	// construction/destruction
	cdp1879_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(read);
	DECLARE_WRITE8_MEMBER(write);

	template<class _Object> static devcb_base &set_irq_cb(device_t &device, _Object wr) { return downcast<cdp1879_device &>(device).m_irq_w.set_callback(wr); }

	devcb_write_line m_irq_w;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// device_rtc_interface overrides
	virtual void rtc_clock_updated(int year, int month, int day, int day_of_week, int hour, int minute, int second) override;
	virtual bool rtc_feature_leap_year() const override { return true; }

	void set_irq(int bit);
	void update_rtc();

private:
	// registers
	enum
	{
		R_CNT_SECONDS = 2,      // 2 = seconds
		R_CNT_MINUTES,          // 3 = minutes
		R_CNT_HOURS,            // 4 = hours
		R_CNT_DAYOFMONTH,       // 5 = day of the month
		R_CNT_MONTH,            // 6 = month
		R_CTL_IRQSTATUS = 7,    // 7 = IRQ status
		R_CTL_CONTROL = 7,   // 7 = IRQ control
		R_ALM_SECONDS,
		R_ALM_MINUTES,
		R_ALM_HOURS
	};

	u8 m_regs[10];
	bool m_comparator_state;

	// timers
	emu_timer *m_clock_timer;
};

// device type definition
DECLARE_DEVICE_TYPE(CDP1879, cdp1879_device)

#endif // MAME_MACHINE_CDP1879_H
