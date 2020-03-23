// license:BSD-3-Clause
// copyright-holders:smf
#ifndef MAME_MACHINE_CLOCK_H
#define MAME_MACHINE_CLOCK_H

#pragma once


#define MCFG_CLOCK_ADD(_tag, _clock) \
	MCFG_DEVICE_ADD(_tag, CLOCK, _clock)

#define MCFG_CLOCK_SIGNAL_HANDLER(_devcb) \
	devcb = &clock_device::set_signal_handler(*device, DEVCB_##_devcb);

class clock_device : public device_t
{
public:
	clock_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_signal_handler(device_t &device, Object &&cb) { return downcast<clock_device &>(device).m_signal_handler.set_callback(std::forward<Object>(cb)); }

protected:
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual void device_clock_changed() override;

private:
	void update_timer();
	attotime period();

	int m_signal;
	emu_timer *m_timer;

	devcb_write_line m_signal_handler;
};

DECLARE_DEVICE_TYPE(CLOCK, clock_device)

#endif // MAME_MACHINE_CLOCK_H
