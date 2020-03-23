// license:BSD-3-Clause
// copyright-holders:Ernesto Corvi
#ifndef MAME_SOUND_2203INTF_H
#define MAME_SOUND_2203INTF_H

#pragma once

#include "ay8910.h"


struct ssg_callbacks;


#define MCFG_YM2203_IRQ_HANDLER(cb) \
		devcb = &ym2203_device::set_irq_handler(*device, (DEVCB_##cb));

class ym2203_device : public ay8910_device
{
public:
	ym2203_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	template <class Object> static devcb_base &set_irq_handler(device_t &device, Object &&cb) { return downcast<ym2203_device &>(device).m_irq_handler.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

	DECLARE_READ8_MEMBER( status_port_r );
	DECLARE_READ8_MEMBER( read_port_r );
	DECLARE_WRITE8_MEMBER( control_port_w );
	DECLARE_WRITE8_MEMBER( write_port_w );

	// update request from fm.cpp
	static void update_request(device_t *param) { downcast<ym2203_device *>(param)->update_request(); }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_post_load() override;
	virtual void device_stop() override;
	virtual void device_reset() override;
	virtual void device_clock_changed() override;

	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	void stream_generate(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples);

private:
	void irq_handler(int irq);
	void timer_handler(int c, int count, int clock);
	void update_request() { m_stream->update(); }

	void calculate_rates();

	static void static_irq_handler(device_t *param, int irq) { downcast<ym2203_device *>(param)->irq_handler(irq); }
	static void static_timer_handler(device_t *param, int c, int count, int clock) { downcast<ym2203_device *>(param)->timer_handler(c, count, clock); }

	// internal state
	sound_stream *  m_stream;
	emu_timer *     m_timer[2];
	void *          m_chip;
	devcb_write_line m_irq_handler;

	static const ssg_callbacks psgintf;
};

DECLARE_DEVICE_TYPE(YM2203, ym2203_device)

#endif // MAME_SOUND_2203INTF_H
