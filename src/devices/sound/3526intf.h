// license:BSD-3-Clause
// copyright-holders:Ernesto Corvi
#ifndef MAME_SOUND_3526INTF_H
#define MAME_SOUND_3526INTF_H

#pragma once


#define MCFG_YM3526_IRQ_HANDLER(cb) \
		devcb = &ym3526_device::set_irq_handler(*device, (DEVCB_##cb));

class ym3526_device : public device_t, public device_sound_interface
{
public:
	ym3526_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	template <class Object> static devcb_base &set_irq_handler(device_t &device, Object &&cb) { return downcast<ym3526_device &>(device).m_irq_handler.set_callback(std::forward<Object>(cb)); }

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

	DECLARE_READ8_MEMBER( status_port_r );
	DECLARE_READ8_MEMBER( read_port_r );
	DECLARE_WRITE8_MEMBER( control_port_w );
	DECLARE_WRITE8_MEMBER( write_port_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_stop() override;
	virtual void device_reset() override;
	virtual void device_clock_changed() override;

	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:
	void irq_handler(int irq);
	void timer_handler(int c, const attotime &period);
	void update_request() { m_stream->update(); }

	void calculate_rates();

	static void static_irq_handler(device_t *param, int irq) { downcast<ym3526_device *>(param)->irq_handler(irq); }
	static void static_timer_handler(device_t *param, int c, const attotime &period) { downcast<ym3526_device *>(param)->timer_handler(c, period); }
	static void static_update_request(device_t *param, int interval) { downcast<ym3526_device *>(param)->update_request(); }

	// internal state
	sound_stream *  m_stream;
	emu_timer *     m_timer[2];
	void *          m_chip;
	devcb_write_line m_irq_handler;
};

DECLARE_DEVICE_TYPE(YM3526, ym3526_device)

#endif // MAME_SOUND_3526INTF_H
