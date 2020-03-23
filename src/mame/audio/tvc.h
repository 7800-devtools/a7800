// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/*****************************************************************************
 *
 *   includes/tvc.h
 *
 ****************************************************************************/

#ifndef MAME_AUDIO_TVC_H
#define MAME_AUDIO_TVC_H

#pragma once

#define MCFG_TVC_SOUND_SNDINT_CALLBACK(_write) \
	devcb = &tvc_sound_device::set_sndint_wr_callback(*device, DEVCB_##_write);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> tvc_sound_device

class tvc_sound_device : public device_t, public device_sound_interface
{
public:
	// construction/destruction
	tvc_sound_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_sndint_wr_callback(device_t &device, Object &&cb) { return downcast<tvc_sound_device &>(device).m_write_sndint.set_callback(std::forward<Object>(cb)); }

	DECLARE_WRITE8_MEMBER(write);
	void reset_divider();

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:
	static const device_timer_id TIMER_SNDINT = 0;

	sound_stream *  m_stream;
	int             m_freq;
	int             m_enabled;
	int             m_volume;
	int             m_incr;
	int             m_signal;
	uint8_t           m_ports[3];
	emu_timer *     m_sndint_timer;
	devcb_write_line   m_write_sndint;
};

// device type definition
extern const device_type TVC_SOUND;
DECLARE_DEVICE_TYPE(TVC_SOUND, tvc_sound_device)

#endif // MAME_AUDIO_TVC_H
