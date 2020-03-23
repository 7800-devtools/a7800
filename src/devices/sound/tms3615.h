// license:BSD-3-Clause
// copyright-holders:Juergen Buchmueller
#ifndef MAME_SOUND_TMS3615_H
#define MAME_SOUND_TMS3615_H

#pragma once


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_TMS3615_ADD(_tag, _clock) \
	MCFG_DEVICE_ADD(_tag, TMS3615, _clock)
#define MCFG_TMS3615_REPLACE(_tag, _clock) \
	MCFG_DEVICE_REPLACE(_tag, TMS3615, _clock)


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************


// ======================> tms3615_device

class tms3615_device : public device_t, public device_sound_interface
{
public:
	static constexpr unsigned FOOTAGE_8 = 0;
	static constexpr unsigned FOOTAGE_16 = 1;

	tms3615_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	void enable_w(int enable);

protected:
	// device-level overrides
	virtual void device_start() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:
	static constexpr unsigned TMS3615_TONES = 13;
	static const int divisor[TMS3615_TONES];

	sound_stream *m_channel;        // returned by stream_create()
	int m_samplerate;               // output sample rate
	int m_basefreq;                 // chip's base frequency
	int m_counter8[TMS3615_TONES];  // tone frequency counter for 8'
	int m_counter16[TMS3615_TONES]; // tone frequency counter for 16'
	int m_output8;                  // output signal bits for 8'
	int m_output16;                 // output signal bits for 16'
	int m_enable;                   // mask which tones to play
};

[[deprecated("Use TMS36XX instead")]]
extern const device_type TMS3615;

#endif // MAME_SOUND_TMS3615_H
