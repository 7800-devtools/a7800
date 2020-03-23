// license:BSD-3-Clause
// copyright-holders:Richard Davies
#ifndef MAME_AUDIO_PHOENIX_H
#define MAME_AUDIO_PHOENIX_H

#pragma once

#include "sound/discrete.h"
#include "sound/tms36xx.h"


class phoenix_sound_device : public device_t, public device_sound_interface
{
public:
	phoenix_sound_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_WRITE8_MEMBER( control_a_w );
	DECLARE_WRITE8_MEMBER( control_b_w );

protected:
	// device-level overrides
	virtual void device_start() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:
	struct c_state
	{
		int32_t counter;
		int32_t level;
	};

	struct n_state
	{
		int32_t counter;
		int32_t polyoffs;
		int32_t polybit;
		int32_t lowpass_counter;
		int32_t lowpass_polybit;
	};

	// internal state
	struct c_state      m_c24_state;
	struct c_state      m_c25_state;
	struct n_state      m_noise_state;
	uint8_t               m_sound_latch_a;
	sound_stream *      m_channel;
	std::unique_ptr<uint32_t[]>                m_poly18;
	discrete_device *m_discrete;
	tms36xx_device *m_tms;

	int update_c24(int samplerate);
	int update_c25(int samplerate);
	int noise(int samplerate);
};

DECLARE_DEVICE_TYPE(PHOENIX, phoenix_sound_device)

DISCRETE_SOUND_EXTERN(phoenix);

#endif // MAME_AUDIO_PHOENIX_H
