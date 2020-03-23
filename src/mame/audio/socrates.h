// license:BSD-3-Clause
// copyright-holders:Jonathan Gevaryahu
#ifndef MAME_AUDIO_SOCRATES_H
#define MAME_AUDIO_SOCRATES_H

#pragma once

class socrates_snd_device : public device_t,
							public device_sound_interface
{
public:
	socrates_snd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	void reg0_w(int data);
	void reg1_w(int data);
	void reg2_w(int data);
	void reg3_w(int data);
	void reg4_w(int data);

protected:
	// device-level overrides
	virtual void device_start() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;
private:
	void snd_clock();
	static const uint8_t s_volumeLUT[];

	// internal state
	sound_stream *  m_stream;
	uint8_t           m_freq[2];      // channel 1,2 frequencies
	uint8_t           m_vol[2];       // channel 1,2 volume
	uint8_t           m_enable[2];    // channel 1,2 enable
	uint8_t           m_channel3;     // channel 3 weird register
	uint8_t           m_state[3];     // output states for channels 1,2,3
	uint8_t           m_accum[3];     // accumulators for channels 1,2,3
	uint16_t          m_DAC_output;   // output
};

DECLARE_DEVICE_TYPE(SOCRATES_SOUND, socrates_snd_device)


#endif // MAME_AUDIO_SOCRATES_H
