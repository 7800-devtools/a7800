// license:BSD-3-Clause
// copyright-holders:Allard van der Bas
#ifndef MAME_AUDIO_WARPWARP_H
#define MAME_AUDIO_WARPWARP_H

#pragma once


class warpwarp_sound_device : public device_t, public device_sound_interface
{
public:
	warpwarp_sound_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	enum
	{
		TIMER_SOUND_VOLUME_DECAY,
		TIMER_MUSIC_VOLUME_DECAY
	};


	DECLARE_WRITE8_MEMBER( sound_w );
	DECLARE_WRITE8_MEMBER( music1_w );
	DECLARE_WRITE8_MEMBER( music2_w );

protected:
	// device-level overrides
	virtual void device_start() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	// internal state
	std::unique_ptr<int16_t[]> m_decay;
	sound_stream *m_channel;
	int m_sound_latch;
	int m_music1_latch;
	int m_music2_latch;
	int m_sound_signal;
	int m_sound_volume;
	emu_timer   *m_sound_volume_timer;
	int m_music_signal;
	int m_music_volume;
	emu_timer   *m_music_volume_timer;
	int m_noise;

	int m_vcarry;
	int m_vcount;
	int m_mcarry;
	int m_mcount;
};

DECLARE_DEVICE_TYPE(WARPWARP, warpwarp_sound_device)

#endif // MAME_AUDIO_WARPWARP_H
