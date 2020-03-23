// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*
    vboy.h - Virtual Boy audio emulation

    By Richard Bannister and Gil Pedersen.
    MESS device adaptation by R. Belmont
*/
#ifndef MAME_AUDIO_VBOY_H
#define MAME_AUDIO_VBOY_H

#pragma once


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_VBOYSND_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, VBOYSND, vboysnd_device::AUDIO_FREQ)

#define MCFG_VBOYSND_REPLACE(_tag) \
	MCFG_DEVICE_REPLACE(_tag, VBOYSND, vboysnd_device::AUDIO_FREQ)

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> vboysnd_device

class vboysnd_device : public device_t, public device_sound_interface
{
public:
	static constexpr unsigned AUDIO_FREQ      = 44100;

	// construction/destruction
	vboysnd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(read);
	DECLARE_WRITE8_MEMBER(write);

	sound_stream *m_stream;

protected:
	static constexpr unsigned CHANNELS        = 4;

	struct s_snd_channel {
		int8_t        playing;    // the sound is playing

		// state when sound was enabled
		uint32_t      env_steptime;       // Envelope step time
		uint8_t       env0;               // Envelope data
		uint8_t       env1;               // Envelope data
		uint8_t       volLeft;            // Left output volume
		uint8_t       volRight;           // Right output volume
		uint8_t       sample[580];        // sample to play
		int         sample_len;         // length of sample

		// values that change, as the sample is played
		int         offset;             // current offset in sample
		int         time;               // the duration that this sample is to be played
		uint8_t       envelope;           // Current envelope level (604)
		int         env_time;           // The duration between envelope decay/grow (608)
	};

	struct s_regchan {
		int32_t sINT;
		int32_t sLRV;
		int32_t sFQL;
		int32_t sFQH;
		int32_t sEV0;
		int32_t sEV1;
		int32_t sRAM;
	};

	struct s_sreg {
		// Sound registers structure
		s_regchan c[4];
	};

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

	s_snd_channel snd_channel[5];

	uint16_t waveFreq2LenTbl[2048];
	uint16_t waveTimer2LenTbl[32];
	uint16_t waveEnv2LenTbl[8];

	emu_timer *m_timer;

	uint8_t m_aram[0x600];
};

// device type definition
DECLARE_DEVICE_TYPE(VBOYSND, vboysnd_device)

#endif //MAME_AUDIO_VBOY_H
