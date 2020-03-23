// license:GPL-2.0+
// copyright-holders:Peter Trauner
/*****************************************************************************
 *
 * includes/gamate.h
 *
 ****************************************************************************/

#ifndef MAME_AUDIO_GAMATE_H
#define MAME_AUDIO_GAMATE_H


// ======================> gamate_sound_device

class gamate_sound_device : public device_t, public device_sound_interface
{
public:
	gamate_sound_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_WRITE8_MEMBER( device_w );
	DECLARE_READ8_MEMBER( device_r );

protected:
	// device-level overrides
	virtual void device_start() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:

	static const int DAConverter[];
	static int Value2Volume(int volume) { return DAConverter[volume]*1; }

	sound_stream *m_mixer_channel;
	struct Tone
	{
		Tone() :
			envelope_on(false),
			level(false),
			tone(false), full_cycle(false),
			volume(0),
			pos(0),
			size(0)
		{
		}

		bool envelope_on, level;
		bool tone/*else noise*/, full_cycle/* else square signal/pulse */;
		int volume;
		int pos, size;
	};

	enum
	{
		Right,
		Left,
		Both
	};

	Tone m_channels[3];

	struct Noise
	{
		Noise():
			state(1),
			level(false),
			step(0.0),
			pos(0.0)
			{}

		int state;
		bool level;
		double step, pos;
	} noise;

	struct Envelope
	{
		Envelope():
			control(0),
			index(0),
			first(false)
			{}

		int control;
		int index;
		bool first;
		double step, pos;
	} envelope;

	uint8_t reg[14];
};

DECLARE_DEVICE_TYPE(GAMATE_SND, gamate_sound_device)

#endif // MAME_AUDIO_GAMATE_H
