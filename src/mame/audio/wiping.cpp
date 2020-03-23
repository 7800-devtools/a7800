// license:BSD-3-Clause
// copyright-holders:Allard van der Bas
/***************************************************************************

    Wiping sound driver (quick hack of the Namco sound driver)

    used by wiping.c and clshroad.c

***************************************************************************/

#include "emu.h"
#include "audio/wiping.h"

static constexpr int samplerate = 48000;
static constexpr int defgain = 48;

DEFINE_DEVICE_TYPE(WIPING, wiping_sound_device, "wiping_sound", "Wiping Audio Custom")

wiping_sound_device::wiping_sound_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, WIPING, tag, owner, clock),
	device_sound_interface(mconfig, *this),
	m_last_channel(nullptr),
	m_sound_prom(nullptr),
	m_sound_rom(nullptr),
	m_num_voices(0),
	m_sound_enable(0),
	m_stream(nullptr),
	m_mixer_table(nullptr),
	m_mixer_lookup(nullptr),
	m_mixer_buffer(nullptr),
	m_mixer_buffer_2(nullptr)
{
	memset(m_channel_list, 0, sizeof(wp_sound_channel)*MAX_VOICES);
	memset(m_soundregs, 0, sizeof(uint8_t)*0x4000);
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void wiping_sound_device::device_start()
{
	wp_sound_channel *voice;

	/* get stream channels */
	m_stream = machine().sound().stream_alloc(*this, 0, 1, samplerate);

	/* allocate a pair of buffers to mix into - 1 second's worth should be more than enough */
	m_mixer_buffer   = make_unique_clear<short[]>(samplerate);
	m_mixer_buffer_2 = make_unique_clear<short[]>(samplerate);

	/* build the mixer table */
	make_mixer_table(8, defgain);

	/* extract globals from the interface */
	m_num_voices = 8;
	m_last_channel = m_channel_list + m_num_voices;

	m_sound_rom = machine().root_device().memregion("samples")->base();
	m_sound_prom = machine().root_device().memregion("soundproms")->base();

	/* start with sound enabled, many games don't have a sound enable register */
	m_sound_enable = 1;

	/* reset all the voices */
	for (voice = m_channel_list; voice < m_last_channel; voice++)
	{
		voice->frequency = 0;
		voice->volume = 0;
		voice->wave = &m_sound_prom[0];
		voice->counter = 0;
	}

	save_item(NAME(m_soundregs));

	for (int i = 0; i < MAX_VOICES; i++)
	{
		save_item(NAME(m_channel_list[i].frequency), i);
		save_item(NAME(m_channel_list[i].counter), i);
		save_item(NAME(m_channel_list[i].volume), i);
		save_item(NAME(m_channel_list[i].oneshot), i);
		save_item(NAME(m_channel_list[i].oneshotplaying), i);
	}
}

/* build a table to divide by the number of voices; gain is specified as gain*16 */
void wiping_sound_device::make_mixer_table(int voices, int gain)
{
	int count = voices * 128;
	int i;

	/* allocate memory */
	m_mixer_table = make_unique_clear<int16_t[]>(256 * voices);

	/* find the middle of the table */
	m_mixer_lookup = m_mixer_table.get() + (128 * voices);

	/* fill in the table - 16 bit case */
	for (i = 0; i < count; i++)
	{
		int val = i * gain * 16 / voices;
		if (val > 32767) val = 32767;
		m_mixer_lookup[ i] = val;
		m_mixer_lookup[-i] = -val;
	}
}


/********************************************************************************/

WRITE8_MEMBER( wiping_sound_device::sound_w )
{
	wp_sound_channel *voice;
	int base;

	/* update the streams */
	m_stream->update();

	/* set the register */
	m_soundregs[offset] = data;

	/* recompute all the voice parameters */
	if (offset <= 0x3f)
	{
		for (base = 0, voice = m_channel_list; voice < m_last_channel; voice++, base += 8)
		{
			voice->frequency = m_soundregs[0x02 + base] & 0x0f;
			voice->frequency = voice->frequency * 16 + ((m_soundregs[0x01 + base]) & 0x0f);
			voice->frequency = voice->frequency * 16 + ((m_soundregs[0x00 + base]) & 0x0f);

			voice->volume = m_soundregs[0x07 + base] & 0x0f;
			if (m_soundregs[0x5 + base] & 0x0f)
			{
				voice->wave = &m_sound_rom[128 * (16 * (m_soundregs[0x5 + base] & 0x0f)
						+ (m_soundregs[0x2005 + base] & 0x0f))];
				voice->oneshot = 1;
			}
			else
			{
				voice->wave = &m_sound_rom[16 * (m_soundregs[0x3 + base] & 0x0f)];
				voice->oneshot = 0;
				voice->oneshotplaying = 0;
			}
		}
	}
	else if (offset >= 0x2000)
	{
		voice = &m_channel_list[(offset & 0x3f)/8];
		if (voice->oneshot)
		{
			voice->counter = 0;
			voice->oneshotplaying = 1;
		}
	}
}

//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void wiping_sound_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	stream_sample_t *buffer = outputs[0];
	wp_sound_channel *voice;
	short *mix;
	int i;

	/* if no sound, we're done */
	if (m_sound_enable == 0)
	{
		memset(buffer, 0, samples * sizeof(*buffer));
		return;
	}

	/* zap the contents of the mixer buffer */
	memset(m_mixer_buffer.get(), 0, samples * sizeof(short));

	/* loop over each voice and add its contribution */
	for (voice = m_channel_list; voice < m_last_channel; voice++)
	{
		int f = 16*voice->frequency;
		int v = voice->volume;

		/* only update if we have non-zero volume and frequency */
		if (v && f)
		{
			const uint8_t *w = voice->wave;
			int c = voice->counter;

			mix = m_mixer_buffer.get();

			/* add our contribution */
			for (i = 0; i < samples; i++)
			{
				int offs;

				c += f;

				if (voice->oneshot)
				{
					if (voice->oneshotplaying)
					{
						offs = (c >> 15);
						if (w[offs>>1] == 0xff)
						{
							voice->oneshotplaying = 0;
						}

						else
						{
							/* use full byte, first the high 4 bits, then the low 4 bits */
							if (offs & 1)
								*mix++ += ((w[offs>>1] & 0x0f) - 8) * v;
							else
								*mix++ += (((w[offs>>1]>>4) & 0x0f) - 8) * v;
						}
					}
				}
				else
				{
					offs = (c >> 15) & 0x1f;

					/* use full byte, first the high 4 bits, then the low 4 bits */
					if (offs & 1)
						*mix++ += ((w[offs>>1] & 0x0f) - 8) * v;
					else
						*mix++ += (((w[offs>>1]>>4) & 0x0f) - 8) * v;
				}
			}

			/* update the counter for this voice */
			voice->counter = c;
		}
	}

	/* mix it down */
	mix = m_mixer_buffer.get();
	for (i = 0; i < samples; i++)
		*buffer++ = m_mixer_lookup[*mix++];
}
