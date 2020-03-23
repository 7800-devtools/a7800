// license:BSD-3-Clause
// copyright-holders:Aaron Giles,smf
/*
    CD-DA "Red Book" audio sound hardware handler
    Relies on the actual CD logic and reading in cdrom.c.
*/

#include "emu.h"
#include "cdda.h"

#define MAX_SECTORS ( 4 )


//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void cdda_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	get_audio_data(&outputs[0][0], &outputs[1][0], samples);
	m_audio_volume[0] = (int16_t)outputs[0][0];
	m_audio_volume[1] = (int16_t)outputs[1][0];
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void cdda_device::device_start()
{
	/* allocate an audio cache */
	m_audio_cache = std::make_unique<uint8_t[]>(CD_MAX_SECTOR_DATA * MAX_SECTORS );

	m_stream = machine().sound().stream_alloc(*this, 0, 2, 44100);

	m_audio_playing = 0;
	m_audio_pause = 0;
	m_audio_ended_normally = 0;
	m_audio_lba = 0;
	m_audio_length = 0;
	m_audio_samples = 0;
	m_audio_bptr = 0;
	m_disc = nullptr;

	save_item( NAME(m_audio_playing) );
	save_item( NAME(m_audio_pause) );
	save_item( NAME(m_audio_ended_normally) );
	save_item( NAME(m_audio_lba) );
	save_item( NAME(m_audio_length) );
	save_pointer( NAME(m_audio_cache.get()), CD_MAX_SECTOR_DATA * MAX_SECTORS );
	save_item( NAME(m_audio_samples) );
	save_item( NAME(m_audio_bptr) );
}


/*-------------------------------------------------
    cdda_set_cdrom - set the CD-ROM file for the
    given CDDA stream
-------------------------------------------------*/

void cdda_device::set_cdrom(void *file)
{
	m_disc = (cdrom_file *)file;
}


/*-------------------------------------------------
    cdda_start_audio - begin playback of a Red
    Book audio track
-------------------------------------------------*/

void cdda_device::start_audio(uint32_t startlba, uint32_t numblocks)
{
	m_stream->update();
	m_audio_playing = true;
	m_audio_pause = false;
	m_audio_ended_normally = false;
	m_audio_lba = startlba;
	m_audio_length = numblocks;
	m_audio_samples = 0;
}


/*-------------------------------------------------
    cdda_stop_audio - stop playback of a Red Book
    audio track
-------------------------------------------------*/

void cdda_device::stop_audio()
{
	m_stream->update();
	m_audio_playing = false;
	m_audio_ended_normally = true;
}


/*-------------------------------------------------
    cdda_pause_audio - pause/unpause playback of
    a Red Book audio track
-------------------------------------------------*/

void cdda_device::pause_audio(int pause)
{
	m_stream->update();
	m_audio_pause = pause;
}


/*-------------------------------------------------
    cdda_get_audio_lba - returns the current LBA
    (physical sector) during Red Book playback
-------------------------------------------------*/

uint32_t cdda_device::get_audio_lba()
{
	m_stream->update();
	return m_audio_lba;
}


/*-------------------------------------------------
    cdda_audio_active - returns Red Book audio
    playback status
-------------------------------------------------*/

int cdda_device::audio_active()
{
	m_stream->update();
	return m_audio_playing;
}


/*-------------------------------------------------
    cdda_audio_paused - returns if Red Book
    playback is paused
-------------------------------------------------*/

int cdda_device::audio_paused()
{
	return m_audio_pause;
}


/*-------------------------------------------------
    cdda_audio_ended - returns if a Red Book
    track reached it's natural end
-------------------------------------------------*/

int cdda_device::audio_ended()
{
	m_stream->update();
	return m_audio_ended_normally;
}


/*-------------------------------------------------
    get_audio_data - reads Red Book data off
    the disc if playback is in progress and
    converts it to 2 16-bit 44.1 kHz streams
-------------------------------------------------*/

void cdda_device::get_audio_data(stream_sample_t *bufL, stream_sample_t *bufR, uint32_t samples_wanted)
{
	int i;
	int16_t *audio_cache = (int16_t *) m_audio_cache.get();

	while (samples_wanted > 0)
	{
		/* if no file, audio not playing, audio paused, or out of disc data,
		   just zero fill */
		if (!m_disc || !m_audio_playing || m_audio_pause || (!m_audio_length && !m_audio_samples))
		{
			if( m_disc && m_audio_playing && !m_audio_pause && !m_audio_length )
			{
				m_audio_playing = false;
				m_audio_ended_normally = true;
			}

			memset(bufL, 0, sizeof(stream_sample_t)*samples_wanted);
			memset(bufR, 0, sizeof(stream_sample_t)*samples_wanted);
			return;
		}

		int samples = samples_wanted;
		if (samples > m_audio_samples)
		{
			samples = m_audio_samples;
		}

		for (i = 0; i < samples; i++)
		{
			/* CD-DA data on the disc is big-endian */
			*bufL++ = (int16_t) big_endianize_int16( audio_cache[ m_audio_bptr ] ); m_audio_bptr++;
			*bufR++ = (int16_t) big_endianize_int16( audio_cache[ m_audio_bptr ] ); m_audio_bptr++;
		}

		samples_wanted -= samples;
		m_audio_samples -= samples;

		if (m_audio_samples == 0)
		{
			int sectors = m_audio_length;
			if (sectors > MAX_SECTORS)
			{
				sectors = MAX_SECTORS;
			}

			for (i = 0; i < sectors; i++)
			{
				cdrom_read_data(m_disc, m_audio_lba, &m_audio_cache[CD_MAX_SECTOR_DATA*i], CD_TRACK_AUDIO);

				m_audio_lba++;
			}

			m_audio_samples = (CD_MAX_SECTOR_DATA*sectors)/4;
			m_audio_length -= sectors;

			/* reset feedout ptr */
			m_audio_bptr = 0;
		}
	}
}

/*-------------------------------------------------
    cdda_get_channel_volume - sets CD-DA volume level
    for either speaker, used for volume control display
-------------------------------------------------*/

int16_t cdda_device::get_channel_volume(int channel)
{
	return m_audio_volume[channel];
}

DEFINE_DEVICE_TYPE(CDDA, cdda_device, "cdda", "CD/DA")

cdda_device::cdda_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, CDDA, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_disc(nullptr)
	, m_stream(nullptr)
{
}
