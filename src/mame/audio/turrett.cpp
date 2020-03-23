// license:BSD-3-Clause
// copyright-holders:Philip Bennett
/***************************************************************************

    Turret Tower sound hardware

****************************************************************************/

#include "emu.h"
#include "includes/turrett.h"

DEFINE_DEVICE_TYPE(TURRETT, turrett_device, "ttsnd", "Turret Tower Sound")


//-------------------------------------------------
//  turrett_device - constructor
//-------------------------------------------------

turrett_device::turrett_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, TURRETT, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, device_memory_interface(mconfig, *this)
	, m_space_config("ttsound", ENDIANNESS_LITTLE, 16, 28, 0, nullptr)
{
}


//-------------------------------------------------
//  memory_space_config - configure address space
//-------------------------------------------------

device_memory_interface::space_config_vector turrett_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(0, &m_space_config)
	};
}


//-------------------------------------------------
//  device_start - initialize the device
//-------------------------------------------------

void turrett_device::device_start()
{
	// Find our direct access
	m_direct = &space().direct();

	// Create the sound stream
	m_stream = machine().sound().stream_alloc(*this, 0, 2, 44100);

	// Create the volume table
	for (int i = 0; i < 0x4f; ++i)
		m_volume_table[i] = 65536 * powf(2.0, (-0.375/4) * i);

	// Last entry is effectively mute
	m_volume_table[0x4f] = 0;

	// Register state for saving
	for (int ch = 0; ch < SOUND_CHANNELS; ++ch)
	{
		save_item(NAME(m_channels[ch].m_address), ch);
		save_item(NAME(m_channels[ch].m_volume), ch);
		save_item(NAME(m_channels[ch].m_playing), ch);
	}
}


//-------------------------------------------------
//  device_reset - reset the device
//-------------------------------------------------

void turrett_device::device_reset()
{
	for (auto & elem : m_channels)
		elem.m_playing = false;
}


//-------------------------------------------------
//  sound_stream_update - update sound stream
//-------------------------------------------------

void turrett_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	// Silence the buffers
	memset(outputs[0], 0x00, sizeof(stream_sample_t) * samples);
	memset(outputs[1], 0x00, sizeof(stream_sample_t) * samples);

	for (int ch = 0; ch < SOUND_CHANNELS; ++ch)
	{
		stream_sample_t *l = outputs[0];
		stream_sample_t *r = outputs[1];

		if (m_channels[ch].m_playing)
		{
			uint32_t &addr = m_channels[ch].m_address;
			int32_t lvol = (m_channels[ch].m_volume >> 16) & 0xff;
			int32_t rvol = m_channels[ch].m_volume & 0xff;

			lvol = m_volume_table[lvol];
			rvol = m_volume_table[rvol];

			// Channels 30 and 31 expect interleaved stereo samples
			uint32_t incr = (ch >= 30) ? 2 : 1;

			for (int s = 0; s < samples; ++s)
			{
				int16_t sample = m_direct->read_word(addr << 1);

				if ((uint16_t)sample == 0x8000)
				{
					m_channels[ch].m_playing = false;
					break;
				}

				addr += incr;

				*l++ += (sample * lvol) >> 17;
				*r++ += (sample * rvol) >> 17;
			}
		}
	}
}


//-------------------------------------------------
//  read - host CPU read access
//-------------------------------------------------

READ32_MEMBER( turrett_device::read )
{
	m_stream->update();

	int ch = offset & 0x3f;

	return m_channels[ch].m_playing << 31;
}


//-------------------------------------------------
//  write - host CPU write access
//-------------------------------------------------

WRITE32_MEMBER( turrett_device::write )
{
	m_stream->update();

	int ch = offset & 0x3f;

	if (offset < 0x100/4)
	{
		if (data == 0)
		{
			m_channels[ch].m_playing = false;
		}
		else
		{
			m_channels[ch].m_address = data;
			m_channels[ch].m_playing = true;
		}
	}
	else
	{
		m_channels[ch].m_volume = data;
	}
}
