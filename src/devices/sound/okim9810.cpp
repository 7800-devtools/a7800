// license:BSD-3-Clause
// copyright-holders:Andrew Gardner
/***************************************************************************

    okim9810.h

    OKI MSM9810 ADPCM(2) sound chip.

***************************************************************************/

#include "emu.h"
#include "okim9810.h"


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(OKIM9810, okim9810_device, "okim9810", "OKI MSM9810 ADPCM")

// volume lookup table. The manual lists a full 16 steps, 2dB per step.
// Given the dB values, that seems to map to a 7-bit volume control.
const uint8_t okim9810_device::okim_voice::s_volume_table[16] =
{
	0x80,   //  0 dB
	0x65,   // -2 dB
	0x50,   // -4 dB
	0x40,   // -6 dB
	0x32,   // -8.0 dB
	0x28,   // -10.5 dB
	0x20,   // -12.0 dB
	0x19,   // -14.5 dB
	0x14,   // -16.0 dB
	0x10,   // -18.0 dB
	0x0c,   // -20.0 dB
	0x0a,   // -22.0 dB
	0x08,   // -24.0 dB
	0x06,   // -26.0 dB
	0x05,   // -28.0 dB
	0x04,   // -30.0 dB
};

// sampling frequency lookup table.
const uint32_t okim9810_device::s_sampling_freq_table[16] =
{
	4000,
	8000,
	16000,
	32000,
	0,
	6400,
	12800,
	25600,
	0,
	5300,
	10600,
	21200,
	0,
	0,
	0,
	0
};



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  okim9810_device - constructor
//-------------------------------------------------

okim9810_device::okim9810_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, OKIM9810, tag, owner, clock),
		device_sound_interface(mconfig, *this),
		device_rom_interface(mconfig, *this, 24),
		m_stream(nullptr),
		m_TMP_register(0x00),
		m_global_volume(0x00),
		m_filter_type(SECONDARY_FILTER),
		m_output_level(OUTPUT_TO_DIRECT_DAC)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void okim9810_device::device_start()
{
	// create the stream
	//int divisor = m_pin7 ? 132 : 165;
	m_stream = machine().sound().stream_alloc(*this, 0, 2, clock());

	// save state stuff
	save_item(NAME(m_TMP_register));
	save_item(NAME(m_global_volume));
	save_item(NAME(m_filter_type));
	save_item(NAME(m_output_level));

	for  (int i = 0; i < OKIM9810_VOICES; i++)
	{
		okim_voice *voice = &m_voice[i];

		save_item(NAME(voice->m_adpcm.m_signal), i);
		save_item(NAME(voice->m_adpcm.m_step), i);
		save_item(NAME(voice->m_adpcm2.m_signal), i);
		save_item(NAME(voice->m_adpcm2.m_step), i);
		save_item(NAME(voice->m_playbackAlgo), i);
		save_item(NAME(voice->m_looping), i);
		save_item(NAME(voice->m_startFlags), i);
		save_item(NAME(voice->m_endFlags), i);
		save_item(NAME(voice->m_base_offset), i);
		save_item(NAME(voice->m_count), i);
		save_item(NAME(voice->m_samplingFreq), i);
		save_item(NAME(voice->m_playing), i);
		save_item(NAME(voice->m_sample), i);
		save_item(NAME(voice->m_channel_volume), i);
		save_item(NAME(voice->m_pan_volume_left), i);
		save_item(NAME(voice->m_pan_volume_right), i);
		save_item(NAME(voice->m_startSample), i);
		save_item(NAME(voice->m_endSample), i);
		save_item(NAME(voice->m_interpSampleNum), i);
	}
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void okim9810_device::device_reset()
{
	m_stream->update();
	for (auto & elem : m_voice)
		elem.m_playing = false;
}


//-------------------------------------------------
//  device_post_load - device-specific post-load
//-------------------------------------------------

void okim9810_device::device_post_load()
{
}


//-------------------------------------------------
//  device_clock_changed - called if the clock
//  changes
//-------------------------------------------------

void okim9810_device::device_clock_changed()
{
}



//-------------------------------------------------
//  rom_bank_updated - the rom bank has changed
//-------------------------------------------------

void okim9810_device::rom_bank_updated()
{
	m_stream->update();
}


//-------------------------------------------------
//  stream_generate - handle update requests for
//  our sound stream
//-------------------------------------------------

void okim9810_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	// reset the output streams
	memset(outputs[0], 0, samples * sizeof(*outputs[0]));
	memset(outputs[1], 0, samples * sizeof(*outputs[1]));

	// iterate over voices and accumulate sample data
	for (auto & elem : m_voice)
		elem.generate_audio(*this, outputs, samples, m_global_volume, clock(), m_filter_type);
}


//-------------------------------------------------
//  read_status - read the status register
//-------------------------------------------------

uint8_t okim9810_device::read_status()
{
	uint8_t result = 0x00;
	uint8_t channelMask = 0x01;
	for (int i = 0; i < OKIM9810_VOICES; i++, channelMask <<= 1)
	{
		if (!m_voice[i].m_playing)
			result |= channelMask;
	}
	return result;
}


//-------------------------------------------------
//  read - memory interface for reading the active status
//-------------------------------------------------

READ8_MEMBER( okim9810_device::read )
{
	return read_status();
}


//-------------------------------------------------
//  write - memory interface for write
//-------------------------------------------------

// The command is written when the CMD pin is low
void okim9810_device::write_command(uint8_t data)
{
	const uint8_t cmd = (data & 0xf8) >> 3;
	const uint8_t channel = (data & 0x07);

	switch(cmd)
	{
		case 0x00:  // START
		{
			osd_printf_debug("START channel mask %02x\n", m_TMP_register);
			uint8_t channelMask = 0x01;
			for (int i = 0; i < OKIM9810_VOICES; i++, channelMask <<= 1)
			{
				if (channelMask & m_TMP_register)
				{
					m_voice[i].m_playing = true;
					osd_printf_debug("\t\tPlaying channel %d: encoder type %d @ %dhz (volume = %d %d).  From %08x for %d samples (looping=%d).\n",
										i,
										m_voice[i].m_playbackAlgo,
										m_voice[i].m_samplingFreq,
										m_voice[i].volume_scale(m_global_volume, m_voice[i].m_channel_volume, m_voice[i].m_pan_volume_left),
										m_voice[i].volume_scale(m_global_volume, m_voice[i].m_channel_volume, m_voice[i].m_pan_volume_right),
										m_voice[i].m_base_offset,
										m_voice[i].m_count,
										m_voice[i].m_looping);
				}
			}
			break;
		}
		case 0x01:  // STOP
		{
			osd_printf_debug("STOP  channel mask %02x\n", m_TMP_register);
			uint8_t channelMask = 0x01;
			for (int i = 0; i < OKIM9810_VOICES; i++, channelMask <<= 1)
			{
				if (channelMask & m_TMP_register)
				{
					m_voice[i].m_playing = false;
					osd_printf_debug("\tChannel %d stopping.\n", i);
				}
			}
			break;
		}
		case 0x02:  // LOOP
		{
			osd_printf_debug("LOOP  channel mask %02x\n", m_TMP_register);
			uint8_t channelMask = 0x01;
			for (int i = 0; i < OKIM9810_VOICES; i++, channelMask <<= 1)
			{
				if (channelMask & m_TMP_register)
				{
					m_voice[i].m_looping = true;
					osd_printf_debug("\tChannel %d looping.\n", i);
				}
				else
				{
					m_voice[i].m_looping = false;
					osd_printf_debug("\tChannel %d done looping.\n", i);
				}
			}
			break;
		}
		case 0x03:  // OPT (options)
		{
			osd_printf_debug("OPT   complex data %02x\n", m_TMP_register);
			m_global_volume = (m_TMP_register & 0x18) >> 3;
			m_filter_type =   (m_TMP_register & 0x06) >> 1;
			m_output_level =  (m_TMP_register & 0x01);
			osd_printf_debug("\tOPT setting main volume scale to Vdd/%d\n", m_global_volume+1);
			osd_printf_debug("\tOPT setting output filter type to %d\n", m_filter_type);
			osd_printf_debug("\tOPT setting output amp level to %d\n", m_output_level);
			break;
		}
		case 0x04:  // MUON (silence)
		{
			osd_printf_warning("MUON  channel %d length %02x\n", channel, m_TMP_register);
			osd_printf_warning("MSM9810: UNIMPLEMENTED COMMAND!\n");
			break;
		}

		case 0x05:  // FADR (phrase address)
		{
			const offs_t base = m_TMP_register * 8;

			offs_t startAddr;
			uint8_t startFlags = read_byte(base + 0);
			startAddr  = read_byte(base + 1) << 16;
			startAddr |= read_byte(base + 2) << 8;
			startAddr |= read_byte(base + 3) << 0;

			offs_t endAddr;
			uint8_t endFlags = read_byte(base + 4);
			endAddr  = read_byte(base + 5) << 16;
			endAddr |= read_byte(base + 6) << 8;
			endAddr |= read_byte(base + 7) << 0;

			// Sub-table
			if (startFlags & 0x80)
			{
				offs_t subTable = startAddr;
				// TODO: New startFlags &= 0x80.  Are there further subtables?
				startFlags = read_byte(subTable + 0);
				startAddr  = read_byte(subTable + 1) << 16;
				startAddr |= read_byte(subTable + 2) << 8;
				startAddr |= read_byte(subTable + 3) << 0;

				// TODO: What does byte (subTable + 4) refer to?
				endAddr  = read_byte(subTable + 5) << 16;
				endAddr |= read_byte(subTable + 6) << 8;
				endAddr |= read_byte(subTable + 7) << 0;
			}

			m_voice[channel].m_sample = 0;
			m_voice[channel].m_interpSampleNum = 0;
			m_voice[channel].m_startFlags = startFlags;
			m_voice[channel].m_base_offset = startAddr;
			m_voice[channel].m_endFlags = endFlags;
			m_voice[channel].m_count = (endAddr-startAddr) + 1;     // Is there yet another extra byte at the end?

			m_voice[channel].m_playbackAlgo = (startFlags & 0x30) >> 4;
			m_voice[channel].m_samplingFreq = s_sampling_freq_table[startFlags & 0x0f];
			if (m_voice[channel].m_playbackAlgo == ADPCM_PLAYBACK ||
				m_voice[channel].m_playbackAlgo == ADPCM2_PLAYBACK)
				m_voice[channel].m_count *= 2;
			else
				osd_printf_warning("MSM9810: UNIMPLEMENTED PLAYBACK METHOD %d\n", m_voice[channel].m_playbackAlgo);

			osd_printf_debug("FADR  channel %d phrase offset %02x => ", channel, m_TMP_register);
			osd_printf_debug("startFlags(%02x) startAddr(%06x) endFlags(%02x) endAddr(%06x) bytes(%d)\n", startFlags, startAddr, endFlags, endAddr, endAddr-startAddr);
			break;
		}

		case 0x06:  // DADR (direct address playback)
		{
			osd_printf_warning("DADR  channel %d complex data %02x\n", channel, m_TMP_register);
			osd_printf_warning("MSM9810: UNIMPLEMENTED COMMAND!\n");
			break;
		}
		case 0x07:  // CVOL (channel volume)
		{
			osd_printf_debug("CVOL  channel %d data %02x\n", channel, m_TMP_register);
			osd_printf_debug("\tChannel %d -> volume index %d.\n", channel, m_TMP_register & 0x0f);

			m_voice[channel].m_channel_volume = m_TMP_register & 0x0f;
			break;
		}
		case 0x08:  // PAN
		{
			const uint8_t leftVolIndex = (m_TMP_register & 0xf0) >> 4;
			const uint8_t rightVolIndex = m_TMP_register & 0x0f;
			osd_printf_debug("PAN   channel %d left index: %02x right index: %02x (%02x)\n", channel, leftVolIndex, rightVolIndex, m_TMP_register);
			osd_printf_debug("\tChannel %d left -> %d right -> %d\n", channel, leftVolIndex, rightVolIndex);
			m_voice[channel].m_pan_volume_left = leftVolIndex;
			m_voice[channel].m_pan_volume_right = rightVolIndex;
			break;
		}
		default:
		{
			osd_printf_warning("MSM9810: UNKNOWN COMMAND!\n");
			break;
		}
	}
}

WRITE8_MEMBER( okim9810_device::write )
{
	write_command(data);
}


//-----------------------------------------------------------
//  writeTMP - memory interface for writing the TMP register
//-----------------------------------------------------------

// TMP is written when the CMD pin is high
void okim9810_device::write_tmp_register(uint8_t data)
{
	m_TMP_register = data;
}

WRITE8_MEMBER( okim9810_device::write_tmp_register )
{
	write_tmp_register(data);
}


//**************************************************************************
//  OKIM VOICE
//**************************************************************************

//-------------------------------------------------
//  okim_voice - constructor
//-------------------------------------------------

okim9810_device::okim_voice::okim_voice()
	: m_playbackAlgo(ADPCM2_PLAYBACK),
		m_looping(false),
		m_startFlags(0),
		m_endFlags(0),
		m_base_offset(0),
		m_count(0),
		m_samplingFreq(s_sampling_freq_table[2]),
		m_playing(false),
		m_sample(0),
		m_channel_volume(0x00),
		m_pan_volume_left(0x00),
		m_pan_volume_right(0x00),
		m_startSample(0),
		m_endSample(0),
		m_interpSampleNum(0)
{
}

//-------------------------------------------------
//  generate_audio - generate audio samples and
//  add them to an output stream
//-------------------------------------------------

void okim9810_device::okim_voice::generate_audio(device_rom_interface &rom,
													stream_sample_t **buffers,
													int samples,
													const uint8_t global_volume,
													const uint32_t clock,
													const uint8_t filter_type)
{
	// skip if not active
	if (!m_playing)
		return;

	// separate out left and right channels
	stream_sample_t *outL = buffers[0];
	stream_sample_t *outR = buffers[1];

	// get left and right volumes
	uint8_t volume_scale_left = volume_scale(global_volume, m_channel_volume, m_pan_volume_left);
	uint8_t volume_scale_right = volume_scale(global_volume, m_channel_volume, m_pan_volume_right);

	// total samples per byte
	uint32_t totalInterpSamples = clock / m_samplingFreq;

	// loop while we still have samples to generate
	while (samples-- != 0)
	{
		// If interpSampleNum == 0, we are at the beginning of a new interp chunk, gather data
		if (m_interpSampleNum == 0)
		{
			// If m_sample == 0, we have begun to play a new voice.  Get both the first nibble & the second.
			if (m_sample == 0)
			{
				// fetch the first sample nibble
				int nibble0 = rom.read_byte(m_base_offset + m_sample / 2) >> (((m_sample & 1) << 2) ^ 4);
				switch (m_playbackAlgo)
				{
					case ADPCM_PLAYBACK:
					{
						m_adpcm.reset();
						m_startSample = (int32_t)m_adpcm.clock(nibble0);
						break;
					}
					case ADPCM2_PLAYBACK:
					{
						m_adpcm2.reset();
						m_startSample = (int32_t)m_adpcm2.clock(nibble0);
						break;
					}
					default:
						break;
				}
			}
			else
			{
				// Otherwise just move the second nibble back to the first spot.
				m_startSample = m_endSample;
			}

			// And fetch the second sample nibble
			int nibble1 = rom.read_byte(m_base_offset + (m_sample+1) / 2) >> ((((m_sample+1) & 1) << 2) ^ 4);
			switch (m_playbackAlgo)
			{
				case ADPCM_PLAYBACK:
				{
					m_endSample = (int32_t)m_adpcm.clock(nibble1);
					break;
				}
				case ADPCM2_PLAYBACK:
				{
					m_endSample = (int32_t)m_adpcm2.clock(nibble1);
					break;
				}
				default:
					break;
			}
		}

		// TODO: Interpolate using proper numeric types.
		float progress = (float)m_interpSampleNum / (float)totalInterpSamples;
		int32_t interpValue = (int32_t)((float)m_startSample + (((float)m_endSample-(float)m_startSample) * progress));

		// if filtering is unwanted
		if (filter_type != SECONDARY_FILTER && filter_type != PRIMARY_FILTER)
			interpValue = m_startSample;

		// output to the stereo buffers, scaling by the volume
		// signal in range -2048..2047, volume in range 2..128 => signal * volume / 8 in range -32768..32767
		int32_t interpValueL = (interpValue * (int32_t)volume_scale_left) / 8;
		*outL++ += interpValueL;

		int32_t interpValueR = (interpValue * (int32_t)volume_scale_right) / 8;
		*outR++ += interpValueR;

		// if the interpsample has reached its end, move on to the next sample
		m_interpSampleNum++;
		if (m_interpSampleNum >= totalInterpSamples)
		{
			m_interpSampleNum = 0;
			m_sample++;
		}

		// the end of the stream has been reached
		if (m_sample >= m_count)
		{
			if (!m_looping)
			{
				m_playing = false;
				break;
			}
			else
			{
				m_sample = 0;
			}
		}
	}
}


//-------------------------------------------------
//  volume_scale - computes the volume equation as
//                 seen on page 29 of the docs.
//  Returns a value from the volume lookup table.
//-------------------------------------------------

uint8_t okim9810_device::okim_voice::volume_scale(const uint8_t global_volume_index,
												const uint8_t channel_volume_index,
												const uint8_t pan_volume_index) const
{
	const uint8_t& V = channel_volume_index;
	const uint8_t& L = pan_volume_index;
	const uint8_t& O = global_volume_index;
	uint32_t index = (V+L) + (O*3);

	if (index > 15)
		index = 15;

	return s_volume_table[index];
}
