// license:BSD-3-Clause
// copyright-holders:Mirko Buffoni,Aaron Giles
/***************************************************************************

    okim6295.h

    OKIM 6295 ADCPM sound chip.

****************************************************************************

    Library to transcode from an ADPCM source to raw PCM.
    Written by Buffoni Mirko in 08/06/97
    References: various sources and documents.

    R. Belmont 31/10/2003
    Updated to allow a driver to use both MSM6295s and "raw" ADPCM voices
    (gcpinbal). Also added some error trapping for MAME_DEBUG builds

****************************************************************************

    OKIM 6295 ADPCM chip:

    Command bytes are sent:

        1xxx xxxx = start of 2-byte command sequence, xxxxxxx is the sample
                    number to trigger
        abcd vvvv = second half of command; one of the abcd bits is set to
                    indicate which voice the v bits seem to be volumed

        0abc d000 = stop playing; one or more of the abcd bits is set to
                    indicate which voice(s)

    Status is read:

        ???? abcd = one bit per voice, set to 0 if nothing is playing, or
                    1 if it is active

***************************************************************************/

#include "emu.h"
#include "okim6295.h"


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(OKIM6295, okim6295_device, "okim6295", "OKI MSM6295 ADPCM")

// volume lookup table. The manual lists only 9 steps, ~3dB per step. Given the dB values,
// that seems to map to a 5-bit volume control. Any volume parameter beyond the 9th index
// results in silent playback.
const uint8_t okim6295_device::s_volume_table[16] =
{
	0x20,   //   0 dB
	0x16,   //  -3.2 dB
	0x10,   //  -6.0 dB
	0x0b,   //  -9.2 dB
	0x08,   // -12.0 dB
	0x06,   // -14.5 dB
	0x04,   // -18.0 dB
	0x03,   // -20.5 dB
	0x02,   // -24.0 dB
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
};


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  okim6295_device - constructor
//-------------------------------------------------

okim6295_device::okim6295_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, OKIM6295, tag, owner, clock),
		device_sound_interface(mconfig, *this),
		device_rom_interface(mconfig, *this, 18),
		m_region(*this, DEVICE_SELF),
		m_command(-1),
		m_stream(nullptr),
		m_pin7_state(0)
{
}


//-------------------------------------------------
//  static_set_pin7 - configuration helper to set
//  the pin 7 state
//-------------------------------------------------

void okim6295_device::static_set_pin7(device_t &device, int pin7)
{
	okim6295_device &okim6295 = downcast<okim6295_device &>(device);
	okim6295.m_pin7_state = pin7;
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void okim6295_device::device_start()
{
	// create the stream
	int divisor = m_pin7_state ? 132 : 165;
	m_stream = machine().sound().stream_alloc(*this, 0, 1, clock() / divisor);

	save_item(NAME(m_command));
	save_item(NAME(m_pin7_state));

	for (int voicenum = 0; voicenum < OKIM6295_VOICES; voicenum++)
	{
		save_item(NAME(m_voice[voicenum].m_playing), voicenum);
		save_item(NAME(m_voice[voicenum].m_sample), voicenum);
		save_item(NAME(m_voice[voicenum].m_count), voicenum);
		save_item(NAME(m_voice[voicenum].m_adpcm.m_signal), voicenum);
		save_item(NAME(m_voice[voicenum].m_adpcm.m_step), voicenum);
		save_item(NAME(m_voice[voicenum].m_volume), voicenum);
		save_item(NAME(m_voice[voicenum].m_base_offset), voicenum);
	}
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void okim6295_device::device_reset()
{
	m_stream->update();
	for (auto & elem : m_voice)
		elem.m_playing = false;
}


//-------------------------------------------------
//  device_post_load - device-specific post-load
//-------------------------------------------------

void okim6295_device::device_post_load()
{
	device_clock_changed();
}


//-------------------------------------------------
//  device_clock_changed - called if the clock
//  changes
//-------------------------------------------------

void okim6295_device::device_clock_changed()
{
	int divisor = m_pin7_state ? 132 : 165;
	m_stream->set_sample_rate(clock() / divisor);
}


//-------------------------------------------------
//  stream_generate - handle update requests for
//  our sound stream
//-------------------------------------------------

void okim6295_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	// reset the output stream
	memset(outputs[0], 0, samples * sizeof(*outputs[0]));

	// iterate over voices and accumulate sample data
	for (auto & elem : m_voice)
		elem.generate_adpcm(*this, outputs[0], samples);
}


//-------------------------------------------------
//  rom_bank_updated - the rom bank has changed
//-------------------------------------------------

void okim6295_device::rom_bank_updated()
{
	m_stream->update();
}


//-------------------------------------------------
//  set_pin7 - change the state of pin 7, which
//  alters the frequency we output
//-------------------------------------------------

void okim6295_device::set_pin7(int pin7)
{
	m_pin7_state = pin7;
	device_clock_changed();
}


//-------------------------------------------------
//  read_status - read the status register
//-------------------------------------------------

uint8_t okim6295_device::read_status()
{
	uint8_t result = 0xf0;    // naname expects bits 4-7 to be 1

	// set the bit to 1 if something is playing on a given channel
	m_stream->update();
	for (int voicenum = 0; voicenum < OKIM6295_VOICES; voicenum++)
		if (m_voice[voicenum].m_playing)
			result |= 1 << voicenum;

	return result;
}


//-------------------------------------------------
//  read - memory interface for read
//-------------------------------------------------

READ8_MEMBER( okim6295_device::read )
{
	return read_status();
}


//-------------------------------------------------
//  write_command - write to the command register
//-------------------------------------------------

void okim6295_device::write_command(uint8_t command)
{
	// if a command is pending, process the second half
	if (m_command != -1)
	{
		// the manual explicitly says that it's not possible to start multiple voices at the same time
		int voicemask = command >> 4;
		//if (voicemask != 0 && voicemask != 1 && voicemask != 2 && voicemask != 4 && voicemask != 8)
		//  popmessage("OKI6295 start %x contact MAMEDEV", voicemask);

		// update the stream
		m_stream->update();

		// determine which voice(s) (voice is set by a 1 bit in the upper 4 bits of the second byte)
		for (int voicenum = 0; voicenum < OKIM6295_VOICES; voicenum++, voicemask >>= 1)
			if (voicemask & 1)
			{
				okim_voice &voice = m_voice[voicenum];

				if (!voice.m_playing) // fixes Got-cha and Steel Force
				{
					// determine the start/stop positions
					offs_t base = m_command * 8;

					offs_t start = read_byte(base + 0) << 16;
					start |= read_byte(base + 1) << 8;
					start |= read_byte(base + 2) << 0;
					start &= 0x3ffff;

					offs_t stop = read_byte(base + 3) << 16;
					stop |= read_byte(base + 4) << 8;
					stop |= read_byte(base + 5) << 0;
					stop &= 0x3ffff;

					if (start < stop)
					{
						// set up the voice to play this sample
						voice.m_playing = true;
						voice.m_base_offset = start;
						voice.m_sample = 0;
						voice.m_count = 2 * (stop - start + 1);

						// also reset the ADPCM parameters
						voice.m_adpcm.reset();
						voice.m_volume = s_volume_table[command & 0x0f];
					}

					// invalid samples go here
					else
					{
						logerror("Requested to play invalid sample %02x\n", m_command);
					}
				}
				else
				{
					logerror("Requested to play sample %02x on non-stopped voice\n", m_command);
				}
			}

		// reset the command
		m_command = -1;
	}

	// if this is the start of a command, remember the sample number for next time
	else if (command & 0x80)
		m_command = command & 0x7f;

	// otherwise, see if this is a silence command
	else
	{
		// update the stream, then turn it off
		m_stream->update();

		// determine which voice(s) (voice is set by a 1 bit in bits 3-6 of the command
		int voicemask = command >> 3;
		for (int voicenum = 0; voicenum < OKIM6295_VOICES; voicenum++, voicemask >>= 1)
			if (voicemask & 1)
				m_voice[voicenum].m_playing = false;
	}
}


//-------------------------------------------------
//  write - memory interface for write
//-------------------------------------------------

WRITE8_MEMBER( okim6295_device::write )
{
	write_command(data);
}



//**************************************************************************
//  OKIM VOICE
//**************************************************************************

//-------------------------------------------------
//  okim_voice - constructor
//-------------------------------------------------

okim6295_device::okim_voice::okim_voice()
	: m_playing(false),
		m_base_offset(0),
		m_sample(0),
		m_count(0),
		m_volume(0)
{
}


//-------------------------------------------------
//  generate_adpcm - generate ADPCM samples and
//  add them to an output stream
//-------------------------------------------------

void okim6295_device::okim_voice::generate_adpcm(device_rom_interface &rom, stream_sample_t *buffer, int samples)
{
	// skip if not active
	if (!m_playing)
		return;

	// loop while we still have samples to generate
	while (samples-- != 0)
	{
		// fetch the next sample byte
		int nibble = rom.read_byte(m_base_offset + m_sample / 2) >> (((m_sample & 1) << 2) ^ 4);

		// output to the buffer, scaling by the volume
		// signal in range -2048..2047, volume in range 2..32 => signal * volume / 2 in range -32768..32767
		*buffer++ += m_adpcm.clock(nibble) * m_volume / 2;

		// next!
		if (++m_sample >= m_count)
		{
			m_playing = false;
			break;
		}
	}
}
