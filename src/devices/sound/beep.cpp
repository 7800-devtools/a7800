// license:BSD-3-Clause
// copyright-holders:Kevin Thacker
/***************************************************************************

    beep.c

    This is used for computers/systems which can only output a constant tone.
    This tone can be turned on and off.
    e.g. PCW and PCW16 computer systems
    KT - 25-Jun-2000

    Sound handler

****************************************************************************/

#include "emu.h"
#include "sound/beep.h"

#define BEEP_RATE (48000)


// device type definition
DEFINE_DEVICE_TYPE(BEEP, beep_device, "beep", "Beep")


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  beep_device - constructor
//-------------------------------------------------

beep_device::beep_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, BEEP, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_stream(nullptr)
	, m_enable(0)
	, m_frequency(clock)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void beep_device::device_start()
{
	m_stream = stream_alloc(0, 1, BEEP_RATE);
	m_enable = 0;
	m_signal = 0x07fff;

	// register for savestates
	save_item(NAME(m_enable));
	save_item(NAME(m_frequency));
	save_item(NAME(m_incr));
	save_item(NAME(m_signal));
}


//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void beep_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	stream_sample_t *buffer = outputs[0];
	int16_t signal = m_signal;
	int clock = 0, rate = BEEP_RATE / 2;

	/* get progress through wave */
	int incr = m_incr;

	if (m_frequency > 0)
		clock = m_frequency;

	/* if we're not enabled, just fill with 0 */
	if ( !m_enable || clock == 0 )
	{
		memset( buffer, 0, samples * sizeof(*buffer) );
		return;
	}

	/* fill in the sample */
	while( samples-- > 0 )
	{
		*buffer++ = signal;
		incr -= clock;
		while( incr < 0 )
		{
			incr += rate;
			signal = -signal;
		}
	}

	/* store progress through wave */
	m_incr = incr;
	m_signal = signal;
}


//-------------------------------------------------
//  changing state to on from off will restart tone
//-------------------------------------------------

WRITE_LINE_MEMBER(beep_device::set_state)
{
	/* only update if new state is not the same as old state */
	int on = (state) ? 1 : 0;
	if (m_enable == on)
		return;

	m_stream->update();
	m_enable = on;

	/* restart wave from beginning */
	m_incr = 0;
	m_signal = 0x07fff;
}


//-------------------------------------------------
//  setting new frequency starts from beginning
//-------------------------------------------------

void beep_device::set_clock(uint32_t frequency)
{
	if (m_frequency == frequency)
		return;

	m_stream->update();
	m_frequency = frequency;
	m_signal = 0x07fff;
	m_incr = 0;
}
