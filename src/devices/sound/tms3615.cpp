// license:BSD-3-Clause
// copyright-holders:Juergen Buchmueller
#include "emu.h"
#include "tms3615.h"

#define VMIN    0x0000
#define VMAX    0x7fff

const int tms3615_device::divisor[TMS3615_TONES] = { 478, 451, 426, 402, 379, 358, 338, 319, 301, 284, 268, 253, 239 };


// device type definition
DEFINE_DEVICE_TYPE(TMS3615, tms3615_device, "tms3615", "TMS3615")


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  tms3615_device - constructor
//-------------------------------------------------

tms3615_device::tms3615_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, TMS3615, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_channel(nullptr)
	, m_samplerate(0)
	, m_basefreq(0)
	, m_output8(0)
	, m_output16(0)
	, m_enable(0)
{
	std::fill(std::begin(m_counter8), std::end(m_counter8), 0);
	std::fill(std::begin(m_counter16), std::end(m_counter16), 0);
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void tms3615_device::device_start()
{
	m_channel = stream_alloc(0, 2, clock()/8);
	m_samplerate = clock()/8;
	m_basefreq = clock();
}


//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void tms3615_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	int samplerate = m_samplerate;
	stream_sample_t *buffer8 = outputs[FOOTAGE_8];
	stream_sample_t *buffer16 = outputs[FOOTAGE_16];

	while( samples-- > 0 )
	{
		int sum8 = 0, sum16 = 0, tone = 0;

		for (tone = 0; tone < TMS3615_TONES; tone++)
		{
			// 8'

			m_counter8[tone] -= (m_basefreq / divisor[tone]);

			while( m_counter8[tone] <= 0 )
			{
				m_counter8[tone] += samplerate;
				m_output8 ^= 1 << tone;
			}

			if (m_output8 & m_enable & (1 << tone))
			{
				sum8 += VMAX;
			}

			// 16'

			m_counter16[tone] -= (m_basefreq / divisor[tone] / 2);

			while( m_counter16[tone] <= 0 )
			{
				m_counter16[tone] += samplerate;
				m_output16 ^= 1 << tone;
			}

			if (m_output16 & m_enable & (1 << tone))
			{
				sum16 += VMAX;
			}
		}

		*buffer8++ = sum8 / TMS3615_TONES;
		*buffer16++ = sum16 / TMS3615_TONES;
	}

	m_enable = 0;
}


void tms3615_device::enable_w(int enable)
{
	m_enable = enable;
}
