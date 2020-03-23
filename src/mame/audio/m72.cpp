// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria
/***************************************************************************

IREM "M72" sound hardware

All games have a YM2151 for music, and most of them also samples. Samples
are not handled consistently by all the games, some use a high frequency NMI
handler to push them through a DAC, others use external hardware.
In the following table, the NMI column indicates with a No the games whose
NMI handler only consists of RETN. R-Type is an exception, it doesn't have
a valid NMI handler at all.

Game                                    Year  ID string     NMI
--------------------------------------  ----  ------------  ---
R-Type                                  1987  - (earlier version, no samples)
Battle Chopper / Mr. Heli               1987  Rev 2.20      Yes
Vigilante                               1988  Rev 2.20      Yes
Ninja Spirit                            1988  Rev 2.20      Yes
Image Fight                             1988  Rev 2.20      Yes
Legend of Hero Tonma                    1989  Rev 2.20      Yes
X Multiply                              1989  Rev 2.20      Yes
Dragon Breed                            1989  Rev 2.20      Yes
Kickle Cubicle                          1988  Rev 2.21      Yes
Shisensho                               1989  Rev 2.21      Yes
R-Type II                               1989  Rev 2.21      Yes
Major Title                             1990  Rev 2.21      Yes
Air Duel                                1990  Rev 3.14 M72   No
Daiku no Gensan                         1990  Rev 3.14 M81  Yes
Daiku no Gensan (M72)                   1990  Rev 3.15 M72   No
Hammerin' Harry                         1990  Rev 3.15 M81  Yes
Ken-Go                                  1991  Rev 3.15 M81  Yes
Pound for Pound                         1990  Rev 3.15 M83   No
Cosmic Cop                              1991  Rev 3.15 M81  Yes
Gallop - Armed Police Unit              1991  Rev 3.15 M72   No
Hasamu                                  1991  Rev 3.15 M81  Yes
Bomber Man                              1991  Rev 3.15 M81  Yes
Bomber Man World (Japan)                1992  Rev 3.31 M81  Yes
Bomber Man World (World) / Atomic Punk  1992  Rev 3.31 M99   No
Quiz F-1 1,2finish                      1992  Rev 3.33 M81  Yes
Risky Challenge                         1993  Rev 3.34 M81  Yes
Shisensho II                            1993  Rev 3.34 M81  Yes

***************************************************************************/

#include "emu.h"
#include "m72.h"


DEFINE_DEVICE_TYPE(IREM_M72_AUDIO, m72_audio_device, "m72_audio", "Irem M72 Audio")

m72_audio_device::m72_audio_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, IREM_M72_AUDIO, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_irqvector(0)
	, m_sample_addr(0)
	, m_samples(*this, "^samples")
	, m_samples_size(0)
	, m_dac(*this, "^dac")
	, m_soundlatch(*this, "^soundlatch")
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void m72_audio_device::device_start()
{
	m_samples_size = m_samples.bytes();
	m_space = &machine().device("soundcpu")->memory().space(AS_IO);

	save_item(NAME(m_irqvector));
	save_item(NAME(m_sample_addr));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void m72_audio_device::device_reset()
{
	m_irqvector = 0xff;
}


/*

  The sound CPU runs in interrup mode 0. IRQ is shared by two sources: the
  YM2151 (bit 4 of the vector), and the main CPU (bit 5).
  Since the vector can be changed from different contexts (the YM2151 timer
  callback, the main CPU context, and the sound CPU context), it's important
  to accurately arbitrate the changes to avoid out-of-order execution. We do
  that by handling all vector changes in a single timer callback.

*/

TIMER_CALLBACK_MEMBER( m72_audio_device::setvector_callback )
{
	switch(param)
	{
		case YM2151_ASSERT:
			m_irqvector &= 0xef;
			break;

		case YM2151_CLEAR:
			m_irqvector |= 0x10;
			break;

		case Z80_ASSERT:
			m_irqvector &= 0xdf;
			break;

		case Z80_CLEAR:
			m_irqvector |= 0x20;
			break;
	}

	machine().device("soundcpu")->execute().set_input_line_and_vector(0, (m_irqvector == 0xff) ? CLEAR_LINE : ASSERT_LINE, m_irqvector);
}


WRITE_LINE_MEMBER(m72_audio_device::ym2151_irq_handler)
{
	machine().scheduler().synchronize(timer_expired_delegate(FUNC(m72_audio_device::setvector_callback), this), state ? YM2151_ASSERT : YM2151_CLEAR);
}

WRITE16_MEMBER( m72_audio_device::sound_command_w )
{
	if (ACCESSING_BITS_0_7)
	{
		m_soundlatch->write(*m_space, offset, data);
		space.machine().scheduler().synchronize(timer_expired_delegate(FUNC(m72_audio_device::setvector_callback), this), Z80_ASSERT);
	}
}

WRITE8_MEMBER( m72_audio_device::sound_command_byte_w )
{
	m_soundlatch->write(*m_space, offset, data);
	space.machine().scheduler().synchronize(timer_expired_delegate(FUNC(m72_audio_device::setvector_callback), this), Z80_ASSERT);
}

WRITE8_MEMBER( m72_audio_device::sound_irq_ack_w )
{
	space.machine().scheduler().synchronize(timer_expired_delegate(FUNC(m72_audio_device::setvector_callback), this), Z80_CLEAR);
}



void m72_audio_device::set_sample_start(int start)
{
	m_sample_addr = start;
}

WRITE8_MEMBER( m72_audio_device::vigilant_sample_addr_w )
{
	if (offset == 1)
		m_sample_addr = (m_sample_addr & 0x00ff) | ((data << 8) & 0xff00);
	else
		m_sample_addr = (m_sample_addr & 0xff00) | ((data << 0) & 0x00ff);
}

WRITE8_MEMBER( m72_audio_device::shisen_sample_addr_w )
{
	m_sample_addr >>= 2;

	if (offset == 1)
		m_sample_addr = (m_sample_addr & 0x00ff) | ((data << 8) & 0xff00);
	else
		m_sample_addr = (m_sample_addr & 0xff00) | ((data << 0) & 0x00ff);

	m_sample_addr <<= 2;
}

WRITE8_MEMBER( m72_audio_device::rtype2_sample_addr_w )
{
	m_sample_addr >>= 5;

	if (offset == 1)
		m_sample_addr = (m_sample_addr & 0x00ff) | ((data << 8) & 0xff00);
	else
		m_sample_addr = (m_sample_addr & 0xff00) | ((data << 0) & 0x00ff);

	m_sample_addr <<= 5;
}

WRITE8_MEMBER( m72_audio_device::poundfor_sample_addr_w )
{
	/* poundfor writes both sample start and sample END - a first for Irem...
	   we don't handle the end written here, 00 marks the sample end as usual. */
	if (offset > 1) return;

	m_sample_addr >>= 4;

	if (offset == 1)
		m_sample_addr = (m_sample_addr & 0x00ff) | ((data << 8) & 0xff00);
	else
		m_sample_addr = (m_sample_addr & 0xff00) | ((data << 0) & 0x00ff);

	m_sample_addr <<= 4;
}

READ8_MEMBER( m72_audio_device::sample_r )
{
	return m_samples[m_sample_addr];
}

WRITE8_MEMBER( m72_audio_device::sample_w )
{
	m_dac->write(data);
	m_sample_addr = (m_sample_addr + 1) & (m_samples_size - 1);
}


//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void m72_audio_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
}
