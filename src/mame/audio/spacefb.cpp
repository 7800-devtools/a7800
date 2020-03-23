// license:BSD-3-Clause
// copyright-holders:Chris Hardy
/***************************************************************************

    Space Firebird hardware

****************************************************************************/

#include "emu.h"
#include "includes/spacefb.h"

#include "cpu/mcs48/mcs48.h"
#include "sound/dac.h"
#include "sound/samples.h"
#include "sound/volt_reg.h"
#include "speaker.h"


READ8_MEMBER(spacefb_state::audio_p2_r)
{
	return (m_sound_latch & 0x18) << 1;
}


READ_LINE_MEMBER(spacefb_state::audio_t0_r)
{
	return BIT(m_sound_latch, 6);
}


READ_LINE_MEMBER(spacefb_state::audio_t1_r)
{
	return BIT(m_sound_latch, 2);
}


WRITE8_MEMBER(spacefb_state::port_1_w)
{
	m_audiocpu->set_input_line(0, (data & 0x02) ? CLEAR_LINE : ASSERT_LINE);

	/* enemy killed */
	if (!(data & 0x01) && (m_sound_latch & 0x01))  m_samples->start(0,0);

	/* ship fire */
	if (!(data & 0x40) && (m_sound_latch & 0x40))  m_samples->start(1,1);

	/*
	 *  Explosion Noise
	 *
	 *  Actual sample has a bit of attack at the start, but these doesn't seem to be an easy way
	 *  to play the attack part, then loop the middle bit until the sample is turned off
	 *  Fortunately it seems like the recorded sample of the spaceship death is the longest the sample plays for.
	 *  We loop it just in case it runs out
	 */
	if ((data & 0x80) != (m_sound_latch & 0x80))
	{
		if (data & 0x80)
			/* play decaying noise */
			m_samples->start(2,3);
		else
			/* start looping noise */
			m_samples->start(2,2, true);
	}

	m_sound_latch = data;
}


static const char *const spacefb_sample_names[] =
{
	"*spacefb",
	"ekilled",
	"shipfire",
	"explode1",
	"explode2",
	nullptr
};


MACHINE_CONFIG_START( spacefb_audio )
	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_ADD("dac", DAC_8BIT_R2R, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.25) // unknown DAC
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac", -1.0, DAC_VREF_NEG_INPUT)

	MCFG_SOUND_ADD("samples", SAMPLES, 0)
	MCFG_SAMPLES_CHANNELS(3)
	MCFG_SAMPLES_NAMES(spacefb_sample_names)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 1.0)
MACHINE_CONFIG_END
