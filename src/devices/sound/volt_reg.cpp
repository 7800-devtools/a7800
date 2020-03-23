// license:BSD-3-Clause
// copyright-holders:smf
/***************************************************************************

    volt_reg.c

    Direct current.

***************************************************************************/

#include "emu.h"
#include "volt_reg.h"

DEFINE_DEVICE_TYPE(VOLTAGE_REGULATOR, voltage_regulator_device, "volt_reg", "Voltage Regulator")

voltage_regulator_device::voltage_regulator_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, VOLTAGE_REGULATOR, tag, owner, clock),
	device_sound_interface(mconfig, *this),
	m_stream(nullptr),
	m_output(0)
{
}

void voltage_regulator_device::device_start()
{
	m_stream = stream_alloc(0, 1, 500);
}

void voltage_regulator_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	for (int samp = 0; samp < samples; samp++)
	{
		outputs[0][samp] = m_output;
	}
}
