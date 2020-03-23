// license:BSD-3-Clause
// copyright-holders:Sergey Svishchev
/**********************************************************************

    Poisk-1 sound card

    Copyright MESS Team.
    Visit http://mamedev.org for licensing and usage restrictions.

**********************************************************************/

#ifndef MAME_BUS_P1_SOUND_H
#define MAME_BUS_P1_SOUND_H

#pragma once


#include "isa.h"
#include "bus/midi/midi.h"
#include "machine/i8251.h"
#include "machine/pit8253.h"
#include "sound/dac.h"
#include "sound/flt_rc.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class p1_sound_device : public device_t,
	public device_isa8_card_interface
{
public:
	// construction/destruction
	p1_sound_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(d14_r);
	DECLARE_READ8_MEMBER(d16_r);
	DECLARE_READ8_MEMBER(d17_r);
	DECLARE_WRITE8_MEMBER(d14_w);
	DECLARE_WRITE8_MEMBER(d16_w);
	DECLARE_WRITE8_MEMBER(d17_w);

	DECLARE_READ8_MEMBER(adc_r);
	DECLARE_WRITE8_MEMBER(dac_w);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// Optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;

private:
	DECLARE_WRITE_LINE_MEMBER(sampler_sync);

	uint8_t m_dac_data[16];
	int m_dac_ptr;

	required_device<dac_byte_interface> m_dac;
	optional_device<filter_rc_device> m_filter;
	required_device<i8251_device> m_midi;
	required_device<pit8253_device> m_d14;
	required_device<pit8253_device> m_d16;
	required_device<pit8253_device> m_d17;
};


// device type definition
DECLARE_DEVICE_TYPE(P1_SOUND, p1_sound_device)


#endif // MAME_BUS_P1_SOUND_H
