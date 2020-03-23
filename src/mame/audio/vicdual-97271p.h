// license:BSD-3-Clause
// copyright-holders:Ariane Fugmann
#ifndef MAME_AUDIO_VICDUAL_97271P_H
#define MAME_AUDIO_VICDUAL_97271P_H

#pragma once

#include "sound/samples.h"

#define MCFG_S97271P_ADD(_tag ) \
	MCFG_DEVICE_ADD(_tag, S97271P, 0)

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class s97271p_device : public device_t
{
public:
	// construction/destruction
	s97271p_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// daughterboard logic
	void port_w(uint8_t data);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	required_device<samples_device> m_samples;

	uint8_t m_state;
};

// device type definition
DECLARE_DEVICE_TYPE(S97271P, s97271p_device)

#endif // MAME_AUDIO_VICDUAL_97271P_H
