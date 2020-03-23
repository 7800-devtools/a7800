// license:BSD-3-Clause
// copyright-holders:Nigel Barnes
/**********************************************************************

    Power Software Joystick Interface

**********************************************************************/

#ifndef MAME_BUS_ELECTRON_PWRJOY_H
#define MAME_BUS_ELECTRON_PWRJOY_H

#pragma once


#include "exp.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> electron_pwrjoy_device

class electron_pwrjoy_device :
	public device_t,
	public device_electron_expansion_interface
{
public:
	// construction/destruction
	electron_pwrjoy_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual ioport_constructor device_input_ports() const override;

	DECLARE_READ8_MEMBER(joystick_r);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	required_memory_region m_exp_rom;
	required_ioport m_joy;
};


// device type definition
DECLARE_DEVICE_TYPE(ELECTRON_PWRJOY, electron_pwrjoy_device)

#endif // MAME_BUS_ELECTRON_PWRJOY_H
