// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*********************************************************************

    a2echoii.h

    Implementation of the Street Electronics Echo II speech card

*********************************************************************/

#ifndef MAME_BUS_A2BUS_A2ECHOII_H
#define MAME_BUS_A2BUS_A2ECHOII_H

#pragma once

#include "a2bus.h"
#include "sound/tms5220.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class a2bus_echoii_device:
	public device_t,
	public device_a2bus_card_interface
{
public:
	// construction/destruction
	a2bus_echoii_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	required_device<tms5220_device> m_tms;

protected:
	a2bus_echoii_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

	// overrides of standard a2bus slot functions
	virtual uint8_t read_c0nx(address_space &space, uint8_t offset) override;
	virtual void write_c0nx(address_space &space, uint8_t offset, uint8_t data) override;
	virtual bool take_c800() override;
};

// device type definition
extern const device_type A2BUS_ECHOII;
DECLARE_DEVICE_TYPE(A2BUS_ECHOII, a2bus_echoii_device)

#endif // MAME_BUS_A2BUS_A2ECHOII_H
