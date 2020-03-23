// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_VCS_COMPUMAT_H
#define MAME_BUS_VCS_COMPUMAT_H

#pragma once

#include "rom.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> a26_rom_cm_device

class a26_rom_cm_device : public a26_rom_f6_device
{
public:
	// construction/destruction
	a26_rom_cm_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual ioport_constructor device_input_ports() const override;
	virtual void device_reset() override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_rom) override;
};


// device type definition
DECLARE_DEVICE_TYPE(A26_ROM_COMPUMATE, a26_rom_cm_device)

#endif // MAME_BUS_VCS_COMPUMAT_H
