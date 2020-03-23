// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Star NL-10 Printer Interface Cartridge emulation

**********************************************************************/

#ifndef MAME_BUS_CBMIEC_C64_NL10_H
#define MAME_BUS_CBMIEC_C64_NL10_H

#pragma once

#include "cbmiec.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> c64_nl10_interface_device

class c64_nl10_interface_device : public device_t, public device_cbm_iec_interface
{
public:
	// construction/destruction
	c64_nl10_interface_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_cbm_iec_interface overrides
	void cbm_iec_atn(int state) override;
	void cbm_iec_data(int state) override;
	void cbm_iec_reset(int state) override;
};


// device type definition
DECLARE_DEVICE_TYPE(C64_NL10_INTERFACE, c64_nl10_interface_device)


#endif // MAME_BUS_CBMIEC_C64_NL10_H
