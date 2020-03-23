// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Final Cartridge emulation

**********************************************************************/

#ifndef MAME_BUS_C64_FINAL_H
#define MAME_BUS_C64_FINAL_H

#pragma once


#include "exp.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> c64_final_cartridge_device

class c64_final_cartridge_device : public device_t,
									public device_c64_expansion_card_interface
{
public:
	// construction/destruction
	c64_final_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual ioport_constructor device_input_ports() const override;

	DECLARE_INPUT_CHANGED_MEMBER( freeze );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_c64_expansion_card_interface overrides
	virtual uint8_t c64_cd_r(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2) override;
	virtual void c64_cd_w(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2) override;
};


// device type definition
DECLARE_DEVICE_TYPE(C64_FINAL, c64_final_cartridge_device)


#endif // MAME_BUS_C64_FINAL_H
