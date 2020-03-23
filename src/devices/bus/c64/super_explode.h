// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Super Explode! cartridge emulation

**********************************************************************/

#ifndef MAME_BUS_C64_SUPER_EXPLODE_H
#define MAME_BUS_C64_SUPER_EXPLODE_H

#pragma once


#include "exp.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> c64_super_explode_cartridge_device

class c64_super_explode_cartridge_device : public device_t,
											public device_c64_expansion_card_interface
{
public:
	// construction/destruction
	c64_super_explode_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// device_c64_expansion_card_interface overrides
	virtual uint8_t c64_cd_r(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2) override;
	virtual void c64_cd_w(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2) override;

private:
	uint8_t m_bank;

	emu_timer *m_exrom_timer;
};


// device type definition
DECLARE_DEVICE_TYPE(C64_SUPER_EXPLODE, c64_super_explode_cartridge_device)


#endif // MAME_BUS_C64_SUPER_EXPLODE_H
