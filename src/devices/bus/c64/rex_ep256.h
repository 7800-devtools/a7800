// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Rex Datentechnik 256KB EPROM cartridge emulation

**********************************************************************/

#ifndef MAME_BUS_C64_REX_EP256_H
#define MAME_BUS_C64_REX_EP256_H

#pragma once


#include "bus/generic/slot.h"
#include "bus/generic/carts.h"
#include "exp.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> c64_rex_ep256_cartridge_device

class c64_rex_ep256_cartridge_device : public device_t,
										public device_c64_expansion_card_interface
{
public:
	// construction/destruction
	c64_rex_ep256_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;

	// device_c64_expansion_card_interface overrides
	virtual uint8_t c64_cd_r(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2) override;
	virtual void c64_cd_w(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2) override;

private:
	generic_slot_device *m_eproms[8];

	uint8_t m_bank, m_socket;
	int m_reset;
};


// device type definition
DECLARE_DEVICE_TYPE(C64_REX_EP256, c64_rex_ep256_cartridge_device)


#endif // MAME_BUS_C64_REX_EP256_H
