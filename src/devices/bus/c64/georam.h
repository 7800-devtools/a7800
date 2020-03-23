// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Berkeley Softworks GeoRAM emulation

**********************************************************************/

#ifndef MAME_BUC_C64_GEORAM_H
#define MAME_BUC_C64_GEORAM_H

#pragma once


#include "exp.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> c64_georam_cartridge_device

class c64_georam_cartridge_device : public device_t,
									public device_c64_expansion_card_interface
{
public:
	// construction/destruction
	c64_georam_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_c64_expansion_card_interface overrides
	virtual uint8_t c64_cd_r(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2) override;
	virtual void c64_cd_w(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2) override;

private:
	optional_shared_ptr<uint8_t> m_ram;

	uint16_t m_bank;
};


// device type definition
DECLARE_DEVICE_TYPE(C64_GEORAM, c64_georam_cartridge_device)


#endif // MAME_BUC_C64_GEORAM_H
