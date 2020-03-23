// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    VideoBrain Standard 2K/4K cartridge emulation

**********************************************************************/

#ifndef MAME_BUS_VIDBRAIN_STD_H
#define MAME_BUS_VIDBRAIN_STD_H

#pragma once

#include "exp.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> videobrain_standard_cartridge_device

class videobrain_standard_cartridge_device : public device_t,
												public device_videobrain_expansion_card_interface
{
public:
	// construction/destruction
	videobrain_standard_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;

	// device_videobrain_expansion_card_interface overrides
	virtual uint8_t videobrain_bo_r(address_space &space, offs_t offset, int cs1, int cs2) override;
};


// device type definition
DECLARE_DEVICE_TYPE(VB_STD, videobrain_standard_cartridge_device)

#endif // MAME_BUS_VIDBRAIN_STD_H
