// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Miracle Systems Gold Card emulation

**********************************************************************/

#ifndef MAME_BUS_QL_MIRACLE_GOLD_CARD_H
#define MAME_BUS_QL_MIRACLE_GOLD_CARD_H

#pragma once

#include "exp.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> miracle_gold_card_device

class miracle_gold_card_device : public device_t, public device_ql_expansion_card_interface
{
public:
	// construction/destruction
	miracle_gold_card_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;

protected:
	// device-level overrides
	virtual void device_start() override;

	// device_ql_expansion_card_interface overrides
	virtual uint8_t read(address_space &space, offs_t offset, uint8_t data) override;
	virtual void write(address_space &space, offs_t offset, uint8_t data) override;
};



// device type definition
DECLARE_DEVICE_TYPE(MIRACLE_GOLD_CARD, miracle_gold_card_device)

#endif // MAME_BUS_QL_MIRACLE_GOLD_CARD_H
