// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_SEGA8_CCATCH_H
#define MAME_BUS_SEGA8_CCATCH_H

#pragma once

#include "sega8_slot.h"
#include "rom.h"

// ======================> sega8_cardcatch_device

class sega8_cardcatch_device : public sega8_rom_device
{
public:
	// construction/destruction
	sega8_cardcatch_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_cart) override;
	virtual DECLARE_WRITE8_MEMBER(write_cart) override;
	virtual DECLARE_WRITE8_MEMBER(write_mapper) override {}

protected:
	virtual void device_add_mconfig(machine_config &config) override;

	required_device<sega8_card_slot_device> m_card;
};


// device type definition
DECLARE_DEVICE_TYPE(SEGA8_ROM_CARDCATCH, sega8_cardcatch_device)

#endif // MAME_BUS_SEGA8_CCATCH_H
