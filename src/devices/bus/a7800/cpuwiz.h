// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
#ifndef MAME_BUS_A7800_CPUWIZ_H
#define MAME_BUS_A7800_CPUWIZ_H

#pragma once

#include "a78_slot.h"
#include "rom.h"


// ======================> a78_versaboard_device

class a78_versaboard_device : public a78_rom_sg_device
{
public:
	// construction/destruction
	a78_versaboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override;

protected:
	a78_versaboard_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	int m_ram_bank;
};


// ======================> a78_megacart_device

class a78_megacart_device : public a78_versaboard_device
{
public:
	// construction/destruction
	a78_megacart_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// reading and writing
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override;
};


// VersaBoard variants of the standard carts + POKEY at 0x0450!

// ======================> a78_rom_p450_vb_device

class a78_rom_p450_vb_device : public a78_versaboard_device
{
public:
	// construction/destruction
	a78_rom_p450_vb_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_04xx) override { if (offset >= 0x50 && offset < 0x60) return m_pokey450->read(space, offset & 0x0f); else return 0xff; }
	virtual DECLARE_WRITE8_MEMBER(write_04xx) override { if (offset >= 0x50 && offset < 0x60) m_pokey450->write(space, offset & 0x0f, data); }

protected:
	virtual void device_add_mconfig(machine_config &config) override;

	required_device<pokey_device> m_pokey450;
};



// device type definition
DECLARE_DEVICE_TYPE(A78_ROM_VERSABOARD, a78_versaboard_device)
DECLARE_DEVICE_TYPE(A78_ROM_MEGACART, a78_megacart_device)

DECLARE_DEVICE_TYPE(A78_ROM_P450_VB, a78_rom_p450_vb_device)

#endif // MAME_BUS_A7800_CPUWIZ_H
