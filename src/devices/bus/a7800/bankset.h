// license:BSD-3-Clause
// copyright-holders:Mike Saarna

#ifndef MAME_BUS_A7800_BANKSET_H
#define MAME_BUS_A7800_BANKSET_H

#pragma once

#include "a78_slot.h"
#include "rom.h"


// ======================> a78_bankset_sg_device

class a78_bankset_sg_device : public a78_rom_sg_device
{
public:
	// construction/destruction
	a78_bankset_sg_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override;

protected:
	a78_bankset_sg_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

};



// ======================> a78_bankset_sg_bankram_device

class a78_bankset_sg_bankram_device : public a78_rom_sg_device
{
public:
	// construction/destruction
	a78_bankset_sg_bankram_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override;

protected:
	a78_bankset_sg_bankram_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

};

// ======================> a78_bankset_rom_device

class a78_bankset_rom_device : public a78_rom_device
{
public:
	// construction/destruction
	a78_bankset_rom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;

protected:
	a78_bankset_rom_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

};

// ======================> a78_bankset_rom_device

class a78_bankset_bankram_device : public a78_rom_device
{
public:
	// construction/destruction
	a78_bankset_bankram_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_40xx) override;
	virtual DECLARE_WRITE8_MEMBER(write_40xx) override;


protected:
	a78_bankset_bankram_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

};



// device type definitions
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_SG, a78_bankset_sg_device)
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_SG_BANKRAM, a78_bankset_sg_bankram_device)
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_ROM, a78_bankset_rom_device)
DECLARE_DEVICE_TYPE(A78_ROM_BANKSET_BANKRAM, a78_bankset_bankram_device)

#endif // MAME_BUS_A7800_BANKSET_H
