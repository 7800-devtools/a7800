// license:BSD-3-Clause
// copyright-holders:David Haywood
#ifndef MAME_BUS_VCS_HARMONY_MELODY_H
#define MAME_BUS_VCS_HARMONY_MELODY_H

#pragma once

#include "rom.h"
#include "cpu/arm7/lpc210x.h"


// ======================> a26_rom_harmony_device

class a26_rom_harmony_device : public a26_rom_f8_device
{
public:
	// construction/destruction
	a26_rom_harmony_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_rom) override;
	virtual DECLARE_WRITE8_MEMBER(write_bank) override;

	DECLARE_READ8_MEMBER(read8_r);

private:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	virtual void device_add_mconfig(machine_config &config) override;

	void check_bankswitch(offs_t offset);

	required_device<lpc210x_device> m_cpu;
};


// device type definition
DECLARE_DEVICE_TYPE(A26_ROM_HARMONY, a26_rom_harmony_device)

#endif // MAME_BUS_VCS_HARMONY_MELODY_H
