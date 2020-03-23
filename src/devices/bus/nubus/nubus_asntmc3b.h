// license:BSD-3-Clause
// copyright-holders:R. Belmont
#ifndef MAME_BUS_NUBUS_NUBUS_ASNTMC3B_H
#define MAME_BUS_NUBUS_NUBUS_ASNTMC3B_H

#pragma once

#include "nubus.h"
#include "machine/dp8390.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> nubus_mac8390_device

class nubus_mac8390_device :
		public device_t,
		public device_nubus_card_interface
{
protected:
	// construction/destruction
	nubus_mac8390_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

	DECLARE_READ8_MEMBER(asntm3b_ram_r);
	DECLARE_WRITE8_MEMBER(asntm3b_ram_w);
	DECLARE_READ32_MEMBER(en_r);
	DECLARE_WRITE32_MEMBER(en_w);

	required_device<dp8390_device> m_dp83902;

private:
	void dp_irq_w(int state);
	DECLARE_READ8_MEMBER(dp_mem_read);
	DECLARE_WRITE8_MEMBER(dp_mem_write);

	uint8_t m_ram[0x20000];
	uint8_t m_prom[16];
};

class nubus_asntmc3nb_device : public nubus_mac8390_device
{
public:
	nubus_asntmc3nb_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

class nubus_appleenet_device : public nubus_mac8390_device
{
public:
	nubus_appleenet_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual const tiny_rom_entry *device_rom_region() const override;
};

// device type definition
DECLARE_DEVICE_TYPE(NUBUS_ASNTMC3NB, nubus_asntmc3nb_device)
DECLARE_DEVICE_TYPE(NUBUS_APPLEENET, nubus_appleenet_device)

#endif // MAME_BUS_NUBUS_NUBUS_ASNTMC3B_H
