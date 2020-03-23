// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
#ifndef MAME_BUS_ISA_SVGA_CIRRUS_H
#define MAME_BUS_ISA_SVGA_CIRRUS_H

#pragma once

#include "isa.h"
#include "video/clgd542x.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class isa16_svga_cirrus_device :
		public device_t,
		public device_isa16_card_interface
{
public:
	// construction/destruction
	isa16_svga_cirrus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(input_port_0_r);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	cirrus_gd5430_device *m_vga;
};

class isa16_svga_cirrus_gd542x_device :
		public device_t,
		public device_isa16_card_interface
{
public:
	// construction/destruction
	isa16_svga_cirrus_gd542x_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(input_port_0_r);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	cirrus_gd5428_device *m_vga;
};


// device type definition
DECLARE_DEVICE_TYPE(ISA16_SVGA_CIRRUS,        isa16_svga_cirrus_device)
DECLARE_DEVICE_TYPE(ISA16_SVGA_CIRRUS_GD542X, isa16_svga_cirrus_gd542x_device)

#endif // MAME_BUS_ISA_SVGA_CIRRUS_H
