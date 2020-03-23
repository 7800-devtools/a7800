// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
#ifndef MAME_BUS_ISA_VGA_H
#define MAME_BUS_ISA_VGA_H

#pragma once

#include "isa.h"
#include "video/pc_vga.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> isa8_vga_device

class isa8_vga_device :
		public device_t,
		public device_isa8_card_interface
{
public:
	// construction/destruction
	isa8_vga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(input_port_0_r);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	vga_device *m_vga;
};


// device type definition
DECLARE_DEVICE_TYPE(ISA8_VGA, isa8_vga_device)

#endif // MAME_BUS_ISA_VGA_H
