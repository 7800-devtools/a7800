// license:GPL-2.0+
// copyright-holders:Dirk Best
/***************************************************************************

    Dick Smith VZ-200/300 RS-232 Cartridge (K-6317)

***************************************************************************/

#ifndef MAME_BUS_VTECH_MEMEXP_RS232_H
#define MAME_BUS_VTECH_MEMEXP_RS232_H

#pragma once

#include "memexp.h"
#include "bus/rs232/rs232.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> vtech_rs232_interface_device

class vtech_rs232_interface_device : public device_t, public device_vtech_memexp_interface
{
public:
	// construction/destruction
	vtech_rs232_interface_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	DECLARE_WRITE_LINE_MEMBER( rs232_rx_w );
	DECLARE_READ8_MEMBER( receive_data_r );
	DECLARE_WRITE8_MEMBER( transmit_data_w );

	required_device<rs232_port_device> m_rs232;

	int m_rx;
};

// device type definition
DECLARE_DEVICE_TYPE(VTECH_RS232_INTERFACE, vtech_rs232_interface_device)

#endif // MAME_BUS_VTECH_MEMEXP_RS232_H
