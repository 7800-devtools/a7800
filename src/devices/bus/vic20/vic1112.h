// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Commodore VIC-1112 IEEE-488 Interface Cartridge emulation

    SYS 45065 to start

**********************************************************************/

#ifndef MAME_BUS_VIC20_VIC1112_H
#define MAME_BUS_VIC20_VIC1112_H

#pragma once

#include "exp.h"
#include "bus/ieee488/ieee488.h"
#include "cpu/m6502/m6502.h"
#include "machine/6522via.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> vic1112_device

class vic1112_device :  public device_t,
						public device_vic20_expansion_card_interface
{
public:
	// construction/destruction
	vic1112_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;

	// device_vic20_expansion_card_interface overrides
	virtual uint8_t vic20_cd_r(address_space &space, offs_t offset, uint8_t data, int ram1, int ram2, int ram3, int blk1, int blk2, int blk3, int blk5, int io2, int io3) override;
	virtual void vic20_cd_w(address_space &space, offs_t offset, uint8_t data, int ram1, int ram2, int ram3, int blk1, int blk2, int blk3, int blk5, int io2, int io3) override;

private:
	DECLARE_WRITE_LINE_MEMBER( via0_irq_w );
	DECLARE_READ8_MEMBER( via0_pb_r );
	DECLARE_WRITE8_MEMBER( via0_pb_w );
	DECLARE_WRITE_LINE_MEMBER( via1_irq_w );

	required_device<via6522_device> m_via0;
	required_device<via6522_device> m_via1;
	required_device<ieee488_device> m_bus;

	//uint8_t *m_rom;

	int m_via0_irq;
	int m_via1_irq;
};


// device type definition
DECLARE_DEVICE_TYPE(VIC1112, vic1112_device)

#endif // MAME_BUS_VIC20_VIC1112_H
