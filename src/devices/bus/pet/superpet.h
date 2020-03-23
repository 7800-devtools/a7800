// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Commodore SuperPET emulation

**********************************************************************/

#ifndef MAME_BUS_PET_SUPERPET_H
#define MAME_BUS_PET_SUPERPET_H

#pragma once

#include "exp.h"
#include "machine/mos6551.h"
#include "machine/mos6702.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> superpet_device

class superpet_device : public device_t,
						public device_pet_expansion_card_interface
{
public:
	// construction/destruction
	superpet_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER( read );
	DECLARE_WRITE8_MEMBER( write );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

	// device_pet_expansion_card_interface overrides
	virtual int pet_norom_r(address_space &space, offs_t offset, int sel) override;
	virtual uint8_t pet_bd_r(address_space &space, offs_t offset, uint8_t data, int &sel) override;
	virtual void pet_bd_w(address_space &space, offs_t offset, uint8_t data, int &sel) override;
	virtual int pet_diag_r() override;
	virtual void pet_irq_w(int state) override;

private:
	DECLARE_WRITE_LINE_MEMBER( acia_irq_w );

	required_device<cpu_device> m_maincpu;
	required_device<mos6551_device> m_acia;
	required_device<mos6702_device> m_dongle;
	required_memory_region m_rom;
	optional_shared_ptr<uint8_t> m_ram;
	required_ioport m_io_sw1;
	required_ioport m_io_sw2;

	inline void update_cpu();
	inline bool is_ram_writable();

	uint8_t m_system;
	uint8_t m_bank;
	uint8_t m_sw1;
	uint8_t m_sw2;
	int m_sel9_rom;
	int m_pet_irq;
	int m_acia_irq;
};


// device type definition
DECLARE_DEVICE_TYPE(SUPERPET, superpet_device)


#endif // MAME_BUS_PET_SUPERPET_H
