// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    CBM 500/600/700 High Resolution Graphics cartridge emulation

**********************************************************************/

#ifndef MAME_BUS_CBM2_HRG_H
#define MAME_BUS_CBM2_HRG_H

#pragma once

#include "exp.h"
#include "video/ef9365.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> cbm2_hrg_device

class cbm2_hrg_device : public device_t,
					public device_cbm2_expansion_card_interface
{
public:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;

protected:
	// construction/destruction
	cbm2_hrg_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_cbm2_expansion_card_interface overrides
	virtual uint8_t cbm2_bd_r(address_space &space, offs_t offset, uint8_t data, int csbank1, int csbank2, int csbank3) override;
	virtual void cbm2_bd_w(address_space &space, offs_t offset, uint8_t data, int csbank1, int csbank2, int csbank3) override;

private:
	required_device<ef9365_device> m_gdc;
	required_memory_region m_bank3;
};


// ======================> cbm2_hrg_a_device

class cbm2_hrg_a_device :  public cbm2_hrg_device
{
public:
	// construction/destruction
	cbm2_hrg_a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
};


// ======================> cbm2_hrg_b_device

class cbm2_hrg_b_device :  public cbm2_hrg_device
{
public:
	// construction/destruction
	cbm2_hrg_b_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
};


// device type definition
DECLARE_DEVICE_TYPE(CBM2_HRG_A, cbm2_hrg_a_device)
DECLARE_DEVICE_TYPE(CBM2_HRG_B, cbm2_hrg_b_device)


#endif // MAME_BUS_CBM2_HRG_H
