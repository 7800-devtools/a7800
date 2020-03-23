// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    CBM 8000 High Speed Graphics (324402-01) card emulation

**********************************************************************/

#ifndef MAME_BUS_PET_HSG_H
#define MAME_BUS_PET_HSG_H

#pragma once

#include "exp.h"
#include "video/ef9365.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> cbm8000_hsg_device

class cbm8000_hsg_device : public device_t, public device_pet_expansion_card_interface
{
public:
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;

	// device_pet_expansion_card_interface overrides
	virtual int pet_norom_r(address_space &space, offs_t offset, int sel) override;
	virtual uint8_t pet_bd_r(address_space &space, offs_t offset, uint8_t data, int &sel) override;
	virtual void pet_bd_w(address_space &space, offs_t offset, uint8_t data, int &sel) override;

protected:
	// construction/destruction
	cbm8000_hsg_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	required_device<ef9365_device> m_gdc;
	required_memory_region m_9000;
	required_memory_region m_a000;
};


// ======================> cbm8000_hsg_a_device

class cbm8000_hsg_a_device :  public cbm8000_hsg_device
{
public:
	// construction/destruction
	cbm8000_hsg_a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
};


// ======================> cbm8000_hsg_b_device

class cbm8000_hsg_b_device :  public cbm8000_hsg_device
{
public:
	// construction/destruction
	cbm8000_hsg_b_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
};


// device type definition
DECLARE_DEVICE_TYPE(CBM8000_HSG_A, cbm8000_hsg_a_device)
DECLARE_DEVICE_TYPE(CBM8000_HSG_B, cbm8000_hsg_b_device)

#endif // MAME_BUS_PET_HSG_H
