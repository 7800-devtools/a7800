// license:BSD-3-Clause
// copyright-holders:Angelo Salese
/***************************************************************************

    Hudson/NEC HuC6271 "Rainbow" device

***************************************************************************/

#ifndef MAME_VIDEO_HUC6271_H
#define MAME_VIDEO_HUC6271_H

#pragma once



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_HUC6271_ADD(tag, freq) \
		MCFG_DEVICE_ADD((tag), HUC6271, (freq))

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> huc6271_device

class huc6271_device : public device_t,
					   public device_memory_interface
{
public:
	// construction/destruction
	huc6271_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// I/O operations
	DECLARE_ADDRESS_MAP(regs, 16);
	//void data_transfer(uint32_t offset, uint32_t data);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual space_config_vector memory_space_config() const override;

private:
	const address_space_config      m_data_space_config;
};


// device type definition
DECLARE_DEVICE_TYPE(HUC6271, huc6271_device)

#endif // MAME_VIDEO_HUC6271_H
