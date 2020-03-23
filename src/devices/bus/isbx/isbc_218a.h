// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    ISBX 218a with ISBC configuration

**********************************************************************/

#ifndef MAME_BUS_ISBX_ISBC_218A_H
#define MAME_BUS_ISBX_ISBC_218A_H

#pragma once

#include "isbx.h"
#include "formats/pc_dsk.h"
#include "machine/upd765.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> compis_fdc_device

class isbc_218a_device : public device_t,
							public device_isbx_card_interface
{
public:
	// construction/destruction
	isbc_218a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;

	// device_isbx_card_interface overrides
	virtual uint8_t mcs0_r(address_space &space, offs_t offset) override;
	virtual void mcs0_w(address_space &space, offs_t offset, uint8_t data) override;
	virtual uint8_t mcs1_r(address_space &space, offs_t offset) override;
	virtual void mcs1_w(address_space &space, offs_t offset, uint8_t data) override;
	virtual uint8_t mdack_r(address_space &space, offs_t offset) override;
	virtual void mdack_w(address_space &space, offs_t offset, uint8_t data) override;
	virtual void opt0_w(int state) override;

private:
	DECLARE_WRITE_LINE_MEMBER( fdc_irq );
	DECLARE_WRITE_LINE_MEMBER( fdc_drq );
	DECLARE_FLOPPY_FORMATS( floppy_formats );

	required_device<i8272a_device> m_fdc;
	required_device<floppy_connector> m_floppy0;

	bool m_reset, m_motor, m_fd8;
};


// device type definition
DECLARE_DEVICE_TYPE(ISBC_218A, isbc_218a_device)


#endif // MAME_BUS_ISBX_ISBC_218A_H
