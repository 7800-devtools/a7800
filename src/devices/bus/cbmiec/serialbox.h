// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Serial Box 64K Serial Port Buffer emulation

**********************************************************************/

#ifndef MAME_BUS_CBMIEC_SERIALBOX_H
#define MAME_BUS_CBMIEC_SERIALBOX_H

#pragma once

#include "cpu/m6502/m65c02.h"
#include "bus/cbmiec/cbmiec.h"



//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define SERIAL_BOX_TAG          "serialbox"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> serial_box_device

class serial_box_device : public device_t, public device_cbm_iec_interface
{
public:
	// construction/destruction
	serial_box_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;

	// device_cbm_iec_interface overrides
	void cbm_iec_atn(int state) override;
	void cbm_iec_data(int state) override;
	void cbm_iec_reset(int state) override;

private:
	required_device<m65c02_device> m_maincpu;
};


// device type definition
DECLARE_DEVICE_TYPE(SERIAL_BOX, serial_box_device)


#endif // MAME_BUS_CBMIEC_SERIALBOX_H
