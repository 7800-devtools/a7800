// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    CMD HD disk drive emulation

**********************************************************************/

#ifndef MAME_BUS_CBMIEC_CMDHD_H
#define MAME_BUS_CBMIEC_CMDHD_H

#pragma once

#include "cbmiec.h"
#include "cpu/m6502/m6502.h"
#include "imagedev/harddriv.h"
#include "machine/6522via.h"
#include "machine/i8255.h"
#include "bus/scsi/scsihd.h"



//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define CMD_HD_TAG          "cmdhd"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> cmd_hd_device

class cmd_hd_device : public device_t, public device_cbm_iec_interface
{
public:
	// construction/destruction
	cmd_hd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_WRITE8_MEMBER( led_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;

	// device_cbm_iec_interface overrides
	void cbm_iec_srq(int state) override;
	void cbm_iec_atn(int state) override;
	void cbm_iec_data(int state) override;
	void cbm_iec_reset(int state) override;

	required_device<cpu_device> m_maincpu;
	required_device<scsi_port_device> m_scsibus;
};


// device type definition
DECLARE_DEVICE_TYPE(CMD_HD, cmd_hd_device)


#endif // MAME_BUS_CBMIEC_CMDHD_H
