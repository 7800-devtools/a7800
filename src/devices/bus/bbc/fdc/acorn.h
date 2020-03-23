// license:BSD-3-Clause
// copyright-holders:Nigel Barnes
/**********************************************************************

    Acorn 8271 and 1770 FDC

**********************************************************************/


#ifndef MAME_BUS__BBC_FDC_ACORN_H
#define MAME_BUS__BBC_FDC_ACORN_H

#pragma once

#include "fdc.h"
#include "machine/i8271.h"
#include "machine/wd_fdc.h"
#include "formats/acorn_dsk.h"
#include "formats/fsd_dsk.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class bbc_acorn8271_device :
	public device_t,
	public device_bbc_fdc_interface

{
public:
	// construction/destruction
	bbc_acorn8271_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	DECLARE_FLOPPY_FORMATS(floppy_formats);

	DECLARE_WRITE_LINE_MEMBER(fdc_intrq_w);
	DECLARE_WRITE_LINE_MEMBER(motor_w);
	DECLARE_WRITE_LINE_MEMBER(side_w);

	required_memory_region m_dfs_rom;
	required_device<i8271_device> m_fdc;
	required_device<floppy_connector> m_floppy0;
	optional_device<floppy_connector> m_floppy1;
};

class bbc_acorn1770_device :
	public device_t,
	public device_bbc_fdc_interface

{
public:
	// construction/destruction
	bbc_acorn1770_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(wd1770l_read);
	DECLARE_WRITE8_MEMBER(wd1770l_write);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	DECLARE_FLOPPY_FORMATS(floppy_formats);

	DECLARE_WRITE_LINE_MEMBER(fdc_intrq_w);
	DECLARE_WRITE_LINE_MEMBER(fdc_drq_w);

	required_memory_region m_dfs_rom;
	required_device<wd1770_device> m_fdc;
	required_device<floppy_connector> m_floppy0;
	optional_device<floppy_connector> m_floppy1;

	int m_drive_control;
};


// device type definition
DECLARE_DEVICE_TYPE(BBC_ACORN8271, bbc_acorn8271_device)
DECLARE_DEVICE_TYPE(BBC_ACORN1770, bbc_acorn1770_device)


#endif // MAME_BUS__BBC_FDC_ACORN_H
