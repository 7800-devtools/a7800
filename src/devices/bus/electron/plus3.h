// license:BSD-3-Clause
// copyright-holders:Nigel Barnes
/**********************************************************************

    ALA13 - Acorn Plus 3

**********************************************************************/

#ifndef MAME_BUS_ELECTRON_PLUS3_H
#define MAME_BUS_ELECTRON_PLUS3_H

#include "exp.h"
#include "machine/wd_fdc.h"
#include "formats/acorn_dsk.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class electron_plus3_device:
	public device_t,
	public device_electron_expansion_interface
{
public:
	// construction/destruction
	electron_plus3_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER(wd1770_status_r);
	DECLARE_WRITE8_MEMBER(wd1770_status_w);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	DECLARE_FLOPPY_FORMATS(floppy_formats);

	required_memory_region m_exp_rom;
	required_device<wd1770_device> m_fdc;
	required_device<floppy_connector> m_floppy0;
	optional_device<floppy_connector> m_floppy1;

	int m_drive_control;
};


// device type definition
DECLARE_DEVICE_TYPE(ELECTRON_PLUS3, electron_plus3_device)


#endif // MAME_BUS_ELECTRON_PLUS3_H
