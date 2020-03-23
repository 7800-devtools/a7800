// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***************************************************************************

    Konami 033906

***************************************************************************/

#ifndef MAME_MACHINE_K033906_H
#define MAME_MACHINE_K033906_H

#pragma once

#include "video/voodoo.h"


/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

#define MCFG_K033906_VOODOO(_tag) \
	k033906_device::set_voodoo_tag(*device, "^" _tag);

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/


// ======================> k033906_device

class k033906_device :  public device_t
{
public:
	// construction/destruction
	k033906_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void set_voodoo_tag(device_t &device, const char *tag) { downcast<k033906_device &>(device).m_voodoo.set_tag(tag); }

	DECLARE_READ32_MEMBER( read );
	DECLARE_WRITE32_MEMBER( write );
	DECLARE_WRITE_LINE_MEMBER( set_reg );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override { }
	virtual void device_post_load() override { }
	virtual void device_clock_changed() override { }

private:
	uint32_t reg_r(int reg);
	void reg_w(int reg, uint32_t data);

	/* i/o lines */

	int          m_reg_set; // 1 = access reg / 0 = access ram

	required_device<voodoo_device> m_voodoo;

	uint32_t       m_reg[256];
	uint32_t       m_ram[32768];
};


// device type definition
DECLARE_DEVICE_TYPE(K033906, k033906_device)

#endif // MAME_MACHINE_K033906_H
