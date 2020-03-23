// license:BSD-3-Clause
// copyright-holders:Angelo Salese,Takahiro Nogi
/***************************************************************************

    Nichibutsu sound HW

***************************************************************************/

#ifndef MAME_AUDIO_NICHISND_H
#define MAME_AUDIO_NICHISND_H

#pragma once

#include "cpu/z80/tmpz84c011.h"
#include "sound/3812intf.h"
#include "sound/dac.h"
#include "sound/volt_reg.h"
#include "speaker.h"
#include "machine/gen_latch.h"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_NICHISND_ADD(tag) \
		MCFG_DEVICE_ADD((tag), NICHISND, (0))


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> nichisnd_device

class nichisnd_device : public device_t
{
public:
	// construction/destruction
	nichisnd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// I/O operations
	DECLARE_WRITE8_MEMBER( soundlatch_clear_w );
	DECLARE_WRITE8_MEMBER( soundbank_w );
	DECLARE_WRITE8_MEMBER( sound_host_command_w );

protected:
	// device-level overrides
	//virtual void device_validity_check(validity_checker &valid) const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	required_device<generic_latch_8_device> m_soundlatch;
	required_region_ptr<uint8_t> m_sound_rom;
};


// device type definition
DECLARE_DEVICE_TYPE(NICHISND, nichisnd_device)



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************


#endif // MAME_MACHINE_NICHISND_H
