// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Commodore 64 CP/M cartridge emulation

**********************************************************************/

#ifndef MAME_BUS_C64_CPM_H
#define MAME_BUS_C64_CPM_H

#pragma once

#include "exp.h"
#include "cpu/z80/z80.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> c64_cpm_cartridge_device

class c64_cpm_cartridge_device : public device_t,
									public device_c64_expansion_card_interface
{
public:
	// construction/destruction
	c64_cpm_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// not really public
	DECLARE_READ8_MEMBER( dma_r );
	DECLARE_WRITE8_MEMBER( dma_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;

	// device_c64_expansion_card_interface overrides
	virtual void c64_cd_w(address_space &space, offs_t offset, uint8_t data, int sphi2, int ba, int roml, int romh, int io1, int io2) override;
	virtual int c64_game_r(offs_t offset, int sphi2, int ba, int rw) override;

private:
	inline void update_signals();

	required_device<cpu_device> m_maincpu;

	int m_enabled;
	int m_ba;

	int m_reset;
};


// device type definition
DECLARE_DEVICE_TYPE(C64_CPM, c64_cpm_cartridge_device)


#endif // MAME_BUS_C64_CPM_H
