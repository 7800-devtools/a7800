// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Commodore 1551 Single Disk Drive emulation

**********************************************************************/

#ifndef MAME_BUS_PLUS4_C1551_H
#define MAME_BUS_PLUS4_C1551_H

#pragma once

#include "exp.h"
#include "cpu/m6502/m6510t.h"
#include "machine/64h156.h"
#include "machine/6525tpi.h"
#include "machine/pla.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> c1551_device

class c1551_device : public device_t, public device_plus4_expansion_card_interface
{
public:
	// construction/destruction
	c1551_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_READ8_MEMBER( tpi0_r );
	DECLARE_WRITE8_MEMBER( tpi0_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

	// device_plus4_expansion_card_interface overrides
	virtual uint8_t plus4_cd_r(address_space &space, offs_t offset, uint8_t data, int ba, int cs0, int c1l, int c2l, int cs1, int c1h, int c2h) override;
	virtual void plus4_cd_w(address_space &space, offs_t offset, uint8_t data, int ba, int cs0, int c1l, int c2l, int cs1, int c1h, int c2h) override;

private:
	DECLARE_READ8_MEMBER( port_r );
	DECLARE_WRITE8_MEMBER( port_w );

	DECLARE_READ8_MEMBER( tcbm_data_r );
	DECLARE_WRITE8_MEMBER( tcbm_data_w );
	DECLARE_READ8_MEMBER( tpi0_pc_r );
	DECLARE_WRITE8_MEMBER( tpi0_pc_w );

	DECLARE_READ8_MEMBER( tpi1_pb_r );
	DECLARE_READ8_MEMBER( tpi1_pc_r );
	DECLARE_WRITE8_MEMBER( tpi1_pc_w );

	DECLARE_FLOPPY_FORMATS( floppy_formats );

	enum
	{
		LED_POWER = 0,
		LED_ACT
	};

	bool tpi1_selected(offs_t offset);

	required_device<m6510t_device> m_maincpu;
	required_device<tpi6525_device> m_tpi0;
	required_device<tpi6525_device> m_tpi1;
	required_device<c64h156_device> m_ga;
	required_device<pla_device> m_pla;
	required_device<floppy_image_device> m_floppy;
	required_device<plus4_expansion_slot_device> m_exp;
	required_ioport m_jp1;

	// TCBM bus
	uint8_t m_tcbm_data;                      // data
	int m_status;                           // status
	int m_dav;                              // data valid
	int m_ack;                              // acknowledge
	int m_dev;                              // device number

	// timers
	emu_timer *m_irq_timer;
};



// device type definition
extern const device_type C1551;
DECLARE_DEVICE_TYPE(C1551, c1551_device)

#endif // MAME_BUS_PLUS4_C1551_H
