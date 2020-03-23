// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/***************************************************************************

    Microsoft Natural Keyboard

***************************************************************************/

#ifndef MAME_BUS_PC_KB_MSNAT_H
#define MAME_BUS_PC_KB_MSNAT_H

#pragma once

#include "pc_kbdc.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class pc_kbd_microsoft_natural_device : public device_t,
										public device_pc_kbd_interface
{
public:
	// construction/destruction
	pc_kbd_microsoft_natural_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE_LINE_MEMBER(clock_write) override;
	virtual DECLARE_WRITE_LINE_MEMBER(data_write) override;

	DECLARE_READ8_MEMBER(p0_read);
	DECLARE_WRITE8_MEMBER(p0_write);
	DECLARE_WRITE8_MEMBER(p1_write);
	DECLARE_WRITE8_MEMBER(p2_write);
	DECLARE_READ8_MEMBER(p3_read);
	DECLARE_WRITE8_MEMBER(p3_write);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;
	virtual const tiny_rom_entry *device_rom_region() const override;

	required_device<cpu_device> m_cpu;

	required_ioport_array<8> m_p2_r;
	required_ioport_array<8> m_p1_r;

	uint8_t   m_p0;
	uint8_t   m_p1;
	uint8_t   m_p2;
	uint8_t   m_p3;
};


// device type definition
extern const device_type PC_KBD_MICROSOFT_NATURAL;

#endif // MAME_BUS_PC_KB_MSNAT_H
