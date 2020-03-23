// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Victor 9000 keyboard emulation

*********************************************************************/

#ifndef MAME_MACHINE_VICTOR9K_KB_H
#define MAME_MACHINE_VICTOR9K_KB_H


#pragma once

#include "cpu/mcs48/mcs48.h"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_VICTOR9K_KBRDY_HANDLER(_devcb) \
	devcb = &victor_9000_keyboard_device::set_kbrdy_cb(*device, DEVCB_##_devcb);

#define MCFG_VICTOR9K_KBDATA_HANDLER(_devcb) \
	devcb = &victor_9000_keyboard_device::set_kbdata_cb(*device, DEVCB_##_devcb);


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> victor_9000_keyboard_device

class victor_9000_keyboard_device :  public device_t
{
public:
	// construction/destruction
	victor_9000_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template<class _Object> static devcb_base &set_kbrdy_cb(device_t &device, _Object object) { return downcast<victor_9000_keyboard_device &>(device).m_kbrdy_cb.set_callback(object); }
	template<class _Object> static devcb_base &set_kbdata_cb(device_t &device, _Object object) { return downcast<victor_9000_keyboard_device &>(device).m_kbdata_cb.set_callback(object); }

	DECLARE_WRITE_LINE_MEMBER( kback_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	// optional information overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

private:
	required_device<cpu_device> m_maincpu;
	required_ioport_array<13> m_y;

	devcb_write_line   m_kbrdy_cb;
	devcb_write_line   m_kbdata_cb;

	uint8_t m_p1;
	uint8_t m_keylatch;
	int m_stb;
	int m_y12;
	int m_kbrdy;
	int m_kbdata;
	int m_kback;

	DECLARE_READ8_MEMBER( kb_p1_r );
	DECLARE_WRITE8_MEMBER( kb_p1_w );
	DECLARE_WRITE8_MEMBER( kb_p2_w );
	DECLARE_READ_LINE_MEMBER( kb_t1_r );
};


// device type definition
DECLARE_DEVICE_TYPE(VICTOR9K_KEYBOARD, victor_9000_keyboard_device)



#endif // MAME_MACHINE_VICTOR9K_KB_H
