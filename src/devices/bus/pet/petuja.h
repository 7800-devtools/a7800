// license:BSD-3-Clause
// copyright-holders:smf
/**********************************************************************

    Commodore PET userport joystick adapter emulation

**********************************************************************/

#ifndef MAME_BUS_PET_PETUJA_H
#define MAME_BUS_PET_PETUJA_H

#pragma once


#include "user.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> pet_userport_joystick_adapter_device

class pet_userport_joystick_adapter_device : public device_t,
	public device_pet_user_port_interface
{
public:
	// construction/destruction
	pet_userport_joystick_adapter_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual ioport_constructor device_input_ports() const override;

	// device_pet_user_port_interface overrides
	WRITE_LINE_MEMBER( write_up1 ) { m_up1 = state; update_port1(); }
	WRITE_LINE_MEMBER( write_down1 ) { m_down1 = state; update_port1(); }
	WRITE_LINE_MEMBER( write_fire1 ) { m_fire1 = state; update_port1(); }
	WRITE_LINE_MEMBER( write_up2 ) { m_up2 = state; update_port2(); }
	WRITE_LINE_MEMBER( write_down2 ) { m_down2 = state; update_port2(); }
	WRITE_LINE_MEMBER( write_fire2 ) { m_fire2 = state; update_port2(); }

protected:
	// device-level overrides
	virtual void device_start() override;

	void update_port1();
	void update_port2();
	int m_up1;
	int m_down1;
	int m_fire1;
	int m_up2;
	int m_down2;
	int m_fire2;
};


// device type definition
DECLARE_DEVICE_TYPE(PET_USERPORT_JOYSTICK_ADAPTER, pet_userport_joystick_adapter_device)


#endif // MAME_BUS_PET_PETUJA_H
