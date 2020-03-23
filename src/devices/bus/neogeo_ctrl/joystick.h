// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/**********************************************************************

    SNK Neo Geo Joystick emulation

**********************************************************************/

#ifndef MAME_BUS_NEOGEO_CTRL_JOYSTICK_H
#define MAME_BUS_NEOGEO_CTRL_JOYSTICK_H

#pragma once


#include "ctrl.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> neogeo_joystick_device

class neogeo_joystick_device : public device_t, public device_neogeo_control_port_interface
{
public:
	// construction/destruction
	neogeo_joystick_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual ioport_constructor device_input_ports() const override;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_neogeo_control_port_interface overrides
	virtual uint8_t read_ctrl() override;
	virtual uint8_t read_start_sel() override;

private:
	required_ioport m_joy;
	required_ioport m_ss;
};


// ======================> neogeo_joy_ac_device

class neogeo_joy_ac_device : public device_t, public device_neogeo_ctrl_edge_interface
{
public:
	// construction/destruction
	neogeo_joy_ac_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual ioport_constructor device_input_ports() const override;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_neogeo_ctrl_edge_interface overrides
	virtual DECLARE_READ8_MEMBER( in0_r ) override;
	virtual DECLARE_READ8_MEMBER( in1_r ) override;

private:
	required_ioport m_joy1;
	required_ioport m_joy2;
};


// device type definition
DECLARE_DEVICE_TYPE(NEOGEO_JOY,    neogeo_joystick_device)
DECLARE_DEVICE_TYPE(NEOGEO_JOY_AC, neogeo_joy_ac_device)


#endif // MAME_BUS_NEOGEO_CTRL_JOYSTICK_H
