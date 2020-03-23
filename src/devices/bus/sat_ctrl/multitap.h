// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/**********************************************************************

    Sega Saturn multitap emulation

**********************************************************************/

#ifndef MAME_BUS_SAT_CTRL_MULTITAP_H
#define MAME_BUS_SAT_CTRL_MULTITAP_H

#pragma once


#include "ctrl.h"



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> saturn_multitap_device

class saturn_multitap_device : public device_t,
							public device_saturn_control_port_interface
{
public:
	// construction/destruction
	saturn_multitap_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;

	// device_saturn_control_port_interface overrides
	virtual uint8_t read_ctrl(uint8_t offset) override;
	virtual uint8_t read_status() override { return 0x16; }
	virtual uint8_t read_id(int idx) override;

private:
	required_device<saturn_control_port_device> m_subctrl1_port;
	required_device<saturn_control_port_device> m_subctrl2_port;
	required_device<saturn_control_port_device> m_subctrl3_port;
	required_device<saturn_control_port_device> m_subctrl4_port;
	required_device<saturn_control_port_device> m_subctrl5_port;
	required_device<saturn_control_port_device> m_subctrl6_port;
};


// device type definition
DECLARE_DEVICE_TYPE(SATURN_MULTITAP, saturn_multitap_device)

#endif // MAME_BUS_SAT_CTRL_MULTITAP_H
