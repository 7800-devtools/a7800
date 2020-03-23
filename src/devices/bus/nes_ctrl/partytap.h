// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/**********************************************************************

    Nintendo Family Computer Yonezawa / PartyRoom 21 Party Tap Controller

**********************************************************************/

#ifndef MAME_BUS_NES_CTRL_PARTYTAP_H
#define MAME_BUS_NES_CTRL_PARTYTAP_H

#pragma once

#include "ctrl.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> nes_partytap_device

class nes_partytap_device : public device_t,
							public device_nes_control_port_interface
{
public:
	// construction/destruction
	nes_partytap_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual ioport_constructor device_input_ports() const override;

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	virtual uint8_t read_exp(offs_t offset) override;
	virtual void write(uint8_t data) override;

	required_ioport m_inputs;
	uint8_t m_mode;
	uint32_t m_latch;
};

// device type definition
DECLARE_DEVICE_TYPE(NES_PARTYTAP, nes_partytap_device)

#endif // MAME_BUS_NES_CTRL_PARTYTAP_H
