// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/**********************************************************************

    Nintendo Family Computer - Epoch Barcode Battler

**********************************************************************/

#ifndef MAME_BUS_NES_CTRL_BCBATTLE_H
#define MAME_BUS_NES_CTRL_BCBATTLE_H

#pragma once

#include "ctrl.h"
#include "machine/bcreader.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> nes_bcbattle_device

class nes_bcbattle_device : public device_t,
							public device_nes_control_port_interface
{
public:
	// construction/destruction
	nes_bcbattle_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	static constexpr device_timer_id TIMER_BATTLER = 1;

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	virtual void device_add_mconfig(machine_config &config) override;

	virtual uint8_t read_exp(offs_t offset) override;
	int read_current_bit();

	required_device<barcode_reader_device> m_reader;
	uint8_t m_current_barcode[20];
	int m_pending_code, m_new_code, m_transmitting, m_cur_bit, m_cur_byte;
	emu_timer *battler_timer;
};

// device type definition
DECLARE_DEVICE_TYPE(NES_BARCODE_BATTLER, nes_bcbattle_device)

#endif // MAME_BUS_NES_CTRL_BCBATTLE_H
