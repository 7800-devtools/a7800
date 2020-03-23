// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
#ifndef MAME_BUS_DMV_K803_H
#define MAME_BUS_DMV_K803_H

#pragma once

#include "dmvbus.h"
#include "machine/mm58167.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> dmv_k803_device

class dmv_k803_device :
		public device_t,
		public device_dmvslot_interface
{
public:
	// construction/destruction
	dmv_k803_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// optional information overrides
	virtual ioport_constructor device_input_ports() const override;
	virtual void device_add_mconfig(machine_config &config) override;

	virtual void io_read(address_space &space, int ifsel, offs_t offset, uint8_t &data) override;
	virtual void io_write(address_space &space, int ifsel, offs_t offset, uint8_t data) override;

	void update_int();

private:
	DECLARE_WRITE_LINE_MEMBER(rtc_irq_w);

	required_device<mm58167_device> m_rtc;
	required_ioport m_dsw;
	dmvcart_slot_device * m_bus;
	uint8_t   m_latch;
	int     m_rtc_int;
};


// device type definition
DECLARE_DEVICE_TYPE(DMV_K803, dmv_k803_device)

#endif // MAME_BUS_DMV_K803_H
