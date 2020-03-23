// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*********************************************************************

    corvfdc02.h

    Implemention of the Corvus Systems CORVUS02 floppy controller

*********************************************************************/

#ifndef MAME_BUS_A2BUS_CORVFDC02_H
#define MAME_BUS_A2BUS_CORVFDC02_H

#pragma once

#include "a2bus.h"
#include "machine/upd765.h"
#include "formats/imd_dsk.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class a2bus_corvfdc02_device:
	public device_t,
	public device_a2bus_card_interface
{
public:
	// construction/destruction
	a2bus_corvfdc02_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	a2bus_corvfdc02_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

	// overrides of standard a2bus slot functions
	virtual uint8_t read_c0nx(address_space &space, uint8_t offset) override;
	virtual void write_c0nx(address_space &space, uint8_t offset, uint8_t data) override;
	virtual uint8_t read_cnxx(address_space &space, uint8_t offset) override;

	required_device<upd765a_device> m_fdc;
	required_device<floppy_connector> m_con1;
	required_device<floppy_connector> m_con2;
	required_device<floppy_connector> m_con3;
	required_device<floppy_connector> m_con4;

private:
	DECLARE_WRITE_LINE_MEMBER(intrq_w);
	DECLARE_WRITE_LINE_MEMBER(drq_w);

	DECLARE_FLOPPY_FORMATS(corv_floppy_formats);

	uint8_t *m_rom;
	uint8_t m_fdc_local_status, m_fdc_local_command;
	uint16_t m_bufptr;
	uint8_t m_buffer[2048];   // 1x6116 SRAM
	floppy_image_device *m_curfloppy;
	bool m_in_drq;
	emu_timer *m_timer;
};

// device type definition
DECLARE_DEVICE_TYPE(A2BUS_CORVFDC02, a2bus_corvfdc02_device)

#endif // MAME_BUS_A2BUS_CORVFDC02_H
