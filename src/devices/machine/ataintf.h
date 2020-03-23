// license:BSD-3-Clause
// copyright-holders:smf
/***************************************************************************

    ataintf.h

    ATA Interface implementation.

***************************************************************************/

#ifndef MAME_MACHINE_ATAINTF_H
#define MAME_MACHINE_ATAINTF_H

#pragma once

#include "atadev.h"

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

// ======================> ata_slot_device

class ata_slot_device : public device_t,
						public device_slot_interface
{
public:
	// construction/destruction
	ata_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	device_ata_interface *dev() { return m_dev; }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_config_complete() override;

private:
	device_ata_interface *m_dev;
};

// device type definition
DECLARE_DEVICE_TYPE(ATA_SLOT, ata_slot_device)

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

#define MCFG_ATA_INTERFACE_IRQ_HANDLER(_devcb) \
	devcb = &ata_interface_device::set_irq_handler(*device, DEVCB_##_devcb);

#define MCFG_ATA_INTERFACE_DMARQ_HANDLER(_devcb) \
	devcb = &ata_interface_device::set_dmarq_handler(*device, DEVCB_##_devcb);

#define MCFG_ATA_INTERFACE_DASP_HANDLER(_devcb) \
	devcb = &ata_interface_device::set_dasp_handler(*device, DEVCB_##_devcb);

SLOT_INTERFACE_EXTERN(ata_devices);

/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

#define MCFG_ATA_INTERFACE_ADD(_tag, _slot_intf, _master, _slave, _fixed) \
	MCFG_DEVICE_ADD(_tag, ATA_INTERFACE, 0) \
	MCFG_DEVICE_MODIFY(_tag ":0") \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _master, _fixed) \
	MCFG_DEVICE_MODIFY(_tag ":1") \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _slave, _fixed) \
	MCFG_DEVICE_MODIFY(_tag)

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

/* ----- device interface ----- */

class ata_interface_device : public device_t
{
public:
	ata_interface_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// static configuration helpers
	template <class Object> static devcb_base &set_irq_handler(device_t &device, Object &&cb) { return downcast<ata_interface_device &>(device).m_irq_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_dmarq_handler(device_t &device, Object &&cb) { return downcast<ata_interface_device &>(device).m_dmarq_handler.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_dasp_handler(device_t &device, Object &&cb) { return downcast<ata_interface_device &>(device).m_dasp_handler.set_callback(std::forward<Object>(cb)); }

	uint16_t read_dma();
	virtual DECLARE_READ16_MEMBER(read_cs0);
	virtual DECLARE_READ16_MEMBER(read_cs1);

	void write_dma(uint16_t data);
	virtual DECLARE_WRITE16_MEMBER(write_cs0);
	virtual DECLARE_WRITE16_MEMBER(write_cs1);
	DECLARE_WRITE_LINE_MEMBER(write_dmack);

protected:
	ata_interface_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_add_mconfig(machine_config &config) override;

	virtual void set_irq(int state);
	virtual void set_dmarq(int state);
	virtual void set_dasp(int state);

private:
	DECLARE_WRITE_LINE_MEMBER(irq0_write_line);
	DECLARE_WRITE_LINE_MEMBER(dmarq0_write_line);
	DECLARE_WRITE_LINE_MEMBER(dasp0_write_line);
	DECLARE_WRITE_LINE_MEMBER(pdiag0_write_line);

	DECLARE_WRITE_LINE_MEMBER(irq1_write_line);
	DECLARE_WRITE_LINE_MEMBER(dmarq1_write_line);
	DECLARE_WRITE_LINE_MEMBER(dasp1_write_line);
	DECLARE_WRITE_LINE_MEMBER(pdiag1_write_line);

	ata_slot_device *m_slot[2];
	int m_irq[2];
	int m_dmarq[2];
	int m_dasp[2];
	int m_pdiag[2];

	devcb_write_line m_irq_handler;
	devcb_write_line m_dmarq_handler;
	devcb_write_line m_dasp_handler;
};

DECLARE_DEVICE_TYPE(ATA_INTERFACE, ata_interface_device)

#endif // MAME_MACHINE_ATAINTF_H
