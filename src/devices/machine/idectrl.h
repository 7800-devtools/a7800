// license:BSD-3-Clause
// copyright-holders:smf
/***************************************************************************

    idectrl.h

    Generic (PC-style) IDE controller implementation.

***************************************************************************/

#ifndef MAME_MACHINE_IDECTRL_H
#define MAME_MACHINE_IDECTRL_H

#pragma once

#include "ataintf.h"

/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

#define MCFG_IDE_CONTROLLER_ADD(_tag, _slot_intf, _master, _slave, _fixed) \
		MCFG_DEVICE_ADD(_tag, IDE_CONTROLLER, 0) \
		MCFG_DEVICE_MODIFY(_tag ":0") \
		MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _master, _fixed) \
		MCFG_DEVICE_MODIFY(_tag ":1") \
		MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _slave, _fixed) \
		MCFG_DEVICE_MODIFY(_tag)

class ide_controller_device : public ata_interface_device
{
public:
	ide_controller_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ16_MEMBER(read_cs0) override;
	virtual DECLARE_READ16_MEMBER(read_cs1) override;
	virtual DECLARE_WRITE16_MEMBER(write_cs0) override;
	virtual DECLARE_WRITE16_MEMBER(write_cs1) override;

protected:
	ide_controller_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);
};

DECLARE_DEVICE_TYPE(IDE_CONTROLLER, ide_controller_device)


#define MCFG_IDE_CONTROLLER_32_ADD(_tag, _slot_intf, _master, _slave, _fixed) \
		MCFG_DEVICE_ADD(_tag, IDE_CONTROLLER_32, 0) \
		MCFG_DEVICE_MODIFY(_tag ":0") \
		MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _master, _fixed) \
		MCFG_DEVICE_MODIFY(_tag ":1") \
		MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _slave, _fixed) \
		MCFG_DEVICE_MODIFY(_tag)

class ide_controller_32_device : public ide_controller_device
{
public:
	ide_controller_32_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_READ32_MEMBER(read_cs0);
	virtual DECLARE_READ32_MEMBER(read_cs1);
	virtual DECLARE_WRITE32_MEMBER(write_cs0);
	virtual DECLARE_WRITE32_MEMBER(write_cs1);

protected:
	ide_controller_32_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

private:
	using ide_controller_device::read_cs0;
	using ide_controller_device::read_cs1;
	using ide_controller_device::write_cs0;
	using ide_controller_device::write_cs1;
};

DECLARE_DEVICE_TYPE(IDE_CONTROLLER_32, ide_controller_32_device)


#define MCFG_BUS_MASTER_IDE_CONTROLLER_ADD(_tag, _slot_intf, _master, _slave, _fixed) \
		MCFG_DEVICE_ADD(_tag, BUS_MASTER_IDE_CONTROLLER, 0) \
		MCFG_DEVICE_MODIFY(_tag ":0") \
		MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _master, _fixed) \
		MCFG_DEVICE_MODIFY(_tag ":1") \
		MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _slave, _fixed) \
		MCFG_DEVICE_MODIFY(_tag)

#define MCFG_BUS_MASTER_IDE_CONTROLLER_SPACE(bmcpu, bmspace) \
		bus_master_ide_controller_device::set_bus_master_space(*device, bmcpu, bmspace);

class bus_master_ide_controller_device : public ide_controller_32_device
{
public:
	bus_master_ide_controller_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	static void set_bus_master_space(device_t &device, const char *bmcpu, uint32_t bmspace) { bus_master_ide_controller_device &ide = downcast<bus_master_ide_controller_device &>(device); ide.m_bmcpu = bmcpu; ide.m_bmspace = bmspace; }

	DECLARE_READ32_MEMBER( bmdma_r );
	DECLARE_WRITE32_MEMBER( bmdma_w );

protected:
	virtual void device_start() override;

	virtual void set_irq(int state) override;
	virtual void set_dmarq(int state) override;

private:
	void execute_dma();

	const char *m_bmcpu;
	uint32_t m_bmspace;
	address_space * m_dma_space;
	uint8_t m_dma_address_xor;

	offs_t m_dma_address;
	uint32_t m_dma_bytes_left;
	offs_t m_dma_descriptor;
	uint8_t m_dma_last_buffer;
	uint8_t m_bus_master_command;
	uint8_t m_bus_master_status;
	uint32_t m_bus_master_descriptor;
	int m_irq;
	int m_dmarq;
};

DECLARE_DEVICE_TYPE(BUS_MASTER_IDE_CONTROLLER, bus_master_ide_controller_device)

#endif // MAME_MACHINE_IDECTRL_H
