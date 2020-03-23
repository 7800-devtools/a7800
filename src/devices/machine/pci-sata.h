// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#ifndef MAME_MACHINE_PCI_SATA_H
#define MAME_MACHINE_PCI_SATA_H

#pragma once

#include "pci.h"

#define MCFG_SATA_ADD(_tag, _main_id, _revision, _subdevice_id) \
	MCFG_PCI_DEVICE_ADD(_tag, SATA, _main_id, _revision, 0x01018a, _subdevice_id)

class sata_device : public pci_device {
public:
	sata_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	DECLARE_ADDRESS_MAP(primary_command_map, 32);
	DECLARE_ADDRESS_MAP(primary_control_map, 32);
	DECLARE_ADDRESS_MAP(secondary_command_map, 32);
	DECLARE_ADDRESS_MAP(secondary_control_map, 32);
	DECLARE_ADDRESS_MAP(bus_master_map, 32);
	DECLARE_ADDRESS_MAP(ide_command_posting_map, 32);
};

DECLARE_DEVICE_TYPE(SATA, sata_device)

#endif // MAME_MACHINE_PCI_SATA_H
