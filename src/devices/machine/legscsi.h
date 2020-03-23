// license:BSD-3-Clause
// copyright-holders:smf
#ifndef MAME_MACHINE_LEGSCSI_H
#define MAME_MACHINE_LEGSCSI_H

#pragma once

#include "bus/scsi/scsihle.h"

#define MCFG_LEGACY_SCSI_PORT(_tag) \
	legacy_scsi_host_adapter::set_scsi_port(*device, "^" _tag);

class legacy_scsi_host_adapter : public device_t
{
public:
	static void set_scsi_port(device_t &device, const char *tag) { downcast<legacy_scsi_host_adapter &>(device).m_scsi_port.set_tag(tag); }

protected:
	legacy_scsi_host_adapter(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;

	void reset_bus();
	bool select(int id);
	void send_command(uint8_t *data, int bytes);
	int get_length();
	int get_phase();
	void read_data(uint8_t *data, int bytes);
	void write_data(uint8_t *data, int bytes);
	uint8_t get_status();

private:
	int m_selected;
	scsihle_device *get_device(int id);

	required_device<scsi_port_device> m_scsi_port;
};

#endif // MAME_MACHINE_LEGSCSI_H
