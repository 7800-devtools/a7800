// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#ifndef MAME_MACHINE_LPC_PIT_H
#define MAME_MACHINE_LPC_PIT_H

#pragma once

#include "lpc.h"

#define MCFG_LPC_PIT_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, LPC_PIT, 0)

class lpc_pit_device : public lpc_device {
public:
	lpc_pit_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void map_device(uint64_t memory_window_start, uint64_t memory_window_end, uint64_t memory_offset, address_space *memory_space,
							uint64_t io_window_start, uint64_t io_window_end, uint64_t io_offset, address_space *io_space) override;

	DECLARE_READ8_MEMBER( status_r);
	DECLARE_WRITE8_MEMBER(access_w);
	DECLARE_WRITE8_MEMBER(control_w);

protected:
	void device_start() override;
	void device_reset() override;

private:
	DECLARE_ADDRESS_MAP(map, 32);
};

DECLARE_DEVICE_TYPE(LPC_PIT, lpc_pit_device)

#endif // MAME_MACHINE_LPC_PIT_H
