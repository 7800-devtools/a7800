// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#ifndef MAME_MACHINE_LPC_RTC_H
#define MAME_MACHINE_LPC_RTC_H

#pragma once

#include "lpc.h"

#define MCFG_LPC_RTC_ADD(_tag) \
	MCFG_DEVICE_ADD(_tag, LPC_RTC, 0)

class lpc_rtc_device : public lpc_device {
public:
	lpc_rtc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void map_device(uint64_t memory_window_start, uint64_t memory_window_end, uint64_t memory_offset, address_space *memory_space,
							uint64_t io_window_start, uint64_t io_window_end, uint64_t io_offset, address_space *io_space) override;

	virtual void map_extdevice(uint64_t memory_window_start, uint64_t memory_window_end, uint64_t memory_offset, address_space *memory_space,
									uint64_t io_window_start, uint64_t io_window_end, uint64_t io_offset, address_space *io_space);

	DECLARE_READ8_MEMBER(  index_r);
	DECLARE_WRITE8_MEMBER( index_w);
	DECLARE_READ8_MEMBER(  target_r);
	DECLARE_WRITE8_MEMBER( target_w);
	DECLARE_READ8_MEMBER(  extindex_r);
	DECLARE_WRITE8_MEMBER( extindex_w);
	DECLARE_READ8_MEMBER(  exttarget_r);
	DECLARE_WRITE8_MEMBER( exttarget_w);

protected:
	void device_start() override;
	void device_reset() override;

private:
	DECLARE_ADDRESS_MAP(map, 32);
	DECLARE_ADDRESS_MAP(extmap, 32);

	uint8_t cur_index, cur_extindex;
	uint8_t ram[256];
};

DECLARE_DEVICE_TYPE(LPC_RTC, lpc_rtc_device)

#endif // MAME_MACHINE_LPC_RTC_H
