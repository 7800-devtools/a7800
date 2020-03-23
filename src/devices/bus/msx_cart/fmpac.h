// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
#ifndef MAME_BUS_MSX_CART_FMPAC_H
#define MAME_BUS_MSX_CART_FMPAC_H

#pragma once

#include "bus/msx_cart/cartridge.h"
#include "sound/ym2413.h"


DECLARE_DEVICE_TYPE(MSX_CART_FMPAC, msx_cart_fmpac_device)


class msx_cart_fmpac_device : public device_t, public msx_cart_interface
{
public:
	msx_cart_fmpac_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void initialize_cartridge() override;

	virtual DECLARE_READ8_MEMBER(read_cart) override;
	virtual DECLARE_WRITE8_MEMBER(write_cart) override;

	DECLARE_WRITE8_MEMBER(write_ym2413);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

	// device-level overrides
	virtual void device_add_mconfig(machine_config &config) override;

	void restore_banks();

private:
	required_device<ym2413_device> m_ym2413;

	uint8_t m_selected_bank;
	uint8_t *m_bank_base;
	bool m_sram_active;
	bool m_opll_active;
	uint8_t m_1ffe;
	uint8_t m_1fff;
	uint8_t m_7ff6;
};


#endif // MAME_BUS_MSX_CART_FMPAC_H
