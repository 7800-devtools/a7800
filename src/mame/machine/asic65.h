// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*************************************
 *
 *  Implementation of ASIC65
 *
 *************************************/
#ifndef MAME_MACHINE_ASIC65_H
#define MAME_MACHINE_ASIC65_H

#pragma once

#include "cpu/tms32010/tms32010.h"

enum {
	ASIC65_STANDARD,
	ASIC65_STEELTAL,
	ASIC65_GUARDIANS,
	ASIC65_ROMBASED
};

class asic65_device : public device_t
{
public:
	asic65_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// (static) configuration helpers
	static void set_type(device_t &device, int type) { downcast<asic65_device &>(device).m_asic65_type = type; }

	void reset_line(int state);
	DECLARE_WRITE16_MEMBER( data_w );
	DECLARE_READ16_MEMBER( read );
	DECLARE_READ16_MEMBER( io_r );

	DECLARE_WRITE16_MEMBER( m68k_w );
	DECLARE_READ16_MEMBER( m68k_r );
	DECLARE_WRITE16_MEMBER( stat_w );
	DECLARE_READ16_MEMBER( stat_r );

	enum
	{
		TIMER_M68K_ASIC65_DEFERRED_W
	};

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	uint8_t   m_asic65_type;
	int     m_command;
	uint16_t  m_param[32];
	uint16_t  m_yorigin;
	uint8_t   m_param_index;
	uint8_t   m_result_index;
	uint8_t   m_reset_state;
	uint8_t   m_last_bank;

	/* ROM-based interface states */
	required_device<cpu_device> m_ourcpu;
	uint8_t   m_tfull;
	uint8_t   m_68full;
	uint8_t   m_cmd;
	uint8_t   m_xflg;
	uint16_t  m_68data;
	uint16_t  m_tdata;

	FILE * m_log;

	DECLARE_READ_LINE_MEMBER( get_bio );
};

DECLARE_DEVICE_TYPE(ASIC65, asic65_device)

#define MCFG_ASIC65_ADD(_tag, _type) \
	MCFG_DEVICE_ADD(_tag, ASIC65, 0) \
	asic65_device::set_type(*device, _type);

#endif // MAME_MACHINE_ASIC65_H
