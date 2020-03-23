// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    bsmt2000.h

    BSMT2000 device emulator.

***************************************************************************/

#ifndef MAME_SOUND_BSMT2000_H
#define MAME_SOUND_BSMT2000_H

#pragma once

#include "cpu/tms32010/tms32010.h"


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_BSMT2000_ADD(_tag, _clock) \
	MCFG_DEVICE_ADD(_tag, BSMT2000, _clock)
#define MCFG_BSMT2000_REPLACE(_tag, _clock) \
	MCFG_DEVICE_REPLACE(_tag, BSMT2000, _clock)
#define MCFG_BSMT2000_READY_CALLBACK(_class, _method) \
	bsmt2000_device::static_set_ready_callback(*device, bsmt2000_device::ready_callback(&_class::_method, #_class "::" #_method, nullptr, (_class *)nullptr));


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************


// ======================> bsmt2000_device

class bsmt2000_device : public device_t,
						public device_sound_interface,
						public device_rom_interface
{
public:
	typedef device_delegate<void ()> ready_callback;

	// construction/destruction
	bsmt2000_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// inline configuration helpers
	static void static_set_ready_callback(device_t &device, ready_callback &&callback);

	// public interface
	uint16_t read_status();
	void write_reg(uint16_t data);
	void write_data(uint16_t data);

protected:
	// device-level overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	// device_sound_interface overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

	// device_rom_interface overrides
	virtual void rom_bank_updated() override;

public:
	// internal TMS I/O callbacks
	DECLARE_READ16_MEMBER( tms_register_r );
	DECLARE_READ16_MEMBER( tms_data_r );
	DECLARE_READ16_MEMBER( tms_rom_r );
	DECLARE_WRITE16_MEMBER( tms_rom_addr_w );
	DECLARE_WRITE16_MEMBER( tms_rom_bank_w );
	DECLARE_WRITE16_MEMBER( tms_left_w );
	DECLARE_WRITE16_MEMBER( tms_right_w );

private:
	// timers
	enum
	{
		TIMER_ID_RESET,
		TIMER_ID_REG_WRITE,
		TIMER_ID_DATA_WRITE
	};

	// configuration state
	ready_callback              m_ready_callback;

	// internal state
	sound_stream *              m_stream;
	required_device<tms32015_device> m_cpu;
	uint16_t                      m_register_select;
	uint16_t                      m_write_data;
	uint16_t                      m_rom_address;
	uint16_t                      m_rom_bank;
	int16_t                       m_left_data;
	int16_t                       m_right_data;
	bool                        m_write_pending;

	DECLARE_READ_LINE_MEMBER( tms_write_pending_r );
};


// device type definition
DECLARE_DEVICE_TYPE(BSMT2000, bsmt2000_device)

#endif // MAME_SOUND_BSMT2000_H
