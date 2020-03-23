// license: BSD-3-Clause
// copyright-holders: Aaron Giles
/***************************************************************************

    Cheap Squeak Deluxe / Artificial Artist Sound Board

***************************************************************************/
#ifndef MAME_AUDIO_CSD_H
#define MAME_AUDIO_CSD_H

#pragma once


#include "cpu/m68000/m68000.h"
#include "machine/6821pia.h"
#include "sound/dac.h"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> midway_cheap_squeak_deluxe_device

class midway_cheap_squeak_deluxe_device : public device_t, public device_mixer_interface
{
public:
	// construction/destruction
	midway_cheap_squeak_deluxe_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// read/write
	DECLARE_READ8_MEMBER(stat_r);
	DECLARE_WRITE8_MEMBER(sr_w);
	DECLARE_WRITE_LINE_MEMBER(sirq_w);
	DECLARE_WRITE_LINE_MEMBER(reset_w);

protected:
	// device-level overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	// devices
	required_device<m68000_device> m_cpu;
	required_device<pia6821_device> m_pia;
	required_device<dac_word_interface> m_dac;

	// internal state
	uint8_t m_status;
	uint16_t m_dacval;

	// internal communications
	DECLARE_WRITE8_MEMBER(porta_w);
	DECLARE_WRITE8_MEMBER(portb_w);
	DECLARE_WRITE_LINE_MEMBER(irq_w);
};

// device type definition
DECLARE_DEVICE_TYPE(MIDWAY_CHEAP_SQUEAK_DELUXE, midway_cheap_squeak_deluxe_device)

#endif // MAME_AUDIO_CSD_H
