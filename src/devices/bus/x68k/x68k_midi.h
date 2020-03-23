// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*
 * x68k_midi.h
 *
 * X68000 MIDI expansion card
 *
 */

#ifndef MAME_BUS_X68K_X68K_MIDI_H
#define MAME_BUS_X68K_X68K_MIDI_H

#pragma once

#include "machine/ym3802.h"
#include "x68kexp.h"

#define MIDI_IRQ_VECTOR 0x80  // does not seem to use the YM3802's vector registers

class x68k_midi_device : public device_t,
						 public device_x68k_expansion_card_interface
{
public:
	// construction/destruction
	x68k_midi_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// optional information overrides
	virtual void device_add_mconfig(machine_config &config) override;

	DECLARE_READ8_MEMBER(x68k_midi_reg_r);
	DECLARE_WRITE8_MEMBER(x68k_midi_reg_w);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	x68k_expansion_slot_device *m_slot;
	required_device<ym3802_device> m_midi;
	void irq_w(int state);
};


// device type definition
DECLARE_DEVICE_TYPE(X68K_MIDI, x68k_midi_device)

#endif // MAME_BUS_X68K_X68K_MIDI_H
