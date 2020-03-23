// license:BSD-3-Clause
// copyright-holders:Angelo Salese
/* Sega 315-5641 / D77591 / 9442CA010 */

// this is the PICO sound chip, we are not sure if it's the same as a 7759 or not, it requires FIFO logic
// which the 7759 does _not_ have but it is possible that is handled somewhere else on the PICO hardware.
#ifndef MAME_SOUND_315_5641_H
#define MAME_SOUND_315_5641_H

#pragma once


#include "upd7759.h"


class sega_315_5641_pcm_device : public upd7759_device
{
public:
	sega_315_5641_pcm_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE8_MEMBER(port_w) override;

	uint8_t get_fifo_space();

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	void advance_state() override;

	uint8_t       m_fifo_data[0x40];
	uint8_t       m_fifo_read;    // last read offset (will read in m_fifo_read+1)
	uint8_t       m_fifo_write;   // write offset
};

DECLARE_DEVICE_TYPE(SEGA_315_5641_PCM, sega_315_5641_pcm_device)

#endif // MAME_SOUND_315_5641_H
