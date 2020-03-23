// license:BSD-3-Clause
// copyright-holders:Angelo Salese
/* Sega 315-5641 / D77591 / 9442CA010 */

#include "emu.h"
#include "315-5641.h"

DEFINE_DEVICE_TYPE(SEGA_315_5641_PCM, sega_315_5641_pcm_device, "315_5641_pcm", "Sega 315-5641 PCM")

sega_315_5641_pcm_device::sega_315_5641_pcm_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: upd7759_device(mconfig, SEGA_315_5641_PCM, tag, owner, clock), m_fifo_read(0), m_fifo_write(0)
{
}

void sega_315_5641_pcm_device::device_start()
{
	save_item(NAME(m_fifo_data), 0x40);
	save_item(NAME(m_fifo_read));
	save_item(NAME(m_fifo_write));

	upd7759_device::device_start();
}

void sega_315_5641_pcm_device::advance_state()
{
	switch (m_state)
	{
		case STATE_DROP_DRQ:
			if (m_rombase == nullptr)
			{
				// Slave Mode: get data from FIFO buffer
				uint8_t fiforead = (m_fifo_read + 1) & 0x3F;
				if (fiforead != m_fifo_write)
				{
					m_fifo_in = m_fifo_data[fiforead];
					m_fifo_read = fiforead;
				}
			}
			break;
	}

	upd775x_device::advance_state();
}


WRITE8_MEMBER( sega_315_5641_pcm_device::port_w )
{
	if (m_rombase != nullptr)
	{
		/* update the FIFO value */
		m_fifo_in = data;
	}
	else
	{
		m_fifo_data[m_fifo_write++] = data;
		m_fifo_write &= 0x3F;
	}
}


uint8_t sega_315_5641_pcm_device::get_fifo_space()
{
	return (m_fifo_read - m_fifo_write) & 0x3F;
}

void sega_315_5641_pcm_device::device_reset()
{
	m_fifo_read          = 0x3F;
	m_fifo_write         = 0x00;

	upd775x_device::device_reset();
}
