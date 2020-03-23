// license:BSD-3-Clause
// copyright-holders:Ernesto Corvi
/******************************************************************************
* FILE
*   Yamaha 3812 emulator interface - MAME VERSION
*
* CREATED BY
*   Ernesto Corvi
*
* UPDATE LOG
*   JB  28-04-2002  Fixed simultaneous usage of all three different chip types.
*                       Used real sample rate when resample filter is active.
*       AAT 12-28-2001  Protected Y8950 from accessing unmapped port and keyboard handlers.
*   CHS 1999-01-09  Fixes new ym3812 emulation interface.
*   CHS 1998-10-23  Mame streaming sound chip update
*   EC  1998        Created Interface
*
* NOTES
*
******************************************************************************/
#include "emu.h"
#include "8950intf.h"
#include "fmopl.h"


void y8950_device::irq_handler(int irq)
{
	m_irq_handler(irq);
}

void y8950_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch(id)
	{
	case 0:
		y8950_timer_over(m_chip,0);
		break;

	case 1:
		y8950_timer_over(m_chip,1);
		break;
	}
}

void y8950_device::timer_handler(int c, const attotime &period)
{
	if( period == attotime::zero )
	{   /* Reset FM Timer */
		m_timer[c]->enable(false);
	}
	else
	{   /* Start FM Timer */
		m_timer[c]->adjust(period);
	}
}


//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void y8950_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	y8950_update_one(m_chip, outputs[0], samples);
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void y8950_device::device_start()
{
	int rate = clock()/72;

	m_irq_handler.resolve_safe();
	m_keyboard_read_handler.resolve_safe(0);
	m_keyboard_write_handler.resolve_safe();
	m_io_read_handler.resolve_safe(0);
	m_io_write_handler.resolve_safe();

	/* stream system initialize */
	m_chip = y8950_init(this,clock(),rate);
	assert_always(m_chip != nullptr, "Error creating Y8950 chip");

	/* ADPCM ROM data */
	y8950_set_delta_t_memory(m_chip, m_region->base(), m_region->bytes());

	m_stream = machine().sound().stream_alloc(*this,0,1,rate);
	/* port and keyboard handler */
	y8950_set_port_handler(m_chip, &y8950_device::static_port_handler_w, &y8950_device::static_port_handler_r, this);
	y8950_set_keyboard_handler(m_chip, &y8950_device::static_keyboard_handler_w, &y8950_device::static_keyboard_handler_r, this);

	/* Y8950 setup */
	y8950_set_timer_handler (m_chip, &y8950_device::static_timer_handler, this);
	y8950_set_irq_handler   (m_chip, &y8950_device::static_irq_handler, this);
	y8950_set_update_handler(m_chip, &y8950_device::static_update_request, this);

	m_timer[0] = timer_alloc(0);
	m_timer[1] = timer_alloc(1);
}

//-------------------------------------------------
//  device_stop - device-specific stop
//-------------------------------------------------

void y8950_device::device_stop()
{
	y8950_shutdown(m_chip);
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void y8950_device::device_reset()
{
	y8950_reset_chip(m_chip);
}


READ8_MEMBER( y8950_device::read )
{
	return y8950_read(m_chip, offset & 1);
}

WRITE8_MEMBER( y8950_device::write )
{
	y8950_write(m_chip, offset & 1, data);
}

READ8_MEMBER( y8950_device::status_port_r ) { return read(space, 0); }
READ8_MEMBER( y8950_device::read_port_r ) { return read(space, 1); }
WRITE8_MEMBER( y8950_device::control_port_w ) { write(space, 0, data); }
WRITE8_MEMBER( y8950_device::write_port_w ) { write(space, 1, data); }


DEFINE_DEVICE_TYPE(Y8950, y8950_device, "y8950", "Y8950 MSX-Audio")

y8950_device::y8950_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, Y8950, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_stream(nullptr)
	, m_timer{ nullptr, nullptr }
	, m_chip(nullptr)
	, m_irq_handler(*this)
	, m_keyboard_read_handler(*this)
	, m_keyboard_write_handler(*this)
	, m_io_read_handler(*this)
	, m_io_write_handler(*this)
	, m_region(*this, DEVICE_SELF)
{
}
