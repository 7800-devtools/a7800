// license:BSD-3-Clause
// copyright-holders:R. Belmont
/**********************************************************************

    General Instruments AY-5-3600 Keyboard Encoder emulation

*********************************************************************/

/*

    TODO:

    - scan timer clock frequency
    - more accurate emulation of real chip

*/

#include "emu.h"
#include "kb3600.h"

//#define VERBOSE 1
#include "logmacro.h"



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// devices
DEFINE_DEVICE_TYPE(AY3600, ay3600_device, "ay3600", "AY-5-3600 Keyboard Encoder")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  ay3600_device - constructor
//-------------------------------------------------

ay3600_device::ay3600_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, AY3600, tag, owner, clock),
	m_read_x0(*this),
	m_read_x1(*this),
	m_read_x2(*this),
	m_read_x3(*this),
	m_read_x4(*this),
	m_read_x5(*this),
	m_read_x6(*this),
	m_read_x7(*this),
	m_read_x8(*this),
	m_read_shift(*this),
	m_read_control(*this),
	m_write_data_ready(*this),
	m_write_ako(*this)
{
	for (auto & elem : m_x_mask)
	{
		elem = 0;
	}
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void ay3600_device::device_start()
{
	// resolve callbacks
	m_read_x0.resolve_safe(0);
	m_read_x1.resolve_safe(0);
	m_read_x2.resolve_safe(0);
	m_read_x3.resolve_safe(0);
	m_read_x4.resolve_safe(0);
	m_read_x5.resolve_safe(0);
	m_read_x6.resolve_safe(0);
	m_read_x7.resolve_safe(0);
	m_read_x8.resolve_safe(0);
	m_read_shift.resolve_safe(0);
	m_read_control.resolve_safe(0);
	m_write_data_ready.resolve_safe();
	m_write_ako.resolve_safe();

	// allocate timers
	m_scan_timer = timer_alloc();
	m_scan_timer->adjust(attotime::from_hz(60), 0, attotime::from_hz(60));

	m_ako = 0;

	// state saving
	save_item(NAME(m_b));
	save_item(NAME(m_ako));
	save_item(NAME(m_x_mask));
}


//-------------------------------------------------
//  device_start - device-specific reset
//-------------------------------------------------

void ay3600_device::device_reset()
{
}

//-------------------------------------------------
//  device_timer - handler timer events
//-------------------------------------------------

void ay3600_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	int ako = 0;

	for (int x = 0; x < 9; x++)
	{
		uint16_t data = 0;

		switch(x)
		{
			case 0: data = m_read_x0(); break;
			case 1: data = m_read_x1(); break;
			case 2: data = m_read_x2(); break;
			case 3: data = m_read_x3(); break;
			case 4: data = m_read_x4(); break;
			case 5: data = m_read_x5(); break;
			case 6: data = m_read_x6(); break;
			case 7: data = m_read_x7(); break;
			case 8: data = m_read_x8(); break;
		}

		for (int y = 0; y < 10; y++)
		{
			int b = (x * 10) + y;

			if (b > 63)
			{
				b -= 64;
				b = 0x100 | b;
			}

			b |= (m_read_shift() << 6);
			b |= (m_read_control() << 7);

			if (BIT(data, y))
			{
				ako = 1;

				if (!(m_x_mask[x] & (1 << y)))
				{
					m_x_mask[x] |= (1 << y);

					if (m_b != b)
					{
						m_b = b;

						m_write_data_ready(1);

						if (ako != m_ako)
						{
							m_write_ako(ako);
							m_ako = ako;
						}
						return;
					}
				}
			}
			else    // key released, unmark it from the "down" info
			{
				m_x_mask[x] &= ~(1 << y);
			}
		}
	}

	if (!ako)
	{
		m_b = -1;
	}

	if (ako != m_ako)
	{
		m_write_ako(ako);
		m_ako = ako;
	}
}


//-------------------------------------------------
//  b_r -
//-------------------------------------------------

uint16_t ay3600_device::b_r()
{
	uint16_t data = m_b;

	m_write_data_ready(0);

	return data;
}
