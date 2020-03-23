// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/*****************************************************************************

    7400 Quad 2-Input NAND Gate

*****************************************************************************/

#include "emu.h"
#include "7400.h"

DEFINE_DEVICE_TYPE(TTL7400, ttl7400_device, "7400", "7400 Quad 2-Input NAND Gate")

ttl7400_device::ttl7400_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, TTL7400, tag, owner, clock)
	, m_y1_func(*this)
	, m_y2_func(*this)
	, m_y3_func(*this)
	, m_y4_func(*this)
	, m_a(0)
	, m_b(0)
	, m_y(0)
{
}

void ttl7400_device::device_start()
{
	init();

	save_item(NAME(m_a));
	save_item(NAME(m_b));
	save_item(NAME(m_y));

	m_y1_func.resolve_safe();
	m_y2_func.resolve_safe();
	m_y3_func.resolve_safe();
	m_y4_func.resolve_safe();
}

void ttl7400_device::device_reset()
{
	init();
}

void ttl7400_device::init()
{
	m_a = 0;
	m_b = 0;
	m_y = 0;
}

void ttl7400_device::update()
{
	uint8_t last_y = m_y;

	m_y = (m_a & m_b) & 0xf;

	if (m_y != last_y)
	{
		for (int bit = 0; bit < 4; bit++)
		{
			if (BIT(m_y, bit) == BIT(last_y, bit))
				continue;

			switch(bit)
			{
				case 0: m_y1_func(BIT(m_y, bit)); break;
				case 1: m_y2_func(BIT(m_y, bit)); break;
				case 2: m_y3_func(BIT(m_y, bit)); break;
				case 3: m_y4_func(BIT(m_y, bit)); break;
			}
		}
	}
}

void ttl7400_device::a_w(uint8_t line, uint8_t state)
{
	uint8_t old_a = m_a;
	m_a &= ~(1 << line);
	m_a |= (state << line);
	if (old_a != m_a)
		update();
}

void ttl7400_device::b_w(uint8_t line, uint8_t state)
{
	uint8_t old_b = m_b;
	m_b &= ~(1 << line);
	m_b |= (state << line);
	if (old_b != m_b)
		update();
}

uint8_t ttl7400_device::y_r(uint8_t line)
{
	return (m_y >> line) & 1;
}


WRITE_LINE_MEMBER( ttl7400_device::a1_w ) { a_w(0, state); }
WRITE_LINE_MEMBER( ttl7400_device::a2_w ) { a_w(1, state); }
WRITE_LINE_MEMBER( ttl7400_device::a3_w ) { a_w(2, state); }
WRITE_LINE_MEMBER( ttl7400_device::a4_w ) { a_w(3, state); }

WRITE_LINE_MEMBER( ttl7400_device::b1_w ) { b_w(0, state); }
WRITE_LINE_MEMBER( ttl7400_device::b2_w ) { b_w(1, state); }
WRITE_LINE_MEMBER( ttl7400_device::b3_w ) { b_w(2, state); }
WRITE_LINE_MEMBER( ttl7400_device::b4_w ) { b_w(3, state); }

READ_LINE_MEMBER( ttl7400_device::y1_r ) { return y_r(0); }
READ_LINE_MEMBER( ttl7400_device::y2_r ) { return y_r(1); }
READ_LINE_MEMBER( ttl7400_device::y3_r ) { return y_r(2); }
READ_LINE_MEMBER( ttl7400_device::y4_r ) { return y_r(3); }
