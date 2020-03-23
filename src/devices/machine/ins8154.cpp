// license:GPL-2.0+
// copyright-holders:Dirk Best
/***************************************************************************

    National Semiconductor INS8154

    N-Channel 128-by-8 Bit RAM Input/Output (RAM I/O)

    TODO: Strobed modes

***************************************************************************/

#include "emu.h"
#include "ins8154.h"

#define VERBOSE 1
#include "logmacro.h"


/***************************************************************************
    CONSTANTS
***************************************************************************/

/* Mode Definition Register */
enum
{
	MDR_BASIC                 = 0x00,
	MDR_STROBED_INPUT         = 0x20,
	MDR_STROBED_OUTPUT        = 0x60,
	MDR_STROBED_OUTPUT_3STATE = 0xe0
};



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(INS8154, ins8154_device, "ins8154", "INS8154 RAM I/O")

//-------------------------------------------------
//  ins8154_device - constructor
//-------------------------------------------------

ins8154_device::ins8154_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, INS8154, tag, owner, clock)
	, m_in_a_cb(*this)
	, m_out_a_cb(*this)
	, m_in_b_cb(*this)
	, m_out_b_cb(*this)
	, m_out_irq_cb(*this)
	, m_in_a(0), m_in_b(0), m_out_a(0), m_out_b(0), m_mdr(0), m_odra(0), m_odrb(0)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void ins8154_device::device_start()
{
	/* resolve callbacks */
	m_in_a_cb.resolve();
	m_out_a_cb.resolve_safe();
	m_in_b_cb.resolve();
	m_out_b_cb.resolve_safe();
	m_out_irq_cb.resolve_safe();

	/* register for state saving */
	save_item(NAME(m_in_a));
	save_item(NAME(m_in_b));
	save_item(NAME(m_out_a));
	save_item(NAME(m_out_b));
	save_item(NAME(m_mdr));
	save_item(NAME(m_odra));
	save_item(NAME(m_odrb));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void ins8154_device::device_reset()
{
	m_in_a = 0;
	m_in_b = 0;
	m_out_a = 0;
	m_out_b = 0;
	m_mdr = 0;
	m_odra = 0;
	m_odrb = 0;
}


READ8_MEMBER(ins8154_device::ins8154_r)
{
	uint8_t val = 0xff;

	if (offset > 0x24)
	{
		LOG("%s: INS8154 Read from invalid offset %02x!\n", machine().describe_context(), offset);
		return 0xff;
	}

	switch (offset)
	{
	case 0x20:
		if (!m_in_a_cb.isnull())
			val = m_in_a_cb(0);
		m_in_a = val;
		break;

	case 0x21:
		if (!m_in_b_cb.isnull())
			val = m_in_b_cb(0);
		m_in_b = val;
		break;

	default:
		if (offset < 0x08)
		{
			if (!m_in_a_cb.isnull())
				val = (m_in_a_cb(0) << (8 - offset)) & 0x80;
			m_in_a = val;
		}
		else
		{
			if (!m_in_b_cb.isnull())
				val = (m_in_b_cb(0) << (8 - (offset >> 4))) & 0x80;
			m_in_b = val;
		}
		break;
	}

	return val;
}

WRITE8_MEMBER(ins8154_device::ins8154_porta_w)
{
	m_out_a = data;

	/* Test if any pins are set as outputs */
	if (m_odra)
		m_out_a_cb(offs_t(0), (data & m_odra) | (m_odra ^ 0xff));
}

WRITE8_MEMBER(ins8154_device::ins8154_portb_w)
{
	m_out_b = data;

	/* Test if any pins are set as outputs */
	if (m_odrb)
		m_out_b_cb(offs_t(0), (data & m_odrb) | (m_odrb ^ 0xff));
}

WRITE8_MEMBER(ins8154_device::ins8154_w)
{
	if (offset > 0x24)
	{
		LOG("%s: INS8154 Write %02x to invalid offset %02x!\n", machine().describe_context(), data, offset);
		return;
	}

	switch (offset)
	{
	case 0x20:
		ins8154_porta_w(space, 0, data);
		break;

	case 0x21:
		ins8154_portb_w(space, 0, data);
		break;

	case 0x22:
		LOG("%s: INS8154 ODRA set to %02x\n", machine().describe_context(), data);
		m_odra = data;
		break;

	case 0x23:
		LOG("%s: INS8154 ODRB set to %02x\n", machine().describe_context(), data);
		m_odrb = data;
		break;

	case 0x24:
		LOG("%s: INS8154 MDR set to %02x\n", machine().describe_context(), data);
		m_mdr = data;
		break;

	default:
		if (offset & 0x10)
		{
			/* Set bit */
			if (offset < 0x08)
				ins8154_porta_w(space, 0, m_out_a |= offset & 0x07);
			else
				ins8154_portb_w(space, 0, m_out_b |= (offset >> 4) & 0x07);
		}
		else
		{
			/* Clear bit */
			if (offset < 0x08)
				ins8154_porta_w(space, 0, m_out_a & ~(offset & 0x07));
			else
				ins8154_portb_w(space, 0, m_out_b & ~((offset >> 4) & 0x07));
		}
		break;
	}
}
