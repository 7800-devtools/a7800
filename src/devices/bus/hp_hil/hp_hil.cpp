// license:BSD-3-Clause
// copyright-holders:Sergey Svishchev
/***************************************************************************

  HP-HIL Keyboard connector interface

***************************************************************************/


#include "emu.h"
#include "hp_hil.h"


#define VERBOSE_DBG 0

#define DBG_LOG(N,M,A) \
	do { \
		if(VERBOSE_DBG>=N) \
		{ \
			if( M ) \
				logerror("%11.6f at %s: %-10s",machine().time().as_double(),machine().describe_context(),(char*)M ); \
			logerror A; \
		} \
	} while (0)


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(HP_HIL_SLOT, hp_hil_slot_device, "hp_hil_slot", "HP-HIL Slot")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  hp_hil_slot_device - constructor
//-------------------------------------------------
hp_hil_slot_device::hp_hil_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, HP_HIL_SLOT, tag, owner, clock)
	, device_slot_interface(mconfig, *this)
{
}


void hp_hil_slot_device::static_set_hp_hil_slot(device_t &device, device_t *owner, const char *mlc_tag)
{
	hp_hil_slot_device &hp_hil = dynamic_cast<hp_hil_slot_device &>(device);
	hp_hil.m_owner = owner;
	hp_hil.m_mlc_tag = mlc_tag;
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void hp_hil_slot_device::device_start()
{
	device_hp_hil_interface *dev = dynamic_cast<device_hp_hil_interface *>(get_card_device());

	if (dev) device_hp_hil_interface::static_set_hp_hil_mlc(*dev,m_owner->subdevice(m_mlc_tag));
}


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(HP_HIL_MLC, hp_hil_mlc_device, "hp_hil_mlc", "HP-HIL Master Link Controller")


//-------------------------------------------------
//  hp_hil_mlc_device - constructor
//-------------------------------------------------
hp_hil_mlc_device::hp_hil_mlc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, HP_HIL_MLC, tag, owner, clock)
	, int_cb(*this)
	, nmi_cb(*this)
{
}

void hp_hil_mlc_device::add_hp_hil_device( device_hp_hil_interface *device )
{
	m_device_list.append(*device);
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------
void hp_hil_mlc_device::device_start()
{
	// resolve callbacks
	int_cb.resolve_safe();
	nmi_cb.resolve_safe();
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------
void hp_hil_mlc_device::device_reset()
{
	// FIFO contents are not initialized at powerup or reset

	m_r2 = 0;
	m_r3 = 0; // HPMLC_R3_NMI;
}


WRITE8_MEMBER(hp_hil_mlc_device::write)
{
	device_hp_hil_interface *entry = m_device_list.first();

	DBG_LOG(2,"Write", ("%d <- %02x\n", offset, data));

	switch (offset)
	{
	case 0:
		DBG_LOG(1,"Transmit", ("%scommand 0x%02x to device %d\n", !m_loop?"loopback ":"", data, m_w1 & 7));
		if (m_loop & 2) // no devices on 2nd link loop
			return;
		if (m_loop == 0)
		{
			if (!m_fifo.full()) {
				m_fifo.enqueue(data | (m_w1 << 8));
				m_r3 |= HPMLC_R3_INT;
				int_cb(ASSERT_LINE);
			}
			return;
		}
		if ((m_w1 & 7) == 0) // broadcast
		{
			while (entry)
			{
				entry->hil_write(data | (m_w1 << 8));
				entry = entry->next();
			}
		} else
		{
			while (entry)
			{
				if (entry->device_id() == (m_w1 & 7))
					entry->hil_write(data | (m_w1 << 8));
				entry = entry->next();
			}
		}
		break;

	case 1:
		m_w1 = data & 0xf;
		break;

	case 2:
		m_w2 = data;
		break;

	case 3:
		m_w3 = data;
		break;

	case 32:    // loopback switch: bit 0 = loop0, bit 1 = loop1
		m_loop = data;
		break;
	}
}

READ8_MEMBER(hp_hil_mlc_device::read)
{
	uint8_t data = 0;

	switch (offset)
	{
	case 0:
		if (!m_fifo.empty())
			data = m_fifo.dequeue() & 255;
		break;

	case 1:
		if (!m_fifo.empty())
			data = m_fifo.peek() >> 8;
		break;

	case 2:
		data = m_r2;
		m_r2 &= ~(HPMLC_R2_PERR|HPMLC_R2_FERR|HPMLC_R2_FOF);
		break;

	case 3:
		data = m_r3;
		m_r3 &= ~(HPMLC_R3_INT|HPMLC_R3_NMI|HPMLC_R3_LERR);
		int_cb(CLEAR_LINE);
		break;
	}

	DBG_LOG(2,"Read", ("%d == %02x\n", offset, data));

	return data;
}

void hp_hil_mlc_device::hil_write(uint16_t data)
{
	DBG_LOG(1,"Receive", ("%s %04X fifo %s\n",
		BIT(data, 11)?"command":"data", data, m_fifo.full()?"full":(m_fifo.empty()?"empty":"ok")));

	if (!m_fifo.full())
	{
		if (!BIT(data, 11))
		{
			m_fifo.enqueue(data);
		}
		else if (!m_fifo.empty() || !(m_w2 & HPMLC_W2_IPF))
		{
			m_fifo.enqueue(data);
			m_r3 |= HPMLC_R3_INT;
			m_w3 &= ~HPMLC_W3_APE;
			int_cb(ASSERT_LINE);
		}
	}
	else
	{
		m_r2 |= HPMLC_R2_FOF;
		m_r3 |= HPMLC_R3_INT;
		int_cb(ASSERT_LINE);
	}
}

WRITE_LINE_MEMBER(hp_hil_mlc_device::ap_w)
{
	if (state && (m_w3 & HPMLC_W3_APE))
	{
		device_hp_hil_interface *entry = m_device_list.first();

		while (entry)
		{
			entry->hil_write(HPMLC_W1_C | HPHIL_POL);
			entry = entry->next();
		}
	}
}


//**************************************************************************
//  DEVICE PC KBD INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_hp_hil_interface - constructor
//-------------------------------------------------

device_hp_hil_interface::device_hp_hil_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig, device)
	, m_hp_hil_mlc(nullptr)
	, m_hp_hil_mlc_dev(nullptr)
	, m_next(nullptr)
{
}


//-------------------------------------------------
//  ~device_hp_hil_interface - destructor
//-------------------------------------------------

device_hp_hil_interface::~device_hp_hil_interface()
{
}


void device_hp_hil_interface::static_set_hp_hil_mlc(device_t &device, device_t *mlc_device)
{
	device_hp_hil_interface &hp_hil = dynamic_cast<device_hp_hil_interface &>(device);
	hp_hil.m_hp_hil_mlc_dev = mlc_device;
}


void device_hp_hil_interface::set_hp_hil_mlc_device()
{
	m_hp_hil_mlc = dynamic_cast<hp_hil_mlc_device *>(m_hp_hil_mlc_dev);
	m_hp_hil_mlc->add_hp_hil_device(this);
}

