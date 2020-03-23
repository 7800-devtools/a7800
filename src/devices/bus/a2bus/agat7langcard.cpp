// license:BSD-3-Clause
// copyright-holders:Sergey Svishchev
/*********************************************************************

    agat7langcard.c

    Implemention of the Agat-7 language card

*********************************************************************/

#include "agat7langcard.h"

/***************************************************************************
    PARAMETERS
***************************************************************************/

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(A2BUS_AGAT7LANGCARD, a2bus_agat7langcard_device, "a7lang", "Agat-7 32K Language Card")

/***************************************************************************
    FUNCTION PROTOTYPES
***************************************************************************/

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

a2bus_agat7langcard_device::a2bus_agat7langcard_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_a2bus_card_interface(mconfig, *this), m_inh_state(0), m_last_offset(0), m_dxxx_bank(0), m_main_bank(0)
{
}

a2bus_agat7langcard_device::a2bus_agat7langcard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	a2bus_agat7langcard_device(mconfig, A2BUS_AGAT7LANGCARD, tag, owner, clock)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void a2bus_agat7langcard_device::device_start()
{
	// set_a2bus_device makes m_slot valid
	set_a2bus_device();

	memset(m_ram, 0, 32*1024);

	save_item(NAME(m_inh_state));
	save_item(NAME(m_ram));
	save_item(NAME(m_dxxx_bank));
	save_item(NAME(m_main_bank));
	save_item(NAME(m_last_offset));
}

void a2bus_agat7langcard_device::device_reset()
{
	m_inh_state = INH_NONE;
	m_dxxx_bank = 0;
	m_main_bank = 0;
	m_last_offset = -1;
	m_mode = 0;
}

void a2bus_agat7langcard_device::do_io(int offset)
{
	int old_inh_state = m_inh_state;

	m_last_offset = offset;

	m_inh_state = INH_WRITE;
	m_dxxx_bank = 0;
	m_main_bank = (offset & 1) * 0x4000;

	if (offset & 0x20)
	{
		m_inh_state = INH_READ;
	}

	if (offset & 0x40)
	{
		m_dxxx_bank = 0x1000;
	}

	if (m_inh_state != old_inh_state)
	{
		recalc_slot_inh();
	}

#if 1
	logerror("LC: (ofs %02x) new state %c%c dxxx=%04x main=%05x\n",
			offset,
			(m_inh_state & INH_READ) ? 'R' : 'x',
			(m_inh_state & INH_WRITE) ? 'W' : 'x',
			m_dxxx_bank, m_main_bank);
#endif
}


/*-------------------------------------------------
    read_cnxx - called for reads from this card's cnxx space
-------------------------------------------------*/

uint8_t a2bus_agat7langcard_device::read_cnxx(address_space &space, uint8_t offset)
{
	return m_last_offset < 0 ? 0x80 : (0x80 | m_last_offset);
}


/*-------------------------------------------------
    write_cnxx - called for writes to this card's cnxx space
-------------------------------------------------*/

void a2bus_agat7langcard_device::write_cnxx(address_space &space, uint8_t offset, uint8_t data)
{
	do_io(offset);
}

uint8_t a2bus_agat7langcard_device::read_inh_rom(address_space &space, uint16_t offset)
{
	assert(m_inh_state & INH_READ); // this should never happen

	if (offset < 0xe000)
	{
		return m_ram[(offset & 0xfff) + m_dxxx_bank + m_main_bank];
	}

	return m_ram[(offset & 0x1fff) + 0x2000 + m_main_bank];
}

void a2bus_agat7langcard_device::write_inh_rom(address_space &space, uint16_t offset, uint8_t data)
{
	// are writes enabled?
	if (!(m_inh_state & INH_WRITE))
	{
		return;
	}

	if (offset < 0xe000)
	{
		m_ram[(offset & 0xfff) + m_dxxx_bank + m_main_bank] = data;
		return;
	}

	m_ram[(offset & 0x1fff) + 0x2000 + m_main_bank] = data;
}

int a2bus_agat7langcard_device::inh_type()
{
	return m_inh_state;
}
