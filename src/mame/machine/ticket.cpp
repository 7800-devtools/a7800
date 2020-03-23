// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    ticket.c

    Generic ticket dispensing device.

***************************************************************************/

#include "emu.h"
#include "machine/ticket.h"


//**************************************************************************
//  DEBUGGING
//**************************************************************************

#define DEBUG_TICKET 0
#define LOG(x) do { if (DEBUG_TICKET) logerror x; } while (0)



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(TICKET_DISPENSER, ticket_dispenser_device, "ticket_dispenser", "Ticket Dispenser")



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  ticket_dispenser_device - constructor
//-------------------------------------------------

ticket_dispenser_device::ticket_dispenser_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, TICKET_DISPENSER, tag, owner, clock),
		m_motor_sense(TICKET_MOTOR_ACTIVE_LOW),
		m_status_sense(TICKET_STATUS_ACTIVE_LOW),
		m_period(attotime::from_msec(100)),
		m_hopper_type(false),
		m_active_bit(0x80),
		m_motoron(0),
		m_ticketdispensed(0),
		m_ticketnotdispensed(0),
		m_status(0),
		m_power(0),
		m_timer(nullptr)
{
}


//-------------------------------------------------
//  ~ticket_dispenser_device - destructor
//-------------------------------------------------

ticket_dispenser_device::~ticket_dispenser_device()
{
}


//**************************************************************************
//  CONFIGURATION HELPERS
//**************************************************************************

//-------------------------------------------------
//  static_set_period - configure the clock period
//  for dispensing
//-------------------------------------------------

void ticket_dispenser_device::static_set_period(device_t &device, const attotime &period)
{
	downcast<ticket_dispenser_device &>(device).m_period = period;
}


//-------------------------------------------------
//  static_set_senses - configure the senses of
//  the motor and status bits
//-------------------------------------------------

void ticket_dispenser_device::static_set_senses(device_t &device, uint8_t motor_sense, uint8_t status_sense, bool hopper_type)
{
	ticket_dispenser_device &ticket = downcast<ticket_dispenser_device &>(device);
	ticket.m_motor_sense = motor_sense;
	ticket.m_status_sense = status_sense;
	ticket.m_hopper_type = hopper_type;
}



//**************************************************************************
//  READ/WRITE HANDLERS
//**************************************************************************

//-------------------------------------------------
//  read - read the status line via the active bit
//  (legacy method)
//-------------------------------------------------

READ8_MEMBER( ticket_dispenser_device::read )
{
	LOG(("%s: Ticket Status Read = %02X\n", machine().describe_context(), m_status));
	return m_status;
}


//-------------------------------------------------
//  line_r - read the status line as a proper line
//-------------------------------------------------

READ_LINE_MEMBER( ticket_dispenser_device::line_r )
{
	return m_status ? 1 : 0;
}


//-------------------------------------------------
//  write - write the control line via the active
//  bit (legacy method)
//-------------------------------------------------

WRITE8_MEMBER( ticket_dispenser_device::write )
{
	// On an activate signal, start dispensing!
	if ((data & m_active_bit) == m_motoron)
	{
		if (!m_power)
		{
			LOG(("%s: Ticket Power On\n", machine().describe_context()));
			m_timer->adjust(m_period);
			m_power = 1;
			m_status = m_ticketnotdispensed;
		}
	}
	else
	{
		if (m_power)
		{
			if (m_hopper_type == false || m_status == m_ticketnotdispensed)
			{
				LOG(("%s: Ticket Power Off\n", machine().describe_context()));
				m_timer->adjust(attotime::never);
				machine().output().set_led_value(2, 0);
			}
			m_power = 0;
		}
	}
}

//-------------------------------------------------
//  motor_w - write the control line as a proper
//  line
//-------------------------------------------------

WRITE_LINE_MEMBER( ticket_dispenser_device::motor_w )
{
	write(machine().dummy_space(), 0, state ? m_active_bit : 0);
}


//**************************************************************************
//  DEVICE INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_start - handle device startup
//-------------------------------------------------

void ticket_dispenser_device::device_start()
{
	m_active_bit = 0x80;
	m_motoron = (m_motor_sense == TICKET_MOTOR_ACTIVE_HIGH) ? m_active_bit : 0;
	m_ticketdispensed = (m_status_sense == TICKET_STATUS_ACTIVE_HIGH) ? m_active_bit : 0;
	m_ticketnotdispensed = m_ticketdispensed ^ m_active_bit;

	m_timer = timer_alloc();

	save_item(NAME(m_status));
	save_item(NAME(m_power));
}


//-------------------------------------------------
//  device_reset - handle device startup
//-------------------------------------------------

void ticket_dispenser_device::device_reset()
{
	m_status = m_ticketnotdispensed;
	m_power = 0x00;
}


//-------------------------------------------------
//  device_timer - handle timer callbacks
//-------------------------------------------------

void ticket_dispenser_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	// if we still have power, keep toggling ticket states
	if (m_power)
	{
		m_status ^= m_active_bit;
		LOG(("Ticket Status Changed to %02X\n", m_status));
		m_timer->adjust(m_period);
	}
	else if (m_hopper_type)
	{
		m_status ^= m_active_bit;
		LOG(("%s: Ticket Power Off\n", machine().describe_context()));
		m_timer->adjust(attotime::never);
		machine().output().set_led_value(2, 0);
	}

	// update LED status (fixme: should map to an output)
	machine().output().set_led_value(2, (m_status == m_ticketdispensed));

	// if we just dispensed, increment global count
	if (m_status == m_ticketdispensed)
	{
		machine().bookkeeping().increment_dispensed_tickets(1);
		LOG(("Ticket Dispensed\n"));
	}
}
