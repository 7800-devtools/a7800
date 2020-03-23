// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Nathan Woods
/**********************************************************************

    Motorola 6821 PIA interface and emulation

**********************************************************************/

#include "emu.h"
#include "6821pia.h"

//**************************************************************************
//  MACROS
//**************************************************************************

#define LOG_GENERAL 0x01
#define LOG_SETUP   0x02
#define LOG_CA1     0x08

//#define VERBOSE (LOG_SETUP | LOG_GENERAL)

#include "logmacro.h"
#define LOGSETUP(...) LOGMASKED(LOG_SETUP,   __VA_ARGS__)
#define LOGCA1(...)   LOGMASKED(LOG_CA1,     __VA_ARGS__)

#define PIA_IRQ1                (0x80)
#define PIA_IRQ2                (0x40)


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(PIA6821, pia6821_device, "pia6821", "6821 PIA")


//-------------------------------------------------
//  pia6821_device - constructor
//-------------------------------------------------

pia6821_device::pia6821_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, PIA6821, tag, owner, clock),
		m_in_a_handler(*this),
		m_in_b_handler(*this),
		m_in_ca1_handler(*this),
		m_in_cb1_handler(*this),
		m_in_ca2_handler(*this),
		m_out_a_handler(*this),
		m_out_b_handler(*this),
		m_ca2_handler(*this),
		m_cb2_handler(*this),
		m_irqa_handler(*this),
		m_irqb_handler(*this), m_in_a(0),
		m_in_ca1(0), m_in_ca2(0), m_out_a(0), m_out_ca2(0), m_port_a_z_mask(0), m_ddr_a(0),
		m_ctl_a(0), m_irq_a1(0), m_irq_a2(0),
		m_irq_a_state(0), m_in_b(0),
		m_in_cb1(0), m_in_cb2(0), m_out_b(0), m_out_cb2(0), m_last_out_cb2_z(0), m_ddr_b(0),
		m_ctl_b(0), m_irq_b1(0), m_irq_b2(0),
		m_irq_b_state(0), m_in_a_pushed(false), m_out_a_needs_pulled(false), m_in_ca1_pushed(false),
		m_in_ca2_pushed(false), m_out_ca2_needs_pulled(false), m_in_b_pushed(false), m_out_b_needs_pulled(false),
		m_in_cb1_pushed(false), m_in_cb2_pushed(false), m_out_cb2_needs_pulled(false), m_logged_port_a_not_connected(false),
		m_logged_port_b_not_connected(false), m_logged_ca1_not_connected(false), m_logged_ca2_not_connected(false),
		m_logged_cb1_not_connected(false), m_logged_cb2_not_connected(false)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void pia6821_device::device_start()
{
	// resolve callbacks
	m_in_a_handler.resolve();
	m_in_b_handler.resolve();
	m_in_ca1_handler.resolve();
	m_in_cb1_handler.resolve();
	m_in_ca2_handler.resolve();
	m_out_a_handler.resolve();
	m_out_b_handler.resolve();
	m_ca2_handler.resolve();
	m_cb2_handler.resolve();
	m_irqa_handler.resolve_safe();
	m_irqb_handler.resolve_safe();

	save_item(NAME(m_in_a));
	save_item(NAME(m_in_ca1));
	save_item(NAME(m_in_ca2));
	save_item(NAME(m_out_a));
	save_item(NAME(m_out_ca2));
	save_item(NAME(m_port_a_z_mask));
	save_item(NAME(m_ddr_a));
	save_item(NAME(m_ctl_a));
	save_item(NAME(m_irq_a1));
	save_item(NAME(m_irq_a2));
	save_item(NAME(m_irq_a_state));
	save_item(NAME(m_in_b));
	save_item(NAME(m_in_cb1));
	save_item(NAME(m_in_cb2));
	save_item(NAME(m_out_b));
	save_item(NAME(m_out_cb2));
	save_item(NAME(m_last_out_cb2_z));
	save_item(NAME(m_ddr_b));
	save_item(NAME(m_ctl_b));
	save_item(NAME(m_irq_b1));
	save_item(NAME(m_irq_b2));
	save_item(NAME(m_irq_b_state));
	save_item(NAME(m_in_a_pushed));
	save_item(NAME(m_out_a_needs_pulled));
	save_item(NAME(m_in_ca1_pushed));
	save_item(NAME(m_in_ca2_pushed));
	save_item(NAME(m_out_ca2_needs_pulled));
	save_item(NAME(m_in_b_pushed));
	save_item(NAME(m_out_b_needs_pulled));
	save_item(NAME(m_in_cb1_pushed));
	save_item(NAME(m_in_cb2_pushed));
	save_item(NAME(m_out_cb2_needs_pulled));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void pia6821_device::device_reset()
{
	//
	// set default read values.
	//
	// ports A,CA1,CA2 default to 1
	// ports B,CB1,CB2 are three-state and undefined (set to 0)
	//
	m_in_a = 0xff;
	m_in_ca1 = true;
	m_in_ca2 = true;
	m_out_a = 0;
	m_out_ca2 = 0;
	m_port_a_z_mask = 0;
	m_ddr_a = 0;
	m_ctl_a = 0;
	m_irq_a1 = 0;
	m_irq_a2 = 0;
	m_irq_a_state = 0;
	m_in_b = 0;
	m_in_cb1 = 0;
	m_in_cb2 = 0;
	m_out_b = 0;
	m_out_cb2 = 0;
	m_last_out_cb2_z = 0;
	m_ddr_b = 0;
	m_ctl_b = 0;
	m_irq_b1 = 0;
	m_irq_b2 = 0;
	m_irq_b_state = 0;
	m_in_a_pushed = false;
	m_out_a_needs_pulled = false;
	m_in_ca1_pushed = false;
	m_in_ca2_pushed = false;
	m_out_ca2_needs_pulled = false;
	m_in_b_pushed = false;
	m_out_b_needs_pulled = false;
	m_in_cb1_pushed = false;
	m_in_cb2_pushed = false;
	m_out_cb2_needs_pulled = false;
	m_logged_port_a_not_connected = false;
	m_logged_port_b_not_connected = false;
	m_logged_ca1_not_connected = false;
	m_logged_ca2_not_connected = false;
	m_logged_cb1_not_connected = false;
	m_logged_cb2_not_connected = false;


	// clear the IRQs
	m_irqa_handler(false);
	m_irqb_handler(false);
}


//-------------------------------------------------
//  update_interrupts
//-------------------------------------------------

void pia6821_device::update_interrupts()
{
	// start with IRQ A
	int new_state = (m_irq_a1 && irq1_enabled(m_ctl_a)) || (m_irq_a2 && irq2_enabled(m_ctl_a));

	if (new_state != m_irq_a_state)
	{
		m_irq_a_state = new_state;
		m_irqa_handler(m_irq_a_state);
	}

	// then do IRQ B
	new_state = (m_irq_b1 && irq1_enabled(m_ctl_b)) || (m_irq_b2 && irq2_enabled(m_ctl_b));

	if (new_state != m_irq_b_state)
	{
		m_irq_b_state = new_state;
		m_irqb_handler(m_irq_b_state);
	}
}


//-------------------------------------------------
//  get_in_a_value
//-------------------------------------------------

uint8_t pia6821_device::get_in_a_value()
{
	uint8_t port_a_data = 0;
	uint8_t ret;

	// update the input
	if (!m_in_a_handler.isnull())
	{
		port_a_data = m_in_a_handler(0);
	}
	else
	{
		if (m_in_a_pushed)
		{
			port_a_data = m_in_a;
		}
		else
		{
			// mark all pins disconnected
			m_port_a_z_mask = 0xff;

			if (!m_logged_port_a_not_connected && (m_ddr_a != 0xff))
			{
				logerror("Warning! No port A read handler. Assuming pins 0x%02X not connected\n", m_ddr_a ^ 0xff);
				m_logged_port_a_not_connected = true;
			}
		}
	}

	// - connected pins are always read
	// - disconnected pins read the output buffer in output mode
	// - disconnected pins are HI in input mode
	ret = (~m_port_a_z_mask             & port_a_data) |
			( m_port_a_z_mask &  m_ddr_a & m_out_a) |
			( m_port_a_z_mask & ~m_ddr_a);

	return ret;
}


//-------------------------------------------------
//  get_in_b_value
//-------------------------------------------------

uint8_t pia6821_device::get_in_b_value()
{
	uint8_t ret;

	if (m_ddr_b == 0xff)
	{
		// all output, just return buffer
		ret = m_out_b;
	}
	else
	{
		uint8_t port_b_data;

		// update the input
		if (!m_in_b_handler.isnull())
		{
			port_b_data = m_in_b_handler(0);
		}
		else
		{
			if (m_in_b_pushed)
			{
				port_b_data = m_in_b;
			}
			else
			{
				if (!m_logged_port_b_not_connected && (m_ddr_b != 0xff))
				{
					logerror("Error! No port B read handler. Three-state pins 0x%02X are undefined\n", m_ddr_b ^ 0xff);
					m_logged_port_b_not_connected = true;
				}

				// undefined -- need to return something
				port_b_data = 0x00;
			}
		}

		// the DDR determines if the pin or the output buffer is read
		ret = (m_out_b & m_ddr_b) | (port_b_data & ~m_ddr_b);
	}

	return ret;
}


//-------------------------------------------------
//  get_out_a_value
//-------------------------------------------------

uint8_t pia6821_device::get_out_a_value()
{
	uint8_t ret;

	if (m_ddr_a == 0xff)
	{
		// all output
		ret = m_out_a;
	}
	else
	{
		// input pins don't change
		ret = (m_out_a & m_ddr_a) | (get_in_a_value() & ~m_ddr_a);
	}

	return ret;
}


//-------------------------------------------------
//  get_out_b_value
//-------------------------------------------------

uint8_t pia6821_device::get_out_b_value()
{
	// input pins are high-impedance - we just send them as zeros for backwards compatibility
	return m_out_b & m_ddr_b;
}


//-------------------------------------------------
//  set_out_ca2
//-------------------------------------------------

void pia6821_device::set_out_ca2(int data)
{
	if (data != m_out_ca2)
	{
		m_out_ca2 = data;

		// send to output function
		if (!m_ca2_handler.isnull())
		{
			m_ca2_handler(m_out_ca2);
		}
		else
		{
			if (m_out_ca2_needs_pulled)
			{
				logerror("Warning! No port CA2 write handler. Previous value has been lost!\n");
			}

			m_out_ca2_needs_pulled = true;
		}
	}
}


//-------------------------------------------------
//  set_out_cb2
//-------------------------------------------------

void pia6821_device::set_out_cb2(int data)
{
	int z = cb2_output_z();

	if ((data != m_out_cb2) || (z != m_last_out_cb2_z))
	{
		m_out_cb2 = data;
		m_last_out_cb2_z = z;

		// send to output function
		if (!m_cb2_handler.isnull())
		{
			m_cb2_handler(m_out_cb2);
		}
		else
		{
			if (m_out_cb2_needs_pulled)
			{
				logerror("Warning! No port CB2 write handler. Previous value has been lost!\n");
			}

			m_out_cb2_needs_pulled = true;
		}
	}
}


//-------------------------------------------------
//  port_a_r
//-------------------------------------------------

uint8_t pia6821_device::port_a_r()
{
	uint8_t ret = get_in_a_value();

	// IRQ flags implicitly cleared by a read
	m_irq_a1 = false;
	m_irq_a2 = false;
	update_interrupts();

	// CA2 is configured as output and in read strobe mode
	if(c2_output(m_ctl_a) && c2_strobe_mode(m_ctl_a))
	{
		// this will cause a transition low
		set_out_ca2(false);

		// if the CA2 strobe is cleared by the E, reset it right away
		if(strobe_e_reset(m_ctl_a))
		{
			set_out_ca2(true);
		}
	}

	LOG("PIA port A read = %02X\n", ret);

	return ret;
}


//-------------------------------------------------
//  ddr_a_r
//-------------------------------------------------

uint8_t pia6821_device::ddr_a_r()
{
	uint8_t ret = m_ddr_a;

	LOG("PIA DDR A read = %02X\n", ret);

	return ret;
}


//-------------------------------------------------
//  port_b_r
//-------------------------------------------------

uint8_t pia6821_device::port_b_r()
{
	uint8_t ret = get_in_b_value();

	// This read will implicitly clear the IRQ B1 flag.  If CB2 is in write-strobe
	// mode with CB1 restore, and a CB1 active transition set the flag,
	// clearing it will cause CB2 to go high again.  Note that this is different
	// from what happens with port A.
	if(m_irq_b1 && c2_strobe_mode(m_ctl_b) && strobe_c1_reset(m_ctl_b))
	{
		set_out_cb2(true);
	}

	// IRQ flags implicitly cleared by a read
	m_irq_b1 = false;
	m_irq_b2 = false;
	update_interrupts();

	LOG("PIA port B read = %02X\n", ret);

	return ret;
}


//-------------------------------------------------
//  ddr_b_r
//-------------------------------------------------

uint8_t pia6821_device::ddr_b_r()
{
	uint8_t ret = m_ddr_b;

	LOG("PIA DDR B read = %02X\n", ret);

	return ret;
}


//-------------------------------------------------
//  control_a_r
//-------------------------------------------------

uint8_t pia6821_device::control_a_r()
{
	uint8_t ret;

	// update CA1 & CA2 if callback exists, these in turn may update IRQ's
	if (!m_in_ca1_handler.isnull())
	{
		ca1_w(m_in_ca1_handler());
	}
	else if(!m_logged_ca1_not_connected && (!m_in_ca1_pushed))
	{
		logerror("Warning! No CA1 read handler. Assuming pin not connected\n");
		m_logged_ca1_not_connected = true;
	}

	if (!m_in_ca2_handler.isnull())
	{
		ca2_w(m_in_ca2_handler());
	}
	else if ( !m_logged_ca2_not_connected && c2_input(m_ctl_a) && !m_in_ca2_pushed)
	{
		logerror("Warning! No CA2 read handler. Assuming pin not connected\n");
		m_logged_ca2_not_connected = true;
	}

	// read control register
	ret = m_ctl_a;

	// set the IRQ flags if we have pending IRQs
	if(m_irq_a1)
	{
		ret |= PIA_IRQ1;
	}

	if(m_irq_a2 && c2_input(m_ctl_a))
	{
		ret |= PIA_IRQ2;
	}

	LOG("PIA control A read = %02X\n", ret);

	return ret;
}


//-------------------------------------------------
//  control_b_r
//-------------------------------------------------

uint8_t pia6821_device::control_b_r()
{
	uint8_t ret;

	// update CB1 & CB2 if callback exists, these in turn may update IRQ's
	if(!m_in_cb1_handler.isnull())
	{
		cb1_w(m_in_cb1_handler());
	}
	else if(!m_logged_cb1_not_connected && !m_in_cb1_pushed)
	{
		logerror("Error! no CB1 read handler. Three-state pin is undefined\n");
		m_logged_cb1_not_connected = true;
	}

	if(!m_logged_cb2_not_connected && c2_input(m_ctl_b) && !m_in_cb2_pushed)
	{
		logerror("Error! Three-state pin is undefined\n");
		m_logged_cb2_not_connected = true;
	}

	// read control register
	ret = m_ctl_b;

	// set the IRQ flags if we have pending IRQs
	if(m_irq_b1)
	{
		ret |= PIA_IRQ1;
	}

	if(m_irq_b2 && c2_input(m_ctl_b))
	{
		ret |= PIA_IRQ2;
	}

	LOG("PIA control B read = %02X\n", ret);

	return ret;
}


//-------------------------------------------------
//  read
//-------------------------------------------------

uint8_t pia6821_device::reg_r(uint8_t offset)
{
	uint8_t ret;

	switch (offset & 0x03)
	{
		default: // impossible
		case 0x00:
			if (output_selected(m_ctl_a))
			{
				ret = port_a_r();
			}
			else
			{
				ret = ddr_a_r();
			}
			break;

		case 0x01:
			ret = control_a_r();
			break;

		case 0x02:
			if (output_selected(m_ctl_b))
			{
				ret = port_b_r();
			}
			else
			{
				ret = ddr_b_r();
			}
			break;

		case 0x03:
			ret = control_b_r();
			break;
	}

	return ret;
}



//-------------------------------------------------
//  send_to_out_a_func
//-------------------------------------------------

void pia6821_device::send_to_out_a_func(const char* message)
{
	// input pins are pulled high
	uint8_t data = get_out_a_value();

	LOG("PIA %s = %02X\n", message, data);

	if(!m_out_a_handler.isnull())
	{
		m_out_a_handler((offs_t) 0, data);
	}
	else
	{
		if(m_out_a_needs_pulled)
		{
			logerror("Warning! No port A write handler. Previous value has been lost!\n");
		}

		m_out_a_needs_pulled = true;
	}
}


//-------------------------------------------------
//  send_to_out_b_func
//-------------------------------------------------

void pia6821_device::send_to_out_b_func(const char* message)
{
	// input pins are high-impedance - we just send them as zeros for backwards compatibility
	uint8_t data = get_out_b_value();

	LOG("PIA %s = %02X\n", message, data);

	if(!m_out_b_handler.isnull())
	{
		m_out_b_handler((offs_t)0, data);
	}
	else
	{
		if(m_out_b_needs_pulled)
		{
			logerror("Warning! No port B write handler. Previous value has been lost!\n");
		}

		m_out_b_needs_pulled = true;
	}
}


//-------------------------------------------------
//  port_a_w
//-------------------------------------------------

void pia6821_device::port_a_w(uint8_t data)
{
	// buffer the output value
	m_out_a = data;

	send_to_out_a_func("port A write");
}


//-------------------------------------------------
//  ddr_a_w
//-------------------------------------------------

void pia6821_device::ddr_a_w(uint8_t data)
{
	if(data == 0x00)
	{
		LOGSETUP("PIA DDR A write = %02X (input mode)\n", data);
	}
	else if(data == 0xff)
	{
		LOGSETUP("PIA DDR A write = %02X (output mode)\n", data);
	}
	else
	{
		LOGSETUP("PIA DDR A write = %02X (mixed mode)\n", data);
	}

	if(m_ddr_a != data)
	{
		// DDR changed, call the callback again
		m_ddr_a = data;
		m_logged_port_a_not_connected = false;
		send_to_out_a_func("port A write due to DDR change");
	}
}


//-------------------------------------------------
//  port_b_w
//-------------------------------------------------

void pia6821_device::port_b_w(uint8_t data)
{
	// buffer the output value
	m_out_b = data;

	send_to_out_b_func("port B write");

	// CB2 in write strobe mode
	if(c2_strobe_mode(m_ctl_b))
	{
		// this will cause a transition low
		set_out_cb2(false);

		// if the CB2 strobe is cleared by the E, reset it right away
		if(strobe_e_reset(m_ctl_b))
		{
			set_out_cb2(true);
		}
	}
}


//-------------------------------------------------
//  ddr_b_w
//-------------------------------------------------

void pia6821_device::ddr_b_w(uint8_t data)
{
	if (data == 0x00)
	{
		LOGSETUP("PIA DDR B write = %02X (input mode)\n", data);
	}
	else if (data == 0xff)
	{
		LOGSETUP("PIA DDR B write = %02X (output mode)\n", data);
	}
	else
	{
		LOGSETUP("PIA DDR B write = %02X (mixed mode)\n", data);
	}

	if(m_ddr_b != data)
	{
		// DDR changed, call the callback again
		m_ddr_b = data;
		m_logged_port_b_not_connected = false;
		send_to_out_b_func("port B write due to DDR change");
	}
}


//-------------------------------------------------
//  control_a_w
//-------------------------------------------------

void pia6821_device::control_a_w(uint8_t data)
{
	// bit 7 and 6 are read only
	data &= 0x3f;

	LOGSETUP("PIA control A write = %02X\n", data);

	// update the control register
	m_ctl_a = data;

	// CA2 is configured as output
	if(c2_output(m_ctl_a))
	{
		bool temp;

		if(c2_set_mode(m_ctl_a))
		{
			// set/reset mode - bit value determines the new output
			temp = c2_set(m_ctl_a);
		}
		else
		{
			// strobe mode - output is always high unless strobed
			temp = true;
		}

		set_out_ca2(temp);
	}

	// update externals
	update_interrupts();
}


//-------------------------------------------------
//  control_b_w
//-------------------------------------------------

void pia6821_device::control_b_w(uint8_t data)
{
	bool temp;

	// bit 7 and 6 are read only
	data &= 0x3f;

	LOGSETUP("PIA control B write = %02X\n", data);

	// update the control register
	m_ctl_b = data;

	if (c2_set_mode(m_ctl_b))
	{
		// set/reset mode - bit value determines the new output
		temp = c2_set(m_ctl_b);
	}
	else
	{
		// strobe mode - output is always high unless strobed
		temp = true;
	}

	set_out_cb2(temp);

	// update externals
	update_interrupts();
}


//-------------------------------------------------
//  write
//-------------------------------------------------

void pia6821_device::reg_w(uint8_t offset, uint8_t data)
{
	switch (offset & 0x03)
	{
		default: // impossible
		case 0x00:
			if (output_selected(m_ctl_a))
			{
				port_a_w(data);
			}
			else
			{
				ddr_a_w(data);
			}
			break;

		case 0x01:
			control_a_w( data);
			break;

		case 0x02:
			if(output_selected(m_ctl_b))
			{
				port_b_w(data);
			}
			else
			{
				ddr_b_w(data);
			}
			break;

		case 0x03:
			control_b_w(data);
			break;
	}
}


//-------------------------------------------------
//  set_a_input
//-------------------------------------------------

void pia6821_device::set_a_input(uint8_t data, uint8_t z_mask)
{
	assert_always(m_in_a_handler.isnull(), "pia6821_porta_w() called when in_a_func implemented");

	LOG("Set PIA input port A = %02X\n", data);

	m_in_a = data;
	m_port_a_z_mask = z_mask;
	m_in_a_pushed = true;
}


//-------------------------------------------------
//  pia6821_porta_w
//-------------------------------------------------

void pia6821_device::porta_w(uint8_t data)
{
	set_a_input(data, 0);
}


//-------------------------------------------------
//  a_output
//-------------------------------------------------

uint8_t pia6821_device::a_output()
{
	m_out_a_needs_pulled = false;

	return get_out_a_value();
}


//-------------------------------------------------
//  ca1_w
//-------------------------------------------------

WRITE_LINE_MEMBER( pia6821_device::ca1_w )
{
	LOGCA1("Set PIA input CA1 = %d\n", state);

	// the new state has caused a transition
	if((m_in_ca1 != state) && ((state && c1_low_to_high(m_ctl_a)) || (!state && c1_high_to_low(m_ctl_a))))
	{
		LOGCA1("CA1 triggering\n");

		// mark the IRQ
		m_irq_a1 = true;

		// update externals
		update_interrupts();

		// CA2 is configured as output and in read strobe mode and cleared by a CA1 transition
		if(c2_output(m_ctl_a) && c2_strobe_mode(m_ctl_a) && strobe_c1_reset(m_ctl_a))
		{
			set_out_ca2(true);
		}
	}

	// set the new value for CA1
	m_in_ca1 = state;
	m_in_ca1_pushed = true;
}


//-------------------------------------------------
//  ca2_w
//-------------------------------------------------

WRITE_LINE_MEMBER( pia6821_device::ca2_w )
{
	LOG("Set PIA input CA2 = %d\n", state);

	// if input mode and the new state has caused a transition
	if(c2_input(m_ctl_a) && (m_in_ca2 != state) && ((state && c2_low_to_high(m_ctl_a)) || (!state && c2_high_to_low(m_ctl_a))))
	{
		LOG("CA2 triggering\n");

		// mark the IRQ
		m_irq_a2 = true;

		// update externals
		update_interrupts();
	}

	// set the new value for CA2
	m_in_ca2 = state;
	m_in_ca2_pushed = true;
}


//-------------------------------------------------
//  ca2_output
//-------------------------------------------------

bool pia6821_device::ca2_output()
{
	m_out_ca2_needs_pulled = false;

	return m_out_ca2;
}


//-------------------------------------------------
//  ca2_output_z - version of ca2_output which
//  takes account of internal pullup resistor
//-------------------------------------------------

bool pia6821_device::ca2_output_z()
{
	m_out_ca2_needs_pulled = false;

	// If it's an output, output the bit, if it's an input, it's
	// pulled up
	return m_out_ca2 | c2_input(m_ctl_a);
}


//-------------------------------------------------
//  portb_w
//-------------------------------------------------

void pia6821_device::portb_w(uint8_t data)
{
	assert_always(m_in_b_handler.isnull(), "pia_set_input_b() called when in_b_func implemented");

	LOG("Set PIA input port B = %02X\n", data);

	m_in_b = data;
	m_in_b_pushed = true;
}


//-------------------------------------------------
//  b_output
//-------------------------------------------------

uint8_t pia6821_device::b_output()
{
	m_out_b_needs_pulled = false;

	return get_out_b_value();
}


//-------------------------------------------------
//  cb1_w
//-------------------------------------------------

WRITE_LINE_MEMBER( pia6821_device::cb1_w )
{
	LOG("Set PIA input CB1 = %d\n", state);

	// the new state has caused a transition
	if((m_in_cb1 != state) && ((state && c1_low_to_high(m_ctl_b)) || (!state && c1_high_to_low(m_ctl_b))))
	{
		LOG("CB1 triggering\n");

		// mark the IRQ
		m_irq_b1 = 1;

		// update externals
		update_interrupts();

		// If CB2 is configured as a write-strobe output which is reset by a CB1
		// transition, this reset will only happen when a read from port B implicitly
		// clears the IRQ B1 flag.  So we handle the CB2 reset there.  Note that this
		// is different from what happens with port A.
	}

	// set the new value for CB1
	m_in_cb1 = state;
	m_in_cb1_pushed = true;
}


//-------------------------------------------------
//  cb2_w
//-------------------------------------------------

WRITE_LINE_MEMBER( pia6821_device::cb2_w )
{
	LOG("Set PIA input CB2 = %d\n", state);

	// if input mode and the new state has caused a transition
	if (c2_input(m_ctl_b) &&
		(m_in_cb2 != state) &&
		((state && c2_low_to_high(m_ctl_b)) || (!state && c2_high_to_low(m_ctl_b))))
	{
		LOG("CB2 triggering\n");

		// mark the IRQ
		m_irq_b2 = 1;

		// update externals
		update_interrupts();
	}

	// set the new value for CA2
	m_in_cb2 = state;
	m_in_cb2_pushed = true;
}


//-------------------------------------------------
//  output_cb2
//-------------------------------------------------

bool pia6821_device::cb2_output()
{
	m_out_cb2_needs_pulled = false;

	return m_out_cb2;
}


//-------------------------------------------------
//  cb2_output_z
//-------------------------------------------------

bool pia6821_device::cb2_output_z()
{
	return !c2_output(m_ctl_b);
}


//-------------------------------------------------
//  control byte wrappers
//-------------------------------------------------

bool pia6821_device::irq1_enabled(uint8_t c)    { return  bool((c >> 0) & 0x01); }
bool pia6821_device::c1_low_to_high(uint8_t c)  { return  bool((c >> 1) & 0x01); }
bool pia6821_device::c1_high_to_low(uint8_t c)  { return !bool((c >> 1) & 0x01); }
bool pia6821_device::output_selected(uint8_t c) { return  bool((c >> 2) & 0x01); }
bool pia6821_device::irq2_enabled(uint8_t c)    { return  bool((c >> 3) & 0x01); }
bool pia6821_device::strobe_e_reset(uint8_t c)  { return  bool((c >> 3) & 0x01); }
bool pia6821_device::strobe_c1_reset(uint8_t c) { return !bool((c >> 3) & 0x01); }
bool pia6821_device::c2_set(uint8_t c)          { return  bool((c >> 3) & 0x01); }
bool pia6821_device::c2_low_to_high(uint8_t c)  { return  bool((c >> 4) & 0x01); }
bool pia6821_device::c2_high_to_low(uint8_t c)  { return !bool((c >> 4) & 0x01); }
bool pia6821_device::c2_set_mode(uint8_t c)     { return  bool((c >> 4) & 0x01); }
bool pia6821_device::c2_strobe_mode(uint8_t c)  { return !bool((c >> 4) & 0x01); }
bool pia6821_device::c2_output(uint8_t c)       { return  bool((c >> 5) & 0x01); }
bool pia6821_device::c2_input(uint8_t c)        { return !bool((c >> 5) & 0x01); }
