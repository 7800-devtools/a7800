// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    8259 PIC interface and emulation

    The 8259 is a programmable interrupt controller used to multiplex
    interrupts for x86 and other computers.  The chip is set up by
    writing a series of Initialization Command Words (ICWs) after which
    the chip is operational and capable of dispatching interrupts.  After
    this, Operation Command Words (OCWs) can be written to control further
    behavior.

**********************************************************************/

#include "emu.h"
#include "machine/pic8259.h"

#define LOG_GENERAL (1U << 0)
#define LOG_ICW     (1U << 1)
#define LOG_OCW     (1U << 2)

//#define VERBOSE (LOG_GENERAL | LOG_ICW | LOG_OCW)
#include "logmacro.h"

#define LOGICW(...) LOGMASKED(LOG_ICW, __VA_ARGS__)
#define LOGOCW(...) LOGMASKED(LOG_OCW, __VA_ARGS__)


ALLOW_SAVE_TYPE(pic8259_device::state_t); // allow save_item on a non-fundamental type

void pic8259_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	/* check the various IRQs */
	for (int n = 0, irq = m_prio; n < 8; n++, irq = (irq + 1) & 7)
	{
		uint8_t mask = 1 << irq;

		/* is this IRQ in service and not cascading and sfnm? */
		if ((m_isr & mask) && !(m_master && m_cascade && m_nested && (m_slave & mask)))
		{
			LOG("pic8259_timerproc(): PIC IRQ #%d still in service\n", irq);
			break;
		}

		/* is this IRQ pending and enabled? */
		if ((m_state == state_t::READY) && (m_irr & mask) && !(m_imr & mask))
		{
			LOG("pic8259_timerproc(): PIC triggering IRQ #%d\n", irq);
			m_out_int_func(1);
			return;
		}
		// if sfnm and in-service don't continue
		if((m_isr & mask) && m_master && m_cascade && m_nested && (m_slave & mask))
			break;
	}
	m_out_int_func(0);
}


void pic8259_device::set_irq_line(int irq, int state)
{
	uint8_t mask = (1 << irq);

	if (state)
	{
		/* setting IRQ line */
		LOG("pic8259_set_irq_line(): PIC set IRQ line #%d\n", irq);

		if(m_level_trig_mode || (!m_level_trig_mode && !(m_irq_lines & mask)))
		{
			m_irr |= mask;
		}
		m_irq_lines |= mask;
	}
	else
	{
		/* clearing IRQ line */
		LOG("pic8259_device::set_irq_line(): PIC cleared IRQ line #%d\n", irq);

		m_irq_lines &= ~mask;
		m_irr &= ~mask;
	}
	set_timer();
}


uint32_t pic8259_device::acknowledge()
{
	for (int n = 0, irq = m_prio; n < 8; n++, irq = (irq + 1) & 7)
	{
		uint8_t mask = 1 << irq;

		/* is this IRQ pending and enabled? */
		if ((m_irr & mask) && !(m_imr & mask))
		{
			LOG("pic8259_acknowledge(): PIC acknowledge IRQ #%d\n", irq);
			if (!m_level_trig_mode)
				m_irr &= ~mask;

			if (!m_auto_eoi)
				m_isr |= mask;

			set_timer();

			if ((m_cascade!=0) && (m_master!=0) && (mask & m_slave))
			{
				// it's from slave device
				return m_read_slave_ack_func(irq);
			}
			else
			{
				if (m_is_x86)
				{
					/* For x86 mode*/
					return irq + m_base;
				}
				else
				{
					/* in case of 8080/85) */
					return 0xcd0000 + (m_vector_addr_high << 8) + m_vector_addr_low + (irq << (3-m_vector_size));
				}
			}
		}
	}
	logerror("Spurious IRQ\n");
	if (m_is_x86)
		return m_base + 7;
	else
		return 0xcd0000 + (m_vector_addr_high << 8) + m_vector_addr_low + (7 << (3-m_vector_size));
}


IRQ_CALLBACK_MEMBER(pic8259_device::inta_cb)
{
	return acknowledge();
}


READ8_MEMBER( pic8259_device::read )
{
	/* NPW 18-May-2003 - Changing 0xFF to 0x00 as per Ruslan */
	uint8_t data = 0x00;

	switch(offset)
	{
		case 0: /* PIC acknowledge IRQ */
			if ( m_ocw3 & 0x04 )
			{
				/* Polling mode */
				if ( m_irr & ~m_imr )
				{
					/* check the various IRQs */
					for (int n = 0, irq = m_prio; n < 8; n++, irq = (irq + 1) & 7)
					{
						if ( ( 1 << irq ) & m_irr & ~m_imr )
						{
							data = 0x80 | irq;
							break;
						}
					}
					acknowledge();
				}
			}
			else
			{
				switch ( m_ocw3 & 0x03 )
				{
				case 2:
					data = m_irr;
					break;
				case 3:
					data = m_isr & ~m_imr;
					break;
				default:
					data = 0x00;
					break;
				}
			}
			break;

		case 1: /* PIC mask register */
			data = m_imr;
			break;
	}
	return data;
}


WRITE8_MEMBER( pic8259_device::write )
{
	switch(offset)
	{
		case 0:    /* PIC acknowledge IRQ */
			if (data & 0x10)
			{
				/* write ICW1 - this pretty much resets the chip */
				LOGICW("pic8259_device::write(): ICW1; data=0x%02X\n", data);

				m_imr                = 0x00;
				m_isr                = 0x00;
				m_irr                = 0x00;
				m_level_trig_mode    = (data & 0x08) ? 1 : 0;
				m_vector_size        = (data & 0x04) ? 1 : 0;
				m_cascade            = (data & 0x02) ? 0 : 1;
				m_icw4_needed        = (data & 0x01) ? 1 : 0;
				m_vector_addr_low    = (data & 0xe0);
				m_state          = state_t::ICW2;
				m_out_int_func(0);
			}
			else if (m_state == state_t::READY)
			{
				if ((data & 0x98) == 0x08)
				{
					/* write OCW3 */
					LOGOCW("pic8259_device::write(): OCW3; data=0x%02X\n", data);

					m_ocw3 = data;
				}
				else if ((data & 0x18) == 0x00)
				{
					int n = data & 7;
					uint8_t mask = 1 << n;

					/* write OCW2 */
					LOGOCW("pic8259_device::write(): OCW2; data=0x%02X\n", data);

					switch (data & 0xe0)
					{
						case 0x00:
							m_prio = 0;
							break;
						case 0x20:
							for (n = 0, mask = 1<<m_prio; n < 8; n++, mask = (mask<<1) | (mask>>7))
							{
								if (m_isr & mask)
								{
									m_isr &= ~mask;
									break;
								}
							}
							break;
						case 0x40:
							break;
						case 0x60:
							if( m_isr & mask )
							{
								m_isr &= ~mask;
							}
							break;
						case 0x80:
							m_prio = (m_prio + 1) & 7;
							break;
						case 0xa0:
							for (n = 0, mask = 1<<m_prio; n < 8; n++, mask = (mask<<1) | (mask>>7))
							{
								if( m_isr & mask )
								{
									m_isr &= ~mask;
									m_prio = (m_prio + 1) & 7;
									break;
								}
							}
							break;
						case 0xc0:
							m_prio = (n + 1) & 7;
							break;
						case 0xe0:
							if( m_isr & mask )
							{
								m_isr &= ~mask;
								m_prio = (n + 1) & 7;
							}
							break;
					}
				}
			}
			break;

		case 1:
			switch(m_state)
			{
				case state_t::ICW1:
					break;

				case state_t::ICW2:
					/* write ICW2 */
					LOGICW("pic8259_device::write(): ICW2; data=0x%02X\n", data);

					m_base = data & 0xf8;
					m_vector_addr_high = data ;
					if (m_cascade)
					{
						m_state = state_t::ICW3;
					}
					else
					{
						m_state = m_icw4_needed ? state_t::ICW4 : state_t::READY;
					}
					break;

				case state_t::ICW3:
					/* write ICW3 */
					LOGICW("pic8259_device::write(): ICW3; data=0x%02X\n", data);

					m_slave = data;
					m_state = m_icw4_needed ? state_t::ICW4 : state_t::READY;
					break;

				case state_t::ICW4:
					/* write ICW4 */
					LOGICW("pic8259_device::write(): ICW4; data=0x%02X\n", data);

					m_nested = (data & 0x10) ? 1 : 0;
					m_mode = (data >> 2) & 3;
					m_auto_eoi = (data & 0x02) ? 1 : 0;
					m_is_x86 = (data & 0x01) ? 1 : 0;
					m_state = state_t::READY;
					break;

				case state_t::READY:
					/* write OCW1 - set interrupt mask register */
					LOGOCW("pic8259_device::write(): OCW1; data=0x%02X\n", data);

					//printf("%s %02x\n",m_master ? "master pic8259 mask" : "slave pic8259 mask",data);
					m_imr = data;
					break;
			}
			break;
	}
	set_timer();
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void pic8259_device::device_start()
{
	// resolve callbacks
	m_out_int_func.resolve_safe();
	m_sp_en_func.resolve_safe(1);
	m_read_slave_ack_func.resolve_safe(0);

	// Register save state items
	save_item(NAME(m_state));
	save_item(NAME(m_isr));
	save_item(NAME(m_irr));
	save_item(NAME(m_prio));
	save_item(NAME(m_imr));
	save_item(NAME(m_irq_lines));
	save_item(NAME(m_input));
	save_item(NAME(m_ocw3));
	save_item(NAME(m_master));
	save_item(NAME(m_level_trig_mode));
	save_item(NAME(m_vector_size));
	save_item(NAME(m_cascade));
	save_item(NAME(m_icw4_needed));
	save_item(NAME(m_vector_addr_low));
	save_item(NAME(m_base));
	save_item(NAME(m_vector_addr_high));
	save_item(NAME(m_slave));
	save_item(NAME(m_nested));
	save_item(NAME(m_mode));
	save_item(NAME(m_auto_eoi));
	save_item(NAME(m_is_x86));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void pic8259_device::device_reset()
{
	m_state = state_t::READY;
	m_isr = 0;
	m_irr = 0;
	m_irq_lines = 0;
	m_prio = 0;
	m_imr = 0;
	m_input = 0;
	m_ocw3 = 2;
	m_level_trig_mode = 0;
	m_vector_size = 0;
	m_cascade = 0;
	m_icw4_needed = 0;
	m_base = 0;
	m_slave = 0;
	m_nested = 0;
	m_mode = 0;
	m_auto_eoi = 0;
	m_is_x86 = 0;
	m_vector_addr_low = 0;
	m_vector_addr_high = 0;

	m_master = m_sp_en_func();
}

DEFINE_DEVICE_TYPE(PIC8259, pic8259_device, "pic8259", "Intel 8259 PIC")

pic8259_device::pic8259_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, PIC8259, tag, owner, clock)
	, m_out_int_func(*this)
	, m_sp_en_func(*this)
	, m_read_slave_ack_func(*this)
	, m_irr(0)
	, m_irq_lines(0)
	, m_level_trig_mode(0)
{
}
