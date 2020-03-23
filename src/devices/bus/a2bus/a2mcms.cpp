// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*********************************************************************

    a2mcms.c

    Implementation of the Mountain Computer Music System.
    This was sold standalone and also used as part of the alphaSyntauri
    and SoundChaser systems.

*********************************************************************/

#include "emu.h"
#include "a2mcms.h"
#include "speaker.h"

// the actual sound device (a slot device can't currently also be a sound device so we keep this private here)
enum
{
	CTRL_IRQS = 0,
	CTRL_DMA,
	CTRL_MASTERVOL
};


DEFINE_DEVICE_TYPE(MCMS, mcms_device, "mcmseng", "Mountain Computer Music System engine")

/***************************************************************************
    PARAMETERS
***************************************************************************/

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(A2BUS_MCMS1, a2bus_mcms1_device, "a2mcms1", "Mountain Computer Music System (card 1)")
DEFINE_DEVICE_TYPE(A2BUS_MCMS2, a2bus_mcms2_device, "a2mcms2", "Mountain Computer Music System (card 2)")

#define ENGINE_TAG  "engine"

#define MCFG_MCMS_IRQ_CALLBACK(_cb) \
	devcb = &mcms_device::set_irq_cb(*device, DEVCB_##_cb);

/***************************************************************************
    FUNCTION PROTOTYPES
***************************************************************************/

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( a2bus_mcms1_device::device_add_mconfig )
	MCFG_SPEAKER_STANDARD_STEREO("mcms_l", "mcms_r")

	MCFG_DEVICE_ADD(ENGINE_TAG, MCMS, 1000000)
	MCFG_MCMS_IRQ_CALLBACK(WRITELINE(a2bus_mcms1_device, irq_w))

	MCFG_SOUND_ROUTE(0, "mcms_l", 1.0)
	MCFG_SOUND_ROUTE(1, "mcms_r", 1.0)
MACHINE_CONFIG_END

//**************************************************************************
//  LIVE DEVICE - Card 1
//**************************************************************************

a2bus_mcms1_device::a2bus_mcms1_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_a2bus_card_interface(mconfig, *this),
	m_mcms(*this, ENGINE_TAG)
{
}

a2bus_mcms1_device::a2bus_mcms1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	a2bus_mcms1_device(mconfig, A2BUS_MCMS1, tag, owner, clock)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void a2bus_mcms1_device::device_start()
{
	// set_a2bus_device makes m_slot valid
	set_a2bus_device();
}

void a2bus_mcms1_device::device_reset()
{
	m_mcms->set_bus_device(this);
}

// read once at c0n0 to disable 125 Hz IRQs
// read once at c0n1 to enable 125 Hz IRQs
uint8_t a2bus_mcms1_device::read_c0nx(address_space &space, uint8_t offset)
{
	if (offset == 0)
	{
		m_mcms->control_w(space, CTRL_IRQS, 0);
	}
	else if (offset == 1)
	{
		m_mcms->control_w(space, CTRL_IRQS, 1);
	}

	return 0xff;
}

// read at Cn00: light gun in bit 7, bits 0-5 = 'random' number
uint8_t a2bus_mcms1_device::read_cnxx(address_space &space, uint8_t offset)
{
	return m_mcms->get_pen_rand();
}

// write 0-255 to Cn00 to set the master volume
void a2bus_mcms1_device::write_cnxx(address_space &space, uint8_t offset, uint8_t data)
{
	if (offset == 0)
	{
		m_mcms->control_w(space, CTRL_MASTERVOL, data);
	}
}

mcms_device *a2bus_mcms1_device::get_engine(void)
{
	return m_mcms;
}

WRITE_LINE_MEMBER(a2bus_mcms1_device::irq_w)
{
	if (state == ASSERT_LINE)
	{
		raise_slot_irq();
	}
	else
	{
		lower_slot_irq();
	}
}

//**************************************************************************
//  LIVE DEVICE - Card 2
//**************************************************************************

a2bus_mcms2_device::a2bus_mcms2_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_a2bus_card_interface(mconfig, *this), m_card1(nullptr), m_engine(nullptr)
{
}

a2bus_mcms2_device::a2bus_mcms2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	a2bus_mcms2_device(mconfig, A2BUS_MCMS2, tag, owner, clock)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void a2bus_mcms2_device::device_start()
{
	// set_a2bus_device makes m_slot valid
	set_a2bus_device();

	if (m_slot < 2)
	{
		fatalerror("MCMS: Card 2 must be in slot 2 or greater\n");
	}
}

void a2bus_mcms2_device::device_reset()
{
	m_card1 = static_cast<a2bus_mcms1_device *>(m_a2bus->m_device_list[m_slot-1]);
	m_engine = m_card1->get_engine();
}

// here to soak up false reads from indexed accesses
uint8_t a2bus_mcms2_device::read_c0nx(address_space &space, uint8_t offset)
{
	return 0xff;
}

// write once to c0n0 to disable the card (reset also disables)
// write twice to c0n1 to enable the card (value doesn't matter)
void a2bus_mcms2_device::write_c0nx(address_space &space, uint8_t offset, uint8_t data)
{
	if (offset == 0)
	{
		m_engine->control_w(space, CTRL_DMA, 0);
	}
	else if (offset == 1)
	{
		m_engine->control_w(space, CTRL_DMA, 1);
	}
}

void a2bus_mcms2_device::write_cnxx(address_space &space, uint8_t offset, uint8_t data)
{
	m_engine->voiceregs_w(space, offset, data);
}


/*
    Sound device implementation
*/

mcms_device::mcms_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, MCMS, tag, owner, clock),
	device_sound_interface(mconfig, *this),
	m_write_irq(*this), m_stream(nullptr), m_timer(nullptr), m_clrtimer(nullptr), m_pBusDevice(nullptr), m_enabled(false), m_mastervol(0), m_rand(0)
{
}

void mcms_device::device_start()
{
	m_write_irq.resolve();
	m_stream = machine().sound().stream_alloc(*this, 0, 2, 31250);
	m_timer = timer_alloc(0, nullptr);
	m_clrtimer = timer_alloc(1, nullptr);
	m_enabled = false;
	memset(m_vols, 0, sizeof(m_vols));
	memset(m_table, 0, sizeof(m_table));
	memset(m_freq, 0, sizeof(m_freq));
	memset(m_acc, 0, sizeof(m_acc));

	// the card detect programs volumes and wavetable page but not freq and expects the accumulator to increment
	for (auto & elem : m_freq)
	{
		elem = 0x0040;
	}

	save_item(NAME(m_enabled));
	save_item(NAME(m_vols));
	save_item(NAME(m_table));
	save_item(NAME(m_freq));
	save_item(NAME(m_acc));
	save_item(NAME(m_mastervol));
	save_item(NAME(m_rand));
}

void mcms_device::device_reset()
{
	m_write_irq(CLEAR_LINE);
	m_timer->adjust(attotime::never);
	m_clrtimer->adjust(attotime::never);
	m_enabled = false;
}

void mcms_device::device_timer(emu_timer &timer, device_timer_id tid, int param, void *ptr)
{
	if (tid == 0)
	{
		m_write_irq(ASSERT_LINE);
		// clear this IRQ in 10 cycles (?)
		m_clrtimer->adjust(attotime::from_usec(10), 0);
	}
	else if (tid == 1)
	{
		m_write_irq(CLEAR_LINE);
	}
}

void mcms_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	stream_sample_t *outL, *outR;
	int i, v;
	uint16_t wptr;
	int8_t sample;
	int32_t mixL, mixR;

	outL = outputs[1];
	outR = outputs[0];

	if (m_enabled)
	{
		for (i = 0; i < samples; i++)
		{
			mixL = mixR = 0;

			for (v = 0; v < 16; v++)
			{
				m_acc[v] += m_freq[v];
				wptr = (m_table[v]<<8) | (m_acc[v]>>8);
				m_rand = (m_acc[v]>>8) & 0x1f;

				sample = (m_pBusDevice->slot_dma_read_no_space(wptr) ^ 0x80);
				if (v & 1)
				{
					mixL += sample * m_vols[v];
				}
				else
				{
					mixR += sample * m_vols[v];
				}
			}

			outL[i] = (mixL * m_mastervol)>>9;
			outR[i] = (mixR * m_mastervol)>>9;
		}
	}
	else
	{
		for (i = 0; i < samples; i++)
		{
			outL[i] = outR[i] = 0;
		}
	}
}

WRITE8_MEMBER(mcms_device::voiceregs_w)
{
	m_stream->update();
	if (offset >= 0x20)
	{
		if (offset & 1) // amp
		{
			m_vols[(offset-0x21)/2] = data;
		}
		else    // wavetable page
		{
			m_table[(offset-0x20)/2] = data;
		}
	}
	else
	{
		if (offset & 1) // freq L
		{
			if (offset == 0x1f)
			{
				m_freq[0] &= 0xff00;
				m_freq[0] |= data;
			}
			else
			{
				int reg = (offset/2)+1;
				m_freq[reg] &= 0xff00;
				m_freq[reg] |= data;
			}
		}
		else    // freq H
		{
			int reg = (offset/2);
			m_freq[reg] &= 0x00ff;
			m_freq[reg] |= (data<<8);
		}
	}
}

WRITE8_MEMBER(mcms_device::control_w)
{
	m_stream->update();

	switch (offset)
	{
		case CTRL_IRQS:
			if (data == 0)
			{
				m_timer->adjust(attotime::never);
			}
			else
			{
				m_timer->adjust(attotime::zero, 0, attotime::from_hz(125));
			}
			break;

		case CTRL_DMA:
			m_enabled = (data == 0) ? false : true;
			break;

		case CTRL_MASTERVOL:
			m_mastervol = data;
			break;
	}
}
