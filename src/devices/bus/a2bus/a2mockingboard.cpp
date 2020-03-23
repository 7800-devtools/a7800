// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*********************************************************************

    a2mockingboard.c

    Implementation of the Sweet Micro Systems Mockingboard card
    and friends.

*********************************************************************/

#include "emu.h"
#include "a2mockingboard.h"
#include "speaker.h"


/***************************************************************************
    PARAMETERS
***************************************************************************/

#define VIA1_TAG "mockbd_via1"
#define VIA2_TAG "mockbd_via2"
#define AY1_TAG "mockbd_ay1"
#define AY2_TAG "mockbd_ay2"
#define AY3_TAG "mockbd_ay3"
#define AY4_TAG "mockbd_ay4"
#define E2P_TMS_TAG "tms5220"

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(A2BUS_MOCKINGBOARD, a2bus_mockingboard_device, "a2mockbd", "Sweet Micro Systems Mockingboard")
DEFINE_DEVICE_TYPE(A2BUS_PHASOR,       a2bus_phasor_device,       "a2phasor", "Applied Engineering Phasor")
DEFINE_DEVICE_TYPE(A2BUS_ECHOPLUS,     a2bus_echoplus_device,     "a2echop",  "Street Electronics Echo Plus")

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( a2bus_ayboard_device::device_add_mconfig )
	MCFG_DEVICE_ADD(VIA1_TAG, VIA6522, 1022727)
	MCFG_VIA6522_WRITEPA_HANDLER(WRITE8(a2bus_ayboard_device, via1_out_a))
	MCFG_VIA6522_WRITEPB_HANDLER(WRITE8(a2bus_ayboard_device, via1_out_b))
	MCFG_VIA6522_IRQ_HANDLER(WRITELINE(a2bus_ayboard_device, via1_irq_w))

	MCFG_DEVICE_ADD(VIA2_TAG, VIA6522, 1022727)
	MCFG_VIA6522_WRITEPA_HANDLER(WRITE8(a2bus_ayboard_device, via2_out_a))
	MCFG_VIA6522_WRITEPB_HANDLER(WRITE8(a2bus_ayboard_device, via2_out_b))
	MCFG_VIA6522_IRQ_HANDLER(WRITELINE(a2bus_ayboard_device, via2_irq_w))

	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")
	MCFG_SOUND_ADD(AY1_TAG, AY8913, 1022727)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 1.0)
	MCFG_SOUND_ADD(AY2_TAG, AY8913, 1022727)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 1.0)
MACHINE_CONFIG_END

MACHINE_CONFIG_MEMBER( a2bus_phasor_device::device_add_mconfig )
	MCFG_DEVICE_ADD(VIA1_TAG, VIA6522, 1022727)
	MCFG_VIA6522_WRITEPA_HANDLER(WRITE8(a2bus_ayboard_device, via1_out_a))
	MCFG_VIA6522_WRITEPB_HANDLER(WRITE8(a2bus_ayboard_device, via1_out_b))
	MCFG_VIA6522_IRQ_HANDLER(WRITELINE(a2bus_ayboard_device, via1_irq_w))

	MCFG_DEVICE_ADD(VIA2_TAG, VIA6522, 1022727)
	MCFG_VIA6522_WRITEPA_HANDLER(WRITE8(a2bus_ayboard_device, via2_out_a))
	MCFG_VIA6522_WRITEPB_HANDLER(WRITE8(a2bus_ayboard_device, via2_out_b))
	MCFG_VIA6522_IRQ_HANDLER(WRITELINE(a2bus_ayboard_device, via2_irq_w))

	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")
	MCFG_SPEAKER_STANDARD_STEREO("lspeaker2", "rspeaker2")
	MCFG_SOUND_ADD(AY1_TAG, AY8913, 1022727)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 1.0)
	MCFG_SOUND_ADD(AY2_TAG, AY8913, 1022727)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker2", 1.0)
	MCFG_SOUND_ADD(AY3_TAG, AY8913, 1022727)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 1.0)
	MCFG_SOUND_ADD(AY4_TAG, AY8913, 1022727)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker2", 1.0)
MACHINE_CONFIG_END

MACHINE_CONFIG_MEMBER( a2bus_echoplus_device::device_add_mconfig )
	MCFG_DEVICE_ADD(VIA1_TAG, VIA6522, 1022727)
	MCFG_VIA6522_WRITEPA_HANDLER(WRITE8(a2bus_ayboard_device, via1_out_a))
	MCFG_VIA6522_WRITEPB_HANDLER(WRITE8(a2bus_ayboard_device, via1_out_b))
	MCFG_VIA6522_IRQ_HANDLER(WRITELINE(a2bus_ayboard_device, via1_irq_w))

	MCFG_DEVICE_ADD(VIA2_TAG, VIA6522, 1022727)
	MCFG_VIA6522_WRITEPA_HANDLER(WRITE8(a2bus_ayboard_device, via2_out_a))
	MCFG_VIA6522_WRITEPB_HANDLER(WRITE8(a2bus_ayboard_device, via2_out_b))
	MCFG_VIA6522_IRQ_HANDLER(WRITELINE(a2bus_ayboard_device, via2_irq_w))

	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")
	MCFG_SOUND_ADD(AY1_TAG, AY8913, 1022727)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 1.0)
	MCFG_SOUND_ADD(AY2_TAG, AY8913, 1022727)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 1.0)

	MCFG_SPEAKER_STANDARD_MONO("echosp")
	MCFG_SOUND_ADD(E2P_TMS_TAG, TMS5220, 640000)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "echosp", 1.0)
MACHINE_CONFIG_END

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

a2bus_ayboard_device::a2bus_ayboard_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_a2bus_card_interface(mconfig, *this),
	m_via1(*this, VIA1_TAG),
	m_via2(*this, VIA2_TAG),
	m_ay1(*this, AY1_TAG),
	m_ay2(*this, AY2_TAG),
	m_ay3(*this, AY3_TAG),
	m_ay4(*this, AY4_TAG), m_isPhasor(false), m_PhasorNative(false), m_porta1(0), m_porta2(0)
{
}

a2bus_mockingboard_device::a2bus_mockingboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	a2bus_ayboard_device(mconfig, A2BUS_MOCKINGBOARD, tag, owner, clock)
{
	m_isPhasor = false;
	m_PhasorNative = false;
}

a2bus_phasor_device::a2bus_phasor_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	a2bus_ayboard_device(mconfig, A2BUS_PHASOR, tag, owner, clock)
{
	m_isPhasor = true;
	m_PhasorNative = false;
}

a2bus_echoplus_device::a2bus_echoplus_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	a2bus_ayboard_device(mconfig, A2BUS_ECHOPLUS, tag, owner, clock),
	m_tms(*this, E2P_TMS_TAG)
{
	m_isPhasor = false;
	m_PhasorNative = false;
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void a2bus_ayboard_device::device_start()
{
	// set_a2bus_device makes m_slot valid
	set_a2bus_device();

	save_item(NAME(m_porta1));
	save_item(NAME(m_porta2));
}

void a2bus_ayboard_device::device_reset()
{
	m_porta1 = m_porta2 = 0;
}

/*-------------------------------------------------
    read_cnxx - called for reads from this card's cnxx space
-------------------------------------------------*/

uint8_t a2bus_ayboard_device::read_cnxx(address_space &space, uint8_t offset)
{
//    printf("Mockingboard(%d): read @ Cn%02X (PC=%x)\n", m_slot, offset, space.device().safe_pc());
	if (m_isPhasor)
	{
		uint8_t retVal = 0;
		int viaSel;

		if (m_PhasorNative)
		{
			viaSel = ((offset & 0x80)>> 6) | ((offset & 0x10)>> 4);
		}
		else
		{
			viaSel = (offset & 0x80) ? 2 : 1;
		}

		if ((offset <= 0x20) || (offset >= 0x80 && offset <= 0xa0))
		{
			if (viaSel & 1)
			{
				retVal |= m_via1->read(space, offset & 0xf);
			}

			if (viaSel & 2)
			{
				retVal |=  m_via2->read(space, offset & 0xf);
			}
		}

		return retVal;
	}
	else
	{
		if (offset <= 0x10)
		{
			return m_via1->read(space, offset & 0xf);
		}
		else if (offset >= 0x80 && offset <= 0x90)
		{
			return m_via2->read(space, offset & 0xf);
		}
	}

	return 0;
}

/*-------------------------------------------------
    write_cnxx - called for writes to this card's c0nx space
-------------------------------------------------*/

void a2bus_ayboard_device::write_cnxx(address_space &space, uint8_t offset, uint8_t data)
{
	if (m_isPhasor)
	{
		if ((offset <= 0x20) || (offset >= 0x80 && offset <= 0xa0))
		{
			int viaSel;

			if (m_PhasorNative)
			{
				viaSel = ((offset & 0x80)>> 6) | ((offset & 0x10)>> 4);
			}
			else
			{
				viaSel = (offset & 0x80) ? 2 : 1;
			}

//            printf("Phasor(%d): write %02x to Cn%02X (PC=%x) (native %d viaSel %d)\n", m_slot, data, offset, space.device().safe_pc(), m_PhasorNative ? 1 : 0, viaSel);

			if (viaSel & 1)
			{
				m_via1->write(space, offset&0xf, data);
			}
			if (viaSel & 2)
			{
				m_via2->write(space, offset&0xf, data);
			}
		}
	}
	else
	{
		if (offset <= 0x10)
		{
			m_via1->write(space, offset & 0xf, data);
		}
		else if (offset >= 0x80 && offset <= 0x90)
		{
			m_via2->write(space, offset & 0xf, data);
		}
		else
		{
			printf("Mockingboard(%d): unk write %02x to Cn%02X (PC=%x)\n", m_slot, data, offset, space.device().safe_pc());
		}
	}
}


WRITE_LINE_MEMBER( a2bus_ayboard_device::via1_irq_w )
{
	if (state)
	{
		raise_slot_irq();
	}
	else
	{
		lower_slot_irq();
	}
}

WRITE_LINE_MEMBER( a2bus_ayboard_device::via2_irq_w )
{
	if (state)
	{
		raise_slot_irq();
	}
	else
	{
		lower_slot_irq();
	}
}

WRITE8_MEMBER( a2bus_ayboard_device::via1_out_a )
{
	m_porta1 = data;
}

WRITE8_MEMBER( a2bus_ayboard_device::via1_out_b )
{
	if (!(data & 4))
	{
		m_ay1->reset_w(space, 0, 0);
		if (m_isPhasor && m_PhasorNative)
		{
			m_ay2->reset_w(space, 0, 0);
		}
	}
	else
	{
		if (!m_isPhasor)
		{
			switch (data & 3)
			{
				case 0: // BDIR=0, BC1=0 (inactive)
					break;

				case 1: // BDIR=0, BC1=1 (read PSG)
					m_porta1 = m_ay1->data_r(space, 0);
					break;

				case 2: // BDIR=1, BC1=0 (write PSG)
					m_ay1->data_w(space, 0, m_porta1);
					break;

				case 3: // BDIR=1, BC1=1 (latch)
					m_ay1->address_w(space, 0, m_porta1);
					break;
			}
		}
		else
		{
			int chipSel;

			if (m_PhasorNative)
			{
				chipSel = (~(data >> 3) & 3);
			}
			else
			{
				chipSel = 1;
			}

//            printf("Phasor: %02x to AY1/2 CS %02x (BDIR/BC1 %02x, data %02x)\n", m_porta1, chipSel, data & 3, data);
			switch (data & 3)
			{
				case 0: // BDIR=0, BC1=0 (inactive)
					break;

				case 1: // BDIR=0, BC1=1 (read PSG)
					if (chipSel & 1)
					{
						m_porta1 = m_ay1->data_r(space, 0);
					}
					if (chipSel & 2)
					{
						m_porta1 = m_ay2->data_r(space, 0);
					}
					break;

				case 2: // BDIR=1, BC1=0 (write PSG)
					if (chipSel & 1)
					{
						m_ay1->data_w(space, 0, m_porta1);
					}
					if (chipSel & 2)
					{
						m_ay2->data_w(space, 0, m_porta1);
					}
					break;

				case 3: // BDIR=1, BC1=1 (latch)
					if (chipSel & 1)
					{
						m_ay1->address_w(space, 0, m_porta1);
					}
					if (chipSel & 2)
					{
						m_ay2->address_w(space, 0, m_porta1);
					}
					break;
			}
		}
	}
}

WRITE8_MEMBER( a2bus_ayboard_device::via2_out_a )
{
	m_porta2 = data;
}

WRITE8_MEMBER( a2bus_ayboard_device::via2_out_b )
{
	if (!(data & 4))
	{
		if (m_isPhasor && m_PhasorNative)
		{
			m_ay3->reset_w(space, 0, 0);
			m_ay4->reset_w(space, 0, 0);
		}
		else
		{
			m_ay2->reset_w(space, 0, 0);
		}
	}
	else
	{
		if (!m_isPhasor)
		{
			switch (data & 3)
			{
				case 0: // BDIR=0, BC1=0 (inactive)
					break;

				case 1: // BDIR=0, BC1=1 (read PSG)
					m_porta2 = m_ay2->data_r(space, 0);
					break;

				case 2: // BDIR=1, BC1=0 (write PSG)
					m_ay2->data_w(space, 0, m_porta2);
					break;

				case 3: // BDIR=1, BC1=1 (latch)
					m_ay2->address_w(space, 0, m_porta2);
					break;
			}
		}
		else
		{
			int chipSel;

			if (m_PhasorNative)
			{
				chipSel = (~(data >> 3) & 3);
			}
			else
			{
				chipSel = 1;
			}

//            printf("Phasor: %02x to AY3/4 CS %02x (BDIR/BC1 %02x, data %02x)\n", m_porta2, chipSel, data & 3, data);
			switch (data & 3)
			{
				case 0: // BDIR=0, BC1=0 (inactive)
					break;

				case 1: // BDIR=0, BC1=1 (read PSG)
					if (chipSel & 1)
					{
						m_porta2 = m_ay3->data_r(space, 0);
					}
					if (chipSel & 2)
					{
						m_porta2 = m_ay4->data_r(space, 0);
					}
					break;

				case 2: // BDIR=1, BC1=0 (write PSG)
					if (chipSel & 1)
					{
						m_ay3->data_w(space, 0, m_porta2);
					}
					if (chipSel & 2)
					{
						m_ay4->data_w(space, 0, m_porta2);
					}
					break;

				case 3: // BDIR=1, BC1=1 (latch)
					if (chipSel & 1)
					{
						m_ay3->address_w(space, 0, m_porta2);
					}
					if (chipSel & 2)
					{
						m_ay4->address_w(space, 0, m_porta2);
					}
					break;
			}
		}
	}
}

uint8_t a2bus_ayboard_device::read_c0nx(address_space &space, uint8_t offset)
{
	if (m_isPhasor)
	{
		m_PhasorNative = (offset & 1) ? true : false;
	}

	return 0xff;
}

void a2bus_ayboard_device::write_c0nx(address_space &space, uint8_t offset, uint8_t data)
{
	if (m_isPhasor)
	{
		m_PhasorNative = (offset & 1) ? true : false;
	}
}

uint8_t a2bus_echoplus_device::read_c0nx(address_space &space, uint8_t offset)
{
	switch (offset)
	{
		case 0:
			return 0x1f | m_tms->status_r(space, 0);
	}

	return 0;
}

void a2bus_echoplus_device::write_c0nx(address_space &space, uint8_t offset, uint8_t data)
{
	switch (offset)
	{
		case 0:
			m_tms->data_w(space, offset, data);
			break;
	}
}
