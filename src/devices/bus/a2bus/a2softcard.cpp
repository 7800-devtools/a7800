// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*********************************************************************

    a2softcard.c

    Implementation of the Microsoft SoftCard Z-80 card

*********************************************************************/

#include "emu.h"
#include "a2softcard.h"
#include "cpu/z80/z80.h"

/***************************************************************************
    PARAMETERS
***************************************************************************/

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(A2BUS_SOFTCARD, a2bus_softcard_device, "a2softcard", "Microsoft SoftCard")

#define Z80_TAG         "z80"

static ADDRESS_MAP_START( z80_mem, AS_PROGRAM, 8, a2bus_softcard_device )
	AM_RANGE(0x0000, 0xffff) AM_READWRITE(dma_r, dma_w)
ADDRESS_MAP_END

/***************************************************************************
    FUNCTION PROTOTYPES
***************************************************************************/

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( a2bus_softcard_device::device_add_mconfig )
	MCFG_CPU_ADD(Z80_TAG, Z80, 1021800*2)   // Z80 runs on double the Apple II's clock
	MCFG_CPU_PROGRAM_MAP(z80_mem)
MACHINE_CONFIG_END

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

a2bus_softcard_device::a2bus_softcard_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_a2bus_card_interface(mconfig, *this),
	m_z80(*this, Z80_TAG), m_bEnabled(false), m_FirstZ80Boot(false)
{
}

a2bus_softcard_device::a2bus_softcard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	a2bus_softcard_device(mconfig, A2BUS_SOFTCARD, tag, owner, clock)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void a2bus_softcard_device::device_start()
{
	// set_a2bus_device makes m_slot valid
	set_a2bus_device();

	save_item(NAME(m_bEnabled));
	save_item(NAME(m_FirstZ80Boot));
}

void a2bus_softcard_device::device_reset()
{
	m_bEnabled = false;

	m_FirstZ80Boot = true;
	m_z80->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
}

void a2bus_softcard_device::write_cnxx(address_space &space, uint8_t offset, uint8_t data)
{
	if (!m_bEnabled)
	{
		m_z80->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
		set_maincpu_halt(ASSERT_LINE);

		if (m_FirstZ80Boot)
		{
			m_FirstZ80Boot = false;
			m_z80->reset();
		}

		m_bEnabled = true;
	}
	else
	{
		m_z80->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
		set_maincpu_halt(CLEAR_LINE);
		m_bEnabled = false;
	}
}

READ8_MEMBER( a2bus_softcard_device::dma_r )
{
	if (m_bEnabled)
	{
		if (offset <= 0xafff)
		{
			return slot_dma_read(space, offset+0x1000);
		}
		else if (offset <= 0xbfff)  // LC bank 2 d000-dfff
		{
			return slot_dma_read(space, (offset&0xfff) + 0xd000);
		}
		else if (offset <= 0xcfff)  // LC e000-efff
		{
			return slot_dma_read(space, (offset&0xfff) + 0xe000);
		}
		else if (offset <= 0xdfff)  // LC f000-ffff (or ROM?)
		{
			return slot_dma_read(space, (offset&0xfff) + 0xf000);
		}
		else if (offset <= 0xefff)  // I/O space c000-cfff
		{
			return slot_dma_read(space, (offset&0xfff) + 0xc000);
		}
		else    // zero page
		{
			return slot_dma_read(space, offset&0xfff);
		}
	}

	return 0xff;
}


//-------------------------------------------------
//  dma_w -
//-------------------------------------------------

WRITE8_MEMBER( a2bus_softcard_device::dma_w )
{
	if (m_bEnabled)
	{
		if (offset <= 0xafff)
		{
			slot_dma_write(space, offset+0x1000, data);
		}
		else if (offset <= 0xbfff)  // LC bank 2 d000-dfff
		{
			slot_dma_write(space, (offset&0xfff) + 0xd000, data);
		}
		else if (offset <= 0xcfff)  // LC e000-efff
		{
			slot_dma_write(space, (offset&0xfff) + 0xe000, data);
		}
		else if (offset <= 0xdfff)  // LC f000-ffff (or ROM?)
		{
			slot_dma_write(space, (offset&0xfff) + 0xf000, data);
		}
		else if (offset <= 0xefff)  // I/O space c000-cfff
		{
			slot_dma_write(space, (offset&0xfff) + 0xc000, data);
		}
		else    // zero page
		{
			slot_dma_write(space, offset&0xfff, data);
		}
	}
}

bool a2bus_softcard_device::take_c800()
{
	return false;
}
