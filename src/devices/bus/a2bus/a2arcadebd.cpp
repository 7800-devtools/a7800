// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*********************************************************************

    a2arcadeboard.c

    Implementation of the Third Millenium Engineering Arcade Board

    TODO:
        - VDPTEST program seems to want more than 16K of RAM, but docs/ads/press releases say 16k, period
        - MLDEMO program needs vsync IRQ from the TMS but doesn't program the registers the way our emulation
          wants to enable IRQs

*********************************************************************/

#include "emu.h"
#include "a2arcadebd.h"
#include "speaker.h"


/***************************************************************************
    PARAMETERS
***************************************************************************/

#define TMS_TAG "arcbd_tms"
#define AY_TAG "arcbd_ay"
#define SCREEN_TAG "screen"

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(A2BUS_ARCADEBOARD, a2bus_arcboard_device, "a2arcbd", "Third Millenium Engineering Arcade Board")

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( a2bus_arcboard_device::device_add_mconfig )
	MCFG_DEVICE_ADD( TMS_TAG, TMS9918A, XTAL_10_738635MHz / 2 )
	MCFG_TMS9928A_VRAM_SIZE(0x4000) // 16k of VRAM
	MCFG_TMS9928A_OUT_INT_LINE_CB(WRITELINE(a2bus_arcboard_device, tms_irq_w))
	MCFG_TMS9928A_SCREEN_ADD_NTSC( SCREEN_TAG )
	MCFG_SCREEN_UPDATE_DEVICE( TMS_TAG, tms9918a_device, screen_update )

	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD(AY_TAG, AY8910, 1022727)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

a2bus_arcboard_device::a2bus_arcboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	a2bus_arcboard_device(mconfig, A2BUS_ARCADEBOARD, tag, owner, clock)
{
}

a2bus_arcboard_device::a2bus_arcboard_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_a2bus_card_interface(mconfig, *this),
	m_tms(*this, TMS_TAG),
	m_ay(*this, AY_TAG)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void a2bus_arcboard_device::device_start()
{
	// set_a2bus_device makes m_slot valid
	set_a2bus_device();
}

void a2bus_arcboard_device::device_reset()
{
}

/*
    C0nx map:
    0 - TMS read vram
    1 - TMS read status
    2 - TMS write vram
    3 - TMS write register
    5 - AY register select
    6 - AY data
*/

uint8_t a2bus_arcboard_device::read_c0nx(address_space &space, uint8_t offset)
{
	switch (offset)
	{
		case 0:
			return m_tms->vram_read(space, 0);

		case 1:
			return m_tms->register_read(space, 0);

		case 6:
			return m_ay->data_r(space, 0);
	}

	return 0xff;
}

void a2bus_arcboard_device::write_c0nx(address_space &space, uint8_t offset, uint8_t data)
{
	switch (offset)
	{
		case 2:
			m_tms->vram_write(space, 0, data);
			break;

		case 3:
			m_tms->register_write(space, 0, data);
			break;

		case 5:
			m_ay->address_w(space, 0, data);
			break;

		case 6:
			m_ay->data_w(space, 0, data);
			break;
	}
}

WRITE_LINE_MEMBER( a2bus_arcboard_device::tms_irq_w )
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
