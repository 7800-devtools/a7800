// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/***************************************************************************

    IQ151 STAPER (STAndard PERipheral) module emulation

    STAPER module includes cables for connect:
    - a printer (CONSUL 2112 or 2113)
    - a paper tape puncher (DT-105S)
    - a paper tape reader (FS-1503)

    Currently only the printer is emulated

***************************************************************************/

#include "emu.h"
#include "staper.h"


/***************************************************************************
    IMPLEMENTATION
***************************************************************************/


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(IQ151_STAPER, iq151_staper_device, "iq151_staper", "IQ151 STAPER")

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  iq151_staper_device - constructor
//-------------------------------------------------

iq151_staper_device::iq151_staper_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, IQ151_STAPER, tag, owner, clock)
	, device_iq151cart_interface(mconfig, *this)
	, m_ppi(*this, "ppi8255")
	, m_printer(*this, "printer")
	, m_printer_timer(nullptr)
	, m_ppi_portc(0)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void iq151_staper_device::device_start()
{
	m_printer_timer = timer_alloc(TIMER_PRINTER);
	m_printer_timer->reset();
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( iq151_staper_device::device_add_mconfig )
	MCFG_DEVICE_ADD("ppi8255", I8255A, 0)
	MCFG_I8255_IN_PORTA_CB(READ8(iq151_staper_device, ppi_porta_r))
	MCFG_I8255_OUT_PORTB_CB(WRITE8(iq151_staper_device, ppi_portb_w))
	MCFG_I8255_OUT_PORTC_CB(WRITE8(iq151_staper_device, ppi_portc_w))

	MCFG_DEVICE_ADD("printer", PRINTER, 0)
MACHINE_CONFIG_END

//-------------------------------------------------
//  device_timer - handler timer events
//-------------------------------------------------

void iq151_staper_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	if (id == TIMER_PRINTER)
		m_ppi->pc2_w(0);
}


//-------------------------------------------------
//  IO read
//-------------------------------------------------

void iq151_staper_device::io_read(offs_t offset, uint8_t &data)
{
	address_space& space = machine().device("maincpu")->memory().space(AS_IO);

	if (offset >= 0xf8 && offset < 0xfc)
		data = m_ppi->read(space, offset & 0x03);
}

//-------------------------------------------------
//  IO write
//-------------------------------------------------

void iq151_staper_device::io_write(offs_t offset, uint8_t data)
{
	address_space& space = machine().device("maincpu")->memory().space(AS_IO);

	if (offset >= 0xf8 && offset < 0xfc)
		m_ppi->write(space, offset & 0x03, data);
}


//**************************************************************************
//  I8255  interface
//**************************************************************************

READ8_MEMBER( iq151_staper_device::ppi_porta_r )
{
	// TODO: paper tape reader input
	return 0;
}

WRITE8_MEMBER( iq151_staper_device::ppi_portb_w )
{
	if (m_ppi_portc & 0x80)
	{
		// printer out
		m_printer->output(data);

		// CONSUL 2112/3 usually print 65/70 cps
		m_printer_timer->adjust(attotime::from_msec(15));
	}
	if (m_ppi_portc & 0x40)
	{
		// TODO: paper tape puncher out
	}
}

WRITE8_MEMBER( iq151_staper_device::ppi_portc_w )
{
	/*
	    x--- ----   printer select
	    -x-- ----   punchtape select
	*/

	m_ppi_portc = data;
}
