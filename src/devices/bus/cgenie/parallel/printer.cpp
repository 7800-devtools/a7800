// license:GPL-2.0+
// copyright-holders:Dirk Best
/***************************************************************************

    EACA Colour Genie Printer Interface EG2012

***************************************************************************/

#include "emu.h"
#include "printer.h"


//**************************************************************************
//  CONSTANTS/MACROS
//**************************************************************************

#define VERBOSE 0


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(CGENIE_PRINTER, cgenie_printer_device, "cgenie_printer", "Printer Interface EG2012")

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( cgenie_printer_device::device_add_mconfig )
	MCFG_CENTRONICS_ADD("centronics", centronics_devices, "printer")
	MCFG_CENTRONICS_BUSY_HANDLER(WRITELINE(cgenie_printer_device, busy_w))
	MCFG_CENTRONICS_PERROR_HANDLER(WRITELINE(cgenie_printer_device, perror_w))
	MCFG_CENTRONICS_SELECT_HANDLER(WRITELINE(cgenie_printer_device, select_w))
	MCFG_CENTRONICS_FAULT_HANDLER(WRITELINE(cgenie_printer_device, fault_w))
	MCFG_CENTRONICS_OUTPUT_LATCH_ADD("latch", "centronics")
MACHINE_CONFIG_END


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  cgenie_printer_device - constructor
//-------------------------------------------------

cgenie_printer_device::cgenie_printer_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, CGENIE_PRINTER, tag, owner, clock),
	device_cg_parallel_interface(mconfig, *this),
	m_centronics(*this, "centronics"),
	m_latch(*this, "latch"),
	m_centronics_busy(0),
	m_centronics_out_of_paper(0),
	m_centronics_unit_sel(1),
	m_centronics_ready(1)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void cgenie_printer_device::device_start()
{
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void cgenie_printer_device::device_reset()
{
}


//**************************************************************************
//  IMPLEMENTATION
//**************************************************************************

WRITE_LINE_MEMBER( cgenie_printer_device::busy_w )
{
	m_centronics_busy = state;
}

WRITE_LINE_MEMBER( cgenie_printer_device::perror_w )
{
	m_centronics_out_of_paper = state;
}

WRITE_LINE_MEMBER( cgenie_printer_device::select_w )
{
	m_centronics_unit_sel = state;
}

WRITE_LINE_MEMBER( cgenie_printer_device::fault_w )
{
	m_centronics_ready = state;
}

void cgenie_printer_device::pa_w(uint8_t data)
{
	if (VERBOSE)
		logerror("%s: pa_w %02x\n", tag(), data);

	m_latch->write(data);
}

uint8_t cgenie_printer_device::pb_r()
{
	uint8_t data = 0x0f;

	data |= m_centronics_ready << 4;
	data |= m_centronics_unit_sel << 5;
	data |= m_centronics_out_of_paper << 6;
	data |= m_centronics_busy << 7;

	return data;
}

void cgenie_printer_device::pb_w(uint8_t data)
{
	if (VERBOSE)
		logerror("%s: pa_w %02x\n", tag(), data);

	m_centronics->write_strobe(BIT(data, 0));
}
