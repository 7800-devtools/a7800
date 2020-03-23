// license:LGPL-2.1+
// copyright-holders:Michael Zapf
/****************************************************************************
     I/O port

     This is the connector on the right side of the TI-99/4(A) console and the
     TI-99/8 console.

                                    XXXXXXXXXXXXXXXXXXX
                         XXXXXXXXXXXXXX+__________+XXXX  rear
       front       XXXXXXXXXXXXXXXXXXXX+          +XXXX
                XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

     In previous MAME/MESS releases, the I/O port was not explicitly modeled,
     making the Peripheral Expansion Box (PEB) a subdevice of the console.
     With this change it becomes possible to emulate the side-car devices and
     also game cartriges that were plugged into the I/O port.

     However, this also means the PEB must be plugged into the I/O port
     ("-ioport peb"), and that all PEB slot cards require an "ioport"
     qualifier at the beginning like "-ioport:peb:slot8 hfdc".

     -----

     The TI-99/8 features a different layout with more lines, requiring
     a specific adapter for connecting the Peripheral Expansion Box, called
     the "Armadillo round cable interface". Despite offering this connection,
     TI promoted a different peripheral concept by the "Hexbus". The I/O port
     can be considered a support for legacy peripheral devices.

     Since the TI-99/8 development was stopped with TI's withdrawal from the
     home computer market, no specific peripheral cards were made available.
     The legacy cards of the TI-99/4(A) can be used with limitations. However,
     memory expansion cards do not work. Since the operating system inside the
     99/8 console already assumes that the Hexbus devices are used, it fails
     to correctly operate the floppy controllers, and thus Extended Basic II
     locks up on startup.

     The lines of the 99/8 I/O port starting with P refer to the external bus
     after the AMIGO custom chip, which translates the logical addresses
     to physical addresses (hence the P). There was a first layout (v1) that
     was changed in the last prototypes (v2).

            TI-99/4(A)                        TI-99/8 (v2)

        bot   Front   top                  top   Rear    bot
      ---------------------              ---------------------
        +5V   1||2    SBE              +5V/XBC   2||1   EXTINT*
      RESET*  3||4    EXTINT*               D2   4||3   AUDIOIN
         A5   5||6    A10                   D1   6||5   PA09
         A4   7||8    A11                   D3   8||7   D0
       DBIN   9||10   A3                    D5  10||9   D4
        A12  11||12   READY                 D7  12||11  D6
       LOAD* 13||14   A8                SRESET* 14||13  GND
        A13  15||16   A14                MSAST* 16||15  GND
         A7  17||18   A9                CRUCLK* 18||17  GND
 A15/CRUOUT  19||20   A2                  HOLD* 20||19  GND
        GND  21||22   CRUCLK*             PA11  22||21  PA12
        GND  23||24   PHI3*        PA15/CRUOUT  24||23  nc
        GND  25||26   WE*                 SCLK  26||25  GND
        GND  27||28   MBE*                PA14  28||27  PA01
         A6  29||30   A1                  PA08  30||29  PA13
         A0  31||32   MEMEN*              PA00  32||31  PA05
      CRUIN  33||34   D7                  PA10  34||33  PA04
         D4  35||36   D6                  SRDY  36||35  CRUIN
         D0  37||38   D5                  PA06  38||37  PA07
         D2  39||40   D1                  PA03  40||39  PA02
        IAQ  41||42   D3                 PDBIN  42||41  IAQ/HOLDA
        -5V  43||44   AUDIOIN             LOAD* 44||43  GND
      ---------------------               PHI3* 46||45  GND
        bot   Rear   top                PMEMEN* 48||47  GND
                                           PWE* 50||49  GND
                                       -----------------------
                                          top   Front   bot
      May 2017, Michael Zapf

*****************************************************************************/

#include "emu.h"
#include "ioport.h"
#include "bus/ti99/peb/peribox.h"

DEFINE_DEVICE_TYPE_NS(TI99_IOPORT, bus::ti99::internal, ioport_device, "ti99_ioport", "TI-99 I/O Port")

namespace bus { namespace ti99 { namespace internal {

ioport_device::ioport_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	:   device_t(mconfig, TI99_IOPORT, tag, owner, clock),
		device_slot_interface(mconfig, *this),
		m_console_extint(*this),
		m_console_ready(*this),
		m_connected(nullptr)
{
}

READ8Z_MEMBER(ioport_device::readz)
{
	if (m_connected != nullptr)
		m_connected->readz(space, offset, value);
}

WRITE8_MEMBER(ioport_device::write)
{
	if (m_connected != nullptr)
		m_connected->write(space, offset, data);
}

SETADDRESS_DBIN_MEMBER(ioport_device::setaddress_dbin)
{
	if (m_connected != nullptr)
		m_connected->setaddress_dbin(space, offset, state);
}

READ8Z_MEMBER(ioport_device::crureadz)
{
	if (m_connected != nullptr)
		m_connected->crureadz(space, offset, value);
}

WRITE8_MEMBER(ioport_device::cruwrite)
{
	if (m_connected != nullptr)
		m_connected->cruwrite(space, offset, data);
}

WRITE_LINE_MEMBER(ioport_device::memen_in)
{
	if (m_connected != nullptr)
		m_connected->memen_in(state);
}

WRITE_LINE_MEMBER(ioport_device::msast_in)
{
	if (m_connected != nullptr)
		m_connected->msast_in(state);
}

WRITE_LINE_MEMBER(ioport_device::clock_in)
{
	if (m_connected != nullptr)
		m_connected->clock_in(state);
}

void ioport_device::device_start()
{
	m_console_extint.resolve();
	m_console_ready.resolve();
	if (m_connected != nullptr)
		m_connected->set_ioport(this);
}

void ioport_device::device_config_complete()
{
	m_connected = static_cast<ioport_attached_device*>(subdevices().first());
}


WRITE_LINE_MEMBER(ioport_attached_device::set_extint)
{
	m_ioport->m_console_extint(state);
}

WRITE_LINE_MEMBER(ioport_attached_device::set_ready)
{
	m_ioport->m_console_ready(state);
}

}   }   } // end namespace bus::ti99::internal

SLOT_INTERFACE_START( ti99_io_port )
	SLOT_INTERFACE("peb", TI99_PERIBOX)
SLOT_INTERFACE_END

SLOT_INTERFACE_START( ti99_io_port_ev )
	SLOT_INTERFACE("peb", TI99_PERIBOX_EV)
SLOT_INTERFACE_END

