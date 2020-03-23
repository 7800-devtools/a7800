// license:LGPL-2.1+
// copyright-holders:Michael Zapf
/***************************************************************************

    Geneve "Memex" memory expansion
    may be used together with the GenMod feature to expand the memory to the
    full 2 MiB range.

    Michael Zapf
    February 2011
    February 2012: rewritten as class

****************************************************************************/

#include "emu.h"
#include "memex.h"

DEFINE_DEVICE_TYPE_NS(TI99_MEMEX, bus::ti99::peb, geneve_memex_device, "ti99_memex", "Geneve memory expansion card")

namespace bus { namespace ti99 { namespace peb {

#define RAMREGION "ram2meg"
#define TRACE_CONFIG 0

enum
{
	MDIP1 = 0x01,
	MDIP2 = 0x02,
	MDIP3 = 0x04,
	MDIP4 = 0x08,
	MDIP5 = 0x10,
	MDIP6 = 0x20,
	MDIP7 = 0x40,
	MDIP8 = 0x80
};

geneve_memex_device::geneve_memex_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, TI99_MEMEX, tag, owner, clock),
	device_ti99_peribox_card_interface(mconfig, *this),
	m_ram(*this, RAMREGION)
{
}

bool geneve_memex_device::access_enabled(offs_t offset)
{
	// 1 0111 .... .... .... .... p-box address block 0xxx ... fxxx
	// first two bits are AME, AMD bits available on Genmod only
	// if AMD, AME are not available we assume AMD=0, AME=1
	// must be set on the Geneve board
	// Some traditional cards will not decode the AMx lines, so
	// we may have to lock out those areas
	int page = (offset >> 13)&0xff;

	// SW2: 10xxx010   locked when SW2=off
	//      10111010   locked when SW2=on
	if (page == 0xba) return false;
	if ((page & 0xc7)==0x82 && ((m_switches & MDIP2)==0))
	{
		if (TRACE_CONFIG) logerror("memex blocks page %02x; dip2=%d\n", page,  (m_switches & MDIP2)!=0);
		return false;
	}

	// Switch  page
	// SW3:    111010xx    enabled for SWx=0,blocked for SWx=1
	// SW4:    111011xx
	// SW5:    111100xx
	// SW6:    111101xx
	// SW7:    111110xx
	// SW8:    111111xx

	if (page >= 0xe8 && page <= 0xff)
	{
		return ((m_switches & (4<< (((page>>2) & 0x0f)-10))) == 0);
	}
	return true;
}

/*
    Memory read. The memory is at locations 0x000000-0x1fffff. Some of these
    regions are hidden by onboard devices of the Geneve. We must block some
    areas which would otherwise interfere with peripheral cards.

    Note that the incomplete decoding of the standard Geneve must be
    considered.
*/
READ8Z_MEMBER( geneve_memex_device::readz )
{
	/* If not Genmod, add the upper two address bits 10 */
	if (!m_genmod) offset |= 0x100000;

	// The card is accessed for all addresses in the address space
	if (access_enabled(offset))
		*value = m_ram->pointer()[offset];
}

/*
    Memory write
*/
WRITE8_MEMBER( geneve_memex_device::write )
{
	/* If not Genmod, add the upper two address bits 10 */
	if (!m_genmod) offset |= 0x100000;

	// The card is accessed for all addresses in the address space
	if (access_enabled(offset))
		m_ram->pointer()[offset] = data;
}

/**************************************************************************/

void geneve_memex_device::device_start()
{
	save_item(NAME(m_switches));
}

void geneve_memex_device::device_reset()
{
	m_switches = ioport("MEMEXDIPS")->read();
	if (TRACE_CONFIG) logerror("memex dips = %02x\n", m_switches);
}

INPUT_PORTS_START( memex )
	PORT_START( "MEMEXDIPS" )
	PORT_DIPNAME( MDIP1, MDIP1, "MEMEX SW1" )
		PORT_DIPSETTING( 0x00, "LED half-bright for 0 WS")
		PORT_DIPSETTING( MDIP1, "LED full-bright")
	PORT_DIPNAME( MDIP2, 0x00, "MEMEX SW2" )
		PORT_DIPSETTING( 0x00, "Lock out all BA mirrors")
		PORT_DIPSETTING( MDIP2, "Lock out page BA only")
	PORT_DIPNAME( MDIP3, 0x00, "MEMEX SW3" )
		PORT_DIPSETTING( 0x00, "Enable pages E8-EB")
		PORT_DIPSETTING( MDIP3, "Lock out pages E8-EB")
	PORT_DIPNAME( MDIP4, 0x00, "MEMEX SW4" )
		PORT_DIPSETTING( 0x00, "Enable pages EC-EF")
		PORT_DIPSETTING( MDIP4, "Lock out pages EC-EF")
	PORT_DIPNAME( MDIP5, 0x00, "MEMEX SW5" )
		PORT_DIPSETTING( 0x00, "Enable pages F0-F3")
		PORT_DIPSETTING( MDIP5, "Lock out pages F0-F3")
	PORT_DIPNAME( MDIP6, 0x00, "MEMEX SW6" )
		PORT_DIPSETTING( 0x00, "Enable pages F4-F7")
		PORT_DIPSETTING( MDIP6, "Lock out pages F4-F7")
	PORT_DIPNAME( MDIP7, 0x00, "MEMEX SW7" )
		PORT_DIPSETTING( 0x00, "Enable pages F8-FB")
		PORT_DIPSETTING( MDIP7, "Lock out pages F8-FB")
	PORT_DIPNAME( MDIP8, 0x00, "MEMEX SW8" )
		PORT_DIPSETTING( 0x00, "Enable pages FC-FF")
		PORT_DIPSETTING( MDIP8, "Lock out pages FC-FF")
INPUT_PORTS_END

MACHINE_CONFIG_MEMBER( geneve_memex_device::device_add_mconfig )
	MCFG_RAM_ADD(RAMREGION)
	MCFG_RAM_DEFAULT_SIZE("2M")
	MCFG_RAM_DEFAULT_VALUE(0)
MACHINE_CONFIG_END

ioport_constructor geneve_memex_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( memex );
}

} } } // end namespace bus::ti99::peb
