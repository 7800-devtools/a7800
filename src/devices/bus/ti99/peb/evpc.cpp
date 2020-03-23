// license:LGPL-2.1+
// copyright-holders:Michael Zapf
/****************************************************************************
    SNUG Enhanced Video Processor Card (EVPC)

    This is an expansion card with an own v9938 video processor on board.
    Later releases (EVPC2) can also be equipped with a v9958.

    The EVPC is intended to be used
    1. with the TI-99/4A console
    2. with the SGCPU

    For option 1, the console-internal VDP (TMS9928A) must be removed. This,
    however, raises a problem, because the video interrupt must now be send
    from the EVPC in the Peripheral Expansion Box to the console, but there is
    no line in the PEB cable for this purpose.

    To solve this issue, a separate cable is led from the EVPC to the console
    which delivers the video interrupt, and also the GROM clock which is also
    lost due to the removal of the internal VDP.

    The SGCPU requires this card, as it does not offer any video processor.
    In this configuration, the video interrupt cable is not required.

    Also, the SGCPU does not offer a socket for the sound chip of the TI
    console, and accordingly, the EVPC also gives the sound chip a new home.
    Thus we assume that in the TI console (option 1) the sound chip has
    also been removed.

    Important note: The DSR (firmware) of the EVPC expects a memory expansion
    to be present; otherwise, the configuration (using CALL EVPC) will crash.
    There is no warning if the 32K expansion is not present.

    Michael Zapf

*****************************************************************************/

#include "emu.h"
#include "evpc.h"
#include "speaker.h"

DEFINE_DEVICE_TYPE_NS(TI99_EVPC, bus::ti99::peb, snug_enhanced_video_device, "ti99_evpc", "SNUG Enhanced Video Processor Card")

namespace bus { namespace ti99 { namespace peb {

#define EVPC_CRU_BASE 0x1400

#define TRACE_ADDRESS 0
#define TRACE_CRU 0
#define TRACE_MEM 0

#define NOVRAM_SIZE 256

snug_enhanced_video_device::snug_enhanced_video_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock):
	device_t(mconfig, TI99_EVPC, tag, owner, clock),
	device_ti99_peribox_card_interface(mconfig, *this),
	device_nvram_interface(mconfig, *this),
	m_dsr_page(0),
	m_inDsrArea(false),
	m_novram_accessed(false),
	m_palette_accessed(false),
	m_RAMEN(false),
	m_sound_accessed(false),
	m_video_accessed(false),
	m_intlevel(0),
	m_dsrrom(nullptr),
	m_novram(nullptr),
	m_video(*this, TI_VDP_TAG),
	m_sound(*this, TI_SOUNDCHIP_TAG),
	m_colorbus(*this, COLORBUS_TAG)
{
}

SETADDRESS_DBIN_MEMBER( snug_enhanced_video_device::setaddress_dbin )
{
	// Do not allow setaddress for the debugger. It will mess up the
	// setaddress/memory access pairs when the CPU enters wait states.
	if (machine().side_effect_disabled()) return;

	if (TRACE_ADDRESS) logerror("set address %04x, %s\n", offset, (state==ASSERT_LINE)? "read" : "write");

	m_address = offset;
	bool reading = (state==ASSERT_LINE);
	int offbase = (m_address & 0xfc01);

	// Sound
	m_sound_accessed = ((m_address & 0xff01)==0x8400) && !reading;

	// Video space
	// 8800 / 8802 / 8804 / 8806
	// 8c00 / 8c02 / 8c04 / 8c06
	//
	// Bits 1000 1w00 0000 0xx0
	// Mask 1111 1000 0000 0001
	m_video_accessed = ((offbase==0x8800) && reading) || ((offbase==0x8c00) && !reading);

	// Read a byte in evpc DSR space
	// 0x4000 - 0x5eff   DSR (paged)
	// 0x5f00 - 0x5fef   NOVRAM
	// 0x5ff0 - 0x5fff   Palette
	m_inDsrArea = ((m_address & 0xe000)==0x4000);
	m_novram_accessed = ((m_address & 0xff00)==0x5f00);
	m_palette_accessed = ((m_address & 0xfff0)==0x5ff0);

	// Note that we check the selection in reverse order so that the overlap is avoided
}

//-------------------------------------------------
//  nvram_default - called to initialize NVRAM to
//  its default state
//-------------------------------------------------

void snug_enhanced_video_device::nvram_default()
{
	memset(m_novram.get(), 0, NOVRAM_SIZE);
}

//-------------------------------------------------
//  nvram_read - called to read NVRAM from the
//  .nv file
//-------------------------------------------------

void snug_enhanced_video_device::nvram_read(emu_file &file)
{
	file.read(m_novram.get(), NOVRAM_SIZE);
}

//-------------------------------------------------
//  nvram_write - called to write NVRAM to the
//  .nv file
//-------------------------------------------------

void snug_enhanced_video_device::nvram_write(emu_file &file)
{
	file.write(m_novram.get(), NOVRAM_SIZE);
}

/*
    Read a byte in evpc DSR space, NOVRAM, Palette, or video
    0x4000 - 0x5eff   DSR (paged)
    0x5f00 - 0x5fef   NOVRAM
    0x5ff0 - 0x5fff   Palette (5ff0, 5ff2, 5ff4, 5ff6)
*/
READ8Z_MEMBER(snug_enhanced_video_device::readz)
{
	if (m_selected && m_inDsrArea)
	{
		if (m_palette_accessed)
		{
			switch (m_address & 0x000f)
			{
			case 0:
				// Palette Read Address Register
				*value = m_palette.write_index;
				break;

			case 2:
				// Palette Read Color Value
				if (m_palette.read)
				{
					switch (m_palette.state)
					{
					case 0:
						*value = m_palette.color[m_palette.read_index].red;
						break;
					case 1:
						*value = m_palette.color[m_palette.read_index].green;
						break;
					case 2:
						*value = m_palette.color[m_palette.read_index].blue;
						break;
					}
					m_palette.state++;
					if (m_palette.state == 3)
					{
						m_palette.state = 0;
						m_palette.read_index++;
					}
				}
				break;

			case 4:
				// Palette Read Pixel Mask
				*value = m_palette.mask;
				break;
			case 6:
				// Palette Read Address Register for Color Value
				if (m_palette.read)
					*value = 0;
				else
					*value = 3;
				break;
			}
			return;
		}

		if (m_novram_accessed && m_RAMEN)
		{
			// NOVRAM
			*value = m_novram[offset & 0x00ff];
			return;
		}

		// DSR space
		*value = m_dsrrom[(offset & 0x1fff) | (m_dsr_page<<13)];
		return;
	}

	if (m_video_accessed)
	{
		*value = m_video->read(space, m_address>>1);
	}
}

/*
    Write a byte in evpc DSR space
    0x4000 - 0x5eff   DSR (paged)
    0x5f00 - 0x5fef   NOVRAM
    0x5ff0 - 0x5fff   Palette (5ff8, 5ffa, 5ffc, 5ffe)
*/
WRITE8_MEMBER(snug_enhanced_video_device::write)
{
	if (m_selected && m_inDsrArea)
	{
		if (m_palette_accessed)
		{
			// Palette
			if (TRACE_MEM) logerror("palette write %04x <- %02x\n", offset&0xffff, data);
			switch (m_address & 0x000f)
			{
			case 0x08:
				// Palette Write Address Register
				if (TRACE_MEM) logerror("EVPC palette address write (for write access)\n");
				m_palette.write_index = data;
				m_palette.state = 0;
				m_palette.read = 0;
				break;

			case 0x0a:
				// Palette Write Color Value
				if (TRACE_MEM) logerror("EVPC palette color write\n");
				if (!m_palette.read)
				{
					switch (m_palette.state)
					{
					case 0:
						m_palette.color[m_palette.write_index].red = data;
						break;
					case 1:
						m_palette.color[m_palette.write_index].green = data;
						break;
					case 2:
						m_palette.color[m_palette.write_index].blue = data;
						break;
					}
					m_palette.state++;
					if (m_palette.state == 3)
					{
						m_palette.state = 0;
						m_palette.write_index++;
					}
					//evpc_palette.dirty = 1;
				}
				break;

			case 0x0c:
				// Palette Write Pixel Mask
				if (TRACE_MEM) logerror("EVPC palette mask write\n");
				m_palette.mask = data;
				break;

			case 0x0e:
				// Palette Write Address Register for Color Value
				if (TRACE_MEM) logerror("EVPC palette address write (for read access)\n");
				m_palette.read_index = data;
				m_palette.state = 0;
				m_palette.read = 1;
				break;

			}
			return;
		}

		if (m_novram_accessed && m_RAMEN)
		{
			// NOVRAM
			m_novram[offset & 0x00ff] = data;
			return;
		}
	}

	if (m_video_accessed)
	{
		m_video->write(space, m_address>>1, data);
	}

	if (m_sound_accessed)
	{
		m_sound->write(space, 0, data);
	}
}

/*
    The CRU read handler. Read EVPC DIP switches
    0: Video timing (PAL/NTSC)
    1: -
    2: charset
    3: RAM shift
    4: -
    5: -
    6: -
    7: DIP or NOVRAM
    Logic is inverted
*/
READ8Z_MEMBER(snug_enhanced_video_device::crureadz)
{
	if ((offset & 0xff00)==EVPC_CRU_BASE)
	{
		if ((offset & 0x00f0)==0) // offset 0 delivers bits 0-7 (address 00-0f)
		{
			*value = ~(ioport("EVPC-SW1")->read() | (ioport("EVPC-SW3")->read()<<2)
				| (ioport("EVPC-SW4")->read()<<3) | (ioport("EVPC-SW8")->read()<<7));
		}
	}
}

/*
    The CRU write handler.
    Bit 0: Turn on DSR ROM
    Bit 1: DSR page select (bit 0)
    Bit 2: -
    Bit 3: RAM enable
    Bit 4: DSR page select (bit 2)
    Bit 5: DSR page select (bit 1)
    Bit 6: -
    Bit 7: -
*/
WRITE8_MEMBER(snug_enhanced_video_device::cruwrite)
{
	if ((offset & 0xff00)==EVPC_CRU_BASE)
	{
		int bit = (offset >> 1) & 0x0f;
		switch (bit)
		{
		case 0:
			m_selected = (data!=0);
			if (TRACE_CRU) logerror("Map DSR = %d\n", m_selected);
			break;

		case 1:
			if (data!=0)
				m_dsr_page |= 1;
			else
				m_dsr_page &= ~1;
			break;

		case 3:
			m_RAMEN = (data!=0);
			break;

		case 4:
			if (data!=0)
				m_dsr_page |= 4;
			else
				m_dsr_page &= ~4;
			break;

		case 5:
			if (data!=0)
				m_dsr_page |= 2;
			else
				m_dsr_page &= ~2;
			break;

		case 2:
		case 6:
		case 7:
			break;
		}
	}
}

/*
    READY line for the sound chip
*/
WRITE_LINE_MEMBER( snug_enhanced_video_device::ready_line )
{
	m_slot->set_ready(state);
}

void snug_enhanced_video_device::device_start()
{
	m_dsrrom = memregion(TI99_DSRROM)->base();
	m_novram = std::make_unique<uint8_t[]>(NOVRAM_SIZE);
	m_console_conn = downcast<bus::ti99::internal::evpc_clock_connector*>(machine().device(TI99_EVPC_CONN_TAG));
	save_item(NAME(m_address));
	save_item(NAME(m_dsr_page));
	save_item(NAME(m_inDsrArea));
	save_item(NAME(m_novram_accessed));
	save_item(NAME(m_palette_accessed));
	save_item(NAME(m_RAMEN));
	save_item(NAME(m_sound_accessed));
	save_item(NAME(m_video_accessed));
	save_item(NAME(m_intlevel));
}

void snug_enhanced_video_device::device_reset()
{
	m_select_mask = 0x7e000;
	m_select_value = 0x74000;
	m_dsr_page = 0;
	m_RAMEN = false;
	m_selected = false;
}

void snug_enhanced_video_device::device_stop()
{
	m_novram = nullptr;
}

/*
    This is the extra cable running from the EVPC card right into the TI console.
    It delivers the VDP interrupt and the GROM clock.

    For the SGCPU, the signal is delivered by the LCP line.
*/
WRITE_LINE_MEMBER( snug_enhanced_video_device::video_interrupt_in )
{
	// This method is frequently called without level change, so we only
	// react on changes
	if (state != m_intlevel)
	{
		m_intlevel = state;
		if (m_console_conn != nullptr) m_console_conn->vclock_line(state);
		else m_slot->lcp_line(state);

		if (state!=0) m_colorbus->poll();
	}
}

ROM_START( ti99_evpc )
	ROM_REGION(0x10000, TI99_DSRROM, 0)
	ROM_LOAD("evpc_dsr.u21", 0, 0x10000, CRC(a062b75d) SHA1(6e8060f86e3bb9c36f244d88825e3fe237bfe9a9)) /* evpc DSR ROM */
ROM_END

/*
    Input ports for the EPVC
*/
INPUT_PORTS_START( ti99_evpc )
	PORT_START( "EVPC-SW1" )
	PORT_DIPNAME( 0x01, 0x00, "EVPC video mode" ) PORT_CONDITION( "EVPC-SW8", 0x01, EQUALS, 0x00 )
		PORT_DIPSETTING(    0x00, "PAL" )
		PORT_DIPSETTING(    0x01, "NTSC" )

	PORT_START( "EVPC-SW3" )
	PORT_DIPNAME( 0x01, 0x00, "EVPC charset" ) PORT_CONDITION( "EVPC-SW8", 0x01, EQUALS, 0x00 )
		PORT_DIPSETTING(    0x00, DEF_STR( International ))
		PORT_DIPSETTING(    0x01, DEF_STR( German ))

	PORT_START( "EVPC-SW4" )
	PORT_DIPNAME( 0x01, 0x00, "EVPC VDP RAM" ) PORT_CONDITION( "EVPC-SW8", 0x01, EQUALS, 0x00 )
		PORT_DIPSETTING(    0x00, "shifted" )
		PORT_DIPSETTING(    0x01, "not shifted" )

	PORT_START( "EVPC-SW8" )
	PORT_DIPNAME( 0x01, 0x00, "EVPC Configuration" )
		PORT_DIPSETTING(    0x00, "DIP" )
		PORT_DIPSETTING(    0x01, "NOVRAM" )
INPUT_PORTS_END

const tiny_rom_entry *snug_enhanced_video_device::device_rom_region() const
{
	return ROM_NAME( ti99_evpc );
}

ioport_constructor snug_enhanced_video_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(ti99_evpc);
}

MACHINE_CONFIG_MEMBER( snug_enhanced_video_device::device_add_mconfig )
	// video hardware
	MCFG_V9938_ADD(TI_VDP_TAG, TI_SCREEN_TAG, 0x20000, XTAL_21_4772MHz)  /* typical 9938 clock, not verified */
	MCFG_V99X8_INTERRUPT_CALLBACK(WRITELINE(snug_enhanced_video_device, video_interrupt_in))
	MCFG_V99X8_SCREEN_ADD_NTSC(TI_SCREEN_TAG, TI_VDP_TAG, XTAL_21_4772MHz)

	// Sound hardware
	MCFG_SPEAKER_STANDARD_MONO("sound_out")
	MCFG_SOUND_ADD(TI_SOUNDCHIP_TAG, SN94624, 3579545/8) /* 3.579545 MHz */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "sound_out", 0.75)
	MCFG_SN76496_READY_HANDLER( WRITELINE(snug_enhanced_video_device, ready_line) )

	// Mouse connected to the color bus of the v9938
	MCFG_COLORBUS_MOUSE_ADD( COLORBUS_TAG )

MACHINE_CONFIG_END

} } } // end namespace bus::ti99::peb
