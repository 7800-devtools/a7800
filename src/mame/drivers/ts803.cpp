// license:BSD-3-Clause
// copyright-holders: Gabriele D'Antona, Robbbert
/*************************************************************************************************

TeleVideo TS-803(H)
Driver by friol
Started: 27/12/2015

Addressable memory space has 2 configurations:

PAGE SEL bit in PORT0 set to 0:

  0000-2000 - 8k rom
  2000-4000 - 8k expansion rom
  4000-BFFF - videoram
  C000-FFFF - 16k cpu RAM

PAGE SEL bit in PORT0 set to 1:

  0000-dFFF - 56k ram
  e000-FFFF - OS RAM

  Z80STI:
  - fails hardware test, left out for now.
  - provides baud-rate clock for DART ch B (serial printer)
  - merges FDC and HDC irq into the daisy chain. FDC works without it though.

  Keyboard / Z80DART:
  - Keyboard has 8048 plus undumped rom. There's no schematic.
  - Protocol: 9600 baud, 8 data bits, 2 stop bits, no parity, no handshaking.
  - Problem: every 2nd keystroke is ignored.
  - Problem: some of the disks cause the keyboard to produce rubbish, unable to find
             a baud rate that works.

  To Do:
  - Fix Z80STI and use it
  - Fix weird problem with keyboard, or better, get the rom and emulate it.
  - Hard Drive
  - Videoram has 4 banks, only first bank is currently used for display
  - Option of a further 64k of main ram.
  - Dipswitches
  - 2 more Serial ports (Serial Printer & Mouse; Modem)
  - Optional RS422 board (has Z80SIO +others)
  - Diagnostic LEDs
  - Cleanup
  - The demo refers to video attributes, but there's no mention of them anywhere else

**************************************************************************************************/

#include "emu.h"
#include "bus/rs232/rs232.h"
#include "cpu/z80/z80.h"
#include "cpu/z80/z80daisy.h"
#include "machine/keyboard.h"
#include "machine/z80dart.h"
#include "machine/wd_fdc.h"
#include "machine/z80sti.h"
#include "video/mc6845.h"
#include "screen.h"


class ts803_state : public driver_device
{
public:
	ts803_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_palette(*this, "palette")
		, m_maincpu(*this, "maincpu")
		, m_fdc(*this, "fdc")
		, m_floppy0(*this, "fdc:0")
		, m_floppy1(*this, "fdc:1")
		//, m_sti(*this, "sti")
		, m_dart(*this, "dart")
		, m_crtc(*this,"crtc")
		, m_p_chargen(*this, "chargen")
	{ }

	DECLARE_READ8_MEMBER( ts803_port_r );
	DECLARE_WRITE8_MEMBER( ts803_port_w );
	DECLARE_READ8_MEMBER( ts803_porthi_r );
	DECLARE_WRITE8_MEMBER( ts803_porthi_w );

	DECLARE_WRITE8_MEMBER( disk_0_control_w );
	DECLARE_READ8_MEMBER( disk_0_control_r );

	DECLARE_WRITE8_MEMBER( keyboard_put );

	MC6845_UPDATE_ROW(crtc_update_row);
	MC6845_ON_UPDATE_ADDR_CHANGED(crtc_update_addr);
	DECLARE_WRITE8_MEMBER( crtc_controlreg_w );
	DECLARE_DRIVER_INIT(ts803);
	TIMER_DEVICE_CALLBACK_MEMBER(dart_tick);

	uint32_t screen_update_ts803(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

private:

	std::unique_ptr<uint8_t[]> m_videoram;
	std::unique_ptr<uint8_t[]> m_56kram;
	uint8_t m_sioidxr=0;
	uint8_t m_sioarr[256];
	bool m_graphics_mode;
	bool m_tick;

	virtual void machine_reset() override;
	virtual void machine_start() override;

	required_device<palette_device> m_palette;
	required_device<cpu_device> m_maincpu;
	required_device<fd1793_device> m_fdc;
	required_device<floppy_connector> m_floppy0;
	required_device<floppy_connector> m_floppy1;
	//required_device<z80sti_device> m_sti;
	required_device<z80dart_device> m_dart;
	required_device<sy6545_1_device> m_crtc;
	required_region_ptr<u8> m_p_chargen;
};

static ADDRESS_MAP_START(ts803_mem, AS_PROGRAM, 8, ts803_state)
	AM_RANGE(0x0000, 0x3fff) AM_READ_BANK("bankr0") AM_WRITE_BANK("bankw0")
	AM_RANGE(0x4000, 0xbfff) AM_RAMBANK("bank4")
	AM_RANGE(0xc000, 0xffff) AM_RAM
ADDRESS_MAP_END

/*

I/0 Port Addresses

System Status Switch 1                                  00
Diagnostic Indicators 1 and 2                           10
Diagnostic Indicators 3 and 4                           11
RS-422 Control and Auto Wait                            12
Memory Bank Select                                      13
STI Device (modem)                                      20-2F
DART Dual Asynchronous Receiver Transmitter
Device (keyboard, printer, mouse)                       30-33
RS-422 SIO Device                                       40-43
Floppy Disk Controller                                  80-83
Floppy Disk Drive Decoder                               90
Winchester Disk Controller Reset                        A0
Winchester Disk Controller                              B0-BF
Graphics Controller                                     C0-CF

*/
static ADDRESS_MAP_START(ts803_io, AS_IO, 8, ts803_state)
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x00, 0x2f) AM_READWRITE(ts803_port_r, ts803_port_w)

	//AM_RANGE(0x00, 0x1f) AM_READWRITE(ts803_port_r, ts803_port_w)
	//AM_RANGE(0x20, 0x2f) AM_DEVREADWRITE("sti", z80sti_device, read, write)
	AM_RANGE(0x30, 0x33) AM_DEVREADWRITE("dart", z80dart_device, cd_ba_r, cd_ba_w)

	AM_RANGE(0x80, 0x83) AM_DEVREADWRITE("fdc", fd1793_device, read, write)
	AM_RANGE(0x90, 0x90) AM_READWRITE(disk_0_control_r,disk_0_control_w)

	//AM_RANGE(0x91, 0xff) AM_READWRITE(ts803_porthi_r, ts803_porthi_w)
	AM_RANGE(0x91, 0xbf) AM_READWRITE(ts803_porthi_r, ts803_porthi_w)

	AM_RANGE(0xc0, 0xc0) AM_DEVREADWRITE("crtc", sy6545_1_device, status_r, address_w)
	AM_RANGE(0xc2, 0xc2) AM_DEVREADWRITE("crtc", sy6545_1_device, register_r, register_w)
	AM_RANGE(0xc4, 0xc4) AM_WRITE(crtc_controlreg_w)
ADDRESS_MAP_END

/* Input ports */
static INPUT_PORTS_START( ts803 )
INPUT_PORTS_END

/* keyboard */

WRITE8_MEMBER( ts803_state::keyboard_put )
{
	if (data)
	{
		//printf("Keyboard stroke [%2x]\n",data);

		//m_maincpu->set_input_line_vector(INPUT_LINE_IRQ0, 0x3f);
		//m_maincpu->set_input_line(INPUT_LINE_IRQ0, ASSERT_LINE);

		if (data==0x0d) m_maincpu->space(AS_PROGRAM).write_byte(0xf83f,0xC1);
		else
		{
			m_maincpu->space(AS_PROGRAM).write_byte(0xf83f,0x4f);
			m_maincpu->space(AS_PROGRAM).write_byte(0xf890,data);
		}
	}
}

/* disk drive */

static SLOT_INTERFACE_START( ts803_floppies )
	SLOT_INTERFACE( "drive0", FLOPPY_525_DD )
	SLOT_INTERFACE( "drive1", FLOPPY_525_DD )
SLOT_INTERFACE_END

WRITE8_MEMBER( ts803_state::disk_0_control_w )
{
/*
d0 ready
d1 motor on
d2 Side select (active low)
d3 Double density (active low)
d4 Drive select 0 (active low)
d5 Drive select 1 (active low)
d6 Drive select 2 (active low)
d7 Drive select 3 (active low)
*/
	if ((data & 0xc0)!=0xc0) return;
	floppy_image_device *floppy = nullptr;
	if (BIT(data, 4)==0)
		floppy = m_floppy0->get_device();
	else
	if (BIT(data, 5)==0)
		floppy = m_floppy1->get_device();

	m_fdc->set_floppy(floppy);

	if (floppy)
	{
		floppy->mon_w(BIT(data, 1));
		floppy->ss_w(BIT(data, 2) ? 0: 1);
	}
	m_fdc->dden_w(BIT(data, 3));
}

READ8_MEMBER( ts803_state::disk_0_control_r )
{
	printf("Disk0 control register read\n");
	return 0xff;
}

READ8_MEMBER( ts803_state::ts803_porthi_r )
{
	//printf("PortHI read [%x]\n",offset+0x91);

	switch(offset+0x91)
	{
		case 0xb2:
			printf("B2 WDC read\n");
			return 0x11;

		case 0xb3:
			printf("B3 WDC read\n");
			return 0x15;

		case 0xb4:
			printf("B4 WDC read\n");
			return 0x55;

		case 0xb5:
			printf("B5 WDC read\n");
			return 0x01;

		case 0xb6:
			printf("B6 WDC read\n");
			return 0x25;

	}

	return 0x00;
}

WRITE8_MEMBER( ts803_state::ts803_porthi_w )
{
	//printf("PortHI write [%2x] [%2x]\n",offset+0x91,data);

	switch (offset+0x91)
	{
		case 0xc4:
			//printf("Control Register for Alpha or Graphics Mode Selection\n");
			break;

		default:
			break;
	}
}

READ8_MEMBER( ts803_state::ts803_port_r )
{
	printf("Port read [%x]\n",offset);

	switch (offset)
	{
		case 0x00:
			// system status switch 1 (dip switches)
			// 0,1,2: baud rate
			// 3: ts803-ts803h
			// 4: local-remote
			// 5: required (?)
			// 6: 60hz-50hz
			// 7: required
			return 0x80;

		case 0x20:
			// STI read (at boot time)
			return 0x55;

		case 0x26:
			// STI read (at boot time)
			return 0x55;

		case 0x27:
			// STI read (at boot time)
			return 0x55;

		case 0x2a:
			// STI read (at boot time)
			return 0x55;

		case 0x2d:
			// STI
			return 0x00;

		case 0x2b:
			// STI read (at boot time)
			return 0x55;

		case 0x42:
			// SIO
			m_sioidxr++;
			return m_sioarr[m_sioidxr-1];

		default:
			return 0xff;
	}

	return 0xff;
}

WRITE8_MEMBER( ts803_state::ts803_port_w )
{
	//printf("Port write [%2x] [%2x]\n",offset,data);

	switch (offset)
	{
		case 0x00:
			printf("Writing on system status switch\n");
			break;

		case 0x10:
			data &= 3;
			printf("Writing to diagnostic indicators 1 & 2: %X\n", data);
			break;

		case 0x11:
			data &= 3;
			printf("Writing to diagnostic indicators 3 & 4: %X\n", data);
			break;

		case 0x12:
			data &= 3;
			printf("RS-422 control: %X\n", data);
			break;

		case 0x13:
			data &= 3;
			if (BIT(data, 1)==0)
			{
				membank("bankr0")->set_entry(BIT(data, 0));
				membank("bank4")->set_entry(BIT(data, 0));
			}
			else
				printf("Error: unknown memory config: %X.\n", data);

			break;

		case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2A: case 0x2b: case 0x2c: case 0x2d: case 0x2e: case 0x2f:
			//printf("STI device write [%2x]\n",data);
			break;

		default:
			printf("unknown port [%2.2x] write of [%2.2x]\n",offset,data);
			break;
	}
}

MC6845_ON_UPDATE_ADDR_CHANGED( ts803_state::crtc_update_addr )
{
	//printf("CRTC::address update [%x]\n",address);
}

MC6845_UPDATE_ROW( ts803_state::crtc_update_row )
{
	const rgb_t *pens = m_palette->palette()->entry_list_raw();
	uint8_t chr,gfx,inv;
	uint16_t mem,x;
	uint32_t *p = &bitmap.pix32(y);

	for (x = 0; x < x_count; x++)
	{
		inv = (x == cursor_x) ? 0xff : 0;

		if (m_graphics_mode)
		{
			mem = (ra*0x2000 + ma + x) & 0x7fff;
			gfx = m_videoram[mem] ^ inv;
		}
		else
		{
			mem = 0x1800 + ((ma + x) & 0x7ff);
			chr = m_videoram[mem];
			gfx = (ra > 7) ? 0 : m_p_chargen[(chr<<3) | ((ra+1)&7)] ^ inv;
		}

		/* Display a scanline of a character (8 pixels) */
		*p++ = pens[BIT(gfx, 7)];
		*p++ = pens[BIT(gfx, 6)];
		*p++ = pens[BIT(gfx, 5)];
		*p++ = pens[BIT(gfx, 4)];
		*p++ = pens[BIT(gfx, 3)];
		*p++ = pens[BIT(gfx, 2)];
		*p++ = pens[BIT(gfx, 1)];
		*p++ = pens[BIT(gfx, 0)];
	}
}

WRITE8_MEMBER( ts803_state::crtc_controlreg_w )
{
/*
Bit 0 = 0 alpha mode
                1 graphics mode
Bit 1 = 0 page 1 (alpha mode only)
                1 page 2 (alpha mode only)
Bit 2 = 0 alpha memory access (round off)
                1 graphics memory access (normal CPU address)
*/

	//printf("CRTC::c4 write [%2x]\n",data);
	m_graphics_mode = BIT(data, 0);
}

// baud rate generator for keyboard, 9600 baud.
TIMER_DEVICE_CALLBACK_MEMBER( ts803_state::dart_tick )
{
	m_tick ^= 1;
	m_dart->rxca_w(m_tick);
	m_dart->txca_w(m_tick);
	m_dart->txcb_w(m_tick); // needed to pass the test
}

void ts803_state::machine_start()
{
	//save these 2 so we can examine them in the debugger
	save_pointer(NAME(m_videoram.get()), 0x8000);
	save_pointer(NAME(m_56kram.get()), 0xc000);
}

void ts803_state::machine_reset()
{
	m_graphics_mode = false;
	m_tick = 0;

	m_sioarr[0]=0x00;
	m_sioarr[1]=0xff;

	//m_sti->reset();
	membank("bankr0")->set_entry(0);
	membank("bankw0")->set_entry(0);
	membank("bank4")->set_entry(0);
}

DRIVER_INIT_MEMBER( ts803_state, ts803 )
{
	m_videoram = std::make_unique<uint8_t[]>(0x8000);
	m_56kram = std::make_unique<uint8_t[]>(0xc000);

	uint8_t *rom = memregion("roms")->base();
	membank("bankr0")->configure_entry(0, &rom[0]); // rom
	membank("bankr0")->configure_entry(1, m_56kram.get()); // ram
	membank("bankw0")->configure_entry(0, m_56kram.get()); // ram
	membank("bank4")->configure_entry(0, m_videoram.get()); // vram
	membank("bank4")->configure_entry(1, m_56kram.get()+0x4000); // ram
}

/* Interrupt priority:
0: RS-422 option board (highest priority)
1: Z80A DART (RS-232C serial I/O)
2: Z80 STI (RS 232C modem port)
3: FD 1793 floppy disk controller
4: Winchester disk controller board
5: Time-of-day clock
Interrupts 0 through 2 are prioritized in a daisy-chain
arrangement. Interrupts 3 through 5 are wired to the Z80 STI
interrupt input pins. */
static const z80_daisy_config daisy_chain[] =
{
	//{ "rs422" }, // not emulated
	{ "dart" },
	//{ "sti" },
	{ nullptr }
};

static MACHINE_CONFIG_START( ts803 )
	MCFG_CPU_ADD("maincpu", Z80, XTAL_16MHz/4)
	MCFG_CPU_PROGRAM_MAP(ts803_mem)
	MCFG_CPU_IO_MAP(ts803_io)
	MCFG_Z80_DAISY_CHAIN(daisy_chain)

	/* video hardware */
	MCFG_SCREEN_ADD_MONOCHROME("screen", RASTER, rgb_t::green())
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_SIZE(640,240)
	MCFG_SCREEN_VISIBLE_AREA(0, 640-1, 0, 240-1)
	MCFG_SCREEN_UPDATE_DEVICE("crtc", sy6545_1_device, screen_update)
	MCFG_PALETTE_ADD_MONOCHROME("palette")

	/* crtc */
	MCFG_MC6845_ADD("crtc", SY6545_1, "screen", 13608000 / 8)
	MCFG_MC6845_SHOW_BORDER_AREA(false)
	MCFG_MC6845_CHAR_WIDTH(8)
	MCFG_MC6845_UPDATE_ROW_CB(ts803_state, crtc_update_row)
	MCFG_MC6845_ADDR_CHANGED_CB(ts803_state, crtc_update_addr)

	//MCFG_DEVICE_ADD("sti", Z80STI, XTAL_16MHz/4) // STI baud rates are derived from XTAL_16MHz / 13
	//MCFG_Z80STI_OUT_INT_CB(INPUTLINE("maincpu", INPUT_LINE_IRQ0))

	MCFG_Z80DART_ADD("dart", XTAL_16MHz / 4, 0, 0, 0, 0 )
	MCFG_Z80DART_OUT_INT_CB(INPUTLINE("maincpu", INPUT_LINE_IRQ0))
	//MCFG_Z80DART_OUT_TXDA_CB(DEVWRITELINE("rs232", rs232_port_device, write_txd))
	//MCFG_Z80DART_OUT_DTRA_CB(DEVWRITELINE("rs232", rs232_port_device, write_dtr))
	//MCFG_Z80DART_OUT_RTSA_CB(DEVWRITELINE("rs232", rs232_port_device, write_rts))

	MCFG_RS232_PORT_ADD("rs232", default_rs232_devices, "keyboard")
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE("dart", z80dart_device, rxa_w))
	//MCFG_RS232_CTS_HANDLER(DEVWRITELINE("dart", z80dart_device, ctsa_w))

	/* floppy disk */
	MCFG_FD1793_ADD("fdc", XTAL_1MHz)
	//MCFG_WD_FDC_INTRQ_CALLBACK(DEVWRITELINE("sti", z80sti_device, i7_w))   // add when sti is in
	MCFG_FLOPPY_DRIVE_ADD("fdc:0", ts803_floppies, "drive0", floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("fdc:1", ts803_floppies, "drive1", floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_SOUND(true)

	/* keyboard */
	//MCFG_DEVICE_ADD("keyboard", GENERIC_KEYBOARD, 0)
	//MCFG_GENERIC_KEYBOARD_CB(WRITE8(ts803_state, keyboard_put))
	MCFG_TIMER_DRIVER_ADD_PERIODIC("dart_tick", ts803_state, dart_tick, attotime::from_hz(153600*2))

MACHINE_CONFIG_END

/* ROM definition */
ROM_START( ts803h )
	ROM_REGION(0x4000, "roms", ROMREGION_ERASEFF) // includes space for optional expansion rom
	ROM_LOAD( "180001-37 rev d 803 5 23 84.a57", 0x0000, 0x2000, CRC(0aa658a7) SHA1(42d0a89c2ff9b6588cd88bdb1f800fac540dccbb) )

	ROM_REGION(0x0100, "proms", 0)
	ROM_LOAD( "8000134.a59", 0x0000, 0x0100, CRC(231fe6d6) SHA1(3c052ba4b74547e0e2451fa1ae67bbcb83a18bab) )

	ROM_REGION(0x0800, "chargen", 0)
	ROM_LOAD( "803h_vid.a119", 0x0000, 0x0800, CRC(d5ce2814) SHA1(ce527479464757223dffac384a85ab74b174952c) )
ROM_END



//   YEAR  NAME    PARENT  COMPAT   MACHINE    INPUT  CLASS        INIT    COMPANY     FULLNAME  FLAGS
COMP(1983, ts803h,  0,      0,      ts803,     ts803, ts803_state, ts803, "Televideo", "TS803H", MACHINE_NOT_WORKING )
