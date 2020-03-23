// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic, Robbbert
/***************************************************************************

Sanyo MBC-200

Machine MBC-1200 is identical but sold outside of Japan

16 x HM6116P-3 2K x 8 SRAM soldered onboard (so 32k ram)
4 x HM6116P-3 2K x 8 SRAM socketed (so 8k ram)
4 x MB83256 32K x 8 socketed (128k ram)
Floppy = 5.25"
MBC1200 has one floppy while MBC1250 has 2. The systems are otherwise identical.

Keyboard communicates via RS232 to uart at E0,E1. The processor and rom for it
are undumped / unknown. The input codes are not ascii, so using custom code until
the required details become available.

On back side:
- keyboard DIN connector
- Centronics printer port
- RS-232C 25pin connector

SBASIC:
Running programs: the file names used within SBASIC must be in
uppercase. For example, run "DEMO" .
You can also run a basic program from CP/M: sbasic "GRAPHICS" .
To Break, press either ^N or ^O (display freezes), then ^C .
Some control keys: 0x14 = Home; 0x8 = Left/BS; 0xA = Down; 0xB = Up; 0xC = Right.
GAIJI.BAS doesn't work because GAIJI.FNT is missing.

TODO:
- Other connections to the various PPI's
- UART connections
- Any other devices?

2011-10-31 Skeleton driver.
2014-05-18 Made rom get copied into ram, boot code from disk
           requires that ram is there otherwise you get
           a MEMORY ERROR. CP/M now loads.

2016-07-16 Added keyboard and sound.

****************************************************************************/

#include "emu.h"
#include "cpu/z80/z80.h"
#include "machine/i8251.h"
#include "machine/i8255.h"
#include "machine/keyboard.h"
#include "machine/wd_fdc.h"
#include "sound/beep.h"
#include "sound/spkrdev.h"
#include "video/mc6845.h"
#include "screen.h"
#include "softlist.h"
#include "speaker.h"

class mbc200_state : public driver_device
{
public:
	mbc200_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_palette(*this, "palette")
		, m_crtc(*this, "crtc")
		, m_ppi_m(*this, "ppi_m")
		, m_vram(*this, "vram")
		, m_maincpu(*this, "maincpu")
		, m_beep(*this, "beeper")
		, m_speaker(*this, "speaker")
		, m_fdc(*this, "fdc")
		, m_floppy0(*this, "fdc:0")
		, m_floppy1(*this, "fdc:1")
	{ }

	DECLARE_READ8_MEMBER(p2_porta_r);
	DECLARE_WRITE8_MEMBER(p1_portc_w);
	DECLARE_WRITE8_MEMBER(pm_porta_w);
	DECLARE_WRITE8_MEMBER(pm_portb_w);
	DECLARE_READ8_MEMBER(keyboard_r);
	void kbd_put(u8 data);
	MC6845_UPDATE_ROW(update_row);
	required_device<palette_device> m_palette;

private:
	virtual void machine_start() override;
	virtual void machine_reset() override;
	uint8_t m_comm_latch;
	uint8_t m_term_data;
	required_device<mc6845_device> m_crtc;
	required_device<i8255_device> m_ppi_m;
	required_shared_ptr<uint8_t> m_vram;
	required_device<cpu_device> m_maincpu;
	required_device<beep_device> m_beep;
	required_device<speaker_sound_device> m_speaker;
	required_device<mb8876_device> m_fdc;
	required_device<floppy_connector> m_floppy0;
	required_device<floppy_connector> m_floppy1;
};


static ADDRESS_MAP_START(mbc200_mem, AS_PROGRAM, 8, mbc200_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE( 0x0000, 0x0fff ) AM_RAM AM_REGION("maincpu", 0)
	AM_RANGE( 0x1000, 0xffff ) AM_RAM
ADDRESS_MAP_END

WRITE8_MEMBER( mbc200_state::p1_portc_w )
{
	m_speaker->level_w(BIT(data,4)); // used by beep command in basic
}

WRITE8_MEMBER( mbc200_state::pm_porta_w )
{
	machine().scheduler().synchronize(); // force resync
	//printf("A %02x %c\n",data,data);
	m_comm_latch = data; // to slave CPU
}

WRITE8_MEMBER( mbc200_state::pm_portb_w )
{
	floppy_image_device *floppy = nullptr;

	// to be verified
	switch (data & 0x01)
	{
	case 0: floppy = m_floppy0->get_device(); break;
	case 1: floppy = m_floppy1->get_device(); break;
	}

	m_fdc->set_floppy(floppy);

	if (floppy)
	{
		floppy->mon_w(0);
		floppy->ss_w(BIT(data, 7));
	}
	m_beep->set_state(BIT(data, 1)); // key-click
}

static ADDRESS_MAP_START( mbc200_io , AS_IO, 8, mbc200_state)
	ADDRESS_MAP_UNMAP_HIGH
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	//AM_RANGE(0xe0, 0xe0) AM_DEVREADWRITE("uart1", i8251_device, data_r, data_w)
	//AM_RANGE(0xe1, 0xe1) AM_DEVREADWRITE("uart1", i8251_device, status_r, control_w)
	AM_RANGE(0xe0, 0xe1) AM_READ(keyboard_r) AM_WRITENOP
	AM_RANGE(0xe4, 0xe7) AM_DEVREADWRITE("fdc", mb8876_device, read, write)
	AM_RANGE(0xe8, 0xeb) AM_DEVREADWRITE("ppi_m", i8255_device, read, write)
	AM_RANGE(0xec, 0xec) AM_DEVREADWRITE("uart2", i8251_device, data_r, data_w)
	AM_RANGE(0xed, 0xed) AM_DEVREADWRITE("uart2", i8251_device, status_r, control_w)
ADDRESS_MAP_END



static ADDRESS_MAP_START(mbc200_sub_mem, AS_PROGRAM, 8, mbc200_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE( 0x0000, 0x2fff ) AM_ROM
	AM_RANGE( 0x3000, 0x7fff ) AM_RAM
	AM_RANGE( 0x8000, 0xffff ) AM_RAM AM_SHARE("vram")
ADDRESS_MAP_END

READ8_MEMBER(mbc200_state::p2_porta_r)
{
	machine().scheduler().synchronize(); // force resync
	uint8_t tmp = m_comm_latch;
	m_comm_latch = 0;
	m_ppi_m->pc6_w(0); // ppi_ack
	return tmp;
}

static ADDRESS_MAP_START( mbc200_sub_io , AS_IO, 8, mbc200_state)
	ADDRESS_MAP_UNMAP_HIGH
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x70, 0x73) AM_DEVREADWRITE("ppi_1", i8255_device, read, write)
	AM_RANGE(0xb0, 0xb0) AM_DEVREADWRITE("crtc", mc6845_device, status_r, address_w)
	AM_RANGE(0xb1, 0xb1) AM_DEVREADWRITE("crtc", mc6845_device, register_r, register_w)
	AM_RANGE(0xd0, 0xd3) AM_DEVREADWRITE("ppi_2", i8255_device, read, write)
ADDRESS_MAP_END

/* Input ports */
static INPUT_PORTS_START( mbc200 )
INPUT_PORTS_END

READ8_MEMBER( mbc200_state::keyboard_r )
{
	uint8_t data = 0;
	if (offset)
	{
		if (m_term_data)
		{
			data = 2;
			// handle CTRL key pressed
			if (m_term_data < 0x20)
			{
				data |= 8;
				m_term_data |= 0x40;
			}
		}
	}
	else
	{
		data = m_term_data;
		m_term_data = 0;
	}

	return data;
}

// convert standard control keys to expected code;
void mbc200_state::kbd_put(u8 data)
{
	switch (data)
	{
		case 0x0e:
			m_term_data = 0xe2;
			break;
		case 0x0f:
			m_term_data = 0xe3;
			break;
		case 0x08:
			m_term_data = 0xe4;
			break;
		case 0x09:
			m_term_data = 0xe5;
			break;
		case 0x0a:
			m_term_data = 0xe6;
			break;
		case 0x0d:
			m_term_data = 0xe7;
			break;
		case 0x1b:
			m_term_data = 0xe8;
			break;
		default:
			m_term_data = data;
	}
}

void mbc200_state::machine_start()
{
}

void mbc200_state::machine_reset()
{
	uint8_t* roms = memregion("roms")->base();
	uint8_t* main = memregion("maincpu")->base();
	memcpy(main, roms, 0x1000);
}

static SLOT_INTERFACE_START( mbc200_floppies )
	SLOT_INTERFACE("qd", FLOPPY_525_QD )
SLOT_INTERFACE_END

MC6845_UPDATE_ROW( mbc200_state::update_row )
{
	const rgb_t *palette = m_palette->palette()->entry_list_raw();
	uint8_t gfx;
	uint16_t mem,x;
	uint32_t *p = &bitmap.pix32(y);

	for (x = 0; x < x_count; x++)
	{
		mem = (ma+x)*4+ra;
		gfx = m_vram[mem & 0x7fff];
		*p++ = palette[BIT(gfx, 7)];
		*p++ = palette[BIT(gfx, 6)];
		*p++ = palette[BIT(gfx, 5)];
		*p++ = palette[BIT(gfx, 4)];
		*p++ = palette[BIT(gfx, 3)];
		*p++ = palette[BIT(gfx, 2)];
		*p++ = palette[BIT(gfx, 1)];
		*p++ = palette[BIT(gfx, 0)];
	}
}

static const gfx_layout mbc200_chars_8x8 =
{
	8,8,
	256,
	1,
	{ 0 },
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	8*8
};

static GFXDECODE_START( mbc200 )
	GFXDECODE_ENTRY( "subcpu", 0x1800, mbc200_chars_8x8, 0, 1 )
GFXDECODE_END


static MACHINE_CONFIG_START( mbc200 )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu",Z80, XTAL_8MHz/2) // NEC D780C-1
	MCFG_CPU_PROGRAM_MAP(mbc200_mem)
	MCFG_CPU_IO_MAP(mbc200_io)

	MCFG_CPU_ADD("subcpu",Z80, XTAL_8MHz/2) // NEC D780C-1
	MCFG_CPU_PROGRAM_MAP(mbc200_sub_mem)
	MCFG_CPU_IO_MAP(mbc200_sub_io)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) /* not accurate */
	MCFG_SCREEN_SIZE(640, 400)
	MCFG_SCREEN_VISIBLE_AREA(0, 640-1, 0, 400-1)
	MCFG_SCREEN_UPDATE_DEVICE("crtc", h46505_device, screen_update)
	MCFG_GFXDECODE_ADD("gfxdecode", "palette", mbc200)
	MCFG_PALETTE_ADD_MONOCHROME("palette")

	MCFG_MC6845_ADD("crtc", H46505, "screen", XTAL_8MHz / 4) // HD46505SP
	MCFG_MC6845_SHOW_BORDER_AREA(false)
	MCFG_MC6845_CHAR_WIDTH(8)
	MCFG_MC6845_UPDATE_ROW_CB(mbc200_state, update_row)

	// sound
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("beeper", BEEP, 1000) // frequency unknown
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)

	MCFG_DEVICE_ADD("ppi_1", I8255, 0)
	MCFG_I8255_OUT_PORTC_CB(WRITE8(mbc200_state, p1_portc_w))

	MCFG_DEVICE_ADD("ppi_2", I8255, 0)
	MCFG_I8255_IN_PORTA_CB(READ8(mbc200_state, p2_porta_r))

	MCFG_DEVICE_ADD("ppi_m", I8255, 0)
	MCFG_I8255_OUT_PORTA_CB(WRITE8(mbc200_state, pm_porta_w))
	MCFG_I8255_OUT_PORTB_CB(WRITE8(mbc200_state, pm_portb_w))

	MCFG_DEVICE_ADD("uart1", I8251, 0) // INS8251N

	MCFG_DEVICE_ADD("uart2", I8251, 0) // INS8251A

	MCFG_MB8876_ADD("fdc", XTAL_8MHz / 8) // guess
	MCFG_FLOPPY_DRIVE_ADD("fdc:0", mbc200_floppies, "qd", floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_SOUND(true)
	MCFG_FLOPPY_DRIVE_ADD("fdc:1", mbc200_floppies, "qd", floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_SOUND(true)

	/* Keyboard */
	MCFG_DEVICE_ADD("keyboard", GENERIC_KEYBOARD, 0)
	MCFG_GENERIC_KEYBOARD_CB(PUT(mbc200_state, kbd_put))

	/* software lists */
	MCFG_SOFTWARE_LIST_ADD("flop_list", "mbc200")
MACHINE_CONFIG_END

/* ROM definition */
ROM_START( mbc200 )
	ROM_REGION( 0x1000, "maincpu", ROMREGION_ERASEFF )
	ROM_REGION( 0x1000, "roms", 0 )
	ROM_LOAD( "d2732a.bin",  0x0000, 0x1000, CRC(bf364ce8) SHA1(baa3a20a5b01745a390ef16628dc18f8d682d63b))
	ROM_REGION( 0x3000, "subcpu", ROMREGION_ERASEFF )
	ROM_LOAD( "m5l2764.bin", 0x0000, 0x2000, CRC(377300a2) SHA1(8563172f9e7f84330378a8d179f4138be5fda099))
ROM_END

/* Driver */

//    YEAR  NAME     PARENT   COMPAT   MACHINE    INPUT   CLASS          INIT  COMPANY   FULLNAME   FLAGS
COMP( 1982, mbc200,  0,       0,       mbc200,    mbc200, mbc200_state,  0,    "Sanyo",  "MBC-200", 0 )
