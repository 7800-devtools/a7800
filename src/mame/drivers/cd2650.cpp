// license:BSD-3-Clause
// copyright-holders:Robbbert
/***************************************************************************

Central Data cd2650

2010-04-08 Skeleton driver.

No info available on this computer apart from a few newsletters.
The system only uses 1000-14FF for videoram and 17F0-17FF for
scratch ram. All other ram is optional.

Commands (must be in uppercase):
A    Examine memory; press C to alter memory
B    Set breakpoint?
C    View breakpoint?
D    Dump to tape
E    Execute
I    ?
L    Load
R    ?
V    Verify?
Press Esc to exit most commands.

TODO
- Lots, probably. The computer is a complete mystery. No manuals or schematics exist.
- Cassette doesn't work.

****************************************************************************/

#include "emu.h"
#include "cpu/s2650/s2650.h"
#include "imagedev/cassette.h"
#include "imagedev/snapquik.h"
#include "machine/keyboard.h"
#include "sound/beep.h"
#include "sound/wave.h"
#include "screen.h"
#include "speaker.h"

class cd2650_state : public driver_device
{
public:
	cd2650_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_p_videoram(*this, "videoram")
		, m_p_chargen(*this, "chargen")
		, m_beep(*this, "beeper")
		, m_cass(*this, "cassette")
	{
	}

	DECLARE_READ8_MEMBER(keyin_r);
	DECLARE_WRITE8_MEMBER(beep_w);
	void kbd_put(u8 data);
	DECLARE_READ_LINE_MEMBER(cass_r);
	DECLARE_WRITE_LINE_MEMBER(cass_w);
	DECLARE_QUICKLOAD_LOAD_MEMBER(cd2650);
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

private:
	uint8_t m_term_data;
	virtual void machine_reset() override;
	required_device<cpu_device> m_maincpu;
	required_shared_ptr<uint8_t> m_p_videoram;
	required_region_ptr<u8> m_p_chargen;
	required_device<beep_device> m_beep;
	required_device<cassette_image_device> m_cass;
};


WRITE8_MEMBER( cd2650_state::beep_w )
{
	if (data & 7)
		m_beep->set_state(BIT(data, 3));
}

WRITE_LINE_MEMBER( cd2650_state::cass_w )
{
	m_cass->output(state ? -1.0 : +1.0);
}

READ_LINE_MEMBER( cd2650_state::cass_r )
{
	return (m_cass->input() > 0.03) ? 1 : 0;
}

READ8_MEMBER( cd2650_state::keyin_r )
{
	uint8_t ret = m_term_data;
	m_term_data = ret | 0x80;
	return ret;
}

static ADDRESS_MAP_START(cd2650_mem, AS_PROGRAM, 8, cd2650_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE( 0x0000, 0x03ff) AM_ROM AM_REGION("roms", 0)
	AM_RANGE( 0x1000, 0x7fff) AM_RAM AM_SHARE("videoram")
ADDRESS_MAP_END

static ADDRESS_MAP_START( cd2650_io, AS_IO, 8, cd2650_state)
	ADDRESS_MAP_UNMAP_HIGH
	//AM_RANGE(0x80, 0x84) disk i/o
ADDRESS_MAP_END

static ADDRESS_MAP_START( cd2650_data, AS_DATA, 8, cd2650_state)
	AM_RANGE(S2650_DATA_PORT,S2650_DATA_PORT) AM_READWRITE(keyin_r, beep_w)
ADDRESS_MAP_END

/* Input ports */
static INPUT_PORTS_START( cd2650 )
INPUT_PORTS_END


void cd2650_state::machine_reset()
{
	m_term_data = 0x80;
	m_beep->set_state(0);
}

uint32_t cd2650_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
/* The video is unusual in that the characters in each line are spaced at 16 bytes in memory,
    thus line 1 starts at 1000, line 2 at 1001, etc. There are 16 lines of 80 characters.
    Further, the letters have bit 6 set low, thus the range is 01 to 1A.
    When the bottom of the screen is reached, it does not scroll, it just wraps around. */

	uint16_t offset = 0;
	uint8_t y,ra,chr,gfx;
	uint16_t sy=0,x,mem;

	for (y = 0; y < 16; y++)
	{
		for (ra = 0; ra < 10; ra++)
		{
			uint16_t *p = &bitmap.pix16(sy++);

			for (x = 0; x < 80; x++)
			{
				gfx = 0;
				if ((ra) && (ra < 9))
				{
					mem = offset + y + (x<<4);

					if (mem > 0x4ff)
						mem -= 0x500;

					chr = m_p_videoram[mem] & 0x3f;

					gfx = m_p_chargen[(BITSWAP8(chr,7,6,2,1,0,3,4,5)<<3) | (ra-1) ];
				}

				/* Display a scanline of a character */
				*p++ = BIT(gfx, 7);
				*p++ = BIT(gfx, 6);
				*p++ = BIT(gfx, 5);
				*p++ = BIT(gfx, 4);
				*p++ = BIT(gfx, 3);
				*p++ = BIT(gfx, 2);
				*p++ = BIT(gfx, 1);
				*p++ = BIT(gfx, 0);
			}
		}
	}
	return 0;
}

/* F4 Character Displayer */
static const gfx_layout cd2650_charlayout =
{
	8, 8,                  /* 8 x 8 characters */
	192,                    /* 64 characters in char.rom + 128 characters in char2.rom */
	1,                  /* 1 bits per pixel */
	{ 0 },                  /* no bitplanes */
	/* x offsets */
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	/* y offsets */
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	8*8                    /* every char takes 8 bytes */
};

static GFXDECODE_START( cd2650 )
	GFXDECODE_ENTRY( "chargen", 0x0000, cd2650_charlayout, 0, 1 )
GFXDECODE_END

void cd2650_state::kbd_put(u8 data)
{
	if (data)
		m_term_data = data;
}

QUICKLOAD_LOAD_MEMBER( cd2650_state, cd2650 )
{
	int i;
	image_init_result result = image_init_result::FAIL;

	int quick_length = image.length();
	if (quick_length < 0x1500)
	{
		image.seterror(IMAGE_ERROR_INVALIDIMAGE, "File too short");
		image.message(" File too short");
	}
	else
	if (quick_length > 0x8000)
	{
		image.seterror(IMAGE_ERROR_INVALIDIMAGE, "File too long");
		image.message(" File too long");
	}
	else
	{
		std::vector<uint8_t> quick_data(quick_length);
		int read_ = image.fread( &quick_data[0], quick_length);
		if (read_ != quick_length)
		{
			image.seterror(IMAGE_ERROR_INVALIDIMAGE, "Cannot read the file");
			image.message(" Cannot read the file");
		}
		else
		if (quick_data[0] != 0x40)
		{
			image.seterror(IMAGE_ERROR_INVALIDIMAGE, "Invalid header");
			image.message(" Invalid header");
		}
		else
		{
			int exec_addr = quick_data[1] * 256 + quick_data[2];

			if (exec_addr >= quick_length)
			{
				image.seterror(IMAGE_ERROR_INVALIDIMAGE, "Exec address beyond end of file");
				image.message(" Exec address beyond end of file");
			}
			else
			{
				// do not overwite system area (17E0-17FF) otherwise chess3 has problems
				read_ = 0x17e0;
				if (quick_length < 0x17e0)
					read_ = quick_length;

				for (i = 0x1500; i < read_; i++)
					m_p_videoram[i-0x1000] = quick_data[i];

				if (quick_length > 0x17ff)
					for (i = 0x1800; i < quick_length; i++)
						m_p_videoram[i-0x1000] = quick_data[i];

				/* display a message about the loaded quickload */
				image.message(" Quickload: size=%04X : exec=%04X",quick_length,exec_addr);

				// Start the quickload
				m_maincpu->set_state_int(S2650_PC, exec_addr);

				result = image_init_result::PASS;
			}
		}
	}

	return result;
}

static MACHINE_CONFIG_START( cd2650 )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu",S2650, XTAL_1MHz)
	MCFG_CPU_PROGRAM_MAP(cd2650_mem)
	MCFG_CPU_IO_MAP(cd2650_io)
	MCFG_CPU_DATA_MAP(cd2650_data)
	MCFG_S2650_SENSE_INPUT(READLINE(cd2650_state, cass_r))
	MCFG_S2650_FLAG_OUTPUT(WRITELINE(cd2650_state, cass_w))

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) /* not accurate */
	MCFG_SCREEN_UPDATE_DRIVER(cd2650_state, screen_update)
	MCFG_SCREEN_SIZE(640, 160)
	MCFG_SCREEN_VISIBLE_AREA(0, 639, 0, 159)
	MCFG_SCREEN_PALETTE("palette")
	MCFG_GFXDECODE_ADD("gfxdecode", "palette", cd2650)
	MCFG_PALETTE_ADD_MONOCHROME("palette")

	/* quickload */
	MCFG_QUICKLOAD_ADD("quickload", cd2650_state, cd2650, "pgm", 1)

	/* Sound */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_WAVE_ADD(WAVE_TAG, "cassette")
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)
	MCFG_SOUND_ADD("beeper", BEEP, 950) // guess
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)

	/* Devices */
	MCFG_DEVICE_ADD("keyboard", GENERIC_KEYBOARD, 0)
	MCFG_GENERIC_KEYBOARD_CB(PUT(cd2650_state, kbd_put))
	MCFG_CASSETTE_ADD( "cassette" )
MACHINE_CONFIG_END

/* ROM definition */
ROM_START( cd2650 )
	ROM_REGION( 0x0400, "roms", 0 )
	ROM_LOAD( "cd2650.rom", 0x0000, 0x0400, CRC(5397328e) SHA1(7106fdb60e1ad2bc5e8e45527f348c23296e8d6a))

	ROM_REGION( 0x0600, "chargen", 0 )
	ROM_LOAD( "char.rom",   0x0000, 0x0200, CRC(9b75db2a) SHA1(4367c01afa503d7cba0c38078fde0b95392c6c2c))
	ROM_LOAD_OPTIONAL( "char2.rom",  0x0200, 0x0400, CRC(b450eea8) SHA1(c1bdba52c2dc5698cad03b6b884b942a083465ed)) // not used

	// various unused roms found on Amigan site
	ROM_REGION( 0xc900, "user1", 0 )
	ROM_LOAD_OPTIONAL( "01a_cd_boots.bin", 0x0000, 0x0200, CRC(5336c62f) SHA1(e94cf7be01ea806ff7c7b90aee1a4e88f4f1ba9f))
	ROM_LOAD_OPTIONAL( "01a_cd_dos.bin",   0x0200, 0x2000, CRC(3f177cdd) SHA1(01afd77ad2f065158cbe032aa26682cb20afe7d8))
	ROM_LOAD_OPTIONAL( "01a_cd_pop.bin",   0x2200, 0x1000, CRC(d8f44f11) SHA1(605ab5a045290fa5b99ff4fc8fbfa2a3f202578f))
	ROM_LOAD_OPTIONAL( "01b_cd_alp.bin",   0x3200, 0x2a00, CRC(b05568bb) SHA1(29e74633c0cd731c0be25313288cfffdae374236))
	ROM_LOAD_OPTIONAL( "01b_cd_basic.bin", 0x5c00, 0x3b00, CRC(0cf1e3d8) SHA1(3421e679c238aeea49cd170b34a6f344da4770a6))
	ROM_LOAD_OPTIONAL( "01b_cd_mon_m.bin", 0x9700, 0x0400, CRC(f6f19c08) SHA1(1984d85d57fc2a6c5a3bd51fbc58540d7129a0ae))
	ROM_LOAD_OPTIONAL( "01b_cd_mon_o.bin", 0x9b00, 0x0400, CRC(9d40b4dc) SHA1(35cffcbd983b7b37c878a15af44100568d0659d1))
	ROM_LOAD_OPTIONAL( "02b_cd_alp.bin",   0x9f00, 0x2a00, CRC(a66b7f32) SHA1(2588f9244b0ec6b861dcebe666d37d3fa88dd043))
ROM_END

/* Driver */

//    YEAR  NAME    PARENT  COMPAT   MACHINE    INPUT   CLASS          INIT  COMPANY         FULLNAME   FLAGS
COMP( 1977, cd2650, 0,      0,       cd2650,    cd2650, cd2650_state,  0,    "Central Data", "CD 2650", 0 )
