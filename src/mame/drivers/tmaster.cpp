// license:BSD-3-Clause
// copyright-holders:Luca Elia, Mariusz Wojcieszek, Peter Ferrie
/***************************************************************************

                           -= Touch Master Games =-

         driver by Luca Elia (l.elia@tin.it) and Mariusz Wojcieszek


CPU:    68000
Video:  Blitter with two layers and double buffering (Xilinx FPGA)
Sound:  OKI M6295
Input:  Microtouch touch screen
Other:  Dallas NVRAM + optional RTC

To Do:
 - Coin optics
 - Correct sound banking
 - Proper protection emulation in tm4k and later games
 - Find cause and fix hang in Solitaire Erotic (all Touchmaster version hang in this game)

To be dumped and added:

Touch Master 6000 <-- There doesn't seem to be a "6000" version
Touch Master 7000 *
Touch Master 8000 *

* There are alternate region/state version with modifications due to legal issues
  Touch Master (current set) is a Euro version, all other sets are "DOMESTIC" (AKA "Standard").
  Is there a Touch Master 6000?  TM5K is version 7.10, then TM7K is version 8, TM8K is version 9.xx

- There are known regional versions like California, Minnesota and likely others.  The difference
  between regional sets are not known at this time.

- Starting with Touch Master 2000, each later version is a chipswap for the mainboard.
   IE: Touch Master 8000 chips can update any Touch Master mainboard 2000 through 7000
  Each version (IE: 2000, 3000, 7000 ect) has different girls for Strip Poker ;-)

Touch Master 8000 part lists (from service bulletin):

A-5343-60194-3  U8  Sound
A-5343-60194-2  U51 Program code
A-5343-60194-1  U52 Program code
A-5343-60194-6  U36 Graphics
A-5343-60194-4  U37 Graphics
A-5343-60194-7  U38 Graphics
A-5343-60194-5  U39 Graphics
A-5343-60194-8  U40 Graphics
A-5343-60194-9  U41 Graphics
A-21657-007     Security Key

The above set is an undumped alternate set, maybe a Euro or special version.

+---------------------------------------------------------------+
|  W24257AK                          GRAPHICS.U37  GRAPHICS.U39 |
|             SECURITY.J12                                      |
| PROGRAM.U52          DS1232        GRAPHICS.U36  GRAPHICS.U38 |
|                                                               |
|  W24257AK                          GRAPHICS.U40  GRAPHICS.U41 |
|               68HC000FN12                                     |
| PROGRAM.U51                                                   |
|                                                               |
|  DS1225AB.U62                      XC3042A     W241024AJ (x2) |
|                                                               |
|   3.664MHZ  24MHz                              W241024AJ (x2) |
| SCN68681       CY7C128A       SOUND.U8    32MHz               |
|     LED2 LED1  CY7C128A                                       |
|    U62                              M6295                     |
-                                                               |
 |Serial Port              LED3                               J8|
-                                                            VOL|
|  J11    J2      J5    J3       J10        J9          J6    J1|
+---------------------------------------------------------------+

U62 is a 16 DIN for a RTC chip (optional)
J Connectors used for all input/output and power. PCB is NON-JAMMA

Chips:
   CPU: MC68HC000FN12
 Video: XC3042A (Sigma Xilinx FPGA gate array)
 Sound: OKI M6295
   OSC: 32MHz, 24MHz & 8.664MHz
 Other: SCN68681C1N40 (Serial controller chip)
        DALLAS DS1225AB-85 Nonvolatile SRAM
        DALLAS DS1204V (used for security)
        DALLAS DS1232 (MicroMonitor Chip)
   RAM: W24257AK-15 (x2 used for CPU data)
        CY7C128A-55PC (x2 used for serial communication)
        W241024AJ-15 (x4 used for blitter frame buffer)

***************************************************************************/

#include "emu.h"
#include "cpu/m68000/m68000.h"
#include "machine/ds1204.h"
#include "machine/mc68681.h"
#include "machine/microtch.h"
#include "machine/nvram.h"
#include "machine/watchdog.h"
#include "sound/okim6295.h"
#include "video/cesblit.h"
#include "screen.h"
#include "speaker.h"

/***************************************************************************

                                 General

***************************************************************************/

class tmaster_state : public driver_device
{
public:
	tmaster_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_blitter(*this, "blitter"),
		m_oki(*this, "oki"),
		m_microtouch(*this,"microtouch"),
		m_duart(*this, "duart68681")
	{ }

	DECLARE_WRITE_LINE_MEMBER(blitter_irq_callback);

	TIMER_DEVICE_CALLBACK_MEMBER(scanline_interrupt);
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	DECLARE_WRITE_LINE_MEMBER(write_oki_bank0);
	DECLARE_WRITE_LINE_MEMBER(write_oki_bank1);

	DECLARE_WRITE_LINE_MEMBER(duart_irq_handler);

	DECLARE_READ16_MEMBER(rtc_r);
	DECLARE_WRITE16_MEMBER(rtc_w);

	DECLARE_READ_LINE_MEMBER(read_rand);

protected:
	required_device<cpu_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	required_device<cesblit_device> m_blitter;
	required_device<okim6295_device> m_oki;
	required_device<microtouch_device> m_microtouch;
	required_device<mc68681_device> m_duart;

	int m_okibank;

	uint8_t m_rtc_ram[8];
};

WRITE_LINE_MEMBER(tmaster_state::blitter_irq_callback)
{
//  logerror("%s: Blitter IRQ callback state = %x\n", machine().describe_context(), state);
	m_maincpu->set_input_line(2, state);
}

/***************************************************************************

                                   Video

***************************************************************************/

uint32_t tmaster_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(m_palette->black_pen(), cliprect);
	return m_blitter->screen_update(screen, bitmap, cliprect);
}

/***************************************************************************

                                   Sound

***************************************************************************/

WRITE_LINE_MEMBER(tmaster_state::write_oki_bank0)
{
	if (state)
		m_okibank |= 1;
	else
		m_okibank &= ~1;

	m_oki->set_rom_bank(m_okibank);
}

WRITE_LINE_MEMBER(tmaster_state::write_oki_bank1)
{
	if (state)
		m_okibank |= 2;
	else
		m_okibank &= ~2;

	m_oki->set_rom_bank(m_okibank);
}

/***************************************************************************

    68681 DUART <-> Microtouch touch screen controller communication

***************************************************************************/

WRITE_LINE_MEMBER(tmaster_state::duart_irq_handler)
{
	m_maincpu->set_input_line_and_vector(4, state, m_duart->get_irq_vector());
}

/***************************************************************************

  DS1644 RTC

***************************************************************************/

uint8_t binary_to_bcd(uint8_t data)
{
	data %= 100;
	return ((data / 10) << 4) | (data % 10);
}

READ16_MEMBER(tmaster_state::rtc_r)
{
	system_time systime;

	machine().current_datetime(systime);
	m_rtc_ram[0x1] = binary_to_bcd(systime.local_time.second);
	m_rtc_ram[0x2] = binary_to_bcd(systime.local_time.minute);
	m_rtc_ram[0x3] = binary_to_bcd(systime.local_time.hour);
	m_rtc_ram[0x4] = binary_to_bcd(systime.local_time.weekday);
	m_rtc_ram[0x5] = binary_to_bcd(systime.local_time.mday);
	m_rtc_ram[0x6] = binary_to_bcd(systime.local_time.month+1);
	m_rtc_ram[0x7] = binary_to_bcd(systime.local_time.year % 100);

	return m_rtc_ram[offset];
}

WRITE16_MEMBER(tmaster_state::rtc_w)
{
	if ( offset == 0 )
	{
		m_rtc_ram[0x0] = data & 0xff;
	}
}

/***************************************************************************

                                Memory Maps

***************************************************************************/

READ_LINE_MEMBER(tmaster_state::read_rand)
{
	return machine().rand() & 1;
}

static ADDRESS_MAP_START( tmaster_map, AS_PROGRAM, 16, tmaster_state )
	AM_RANGE( 0x000000, 0x1fffff ) AM_ROM
	AM_RANGE( 0x200000, 0x27ffff ) AM_RAM
	AM_RANGE( 0x280000, 0x28ffef ) AM_RAM AM_SHARE("nvram")
	AM_RANGE( 0x28fff0, 0x28ffff ) AM_READWRITE(rtc_r, rtc_w )

	AM_RANGE( 0x300010, 0x300011 ) AM_READ_PORT("COIN")

	AM_RANGE( 0x300020, 0x30003f ) AM_DEVREADWRITE8("duart68681", mc68681_device, read, write, 0xff )

	AM_RANGE( 0x300040, 0x300041 ) AM_WRITE_PORT("OUT")

	AM_RANGE( 0x300070, 0x300071 ) AM_DEVWRITE("blitter", cesblit_device, addr_hi_w )

	AM_RANGE( 0x500000, 0x500011 ) AM_DEVWRITE("blitter", cesblit_device, regs_w )
	AM_RANGE( 0x500010, 0x500011 ) AM_DEVREAD( "blitter", cesblit_device, status_r )

	AM_RANGE( 0x580000, 0x580001 ) AM_WRITENOP // often

	AM_RANGE( 0x600000, 0x601fff ) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")

	AM_RANGE( 0x800000, 0x800001 ) AM_DEVREADWRITE8("oki", okim6295_device, read, write, 0x00ff )

	AM_RANGE( 0x800010, 0x800011 ) AM_DEVWRITE("blitter", cesblit_device, color_w )
ADDRESS_MAP_END

/***************************************************************************

                                Input Ports

***************************************************************************/

static INPUT_PORTS_START( tm )
	PORT_START("COIN")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW,  IPT_COIN5    )    // "M. Coin 1 Input"
	PORT_BIT( 0x0002, IP_ACTIVE_LOW,  IPT_COIN6    )    // "M. Coin 2 Input"
	PORT_BIT( 0x0004, IP_ACTIVE_LOW,  IPT_BILL1    ) PORT_IMPULSE(2)    // "DBV Input"
	PORT_BIT( 0x0008, IP_ACTIVE_LOW,  IPT_UNKNOWN  )
	PORT_BIT( 0x0010, IP_ACTIVE_LOW,  IPT_UNKNOWN  )    // service coin?
	PORT_SERVICE_NO_TOGGLE( 0x0020, IP_ACTIVE_LOW  )
	PORT_BIT( 0x0040, IP_ACTIVE_LOW,  IPT_SERVICE1 )    // "Calibrate"
	PORT_BIT( 0x0080, IP_ACTIVE_LOW,  IPT_UNKNOWN  )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW,  IPT_UNKNOWN  )
	PORT_BIT( 0x0200, IP_ACTIVE_LOW,  IPT_UNKNOWN  )
	PORT_BIT( 0x0400, IP_ACTIVE_LOW,  IPT_UNKNOWN  )
	PORT_BIT( 0x0800, IP_ACTIVE_HIGH, IPT_SPECIAL  ) PORT_READ_LINE_DEVICE_MEMBER( DEVICE_SELF, tmaster_state, read_rand )
	PORT_BIT( 0x1000, IP_ACTIVE_LOW,  IPT_COIN1    )    // "E. Coin 1" (ECA?) tmaster defaults to e. coin,
	PORT_BIT( 0x2000, IP_ACTIVE_LOW,  IPT_COIN2    )    // "E. Coin 2" (ECA?) rather than m. coin
	PORT_BIT( 0x4000, IP_ACTIVE_LOW,  IPT_COIN3    )    // "E. Coin 3" (ECA?) so they're coin1-coin4
	PORT_BIT( 0x8000, IP_ACTIVE_LOW,  IPT_COIN4    )    // "E. Coin 4" (ECA?)

	PORT_START("OUT")
	//0800?
	PORT_BIT( 0x0200, IP_ACTIVE_HIGH, IPT_OUTPUT ) PORT_WRITE_LINE_DEVICE_MEMBER( DEVICE_SELF, tmaster_state, write_oki_bank1 )
	PORT_BIT( 0x0100, IP_ACTIVE_HIGH, IPT_OUTPUT ) PORT_WRITE_LINE_DEVICE_MEMBER( DEVICE_SELF, tmaster_state, write_oki_bank0 )
	//0002?
INPUT_PORTS_END

static INPUT_PORTS_START( tm2k )
	PORT_INCLUDE( tm )

	PORT_MODIFY("COIN")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_COIN1 )    // "M. Coin 1 Input"
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_COIN2 )    // "M. Coin 2 Input"
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_COIN3 )    // "E. Coin 1" (ECA mech) The rest of the tm games
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_COIN4 )    // "E. Coin 2" (ECA mech) Default to m. coin
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_COIN5 )    // "E. Coin 3" (ECA mech) So these are coin3-coin6
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_COIN6 )    // "E. Coin 4" (ECA mech)
INPUT_PORTS_END

static INPUT_PORTS_START( tm4k )
	PORT_INCLUDE( tm2k )

	PORT_MODIFY("COIN")
	PORT_BIT( 0x0400, IP_ACTIVE_HIGH, IPT_SPECIAL ) PORT_READ_LINE_DEVICE_MEMBER( "ds1204", ds1204_device, read_dq )

	PORT_MODIFY( "OUT" )
	PORT_BIT( 0x1000, IP_ACTIVE_HIGH, IPT_OUTPUT ) PORT_WRITE_LINE_DEVICE_MEMBER( "ds1204", ds1204_device, write_rst )
	PORT_BIT( 0x2000, IP_ACTIVE_LOW,  IPT_OUTPUT ) PORT_WRITE_LINE_DEVICE_MEMBER( "ds1204", ds1204_device, write_dq )
	PORT_BIT( 0x4000, IP_ACTIVE_HIGH, IPT_OUTPUT ) PORT_WRITE_LINE_DEVICE_MEMBER( "ds1204", ds1204_device, write_clk )
INPUT_PORTS_END

/***************************************************************************

                               Machine Drivers

***************************************************************************/

TIMER_DEVICE_CALLBACK_MEMBER(tmaster_state::scanline_interrupt)
{
	int scanline = param;

	if (scanline == 0) // vblank, FIXME
		m_maincpu->set_input_line(3, HOLD_LINE);
	else if ((scanline % 16) == 0)
		m_maincpu->set_input_line(1, HOLD_LINE);

	// lev 2 triggered at the end of the blit
}

int tmaster_compute_addr(uint16_t reg_low, uint16_t reg_mid, uint16_t reg_high)
{
	return (reg_low & 0xff) | ((reg_mid & 0x1ff) << 8) | (reg_high << 17);
}

static MACHINE_CONFIG_START( tm )
	MCFG_CPU_ADD("maincpu", M68000, XTAL_24MHz / 2) /* 12MHz */
	MCFG_CPU_PROGRAM_MAP(tmaster_map)
	MCFG_TIMER_DRIVER_ADD_SCANLINE("scantimer", tmaster_state, scanline_interrupt, "screen", 0, 1)

	MCFG_MC68681_ADD( "duart68681", XTAL_8_664MHz / 2 /*??*/)
	MCFG_MC68681_IRQ_CALLBACK(WRITELINE(tmaster_state, duart_irq_handler))
	MCFG_MC68681_A_TX_CALLBACK(DEVWRITELINE("microtouch", microtouch_device, rx))

	MCFG_MICROTOUCH_ADD( "microtouch", 9600, DEVWRITELINE("duart68681", mc68681_device, rx_a_w) )

	MCFG_NVRAM_ADD_0FILL("nvram")

	// video hardware
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(400, 256)
	MCFG_SCREEN_VISIBLE_AREA(0, 400-1, 0, 256-1)
	MCFG_SCREEN_UPDATE_DRIVER(tmaster_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 0x1000)
	MCFG_PALETTE_FORMAT(xBBBBBGGGGGRRRRR)

	MCFG_CESBLIT_ADD("blitter", "screen", XTAL_32MHz)
	MCFG_CESBLIT_COMPUTE_ADDR(tmaster_compute_addr)
	MCFG_CESBLIT_IRQ_CB(WRITELINE(tmaster_state, blitter_irq_callback))

	// sound hardware
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_OKIM6295_ADD("oki", XTAL_24MHz / 16, PIN7_HIGH)  /* 1.5Mhz? clock frequency & pin 7 not verified */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( tmds1204, tm )
	MCFG_DS1204_ADD("ds1204")
MACHINE_CONFIG_END

/***************************************************************************

                               ROMs Loading

***************************************************************************/

/***************************************************************************

Touch Master
1996, Midway

68000 @ 12MHz
u51 - u52 program code
u36 -> u39 gfx
u8 sound
OKI6295
NVSRAM DS1225a
Philips SCN68681
Xlinx XC3042a

***************************************************************************/

ROM_START( tm )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tmaster_v3.00_euro.u51", 0x000000, 0x080000, CRC(edaa5874) SHA1(48b99bc7f5a6453def265967ca7d8eefdf9dc97b) ) /* Ver: 3.00 Euro 11-25-96 */
	ROM_LOAD16_BYTE( "tmaster_v3.00_euro.u52", 0x000001, 0x080000, CRC(e9fd30fc) SHA1(d91ea05d5f574603883336729fb9df705688945d) ) /* Ver: 3.00 Euro 11-25-96 */

	ROM_REGION( 0x400000, "blitter", ROMREGION_ERASE )  // Blitter gfx
	ROM_LOAD16_BYTE( "tmaster_v2.1.u38", 0x100000, 0x080000, CRC(68885ef6) SHA1(010602b59c33c3e490491a296ddaf8952e315b83) ) /* Marked as Rev 2.1 */
	ROM_LOAD16_BYTE( "tmaster_v2.1.u36", 0x100001, 0x080000, CRC(204096ec) SHA1(9239923b7eedb6003c63ef2e8ff224edee657bbc) ) /* Marked as Rev 2.1 */
	// unused gap
	ROM_LOAD16_BYTE( "tmaster_v3.00.u39", 0x300000, 0x080000, CRC(cbb716cb) SHA1(4e8d8f6cbfb25a8161ff8fe7505d6b209650dd2b) ) /* Marked as Rev 3.00 */
	ROM_LOAD16_BYTE( "tmaster_v3.00.u37", 0x300001, 0x080000, CRC(e0b6a9f7) SHA1(7e057ca87833c682e5be03668469259bbdefbf20) ) /* Marked as Rev 3.00 */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tmaster.u8", 0x40000, 0x040000, CRC(f39ad4cf) SHA1(9bcb9a5dd3636d6541eeb3e737c7253ab0ed4e8d) ) /* Marked as Rev 1.0 */
	ROM_CONTINUE(           0xc0000, 0x040000 )
ROM_END

ROM_START( tmdo )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tmaster_v2.2-01.u51", 0x000000, 0x080000, CRC(12e1b085) SHA1(b01325e0067204991a707db24e3e2036708ebccf) ) /* Ver: 2.2-01 Standard 10-17-96 */
	ROM_LOAD16_BYTE( "tmaster_v2.2-01.u52", 0x000001, 0x080000, CRC(6c2c643f) SHA1(8dd7930f4c499483ca46b0b97bde94cb8d6e06aa) ) /* Ver: 2.2-01 Standard 10-17-96 */

	ROM_REGION( 0x400000, "blitter", ROMREGION_ERASE )  // Blitter gfx
	ROM_LOAD16_BYTE( "tmaster_v2.1.u38", 0x100000, 0x080000, CRC(68885ef6) SHA1(010602b59c33c3e490491a296ddaf8952e315b83) ) /* Marked as Rev 2.1 */
	ROM_LOAD16_BYTE( "tmaster_v2.1.u36", 0x100001, 0x080000, CRC(204096ec) SHA1(9239923b7eedb6003c63ef2e8ff224edee657bbc) ) /* Marked as Rev 2.1 */
	// unused gap
	ROM_LOAD16_BYTE( "tmaster_v2.1.u39", 0x300000, 0x080000, CRC(a4445260) SHA1(915347f69d7ea45f8f299a67d77ff437983495d2) ) /* Marked as Rev 2.1 */
	ROM_LOAD16_BYTE( "tmaster_v2.1.u37", 0x300001, 0x080000, CRC(0e140a3e) SHA1(10a34e3b95c0d36fe687fe8c1124ef244a93d720) ) /* Marked as Rev 2.1 */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tmaster.u8", 0x40000, 0x040000, CRC(f39ad4cf) SHA1(9bcb9a5dd3636d6541eeb3e737c7253ab0ed4e8d) ) /* Marked as Rev 1.0 */
	ROM_CONTINUE(           0xc0000, 0x040000 )
ROM_END

/***************************************************************************

Touchmaster 2000
by Midway (c) 1996
touchscreen game

All chips are ST M27C4001
---------------------------

Name_Board Location        Version               Use             Checksum
-------------------------------------------------------------------------
TM2K_v463.u51              4.63 Game Program & Cpu instructions   2342
TM2K_v463.u52              4.63 Game Program & Cpu instructions   4619

TM2K_v402.u51              4.02 Game Program & Cpu instructions   c517
TM2K_v402.u52              4.02 Game Program & Cpu instructions   e82c

TM2K_v400.u51              4.00 Game Program & Cpu instructions   a49b
TM2K_v400.u52              4.00 Game Program & Cpu instructions   7c2f

TM2K_graphic.u36           4.00 Video Images & Graphics           20cb
TM2K_graphic.u37           4.00 Video Images & Graphics           f5cf
TM2K_graphic.u38           4.00 Video Images & Graphics           14c7
TM2K_graphic.u39           4.00 Video Images & Graphics           043e
TM2K_graphic.u40           4.62 Video Images & Graphics           14c7
TM2K_graphic.u41           4.62 Video Images & Graphics           9334
TM2K_sound.u8              1.0  Audio Program & sounds            dbde

Does not require a security key

***************************************************************************/

ROM_START( tm2k )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm2k_v4.63.u51", 0x000000, 0x100000, CRC(f73fdb23) SHA1(d0a5e98d5de85fd3f29d1efe6b7aebe9e348c59b) ) /* Ver: 4.63 Standard 9-3-97 */
	ROM_LOAD16_BYTE( "tm2k_v4.63.u52", 0x000001, 0x100000, CRC(79ac719c) SHA1(7717362ea42fc36009e0fed430f07c946553bfd3) ) /* Ver: 4.63 Standard 9-3-97 */

	ROM_REGION( 0x600000, "blitter", ROMREGION_ERASE )  // Blitter gfx
	ROM_LOAD16_BYTE( "tm2k_graphic.u38", 0x100000, 0x080000, CRC(22bb6cc5) SHA1(fc6cfd4e1e6e1455d648a7b63f2c8e37cdfe86d6) ) /* First 4 graphic roms marked as Rev 4.00 */
	ROM_LOAD16_BYTE( "tm2k_graphic.u36", 0x100001, 0x080000, CRC(7f0840ac) SHA1(1c3af419d571579a3f2c561617d55914d28ef22b) )
	ROM_LOAD16_BYTE( "tm2k_graphic.u39", 0x300000, 0x080000, CRC(059e1bd8) SHA1(7451c1cfa0d090b0566e353738a1ffba732a8ad2) )
	ROM_LOAD16_BYTE( "tm2k_graphic.u37", 0x300001, 0x080000, CRC(4cf65950) SHA1(74d49166da19ecc4b8fc1e8e3f01361dfb645eea) )
	ROM_LOAD16_BYTE( "tm2k_graphic.u41", 0x500000, 0x080000, CRC(abac4ad3) SHA1(d3944a39b46f3e67ddb0ff7047685c6c716a393c) ) /* Last 2 graphics roms marked as Rev 4.62 */
	ROM_LOAD16_BYTE( "tm2k_graphic.u40", 0x500001, 0x080000, CRC(ca86b9a2) SHA1(bb639af4e0ee48c3231de5f0a0f14de20836216a) )

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm2k_sound.u8", 0x40000, 0x040000, CRC(f39ad4cf) SHA1(9bcb9a5dd3636d6541eeb3e737c7253ab0ed4e8d) ) /* Marked as Rev 1.0 */
	ROM_CONTINUE(              0xc0000, 0x040000 )
ROM_END

ROM_START( tm2ka )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm2k_v4.02.u51", 0x000000, 0x080000, CRC(47269aeb) SHA1(6b7ebfde290f7d21a36a72b00dc6523490581edb) ) /* Ver: 4.02 Standard 5-30-97 */
	ROM_LOAD16_BYTE( "tm2k_v4.02.u52", 0x000001, 0x080000, CRC(2e3564ac) SHA1(9a71f38841bc17c291cb3f513b18ebe50fc18d9f) ) /* Ver: 4.02 Standard 5-30-97 */

	ROM_REGION( 0x400000, "blitter", ROMREGION_ERASE )  // Blitter gfx
	ROM_LOAD16_BYTE( "tm2k_graphic.u38", 0x100000, 0x080000, CRC(22bb6cc5) SHA1(fc6cfd4e1e6e1455d648a7b63f2c8e37cdfe86d6) ) /* All 4 graphic roms marked as Rev 4.00 */
	ROM_LOAD16_BYTE( "tm2k_graphic.u36", 0x100001, 0x080000, CRC(7f0840ac) SHA1(1c3af419d571579a3f2c561617d55914d28ef22b) )
	ROM_LOAD16_BYTE( "tm2k_graphic.u39", 0x300000, 0x080000, CRC(059e1bd8) SHA1(7451c1cfa0d090b0566e353738a1ffba732a8ad2) )
	ROM_LOAD16_BYTE( "tm2k_graphic.u37", 0x300001, 0x080000, CRC(4cf65950) SHA1(74d49166da19ecc4b8fc1e8e3f01361dfb645eea) )
	/* Sockets U40 & U41 not populated with earlier Touchmaster 2000 sets */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm2k_sound.u8", 0x40000, 0x040000, CRC(f39ad4cf) SHA1(9bcb9a5dd3636d6541eeb3e737c7253ab0ed4e8d) ) /* Marked as Rev 1.0 */
	ROM_CONTINUE(              0xc0000, 0x040000 )
ROM_END

ROM_START( tm2kb )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm2k_v4.00.u51", 0x000000, 0x080000, CRC(c110502b) SHA1(e9415ed23b9bb0851548e75c208ebcbe6ac2a708) ) /* Ver: 4.00 Standard 5-16-97 */
	ROM_LOAD16_BYTE( "tm2k_v4.00.u52", 0x000001, 0x080000, CRC(a17c1d6e) SHA1(5ecb8f27f75469ab9600b3f640eb1acc7a3980e0) ) /* Ver: 4.00 Standard 5-16-97 */

	ROM_REGION( 0x400000, "blitter", ROMREGION_ERASE )  // Blitter gfx
	ROM_LOAD16_BYTE( "tm2k_graphic.u38", 0x100000, 0x080000, CRC(22bb6cc5) SHA1(fc6cfd4e1e6e1455d648a7b63f2c8e37cdfe86d6) ) /* All 4 graphic roms marked as Rev 4.00 */
	ROM_LOAD16_BYTE( "tm2k_graphic.u36", 0x100001, 0x080000, CRC(7f0840ac) SHA1(1c3af419d571579a3f2c561617d55914d28ef22b) )
	ROM_LOAD16_BYTE( "tm2k_graphic.u39", 0x300000, 0x080000, CRC(059e1bd8) SHA1(7451c1cfa0d090b0566e353738a1ffba732a8ad2) )
	ROM_LOAD16_BYTE( "tm2k_graphic.u37", 0x300001, 0x080000, CRC(4cf65950) SHA1(74d49166da19ecc4b8fc1e8e3f01361dfb645eea) )
	/* Sockets U40 & U41 not populated with earlier Touchmaster 2000 sets */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm2k_sound.u8", 0x40000, 0x040000, CRC(f39ad4cf) SHA1(9bcb9a5dd3636d6541eeb3e737c7253ab0ed4e8d) ) /* Marked as Rev 1.0 */
	ROM_CONTINUE(              0xc0000, 0x040000 )
ROM_END

ROM_START( tm2kspeval ) /* NOT FOR RELEASE / FOR EVALUATION - still requires credits - Defaults to Spanish but English can be selected */
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm2k_spain_4.xx3.u51", 0x000000, 0x080000, CRC(ba51f4dd) SHA1(bcdf6acbe6546a562e74178d4f9cdb0e167baee2) ) /* Ver: 4.XX3 Spanish 6-26-97 - Evaluation Only */
	ROM_LOAD16_BYTE( "tm2k_spain_4.xx3.u52", 0x000001, 0x080000, CRC(6ca86264) SHA1(2fb04b8d4bd0a51653ca80e1af7fedab5aebb9dd) ) /* Ver: 4.XX3 Spanish 6-26-97 - Evaluation Only */

	ROM_REGION( 0x400000, "blitter", ROMREGION_ERASE )  // Blitter gfx
	ROM_LOAD16_BYTE( "tm2k_graphic.u38", 0x100000, 0x080000, CRC(22bb6cc5) SHA1(fc6cfd4e1e6e1455d648a7b63f2c8e37cdfe86d6) ) /* All 4 graphic roms marked as Rev 4.X */
	ROM_LOAD16_BYTE( "tm2k_graphic.u36", 0x100001, 0x080000, CRC(7f0840ac) SHA1(1c3af419d571579a3f2c561617d55914d28ef22b) )
	ROM_LOAD16_BYTE( "tm2k_graphic.u39", 0x300000, 0x080000, CRC(059e1bd8) SHA1(7451c1cfa0d090b0566e353738a1ffba732a8ad2) )
	ROM_LOAD16_BYTE( "tm2k_graphic.u37", 0x300001, 0x080000, CRC(4cf65950) SHA1(74d49166da19ecc4b8fc1e8e3f01361dfb645eea) )
	/* Sockets U40 & U41 not populated with earlier Touchmaster 2000 sets */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm2k_sound.u8", 0x40000, 0x040000, CRC(f39ad4cf) SHA1(9bcb9a5dd3636d6541eeb3e737c7253ab0ed4e8d) ) /* Marked as Rev 1.0 */
	ROM_CONTINUE(              0xc0000, 0x040000 )
ROM_END

/***************************************************************************

Touchmaster 3000
by Midway (c) 1997
touchscreen game

All chips are SGS 27C801 (some kits/upgrades used mask roms)
---------------------------

Name_Board Location        Version               Use             Checksum
-------------------------------------------------------------------------
TM3K_v502.u51              5.02 Game Program & Cpu instructions   c308
TM3K_v502.u52              5.02 Game Program & Cpu instructions   4d5e

TM3K_v501.u51              5.01 Game Program & Cpu instructions   0c6c
TM3K_v501.u52              5.01 Game Program & Cpu instructions   b2d8

TM3K_graphic.u36           5.0  Video Images & Graphics           54f1
TM3K_graphic.u37           5.0  Video Images & Graphics           4856
TM3K_graphic.u38           5.0  Video Images & Graphics           5493
TM3K_graphic.u39           5.0  Video Images & Graphics           6029
TM3K_graphic.u40           5.0  Video Images & Graphics           ccb4
TM3K_graphic.u41           5.0  Video Images & Graphics           e9ab
TM3K_sound.u8              5.0  Audio Program & sounds            64d5

Does not require a security key

Box labeled as:

TMSTR 3000 UPGRADE KIT USA-DBV
63373100870

***************************************************************************/

ROM_START( tm3k )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm3k_v5.02.u51", 0x000000, 0x100000, CRC(6267e2bd) SHA1(c81e5cd059a9ad2f6a36261738e39740a1a3a03f) ) /* TOUCHMASTER 3000 U51 DOMESTIC 5.02 (Standard 11-17-97) (yellow label) */
	ROM_LOAD16_BYTE( "tm3k_v5.02.u52", 0x000001, 0x100000, CRC(836fdf1e) SHA1(2ee9c0929950afb72f172b253d6c392e9a698037) ) /* TOUCHMASTER 3000 U52 DOMESTIC 5.02 (Standard 11-17-97) (yellow label) */

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm3k_graphic.u38", 0x000000, 0x100000, CRC(a6683899) SHA1(d05024390917cdb1871d030996da8e1eb6460918) ) /* Labeled TOUCHMASTER U38 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u36", 0x000001, 0x100000, CRC(7bde520d) SHA1(77750b689e2f0d47804042456e54bbd9c28deeac) ) /* Labeled TOUCHMASTER U36 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u39", 0x200000, 0x100000, CRC(206b56a6) SHA1(09e5e05bffd0a09abd24d668e2c59b56f2c79134) ) /* Labeled TOUCHMASTER U39 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u37", 0x200001, 0x100000, CRC(18f50eb3) SHA1(a7c9d3b24b5fd110380ec87d9200d55cad473efc) ) /* Labeled TOUCHMASTER U37 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u41", 0x400000, 0x100000, CRC(c35c0536) SHA1(a29fd88e8f3e124f6e84012c3573616f6447eeaa) ) /* Labeled TOUCHMASTER U41 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u40", 0x400001, 0x100000, CRC(353df7ca) SHA1(d6c5d5449af6b6a3acee219778583904c5b554b4) ) /* Labeled TOUCHMASTER U40 STANDARD 5.0 (pink label) */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm3k_sound.u8", 0x00000, 0x100000, CRC(d0ae33c1) SHA1(a079def9a086a091fcc4493a44fec756d2470415) ) /* Labeled TOUCHMASTER U8 5.0 (green label) */
ROM_END

ROM_START( tm3ksp ) /* Defaults to Spanish but can also be played in English */
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm3k_v5.02_spain.u51", 0x000000, 0x100000, CRC(e5d1d465) SHA1(30ecf32c146a611a257d97bd6789b46ce2695024) ) /* TOUCHMASTER U51 SPAIN 5.02 (Spanish 11-25-97) (yellow label) */
	ROM_LOAD16_BYTE( "tm3k_v5.02_spain.u52", 0x000001, 0x100000, CRC(21c5fb8b) SHA1(5575925d6dc7548f5ccb04876d8c15b24601e3c0) ) /* TOUCHMASTER U52 SPAIN 5.02 (Spanish 11-25-97) (yellow label) */

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm3k_graphic.u38", 0x000000, 0x100000, CRC(a6683899) SHA1(d05024390917cdb1871d030996da8e1eb6460918) ) /* Labeled TOUCHMASTER U38 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u36", 0x000001, 0x100000, CRC(7bde520d) SHA1(77750b689e2f0d47804042456e54bbd9c28deeac) ) /* Labeled TOUCHMASTER U36 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u39", 0x200000, 0x100000, CRC(206b56a6) SHA1(09e5e05bffd0a09abd24d668e2c59b56f2c79134) ) /* Labeled TOUCHMASTER U39 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u37", 0x200001, 0x100000, CRC(18f50eb3) SHA1(a7c9d3b24b5fd110380ec87d9200d55cad473efc) ) /* Labeled TOUCHMASTER U37 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u41", 0x400000, 0x100000, CRC(c35c0536) SHA1(a29fd88e8f3e124f6e84012c3573616f6447eeaa) ) /* Labeled TOUCHMASTER U41 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u40", 0x400001, 0x100000, CRC(353df7ca) SHA1(d6c5d5449af6b6a3acee219778583904c5b554b4) ) /* Labeled TOUCHMASTER U40 STANDARD 5.0 (pink label) */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm3k_sound.u8", 0x00000, 0x100000, CRC(d0ae33c1) SHA1(a079def9a086a091fcc4493a44fec756d2470415) ) /* Labeled TOUCHMASTER U8 5.0 (green label) */
ROM_END

ROM_START( tm3ka )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm3k_v5.01.u51", 0x000000, 0x100000, CRC(c9522279) SHA1(e613b791f831271722f05b7e96c35519fa9fc174) ) /* TOUCHMASTER 3000 U51 DOMESTIC 5.01 (Standard 11-4-97) (yellow label) */
	ROM_LOAD16_BYTE( "tm3k_v5.01.u52", 0x000001, 0x100000, CRC(8c6a0db7) SHA1(6b0eae60ea471cd8c4001749ac2677d8d4532567) ) /* TOUCHMASTER 3000 U52 DOMESTIC 5.01 (Standard 11-4-97) (yellow label) */

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm3k_graphic.u38", 0x000000, 0x100000, CRC(a6683899) SHA1(d05024390917cdb1871d030996da8e1eb6460918) ) /* Labeled TOUCHMASTER U38 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u36", 0x000001, 0x100000, CRC(7bde520d) SHA1(77750b689e2f0d47804042456e54bbd9c28deeac) ) /* Labeled TOUCHMASTER U36 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u39", 0x200000, 0x100000, CRC(206b56a6) SHA1(09e5e05bffd0a09abd24d668e2c59b56f2c79134) ) /* Labeled TOUCHMASTER U39 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u37", 0x200001, 0x100000, CRC(18f50eb3) SHA1(a7c9d3b24b5fd110380ec87d9200d55cad473efc) ) /* Labeled TOUCHMASTER U37 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u41", 0x400000, 0x100000, CRC(c35c0536) SHA1(a29fd88e8f3e124f6e84012c3573616f6447eeaa) ) /* Labeled TOUCHMASTER U41 STANDARD 5.0 (pink label) */
	ROM_LOAD16_BYTE( "tm3k_graphic.u40", 0x400001, 0x100000, CRC(353df7ca) SHA1(d6c5d5449af6b6a3acee219778583904c5b554b4) ) /* Labeled TOUCHMASTER U40 STANDARD 5.0 (pink label) */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm3k_sound.u8", 0x00000, 0x100000, CRC(d0ae33c1) SHA1(a079def9a086a091fcc4493a44fec756d2470415) ) /* Labeled TOUCHMASTER U8 5.0 (green label) */
ROM_END

ROM_START( tm3keval ) /* FREEPLAY ONLY / NOT FOR RELEASE / FOR EVALUATION ONLY */
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "u51_touchmaster_10-21-97_a903.u51", 0x000000, 0x100000, CRC(4ddf94aa) SHA1(733134f835a30db48e6edb7eef95dc7a8f680955) ) /* U51  TOUCHMASTER 10-21-97 A903 (Standard AMOA 10-21-97) */
	ROM_LOAD16_BYTE( "u52_touchmaster_10-21-97_dab0.u52", 0x000001, 0x100000, CRC(a1e1eb81) SHA1(d2e116872ff41cce9f6c228ac0ce7a3250f21054) ) /* U51  TOUCHMASTER 10-21-97 DAB0 (Standard AMOA 10-21-97) */

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "u38_touchmaster_10-21-97_0067.u38", 0x000000, 0x100000, CRC(cc2af2b0) SHA1(64de224599e038b98ffedd965ac83b077298097f) ) /* U38  TOUCHMASTER 10-21-97 0067 (date and checksum hand written) */
	ROM_LOAD16_BYTE( "u36_touchmaster_10-21-97_ba57.u36", 0x000001, 0x100000, CRC(73e99b1c) SHA1(812d6958a8746cc8f5cba125486e1f72740a0840) ) /* U36  TOUCHMASTER 10-21-97 BA57 (date and checksum hand written) */
	ROM_LOAD16_BYTE( "u39_touchmaster_10-21-97_4a76.u39", 0x200000, 0x100000, CRC(af92cf87) SHA1(cb6baf8b3afe1e61e440be69a90d5c001eb7b388) ) /* U39  TOUCHMASTER 10-21-97 4A76 (date and checksum hand written) */
	ROM_LOAD16_BYTE( "u37_touchmaster_10-21-97_40F6.u37", 0x200001, 0x100000, CRC(b5e69eca) SHA1(c548e33a049d17381c9755499940ffc459c0cb48) ) /* U37  TOUCHMASTER 10-21-97 40F6 (date and checksum hand written) */
	ROM_LOAD16_BYTE( "u41_touchmaster_10-21-97_f986.u41", 0x400000, 0x100000, CRC(97bc5e19) SHA1(9a9786ec2a9a7b7008d9e2e8624df8301f737dbc) ) /* U41  TOUCHMASTER 10-21-97 F986 (date and checksum hand written) */
	ROM_LOAD16_BYTE( "u40_touchmaster_10-21-97_7086.u40", 0x400001, 0x100000, CRC(e3ae6fa6) SHA1(02c98f59553f4343f0dd855794364ad95aa81063) ) /* U40  TOUCHMASTER 10-21-97 7086 (date and checksum hand written) */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm3k_sound.u8", 0x00000, 0x100000, CRC(d0ae33c1) SHA1(a079def9a086a091fcc4493a44fec756d2470415) ) /* Labeled TOUCHMASTER U8 5.0 (green label) */
ROM_END

/***************************************************************************

Touchmaster 4000
by Midway (c) 1998
touchscreen game

All chips are SGS 27C801 (some kits/upgrades used mask roms)
---------------------------

Name_Board Location        Version               Use                      Checksum
-----------------------------------------------------------------------------------
TM4K_v603.u51              6.03 Game Program & Cpu instructions 96B0
TM4K_v603.u52              6.03 Game Program & Cpu instructions 2842

TM4K_v602.u51              6.02 Game Program & Cpu instructions FEA0
TM4K_v602.u52              6.02 Game Program & Cpu instructions 9A71

TM4K_v601.u51              6.01 Game Program & Cpu instructions 6FF1
TM4K_v601.u52              6.01 Game Program & Cpu instructions 6643

TM4K_graphic.u36           6.0  Video Images & Graphics         54F1 (same as TM3K)
TM4K_graphic.u37           6.0  Video Images & Graphics         609E
TM4K_graphic.u38           6.0  Video Images & Graphics         5493 (same as TM3K)
TM4K_graphic.u39           6.0  Video Images & Graphics         CB90
TM4K_graphic.u40           6.0  Video Images & Graphics         208A
TM4K_graphic.u41           6.0  Video Images & Graphics         385D
TM4K_sound.u8              6.0  Audio Program & sounds          DE0B

J12 DALLAS DS1204V         N/A  Security Key (required for this Version) - Labeled A-21657-003

***************************************************************************/

ROM_START( tm4k )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm4k_v6.03.u51", 0x000000, 0x100000, CRC(0c88c856) SHA1(4c60c09812ab05f9dccef3038a5ddbd4632dbf4e) ) /* TOUCHMASTER 4000 U51 DOMESTIC  6.03 (Standard 6-23-98) */
	ROM_LOAD16_BYTE( "tm4k_v6.03.u52", 0x000001, 0x100000, CRC(9320bfe9) SHA1(cc3a51f439c139ca30efe28a817cf4f68679180e) ) /* TOUCHMASTER 4000 U52 DOMESTIC  6.03 (Standard 6-23-98) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-003", 0x0000, 0x000022, CRC(b9facb2a) SHA1(17157534f45ec6db78f952586bd98f1f7e7215c1) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm4k_graphic.u38", 0x000000, 0x100000, CRC(a6683899) SHA1(d05024390917cdb1871d030996da8e1eb6460918) ) /* Mask rom labeled 5341-15746-03 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u36", 0x000001, 0x100000, CRC(7bde520d) SHA1(77750b689e2f0d47804042456e54bbd9c28deeac) ) /* Mask rom labeled 5341-15746-01 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u39", 0x200000, 0x100000, CRC(bac88cfb) SHA1(26ed169296b890c5f5b50c418c15299355a6592f) ) /* Mask rom labeled 5341-15746-04 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u37", 0x200001, 0x100000, CRC(bf49fafa) SHA1(b400667bf654dc9cd01a85c8b99670459400fd60) ) /* Mask rom labeled 5341-15746-02 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u41", 0x400000, 0x100000, CRC(e97edb1e) SHA1(75510676cf1692ad03efd4ccd57d25af1cc8ef2a) ) /* Mask rom labeled 5341-15746-06 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u40", 0x400001, 0x100000, CRC(f6771a09) SHA1(74f71d5e910006c83a38170f24aa811c38a3e020) ) /* Mask rom labeled 5341-15746-05 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm4k_sound.u8", 0x00000, 0x100000, CRC(48c3782b) SHA1(bfe105ddbde8bbbd84665dfdd565d6d41926834a) ) /* Mask rom labeled 5341-15746-07 U8 SOUND IMAGE */
ROM_END

ROM_START( tm4knj ) /* New Jersey version only */
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm4knj_v6.03.u51", 0x000000, 0x100000, CRC(fcaf10d2) SHA1(67bf584223c4bb7436bd9a03eb799895ecd26a26) ) /* TOUCHMASTER 4000 U51 NEW JERSEY  6.03 (New Jersey 3-23-98) */
	ROM_LOAD16_BYTE( "tm4knj_v6.03.u52", 0x000001, 0x100000, CRC(bed6f9e9) SHA1(695509a2b47e1d70cdbad765f581605b31065b8c) ) /* TOUCHMASTER 4000 U52 NEW JERSEY  6.03 (New Jersey 3-23-98) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-003", 0x0000, 0x000022, CRC(b9facb2a) SHA1(17157534f45ec6db78f952586bd98f1f7e7215c1) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm4k_graphic.u38", 0x000000, 0x100000, CRC(a6683899) SHA1(d05024390917cdb1871d030996da8e1eb6460918) ) /* Mask rom labeled 5341-15746-03 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u36", 0x000001, 0x100000, CRC(7bde520d) SHA1(77750b689e2f0d47804042456e54bbd9c28deeac) ) /* Mask rom labeled 5341-15746-01 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u39", 0x200000, 0x100000, CRC(bac88cfb) SHA1(26ed169296b890c5f5b50c418c15299355a6592f) ) /* Mask rom labeled 5341-15746-04 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u37", 0x200001, 0x100000, CRC(bf49fafa) SHA1(b400667bf654dc9cd01a85c8b99670459400fd60) ) /* Mask rom labeled 5341-15746-02 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u41", 0x400000, 0x100000, CRC(e97edb1e) SHA1(75510676cf1692ad03efd4ccd57d25af1cc8ef2a) ) /* Mask rom labeled 5341-15746-06 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u40", 0x400001, 0x100000, CRC(f6771a09) SHA1(74f71d5e910006c83a38170f24aa811c38a3e020) ) /* Mask rom labeled 5341-15746-05 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm4k_sound.u8", 0x00000, 0x100000, CRC(48c3782b) SHA1(bfe105ddbde8bbbd84665dfdd565d6d41926834a) ) /* Mask rom labeled 5341-15746-07 U8 SOUND IMAGE */
ROM_END

ROM_START( tm4ka )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm4k_v6.02.u51", 0x000000, 0x100000, CRC(3d8d7848) SHA1(31638f23cdd5e6cfbb2270e953f84fe1bd437950) ) /* TOUCHMASTER 4000 U51 DOMESTIC  6.02 (Standard 4-14-98) */
	ROM_LOAD16_BYTE( "tm4k_v6.02.u52", 0x000001, 0x100000, CRC(6d412871) SHA1(ae27c7723b292daf6682c53bafac22e4a3cd1ece) ) /* TOUCHMASTER 4000 U52 DOMESTIC  6.02 (Standard 4-14-98) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-003", 0x0000, 0x000022, CRC(b9facb2a) SHA1(17157534f45ec6db78f952586bd98f1f7e7215c1) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm4k_graphic.u38", 0x000000, 0x100000, CRC(a6683899) SHA1(d05024390917cdb1871d030996da8e1eb6460918) ) /* Mask rom labeled 5341-15746-03 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u36", 0x000001, 0x100000, CRC(7bde520d) SHA1(77750b689e2f0d47804042456e54bbd9c28deeac) ) /* Mask rom labeled 5341-15746-01 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u39", 0x200000, 0x100000, CRC(bac88cfb) SHA1(26ed169296b890c5f5b50c418c15299355a6592f) ) /* Mask rom labeled 5341-15746-04 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u37", 0x200001, 0x100000, CRC(bf49fafa) SHA1(b400667bf654dc9cd01a85c8b99670459400fd60) ) /* Mask rom labeled 5341-15746-02 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u41", 0x400000, 0x100000, CRC(e97edb1e) SHA1(75510676cf1692ad03efd4ccd57d25af1cc8ef2a) ) /* Mask rom labeled 5341-15746-06 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u40", 0x400001, 0x100000, CRC(f6771a09) SHA1(74f71d5e910006c83a38170f24aa811c38a3e020) ) /* Mask rom labeled 5341-15746-05 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm4k_sound.u8", 0x00000, 0x100000, CRC(48c3782b) SHA1(bfe105ddbde8bbbd84665dfdd565d6d41926834a) ) /* Mask rom labeled 5341-15746-07 U8 SOUND IMAGE */
ROM_END

ROM_START( tm4kca ) /* California version only */
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm4k_v6.02ca.u51", 0x000000, 0x100000, CRC(b355e15e) SHA1(e6a6cd74d3439f589ac0a914ec66962ce2885c00) ) /* TOUCHMASTER 4000 U51 CALIFORNIA  6.02 (CA. 4-14-98) */
	ROM_LOAD16_BYTE( "tm4k_v6.02ca.u52", 0x000001, 0x100000, CRC(0adbbfc0) SHA1(4660bd95649bfc65d3e52975d168de4963bd27cb) ) /* TOUCHMASTER 4000 U52 CALIFORNIA  6.02 (CA. 4-14-98) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-003", 0x0000, 0x000022, CRC(b9facb2a) SHA1(17157534f45ec6db78f952586bd98f1f7e7215c1) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm4k_graphic.u38", 0x000000, 0x100000, CRC(a6683899) SHA1(d05024390917cdb1871d030996da8e1eb6460918) ) /* Mask rom labeled 5341-15746-03 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u36", 0x000001, 0x100000, CRC(7bde520d) SHA1(77750b689e2f0d47804042456e54bbd9c28deeac) ) /* Mask rom labeled 5341-15746-01 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u39", 0x200000, 0x100000, CRC(bac88cfb) SHA1(26ed169296b890c5f5b50c418c15299355a6592f) ) /* Mask rom labeled 5341-15746-04 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u37", 0x200001, 0x100000, CRC(bf49fafa) SHA1(b400667bf654dc9cd01a85c8b99670459400fd60) ) /* Mask rom labeled 5341-15746-02 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u41", 0x400000, 0x100000, CRC(e97edb1e) SHA1(75510676cf1692ad03efd4ccd57d25af1cc8ef2a) ) /* Mask rom labeled 5341-15746-06 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u40", 0x400001, 0x100000, CRC(f6771a09) SHA1(74f71d5e910006c83a38170f24aa811c38a3e020) ) /* Mask rom labeled 5341-15746-05 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm4k_sound.u8", 0x00000, 0x100000, CRC(48c3782b) SHA1(bfe105ddbde8bbbd84665dfdd565d6d41926834a) ) /* Mask rom labeled 5341-15746-07 U8 SOUND IMAGE */
ROM_END

ROM_START( tm4kb )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm4k_v6.01.u51", 0x000000, 0x100000, CRC(cdcfd064) SHA1(51f022d25411d119a5f16ff7f09f4bed59b937e1) ) /* TOUCHMASTER 4000 U51 DOMESTIC  6.01 (Standard 3-23-98) */
	ROM_LOAD16_BYTE( "tm4k_v6.01.u52", 0x000001, 0x100000, CRC(e0bf71a9) SHA1(8f5e70dee60cd95aceac4707ff73bdff578a6139) ) /* TOUCHMASTER 4000 U52 DOMESTIC  6.01 (Standard 3-23-98) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-003", 0x0000, 0x000022, CRC(b9facb2a) SHA1(17157534f45ec6db78f952586bd98f1f7e7215c1) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm4k_graphic.u38", 0x000000, 0x100000, CRC(a6683899) SHA1(d05024390917cdb1871d030996da8e1eb6460918) ) /* Mask rom labeled 5341-15746-03 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u36", 0x000001, 0x100000, CRC(7bde520d) SHA1(77750b689e2f0d47804042456e54bbd9c28deeac) ) /* Mask rom labeled 5341-15746-01 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u39", 0x200000, 0x100000, CRC(bac88cfb) SHA1(26ed169296b890c5f5b50c418c15299355a6592f) ) /* Mask rom labeled 5341-15746-04 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u37", 0x200001, 0x100000, CRC(bf49fafa) SHA1(b400667bf654dc9cd01a85c8b99670459400fd60) ) /* Mask rom labeled 5341-15746-02 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u41", 0x400000, 0x100000, CRC(e97edb1e) SHA1(75510676cf1692ad03efd4ccd57d25af1cc8ef2a) ) /* Mask rom labeled 5341-15746-06 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u40", 0x400001, 0x100000, CRC(f6771a09) SHA1(74f71d5e910006c83a38170f24aa811c38a3e020) ) /* Mask rom labeled 5341-15746-05 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm4k_sound.u8", 0x00000, 0x100000, CRC(48c3782b) SHA1(bfe105ddbde8bbbd84665dfdd565d6d41926834a) ) /* Mask rom labeled 5341-15746-07 U8 SOUND IMAGE */
ROM_END

ROM_START( tm4kmn ) /* Minnesota only version */
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm4kmn_v6.01.u51", 0x000000, 0x100000, CRC(66b7f936) SHA1(7d13b24440f15bc29587b35d32f8a34019cc2462) ) /* TOUCHMASTER 4000 U51 MINNESOTA  6.01 (MN. 3-30-98) */
	ROM_LOAD16_BYTE( "tm4kmn_v6.01.u52", 0x000001, 0x100000, CRC(34fa24bb) SHA1(4c4f706f8f09bfd1f81fbf71b377c0eb2cbfa97d) ) /* TOUCHMASTER 4000 U52 MINNESOTA  6.01 (MN. 3-30-98) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-003", 0x0000, 0x000022, CRC(b9facb2a) SHA1(17157534f45ec6db78f952586bd98f1f7e7215c1) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm4k_graphic.u38", 0x000000, 0x100000, CRC(a6683899) SHA1(d05024390917cdb1871d030996da8e1eb6460918) ) /* Mask rom labeled 5341-15746-03 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u36", 0x000001, 0x100000, CRC(7bde520d) SHA1(77750b689e2f0d47804042456e54bbd9c28deeac) ) /* Mask rom labeled 5341-15746-01 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u39", 0x200000, 0x100000, CRC(bac88cfb) SHA1(26ed169296b890c5f5b50c418c15299355a6592f) ) /* Mask rom labeled 5341-15746-04 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u37", 0x200001, 0x100000, CRC(bf49fafa) SHA1(b400667bf654dc9cd01a85c8b99670459400fd60) ) /* Mask rom labeled 5341-15746-02 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u41", 0x400000, 0x100000, CRC(e97edb1e) SHA1(75510676cf1692ad03efd4ccd57d25af1cc8ef2a) ) /* Mask rom labeled 5341-15746-06 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm4k_graphic.u40", 0x400001, 0x100000, CRC(f6771a09) SHA1(74f71d5e910006c83a38170f24aa811c38a3e020) ) /* Mask rom labeled 5341-15746-05 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm4k_sound.u8", 0x00000, 0x100000, CRC(48c3782b) SHA1(bfe105ddbde8bbbd84665dfdd565d6d41926834a) ) /* Mask rom labeled 5341-15746-07 U8 SOUND IMAGE */
ROM_END

/***************************************************************************

Touchmaster 5000
by Midway (c) 1998
touchscreen game

All chips are ST M27C801 (some kits/upgrades used mask roms)
---------------------------

Name Board Location        Version               Use                      Checksum
-----------------------------------------------------------------------------------
tm5k_v7_10.u51             7.10 Game Program & Cpu instructions 1A51
tm5k_v7_10.u52             7.10 Game Program & Cpu instructions 5A01
tm5k_graphic.u36           7.0  Video Images & Graphics         DB7F
tm5k_graphic.u37           7.0  Video Images & Graphics         871B
tm5k_graphic.u38           7.0  Video Images & Graphics         EDCE
tm5k_graphic.u39           7.0  Video Images & Graphics         657F
tm5k_graphic.u40           7.0  Video Images & Graphics         93E0
tm5k_graphic.u41           7.0  Video Images & Graphics         FCA5
tm5k_sound.u8              7.0  Audio Program & sounds          F474

J12 DALLAS DS1204V         N/A  Security Key (required for this Version) - Labeled A-21657-004
-----------------------------------------------------------------------------------

* EPROM versions are labeled with white labels

***************************************************************************/

ROM_START( tm5k )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm5k_v7.10.u51", 0x000000, 0x100000, CRC(df0bd25e) SHA1(db1a197ed4c868743397f3823f3f1d42b9329f80) ) /* TOUCHMASTER 5000 U51 DOMESTIC 7.10 (Standard 10-9-98) (tan label) */
	ROM_LOAD16_BYTE( "tm5k_v7.10.u52", 0x000001, 0x100000, CRC(ddf9e8dc) SHA1(3228f2eba067bdf1bd639116bffc589585ea3e72) ) /* TOUCHMASTER 5000 U52 DOMESTIC 7.10 (Standard 10-9-98) (tan label) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-004", 0x0000, 0x000022, CRC(14b5d50a) SHA1(6c8d3f9a3b44ce5f8273a2f86a6c0f98423b7c7d) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm5k_graphic.u38", 0x000000, 0x100000, CRC(93038e7c) SHA1(448f69bf51ac992f6b35b471cba9675c67984cd7) ) /* Mask rom labeled 5341-15951-07 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u36", 0x000001, 0x100000, CRC(5453a44a) SHA1(094439a56336ca933b0b7ede8c057546d1d490b2) ) /* Mask rom labeled 5341-15951-06 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u39", 0x200000, 0x100000, CRC(1349fdc7) SHA1(5118983be584455320f0d6006133f38e6a8ee0d8) ) /* Mask rom labeled 5341-15951-05 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u37", 0x200001, 0x100000, CRC(8bcc376c) SHA1(0588f6f96090b26a3ce0eb3a933a5cc9d8ce742d) ) /* Mask rom labeled 5341-15951-04 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u41", 0x400000, 0x100000, CRC(c8717fef) SHA1(fbd13321db0f35b7bdf207468f28792a7666bb2e) ) /* Mask rom labeled 5341-15951-09 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u40", 0x400001, 0x100000, CRC(cff3f962) SHA1(2389d94ffa0eaf2d2f62315822273db031eea033) ) /* Mask rom labeled 5341-15951-08 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm5k_sound.u8", 0x00000, 0x100000, CRC(c6070a60) SHA1(2dc20bf2217a36374b5a691133ad43f53dbe29ca) ) /* Mask rom labeled 5341-15951-03 U8 VIDEO IMAGE */
ROM_END

ROM_START( tm5kca ) /* California only version */
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm5kca_v7.10.u51", 0x000000, 0x100000, CRC(7c03708f) SHA1(e1f8198bc03c1f9d2a00662b589fd66fdcc7a2de) ) /* TOUCHMASTER 5000 U51 CALIFORNIA 7.10 (CA. 10-9-98) (tan label) */
	ROM_LOAD16_BYTE( "tm5kca_v7.10.u52", 0x000001, 0x100000, CRC(f1413295) SHA1(ad6b57c590bdae063b0e4a1abcb3b13ee52cd6db) ) /* TOUCHMASTER 5000 U52 CALIFORNIA 7.10 (CA. 10-9-98) (tan label) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-004", 0x0000, 0x000022, CRC(14b5d50a) SHA1(6c8d3f9a3b44ce5f8273a2f86a6c0f98423b7c7d) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm5k_graphic.u38", 0x000000, 0x100000, CRC(93038e7c) SHA1(448f69bf51ac992f6b35b471cba9675c67984cd7) ) /* Mask rom labeled 5341-15951-07 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u36", 0x000001, 0x100000, CRC(5453a44a) SHA1(094439a56336ca933b0b7ede8c057546d1d490b2) ) /* Mask rom labeled 5341-15951-06 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u39", 0x200000, 0x100000, CRC(1349fdc7) SHA1(5118983be584455320f0d6006133f38e6a8ee0d8) ) /* Mask rom labeled 5341-15951-05 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u37", 0x200001, 0x100000, CRC(8bcc376c) SHA1(0588f6f96090b26a3ce0eb3a933a5cc9d8ce742d) ) /* Mask rom labeled 5341-15951-04 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u41", 0x400000, 0x100000, CRC(c8717fef) SHA1(fbd13321db0f35b7bdf207468f28792a7666bb2e) ) /* Mask rom labeled 5341-15951-09 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u40", 0x400001, 0x100000, CRC(cff3f962) SHA1(2389d94ffa0eaf2d2f62315822273db031eea033) ) /* Mask rom labeled 5341-15951-08 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm5k_sound.u8", 0x00000, 0x100000, CRC(c6070a60) SHA1(2dc20bf2217a36374b5a691133ad43f53dbe29ca) ) /* Mask rom labeled 5341-15951-03 U8 VIDEO IMAGE */
ROM_END

ROM_START( tm5kmn ) /* Minnesota only version */
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm5kmn_v7.10.u51", 0x000000, 0x100000, CRC(eb12e430) SHA1(48de1d49025cd3fc844d0399d58efc630c51ab28) ) /* TOUCHMASTER 5000 U51 MINNESOTA 7.10 (MN. 10-9-98) (tan label) */
	ROM_LOAD16_BYTE( "tm5kmn_v7.10.u52", 0x000001, 0x100000, CRC(6a4e75d9) SHA1(b0049b5d5f649aedb568d71aacfcb648320dd53b) ) /* TOUCHMASTER 5000 U52 MINNESOTA 7.10 (MN. 10-9-98) (tan label) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-004", 0x0000, 0x000022, CRC(14b5d50a) SHA1(6c8d3f9a3b44ce5f8273a2f86a6c0f98423b7c7d) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm5k_graphic.u38", 0x000000, 0x100000, CRC(93038e7c) SHA1(448f69bf51ac992f6b35b471cba9675c67984cd7) ) /* Mask rom labeled 5341-15951-07 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u36", 0x000001, 0x100000, CRC(5453a44a) SHA1(094439a56336ca933b0b7ede8c057546d1d490b2) ) /* Mask rom labeled 5341-15951-06 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u39", 0x200000, 0x100000, CRC(1349fdc7) SHA1(5118983be584455320f0d6006133f38e6a8ee0d8) ) /* Mask rom labeled 5341-15951-05 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u37", 0x200001, 0x100000, CRC(8bcc376c) SHA1(0588f6f96090b26a3ce0eb3a933a5cc9d8ce742d) ) /* Mask rom labeled 5341-15951-04 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u41", 0x400000, 0x100000, CRC(c8717fef) SHA1(fbd13321db0f35b7bdf207468f28792a7666bb2e) ) /* Mask rom labeled 5341-15951-09 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u40", 0x400001, 0x100000, CRC(cff3f962) SHA1(2389d94ffa0eaf2d2f62315822273db031eea033) ) /* Mask rom labeled 5341-15951-08 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm5k_sound.u8", 0x00000, 0x100000, CRC(c6070a60) SHA1(2dc20bf2217a36374b5a691133ad43f53dbe29ca) ) /* Mask rom labeled 5341-15951-03 U8 VIDEO IMAGE */
ROM_END

ROM_START( tm5ka )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm5k_v7.01.u51", 0x000000, 0x100000, CRC(3ff68213) SHA1(ec4939899a69f56d2437c806fdd0a5b50e58ec6a) ) /* TOUCHMASTER 5000 U51 DOMESTIC 7.01 (Standard 8-21-98) (pink label) */
	ROM_LOAD16_BYTE( "tm5k_v7.01.u52", 0x000001, 0x100000, CRC(b3de607c) SHA1(4ede5b8c50177d1934a3b93d311a240ef354c450) ) /* TOUCHMASTER 5000 U52 DOMESTIC 7.01 (Standard 8-21-98) (pink label) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-004", 0x0000, 0x000022, CRC(14b5d50a) SHA1(6c8d3f9a3b44ce5f8273a2f86a6c0f98423b7c7d) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm5k_graphic.u38", 0x000000, 0x100000, CRC(93038e7c) SHA1(448f69bf51ac992f6b35b471cba9675c67984cd7) ) /* Mask rom labeled 5341-15951-07 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u36", 0x000001, 0x100000, CRC(5453a44a) SHA1(094439a56336ca933b0b7ede8c057546d1d490b2) ) /* Mask rom labeled 5341-15951-06 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u39", 0x200000, 0x100000, CRC(1349fdc7) SHA1(5118983be584455320f0d6006133f38e6a8ee0d8) ) /* Mask rom labeled 5341-15951-05 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u37", 0x200001, 0x100000, CRC(8bcc376c) SHA1(0588f6f96090b26a3ce0eb3a933a5cc9d8ce742d) ) /* Mask rom labeled 5341-15951-04 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u41", 0x400000, 0x100000, CRC(c8717fef) SHA1(fbd13321db0f35b7bdf207468f28792a7666bb2e) ) /* Mask rom labeled 5341-15951-09 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm5k_graphic.u40", 0x400001, 0x100000, CRC(cff3f962) SHA1(2389d94ffa0eaf2d2f62315822273db031eea033) ) /* Mask rom labeled 5341-15951-08 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm5k_sound.u8", 0x00000, 0x100000, CRC(c6070a60) SHA1(2dc20bf2217a36374b5a691133ad43f53dbe29ca) ) /* Mask rom labeled 5341-15951-03 U8 VIDEO IMAGE */
ROM_END

/***************************************************************************

Touchmaster 7000
by Midway (c) 1999
touchscreen game

All chips are ST M27C801 (some kits/upgrades used mask roms)
---------------------------

Name Board Location        Version               Use                      Checksum
-----------------------------------------------------------------------------------
tm7k_v804.u51              8.04 Game Program & Cpu instructions 321B
tm7k_v804.u52              8.04 Game Program & Cpu instructions 2DED

tm7k_v800.u51              8.00 Game Program & Cpu instructions 82A5
tm7k_v800.u52              8.00 Game Program & Cpu instructions 81E1

tm7k_graphic.u36           8.0  Video Images & Graphics         DB7F (same as TM5K)
tm7k_graphic.u37           8.0  Video Images & Graphics         7461
tm7k_graphic.u38           8.0  Video Images & Graphics         EDCE (same as TM5K)
tm7k_graphic.u39           8.0  Video Images & Graphics         2BB6
tm7k_graphic.u40           8.0  Video Images & Graphics         B103
tm7k_graphic.u41           8.0  Video Images & Graphics         11CA
tm7k_sound.u8              8.0  Audio Program & sounds          F474 (same as TM5K)

J12 DALLAS DS1204V         N/A  Security Key (required for this Version) - Labeled A-21657-005
-----------------------------------------------------------------------------------

***************************************************************************/

ROM_START( tm7k )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm7k_v8.04.u51", 0x000000, 0x100000, CRC(2461af04) SHA1(9cf37c04db0297ff8f9f316fd476d6d5d1c39acf) ) /* TOUCHMASTER 7000 U51 DOMESTIC 8.04 (Standard 06/02/99) (orange label) */
	ROM_LOAD16_BYTE( "tm7k_v8.04.u52", 0x000001, 0x100000, CRC(5d39fad2) SHA1(85e8d110b88e1099117ab7963eaee47dc86ec7c5) ) /* TOUCHMASTER 7000 U52 DOMESTIC 8.04 (Standard 06/02/99) (orange label) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-005", 0x000000, 0x000022, CRC(affa047b) SHA1(5a576ce3d0d0ec35994937c89daab4360f46690d) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm7k_graphic.u38", 0x000000, 0x100000, CRC(93038e7c) SHA1(448f69bf51ac992f6b35b471cba9675c67984cd7) ) /* Mask rom labeled 5341-16262-07 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u36", 0x000001, 0x100000, CRC(5453a44a) SHA1(094439a56336ca933b0b7ede8c057546d1d490b2) ) /* Mask rom labeled 5341-16262-06 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u39", 0x200000, 0x100000, CRC(26af8da8) SHA1(02555b1597a4962f1fd0c3ffc89e5c8338aa3085) ) /* Mask rom labeled 5341-16262-05 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u37", 0x200001, 0x100000, CRC(9a705043) SHA1(cffb31859544c1c4082be78b3bca5ad9cd0d2a45) ) /* Mask rom labeled 5341-16262-04 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u41", 0x400000, 0x100000, CRC(99b6edda) SHA1(c0ee2834fdbfbc1159a6d08c45552d4d9c1c4ea4) ) /* Mask rom labeled 5341-16262-09 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u40", 0x400001, 0x100000, CRC(a3925379) SHA1(74836325ab10466e23105a3b54fc706c0dd5f06c) ) /* Mask rom labeled 5341-16262-08 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm7k_sound.u8", 0x00000, 0x100000, CRC(c6070a60) SHA1(2dc20bf2217a36374b5a691133ad43f53dbe29ca) ) /* Mask rom labeled 5341-16262-03 U8 SOUND */
ROM_END

ROM_START( tm7knj ) /* New Jersey version only - ABC Exception No. 27 */
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm7knj_v8.05.u51", 0x000000, 0x100000, CRC(46a7ed33) SHA1(e9ceb5005a6e88bd179fca0b62477cea19b82443) ) /* TOUCHMASTER 7000 U51 NEW JERSEY 8.05 (Standard 07/22/99) (orange label) */
	ROM_LOAD16_BYTE( "tm7knj_v8.05.u52", 0x000001, 0x100000, CRC(127e8b09) SHA1(017d4cda6205f88256cbecabf0c94e71eef93bf9) ) /* TOUCHMASTER 7000 U52 NEW JERSEY 8.05 (Standard 07/22/99) (orange label) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-005", 0x000000, 0x000022, CRC(affa047b) SHA1(5a576ce3d0d0ec35994937c89daab4360f46690d) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm7k_graphic.u38", 0x000000, 0x100000, CRC(93038e7c) SHA1(448f69bf51ac992f6b35b471cba9675c67984cd7) ) /* Mask rom labeled 5341-16262-07 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u36", 0x000001, 0x100000, CRC(5453a44a) SHA1(094439a56336ca933b0b7ede8c057546d1d490b2) ) /* Mask rom labeled 5341-16262-06 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u39", 0x200000, 0x100000, CRC(26af8da8) SHA1(02555b1597a4962f1fd0c3ffc89e5c8338aa3085) ) /* Mask rom labeled 5341-16262-05 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u37", 0x200001, 0x100000, CRC(9a705043) SHA1(cffb31859544c1c4082be78b3bca5ad9cd0d2a45) ) /* Mask rom labeled 5341-16262-04 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u41", 0x400000, 0x100000, CRC(99b6edda) SHA1(c0ee2834fdbfbc1159a6d08c45552d4d9c1c4ea4) ) /* Mask rom labeled 5341-16262-09 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u40", 0x400001, 0x100000, CRC(a3925379) SHA1(74836325ab10466e23105a3b54fc706c0dd5f06c) ) /* Mask rom labeled 5341-16262-08 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm7k_sound.u8", 0x00000, 0x100000, CRC(c6070a60) SHA1(2dc20bf2217a36374b5a691133ad43f53dbe29ca) ) /* Mask rom labeled 5341-16262-03 U8 SOUND */
ROM_END

ROM_START( tm7kmn ) /* Minnesota only version */
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm7kmn_v8.04.u51", 0x000000, 0x100000, CRC(c85b5adc) SHA1(418776917c1f419fd981deaab9754f4f03a2288f) ) /* TOUCHMASTER 7000 U51 MINNESOTA 8.04 (MN. 06/02/99) (orange label) */
	ROM_LOAD16_BYTE( "tm7kmn_v8.04.u52", 0x000001, 0x100000, CRC(542bc209) SHA1(b8314d695a36380f6be02b45eac83cfab857c258) ) /* TOUCHMASTER 7000 U52 MINNESOTA 8.04 (MN. 06/02/99) (orange label) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-005", 0x000000, 0x000022, CRC(affa047b) SHA1(5a576ce3d0d0ec35994937c89daab4360f46690d) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm7k_graphic.u38", 0x000000, 0x100000, CRC(93038e7c) SHA1(448f69bf51ac992f6b35b471cba9675c67984cd7) ) /* Mask rom labeled 5341-16262-07 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u36", 0x000001, 0x100000, CRC(5453a44a) SHA1(094439a56336ca933b0b7ede8c057546d1d490b2) ) /* Mask rom labeled 5341-16262-06 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u39", 0x200000, 0x100000, CRC(26af8da8) SHA1(02555b1597a4962f1fd0c3ffc89e5c8338aa3085) ) /* Mask rom labeled 5341-16262-05 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u37", 0x200001, 0x100000, CRC(9a705043) SHA1(cffb31859544c1c4082be78b3bca5ad9cd0d2a45) ) /* Mask rom labeled 5341-16262-04 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u41", 0x400000, 0x100000, CRC(99b6edda) SHA1(c0ee2834fdbfbc1159a6d08c45552d4d9c1c4ea4) ) /* Mask rom labeled 5341-16262-09 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u40", 0x400001, 0x100000, CRC(a3925379) SHA1(74836325ab10466e23105a3b54fc706c0dd5f06c) ) /* Mask rom labeled 5341-16262-08 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm7k_sound.u8", 0x00000, 0x100000, CRC(c6070a60) SHA1(2dc20bf2217a36374b5a691133ad43f53dbe29ca) ) /* Mask rom labeled 5341-16262-03 U8 SOUND */
ROM_END

ROM_START( tm7ka )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm7k_v8.00.u51", 0x000000, 0x100000, CRC(83ec3da7) SHA1(37fa7183e7acc2eab35ac431d99cbbfe4862979e) ) /* TOUCHMASTER 7000 U51 DOMESTIC 8.00 (Standard 03/26/99) (orange label) */
	ROM_LOAD16_BYTE( "tm7k_v8.00.u52", 0x000001, 0x100000, CRC(e2004282) SHA1(aa73029f31e2062cabedfcd778db97b314624ae8) ) /* TOUCHMASTER 7000 U52 DOMESTIC 8.00 (Standard 03/26/99) (orange label) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-005", 0x000000, 0x000022, CRC(affa047b) SHA1(5a576ce3d0d0ec35994937c89daab4360f46690d) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm7k_graphic.u38", 0x000000, 0x100000, CRC(93038e7c) SHA1(448f69bf51ac992f6b35b471cba9675c67984cd7) ) /* Labeled GRAPHIC U38  8.0 (orange label) */
	ROM_LOAD16_BYTE( "tm7k_graphic.u36", 0x000001, 0x100000, CRC(5453a44a) SHA1(094439a56336ca933b0b7ede8c057546d1d490b2) ) /* Labeled GRAPHIC U36  8.0 (orange label) */
	ROM_LOAD16_BYTE( "tm7k_graphic.u39", 0x200000, 0x100000, CRC(26af8da8) SHA1(02555b1597a4962f1fd0c3ffc89e5c8338aa3085) ) /* Labeled GRAPHIC U39  8.0 (orange label) */
	ROM_LOAD16_BYTE( "tm7k_graphic.u37", 0x200001, 0x100000, CRC(9a705043) SHA1(cffb31859544c1c4082be78b3bca5ad9cd0d2a45) ) /* Labeled GRAPHIC U37  8.0 (orange label) */
	ROM_LOAD16_BYTE( "tm7k_graphic.u41", 0x400000, 0x100000, CRC(99b6edda) SHA1(c0ee2834fdbfbc1159a6d08c45552d4d9c1c4ea4) ) /* Labeled GRAPHIC U41  8.0 (orange label) */
	ROM_LOAD16_BYTE( "tm7k_graphic.u40", 0x400001, 0x100000, CRC(a3925379) SHA1(74836325ab10466e23105a3b54fc706c0dd5f06c) ) /* Labeled GRAPHIC U40  8.0 (orange label) */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm7k_sound.u8", 0x00000, 0x100000, CRC(c6070a60) SHA1(2dc20bf2217a36374b5a691133ad43f53dbe29ca) ) /* Labeled SOUND U8  8.0 (orange label) */
ROM_END

ROM_START( tm7kmna ) /* Minnesota only version */
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm7kmn_v8.00.u51", 0x000000, 0x100000, CRC(4bc423e9) SHA1(c9ba044b4bc8431281e174cddc8697324cbbf072) ) /* TOUCHMASTER 7000 U51 MINNESOTA 8.00 (MN. 03/26/99) (orange label) */
	ROM_LOAD16_BYTE( "tm7kmn_v8.00.u52", 0x000001, 0x100000, CRC(51968948) SHA1(f8b63df4480cafa453f970fb07f73103d39e2903) ) /* TOUCHMASTER 7000 U52 MINNESOTA 8.00 (MN. 03/26/99) (orange label) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-005", 0x000000, 0x000022, CRC(affa047b) SHA1(5a576ce3d0d0ec35994937c89daab4360f46690d) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm7k_graphic.u38", 0x000000, 0x100000, CRC(93038e7c) SHA1(448f69bf51ac992f6b35b471cba9675c67984cd7) ) /* Mask rom labeled 5341-16262-07 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u36", 0x000001, 0x100000, CRC(5453a44a) SHA1(094439a56336ca933b0b7ede8c057546d1d490b2) ) /* Mask rom labeled 5341-16262-06 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u39", 0x200000, 0x100000, CRC(26af8da8) SHA1(02555b1597a4962f1fd0c3ffc89e5c8338aa3085) ) /* Mask rom labeled 5341-16262-05 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u37", 0x200001, 0x100000, CRC(9a705043) SHA1(cffb31859544c1c4082be78b3bca5ad9cd0d2a45) ) /* Mask rom labeled 5341-16262-04 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u41", 0x400000, 0x100000, CRC(99b6edda) SHA1(c0ee2834fdbfbc1159a6d08c45552d4d9c1c4ea4) ) /* Mask rom labeled 5341-16262-09 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm7k_graphic.u40", 0x400001, 0x100000, CRC(a3925379) SHA1(74836325ab10466e23105a3b54fc706c0dd5f06c) ) /* Mask rom labeled 5341-16262-08 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm7k_sound.u8", 0x00000, 0x100000, CRC(c6070a60) SHA1(2dc20bf2217a36374b5a691133ad43f53dbe29ca) ) /* Mask rom labeled 5341-16262-03 U8 SOUND */
ROM_END

ROM_START( tm7keval ) /* FREEPLAY ONLY / NOT FOR RELEASE / FOR EVALUATION ONLY */
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm7k_v8.1x.u51", 0x000000, 0x100000, CRC(57c88287) SHA1(1008bbcd137ede321be4cc2ad22e3982d880471b) ) /* TOUCHMASTER 7000 U51 8.1X (ASI Standard 03/08/99) (white label) */
	ROM_LOAD16_BYTE( "tm7k_v8.1x.u52", 0x000001, 0x100000, CRC(e45d69bb) SHA1(9e8a42924a9cd573cbd2f3164d0fb468eee7ff51) ) /* TOUCHMASTER 7000 U52 8.1X (ASI Standard 03/08/99) (white label) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-004", 0x000000, 0x000022, CRC(14b5d50a) SHA1(6c8d3f9a3b44ce5f8273a2f86a6c0f98423b7c7d) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm7k_graphic.u38", 0x000000, 0x100000, CRC(93038e7c) SHA1(448f69bf51ac992f6b35b471cba9675c67984cd7) ) /* TOUCHMASTER EDCE V8.X U38 (white label) */
	ROM_LOAD16_BYTE( "tm7k_graphic.u36", 0x000001, 0x100000, CRC(5453a44a) SHA1(094439a56336ca933b0b7ede8c057546d1d490b2) ) /* TOUCHMASTER DB7F V8.X U36 (white label) */
	ROM_LOAD16_BYTE( "tm7k_graphic.u39", 0x200000, 0x100000, CRC(26af8da8) SHA1(02555b1597a4962f1fd0c3ffc89e5c8338aa3085) ) /* TOUCHMASTER 2BB6 V8.X U39 (white label) */
	ROM_LOAD16_BYTE( "tm7k_graphic.u37", 0x200001, 0x100000, CRC(9a705043) SHA1(cffb31859544c1c4082be78b3bca5ad9cd0d2a45) ) /* TOUCHMASTER 7461 V8.X U37 (white label) */
	ROM_LOAD16_BYTE( "tm7k_v8x.u41",     0x400000, 0x100000, CRC(d15757f0) SHA1(d09a83dc00812a4208d5684e8cfba1f5d1e3f097) ) /* TOUCHMASTER 335E V8.X U41 (white label) */
	ROM_LOAD16_BYTE( "tm7k_v8x.u40",     0x400001, 0x100000, CRC(d3596f63) SHA1(a2474bc1a73b5782f8169b0529a535269ea82440) ) /* TOUCHMASTER 7987 V8.X U40 (white label) */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm7k_sound.u8", 0x00000, 0x100000, CRC(c6070a60) SHA1(2dc20bf2217a36374b5a691133ad43f53dbe29ca) ) /* Labeled SOUND U8  8.0 (orange label) */
ROM_END

/***************************************************************************

Touchmaster 8000
by Midway (c) 2000
touchscreen game

All chips are ST M27C801 (some kits/upgrades used mask roms)
---------------------------

Name Board Location        Version               Use                      Checksum
-----------------------------------------------------------------------------------
tm8k_v904.u51              9.04 Game Program & Cpu instructions D40F
tm8k_v904.u52              9.04 Game Program & Cpu instructions 53B2

tm8k_graphic.u36           9.0  Video Images & Graphics         AD8D
tm8k_graphic.u37           9.0  Video Images & Graphics         AF83
tm8k_graphic.u38           9.0  Video Images & Graphics         6BCF
tm8k_graphic.u39           9.0  Video Images & Graphics         C8A6
tm8k_graphic.u40           9.0  Video Images & Graphics         B8C7
tm8k_graphic.u41           9.0  Video Images & Graphics         EF93
tm8k_sound.u8              9.0  Audio Program & sounds          F474 (same as TM5K & TM7K)

J12 DALLAS DS1204V         N/A  Security Key (required for this Version) - Labeled A-21657-007
-----------------------------------------------------------------------------------

***************************************************************************/

/* According to the Distributor notice from Midway, the only thing updated for v9.04 was the "Power Cell" game to 'improve the performance'. */
ROM_START( tm8k )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm8k_v9.04.u51", 0x000000, 0x100000, CRC(28864ec8) SHA1(e703f9ee350dd915102e784bbd04445a95b7d0a5) ) /* TOUCHMASTER 8000 U51 DOMESTIC 9.04 (Standard 04/25/00) */
	ROM_LOAD16_BYTE( "tm8k_v9.04.u52", 0x000001, 0x100000, CRC(c123eec2) SHA1(3e9c84755b18a4fd900068f385ee47107771391d) ) /* TOUCHMASTER 8000 U52 DOMESTIC 9.04 (Standard 04/25/00) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-007", 0x000000, 0x000022, CRC(a1776e90) SHA1(40116d693b565fe472c278ea583b50e2b708630a) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm8k_graphic.u38", 0x000000, 0x100000, CRC(2a971d46) SHA1(6ca4067e9fa40053df415e670b2e853915319dbb) ) /* Mask rom labeled 5341-16513-07 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm8k_graphic.u36", 0x000001, 0x100000, CRC(3bde285e) SHA1(87bf60034665542fb0240b7479adfffb7ba9fad7) ) /* Mask rom labeled 5341-16513-06 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm8k_graphic.u39", 0x200000, 0x100000, CRC(58c6c1d8) SHA1(cc11863c4ea46bde7ea4775075f4328be6d6c6d1) ) /* Mask rom labeled 5341-16513-05 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm8k_graphic.u37", 0x200001, 0x100000, CRC(c0992f7a) SHA1(e4e1ef2414f2f0a784c775f39123122c08950403) ) /* Mask rom labeled 5341-16513-04 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm8k_graphic.u41", 0x400000, 0x100000, CRC(d8bdb82e) SHA1(9bdee261591ccff8a57c5454644f84f8992f614f) ) /* Mask rom labeled 5341-16513-09 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm8k_graphic.u40", 0x400001, 0x100000, CRC(0c3d6347) SHA1(7ef19018c180abf412a8ff9f278b00c2b4321cc2) ) /* Mask rom labeled 5341-16513-08 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm8k_sound.u8", 0x00000, 0x100000, CRC(c6070a60) SHA1(2dc20bf2217a36374b5a691133ad43f53dbe29ca) ) /* Mask rom labeled 5341-16513-03 U8 SOUND */
ROM_END

ROM_START( tm8k902 )
	ROM_REGION( 0x200000, "maincpu", 0 ) // 68000 Code
	ROM_LOAD16_BYTE( "tm8k_v9.02.u51", 0x000000, 0x100000, CRC(c0d95b2c) SHA1(2240d969047ada856ec2fac6cdf72db722753a6d) ) /* TOUCHMASTER 8000 U51 DOMESTIC 9.02 (Standard 03/17/00) */
	ROM_LOAD16_BYTE( "tm8k_v9.02.u52", 0x000001, 0x100000, CRC(5194c1b5) SHA1(28a91d12a022927bfe96afd62b30b268c0ced3ea) ) /* TOUCHMASTER 8000 U52 DOMESTIC 9.02 (Standard 03/17/00) */

	ROM_REGION( 0x000022, "ds1204", 0 )
	ROM_LOAD( "a-21657-007", 0x000000, 0x000022, CRC(a1776e90) SHA1(40116d693b565fe472c278ea583b50e2b708630a) )

	ROM_REGION( 0x600000, "blitter", 0 )    // Blitter gfx
	ROM_LOAD16_BYTE( "tm8k_graphic.u38", 0x000000, 0x100000, CRC(2a971d46) SHA1(6ca4067e9fa40053df415e670b2e853915319dbb) ) /* Mask rom labeled 5341-16513-07 U38 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm8k_graphic.u36", 0x000001, 0x100000, CRC(3bde285e) SHA1(87bf60034665542fb0240b7479adfffb7ba9fad7) ) /* Mask rom labeled 5341-16513-06 U36 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm8k_graphic.u39", 0x200000, 0x100000, CRC(58c6c1d8) SHA1(cc11863c4ea46bde7ea4775075f4328be6d6c6d1) ) /* Mask rom labeled 5341-16513-05 U39 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm8k_graphic.u37", 0x200001, 0x100000, CRC(c0992f7a) SHA1(e4e1ef2414f2f0a784c775f39123122c08950403) ) /* Mask rom labeled 5341-16513-04 U37 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm8k_graphic.u41", 0x400000, 0x100000, CRC(d8bdb82e) SHA1(9bdee261591ccff8a57c5454644f84f8992f614f) ) /* Mask rom labeled 5341-16513-09 U41 VIDEO IMAGE */
	ROM_LOAD16_BYTE( "tm8k_graphic.u40", 0x400001, 0x100000, CRC(0c3d6347) SHA1(7ef19018c180abf412a8ff9f278b00c2b4321cc2) ) /* Mask rom labeled 5341-16513-08 U40 VIDEO IMAGE */

	ROM_REGION( 0x100000, "oki", 0 ) // Samples
	ROM_LOAD( "tm8k_sound.u8", 0x00000, 0x100000, CRC(c6070a60) SHA1(2dc20bf2217a36374b5a691133ad43f53dbe29ca) ) /* Mask rom labeled 5341-16513-03 U8 SOUND */
ROM_END

GAME( 1996, tm,         0,        tm,       tm,       tmaster_state, 0,        ROT0, "Midway Games Inc. / CES Inc.", "Touchmaster (v3.00 Euro)",                          0 )
GAME( 1996, tmdo,       tm,       tm,       tm,       tmaster_state, 0,        ROT0, "Midway Games Inc. / CES Inc.", "Touchmaster (v2.2-01 Standard)",                    0 )
GAME( 1996, tm2k,       0,        tm,       tm2k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 2000 Plus (v4.63 Standard)",            0 )
GAME( 1996, tm2ka,      tm2k,     tm,       tm2k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 2000 (v4.02 Standard)",                 0 )
GAME( 1996, tm2kb,      tm2k,     tm,       tm2k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 2000 (v4.00 Standard)",                 0 )
GAME( 1996, tm2kspeval, tm2k,     tm,       tm2k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 2000 (v4.XX3 Spanish Evaluation)",      0 )
GAME( 1997, tm3k,       0,        tm,       tm2k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 3000 (v5.02 Standard)",                 0 )
GAME( 1997, tm3ksp,     tm3k,     tm,       tm2k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 3000 (v5.02 Spanish)",                  0 )
GAME( 1997, tm3ka,      tm3k,     tm,       tm2k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 3000 (v5.01 Standard)",                 0 )
GAME( 1997, tm3keval,   tm3k,     tm,       tm2k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 3000 (v5.XX Standard AMOA Evaluation)", 0 )
GAME( 1998, tm4k,       0,        tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 4000 (v6.03 Standard)",                 0 )
GAME( 1998, tm4knj,     tm4k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 4000 (v6.03 New Jersey)",               0 )
GAME( 1998, tm4ka,      tm4k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 4000 (v6.02 Standard)",                 0 )
GAME( 1998, tm4kca,     tm4k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 4000 (v6.02 California)",               0 )
GAME( 1998, tm4kb,      tm4k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 4000 (v6.01 Standard)",                 0 )
GAME( 1998, tm4kmn,     tm4k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 4000 (v6.01 Minnesota)",                0 )
GAME( 1998, tm5k,       0,        tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 5000 (v7.10 Standard)",                 0 )
GAME( 1998, tm5kca,     tm5k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 5000 (v7.10 California)",               0 )
GAME( 1998, tm5kmn,     tm5k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 5000 (v7.10 Minnesota)",                0 )
GAME( 1998, tm5ka,      tm5k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 5000 (v7.01 Standard)",                 0 )
GAME( 1999, tm7k,       0,        tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 7000 (v8.04 Standard)",                 0 )
GAME( 1999, tm7knj,     tm7k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 7000 (v8.05 New Jersey)",               0 )
GAME( 1999, tm7kmn,     tm7k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 7000 (v8.04 Minnesota)",                0 )
GAME( 1999, tm7ka,      tm7k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 7000 (v8.00 Standard)",                 0 )
GAME( 1999, tm7kmna,    tm7k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 7000 (v8.00 Minnesota)",                0 )
GAME( 1999, tm7keval,   tm7k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 7000 (v8.1X ASI Standard Evaluation)",  0 )
GAME( 2000, tm8k,       0,        tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 8000 (v9.04 Standard)",                 0 )
GAME( 2000, tm8k902,    tm8k,     tmds1204, tm4k,     tmaster_state, 0,        ROT0, "Midway Games Inc.",            "Touchmaster 8000 (v9.02 Standard)",                 0 )
