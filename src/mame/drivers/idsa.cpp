// license:BSD-3-Clause
// copyright-holders:Ivan Vangelista, Robbbert
/***********************************************************************************************************************
// Skeleton driver for IDSA pinballs.
// Known pinballs to be dumped: Fantastic Car (1986)
// Hardware listing and ROM definitions from PinMAME.

Hardware:
---------
CPU:     Z80 @ 4 MHz
    INT: IRQ @ 977 Hz (4MHz/2048/2) or 488 Hz (4MHz/2048/4)
IO:      DMA, AY8910 ports
DISPLAY: bsktball: 7-digit 7-segment panels with PROM-based 5-bit BCD data (allowing a simple alphabet)
         v1: 6-digit 7-segment panels with BCD decoding
SOUND:   2 x AY8910 @ 2 MHz plus SP0256 @ 3.12 MHz on board

Schematic is terrible, lots of important info left out. Need the V1 manual & schematic.

ToDO:
- Various PROMs are undumped.
- Speech is gibberish
- Display
- Mechanical sounds
- The usual: identify lamps, solenoids, contactors

**********************************************************************************************************************/

#include "emu.h"
#include "machine/genpin.h"

#include "cpu/z80/z80.h"
#include "sound/ay8910.h"
#include "sound/sp0256.h"
#include "machine/clock.h"
#include "speaker.h"


class idsa_state : public genpin_class
{
public:
	idsa_state(const machine_config &mconfig, device_type type, const char *tag)
		: genpin_class(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_speech(*this, "speech")
		{ }

	DECLARE_WRITE_LINE_MEMBER(clock_w);
	DECLARE_READ8_MEMBER(portb0_r);
	DECLARE_WRITE8_MEMBER(port80_w);
	DECLARE_WRITE8_MEMBER(port90_w);
	DECLARE_WRITE8_MEMBER(ay1_a_w);
	DECLARE_WRITE8_MEMBER(ay1_b_w);
	DECLARE_WRITE8_MEMBER(ay2_a_w);
	DECLARE_WRITE8_MEMBER(ay2_b_w);

private:
	uint16_t m_irqcnt;
	virtual void machine_reset() override;
	required_device<cpu_device> m_maincpu;
	required_device<sp0256_device> m_speech;
};

static ADDRESS_MAP_START( maincpu_map, AS_PROGRAM, 8, idsa_state )
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0x87ff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( maincpu_io_map, AS_IO, 8, idsa_state )
	ADDRESS_MAP_UNMAP_HIGH
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x0f) AM_READ_PORT("X0")
	AM_RANGE(0x10, 0x1f) AM_READ_PORT("X1")
	AM_RANGE(0x20, 0x2f) AM_READ_PORT("X2")
	AM_RANGE(0x30, 0x3f) AM_READ_PORT("X3")
	AM_RANGE(0x40, 0x4f) AM_READ_PORT("X4")
	AM_RANGE(0x50, 0x5f) AM_READ_PORT("X5")
	AM_RANGE(0x60, 0x6f) AM_READ_PORT("X6")
	AM_RANGE(0x70, 0x7f) AM_READ_PORT("X7")
	AM_RANGE(0x80, 0x8f) AM_WRITE(port80_w)
	AM_RANGE(0x90, 0x9f) AM_WRITE(port90_w)
	AM_RANGE(0xb0, 0xb3) AM_READ(portb0_r)
	AM_RANGE(0xbd, 0xbd) AM_READ_PORT("X8")
	AM_RANGE(0xd0, 0xdf) AM_DEVWRITE("speech", sp0256_device, ald_w)
	AM_RANGE(0xe0, 0xef) AM_DEVREADWRITE("aysnd1", ay8910_device, data_r, address_data_w)
	AM_RANGE(0xf0, 0xff) AM_DEVREADWRITE("aysnd2", ay8910_device, data_r, address_data_w)
ADDRESS_MAP_END


static INPUT_PORTS_START( idsa )
	PORT_START("X0") // 1-8
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_Q)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_W)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_E)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_R)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_Y)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_U)
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_I)
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_O)

	PORT_START("X1") // 9-16
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_A)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_S)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_D)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_F)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_G)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_H)
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_J)
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_K)

	PORT_START("X2") // 17-24
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_L)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_Z)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_C)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_V)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_B)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_N)
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_X)
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_COMMA)

	PORT_START("X3") // 25-32
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_STOP)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_SLASH)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_COLON)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_QUOTE)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_ENTER)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_MINUS)
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_EQUALS)
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_BACKSPACE)

	PORT_START("X4") // 33-40
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_OPENBRACE)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_CLOSEBRACE)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_BACKSLASH)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_SPACE)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_LEFT)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_UP)
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_DOWN)

	PORT_START("X5") // 41-48
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_0_PAD)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_2_PAD)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_3_PAD)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_4_PAD)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_5_PAD)
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_6_PAD)
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_OTHER ) PORT_CODE(KEYCODE_7_PAD)

	PORT_START("X6")
	PORT_DIPNAME( 0x01, 0x01, "S01")
	PORT_DIPSETTING(    0x01, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x02, 0x02, "S02")
	PORT_DIPSETTING(    0x02, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x04, 0x04, "S03")
	PORT_DIPSETTING(    0x04, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x08, 0x08, "S04")
	PORT_DIPSETTING(    0x08, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x10, 0x00, "S05")
	PORT_DIPSETTING(    0x10, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x20, 0x20, "S06")
	PORT_DIPSETTING(    0x20, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x40, 0x40, "S07")
	PORT_DIPSETTING(    0x40, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x80, 0x00, "S08")
	PORT_DIPSETTING(    0x80, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))

	PORT_START("X7")
	PORT_DIPNAME( 0x01, 0x01, "S09")
	PORT_DIPSETTING(    0x01, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x02, 0x02, "S10")
	PORT_DIPSETTING(    0x02, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x04, 0x04, "S11")
	PORT_DIPSETTING(    0x04, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x08, 0x08, "S12")
	PORT_DIPSETTING(    0x08, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x10, 0x10, "S13")
	PORT_DIPSETTING(    0x10, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x20, 0x00, "S14")
	PORT_DIPSETTING(    0x20, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x40, 0x40, "S15")
	PORT_DIPSETTING(    0x40, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))
	PORT_DIPNAME( 0x80, 0x80, "S16")
	PORT_DIPSETTING(    0x80, DEF_STR(Off))
	PORT_DIPSETTING(    0x00, DEF_STR(On))

	PORT_START("X8") // all bits here are unknown
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNUSED ) // this bit low makes V1 reboot
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_COIN3 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_TILT )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_START )
INPUT_PORTS_END

// This came from pinmame, even though there's no spb640 chip
READ8_MEMBER( idsa_state::portb0_r )
{
	uint16_t data = m_speech->spb640_r(space, offset / 2);
	return offset % 2 ? (uint8_t)(data >> 8) : (uint8_t)(data & 0xff);
}

// ports 80 & 90 for the display
WRITE8_MEMBER( idsa_state::port80_w )
{
}

WRITE8_MEMBER( idsa_state::port90_w )
{
}

// AY ports are for lamps and solenoids
WRITE8_MEMBER( idsa_state::ay1_a_w )
{
}

WRITE8_MEMBER( idsa_state::ay1_b_w )
{
}

WRITE8_MEMBER( idsa_state::ay2_a_w )
{
}

WRITE8_MEMBER( idsa_state::ay2_b_w )
{
}

WRITE_LINE_MEMBER( idsa_state::clock_w )
{
	if (state)
	{
		m_irqcnt++;
		if (m_irqcnt == 1)
			m_maincpu->set_input_line(INPUT_LINE_IRQ0, CLEAR_LINE);
		else
		if (m_irqcnt == 2048)
		{
			m_maincpu->set_input_line(INPUT_LINE_IRQ0, ASSERT_LINE);
			m_irqcnt = 0;
		}
	}
}

void idsa_state::machine_reset()
{
	m_irqcnt = 0;
}

static MACHINE_CONFIG_START( idsa )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, XTAL_8MHz / 2)
	MCFG_CPU_PROGRAM_MAP(maincpu_map)
	MCFG_CPU_IO_MAP(maincpu_io_map)

	MCFG_DEVICE_ADD("irqclk", CLOCK, XTAL_8MHz / 4 )
	MCFG_CLOCK_SIGNAL_HANDLER(WRITELINE(idsa_state, clock_w))

	/* video hardware */
	//MCFG_DEFAULT_LAYOUT()

	/* sound hardware */
	MCFG_FRAGMENT_ADD( genpin_audio )
	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")
	MCFG_SOUND_ADD("speech", SP0256, 3120000)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 1.5)
	MCFG_SOUND_ADD("aysnd1", AY8910, 2000000) // 2Mhz according to pinmame, schematic omits the clock line
	MCFG_AY8910_PORT_A_WRITE_CB(WRITE8(idsa_state, ay1_a_w))
	MCFG_AY8910_PORT_B_WRITE_CB(WRITE8(idsa_state, ay1_b_w))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 0.75)
	MCFG_SOUND_ADD("aysnd2", AY8910, 2000000)
	MCFG_AY8910_PORT_A_WRITE_CB(WRITE8(idsa_state, ay2_a_w))
	MCFG_AY8910_PORT_B_WRITE_CB(WRITE8(idsa_state, ay2_b_w))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 0.75)
MACHINE_CONFIG_END


ROM_START(v1)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("v1.128", 0x0000, 0x4000, CRC(4e08f7bc) SHA1(eb6ef00e489888dd9c53010c525840de06bcd0f3))

	ROM_REGION(0x10000, "speech", 0)
	ROM_LOAD( "sp0256a-al2.1b",   0x1000, 0x0800, CRC(b504ac15) SHA1(e60fcb5fa16ff3f3b69d36c7a6e955744d3feafc) )
ROM_END

ROM_START(bsktbllp)
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("bsktball.256", 0x0000, 0x8000, CRC(d474e29b) SHA1(750cbacef34dde0b3dcb6c1e4679db78a73643fd))

	ROM_REGION(0x10000, "speech", 0)
	ROM_LOAD( "sp0256a-al2.1b",   0x1000, 0x0800, CRC(b504ac15) SHA1(e60fcb5fa16ff3f3b69d36c7a6e955744d3feafc) )
ROM_END


GAME( 1985, v1,       0, idsa, idsa, idsa_state, 0, ROT0, "IDSA", "V.1",         MACHINE_IS_SKELETON_MECHANICAL )
GAME( 1987, bsktbllp, 0, idsa, idsa, idsa_state, 0, ROT0, "IDSA", "Basket Ball", MACHINE_IS_SKELETON_MECHANICAL )
