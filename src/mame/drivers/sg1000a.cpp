// license:BSD-3-Clause
// copyright-holders:Tomasz Slanina
/*********************************************************
Sega hardware based on their SG-1000 console
Driver by Tomasz Slanina  analog [at] op.pl


Supported games :
- Champion Boxing
- Champion Pro Wrestling
- Doki Doki Penguin Land

Memory map :
0x0000 - 0xBFFF ROM
0xC000 - 0xC3FF RAM

CPU:
Z80 A                    3.57954 MHz (Champion Boxing)
315-5114 (encrypted Z80) 3.57954 MHz (Champion Pro Wrestling)

8255 for I/O port work
3 Eproms for program and graphics
TMM2064 for program RAM
TMS9928ANL for graphics ( 3.57954 MHz? )
8 8118 dynamic RAMs for the graphics
74LS139 and 74LS32 for logic gating
ULN2003AN for coin counter output
SN76489AN for music
7808 voltage regulator to a transistorized circuit for TV output
secondary crystal, numbers unknown for the TMS9928

--

Doki Doki Penguin Land
Sega, 1985

PCB Layout
----------

834-5492
|---------------------------------------------|
|    CN3          CN2          CN4            |
|                                  DSW(4)     |
|3.579545MHz                          TD62003 |
|   74HC04     8255             SN76489       |
|                                         CN5 |
|                                     VR5     |
| 6116         Z80A                           |
|                                  LA4460     |
|            74LS32 74LS139                   |
|EPR-7358.IC3                                 |
|              TMS9928                    CN1 |
|                                             |
|                              10.7386MHz     |
|           MB8118  MB8118                    |
|EPR-7357.IC2                                 |
|           MB8118  MB8118                    |
|                                        7808 |
|           MB8118  MB8118     VR4            |
|EPR7356.IC1                                  |
|    7805   MB8118  MB8118       VR3  VR2  VR1|
|---------------------------------------------|
Notes:
      All IC's shown
      CN1/2/3/4/5   - Connectors for power/video/sound/controls
      VR1/2/3       - Potentiometers for RGB adjustment
      VR4           - Potentiometer for Horizontal Sync adjustment
      VR5           - Potentiometer for volume
      TMS9928 clock - 2.68465 [10.7386/4]
      Z80 clock     - 3.579545MHz
      VSync         - 60Hz
      HSync         - 15.58kHz

Doki Doki Penguinland Dip Switches (DIP4) and Pinout

Option               1     2     3     4
------------------------------------------
1coin 1credit        off   off
1c 2cr               on    off
1c 3cr               off   on
2c 1cr               on    on
attract sound  yes               on
               no                off
not used                               off

Hold Service + 1P start = test mode


CN1               CN2                       CN3
1 Red             1 Player 1 UP             1 Player 2 UP
2 Green           2 Player 1 DOWN           2 Player 2 DOWN
3 Blue            3 Player 1 LEFT           3 Player 2 LEFT
4 Gnd             4 Player 1 RIGHT          4 Player 2 RIGHT
5 Sync            5 Player 1 Button B       5 Player 2 Button B
6 Key             6 Player 1 Button A       6 Player 2 Button A
7                 7 Key                     7 Service
8 Speaker +       8 Player 1 START          8 Coin
9 Speaker -       9 Player 2 START          9 Key
10                10 Gnd                    10 Gnd


CN4               CN5
1                 1 +5V
2                 2 +5V
3                 3 +5V
4                 4 Gnd
5 Key             5 Gnd
6 Coin Meter      6 Gnd
7                 7 +12V
8                 8 Key
9 +5V             9 +12V
10 Gnd            10 Gnd

******************************************************/

/* Super Derby II notes from Charles MacDonald

================================================================================
Notes
================================================================================

Z80 memory map

0000-3FFF : IC10 (EPROM, 16Kx8)
4000-7FFF : IC11 (EPROM, 16Kx8)
8000-BFFF : (Unused)
C000-FFFF : IC8 (Work RAM, 2Kx8)

Z80 port map

00-3F : (Unused)
40-7F : 315-5066 (SN76489 part)
80-BF : 315-5066 (TMS9918 part)
C0-FF : KBSEL#

Detail of KBSEL# area

C0-C7 : 8251 UART
C8-CF : 8255 PPI
D0-D7 : (Unused)
D8-DF : (Unused)
E0-E7 : (Unused)
E8-EF : Output port load strobe (IC25 P9)
F0-F7 : Output port load strobe (IC21 P9)
F8-FF : Pulse generator trigger (IC17 P1)

Z80 signals

 INT# from 315-5066 (same as TMS9918 INT#)
 NMI# source is unclear (from unlabeled IC3 nearby)
WAIT# is unused (pulled up)
  CLK is 3.579543 MHz (from 315-5066 which divides 10.73863 XTAL by 3)

The PCB is configured to connect the PSG output of the 315-5066 to an amplifier,
but none of the locations are populated so there is no audio output.

================================================================================
Board markings
================================================================================

On top copper layer:
"(C) SEGA 1984  834-5529"

On bottom copper layer:
"171-5200"

On a sticker:
"REV C"

On a sticker:
"94.05.27"

================================================================================
Parts list
================================================================================

All ICs have a date code of either 1983 or 1984.

 IC1 - LA4460 (unpopulated)
 IC2 - NEC D74HC138C
 IC3 - (Unreadable)
 IC4 - D780C-1 (Z80)
 IC5 - TI SN74LS245N
 IC6 - SN74LS244N
 IC7 - SN73LS244N
 IC8 - Toshiba TC5517APL (backed by a super capacitor)
 IC9 - NEC D8255AC-2
IC10 - 28-pin socket (silkscreen is "27128") for "EPR-6450D" (Fujitsu MB27128-30)
IC11 - 28-pin socket (silkscreen is "27128") for "EPR-6504D" (Fujitsu MB27128-30)
IC12 - Toshiba TC4053BP
IC13 - Toshiba TC74HC4040P
IC14 - NEC D74HC138C
IC15 - Mitsubishi M74LS02P
IC16 - Sharp IR2339
IC17 - TI SN74LS123N
IC18 - National DM74175N
IC19 - Sega 315-5066
IC20 - NEC D8251AC
IC21 - National DM74175N
IC22 - National DM74175N
IC23 - NEC D41416C-15
IC24 - NEC D41416C-15
IC25 - National DM74175N
IC26 - Motorola NE592N
IC27 - Motorola NE592N
IC28 - Motorola NE592N
IC29 - NEC 7808

PC1  - GI B 5102 321J "TLP521-1"
PC2  - GI D 5102 423J
PC3  - GI D 5102 423J
PC4  - GI D 5102 423J
PC5  - GI D 5102 423J
PC6  - GI D 5102 423J
PC7  - GI D 5102 423J
PC8  - GI D 5102 423J
PC9  - GI D 5102 423J

PST1 - "F" (Unreadable)
PST2 - T516A

TA1  - TD62003P
TA2  - TD62003P
TA3  - TD62003P
TA4  - TD62003P
TA5  - TD62003P

TR1  - C945
TR2  - C1B15
TR3  - C458

SW1  - DIP switch, eight switches, all OFF
SW2  - DIP switch, eight switches, all OFF

XTAL - 10.73863

CN1  - 2x20 latching header
CN2  - 2x17 latching header
CN3  - 2x13 latching header
CN4  - 2x5  latching header

LED-1  - Red LED
LED   - ??? (in optical housing)
PTr   - ??? (in optical housing)

================================================================================
Source code and text in EPR-6450D
================================================================================

"KYOKO SAEKI & HIROMITSU MARUYAMA"

POWER DOWN CHANGE
TSEL:
    LD  A,(BETFLG)  ; READ CREDIT
    RRCA            ; CREDIT IN ?
    JR  C,BETSELZ

    LD  A,(SELOLD)
    OR  A
    LD  C,A
    CALL    NZ,OLDCLCG
    LD  HL,0
    LD  (SELOLD),HL
    LD  (DBLSWCT),HL
    LD  (LMPCNT),1

================================================================================
End
================================================================================
*/

#include "emu.h"
#include "cpu/z80/z80.h"
#include "sound/sn76496.h"
#include "video/tms9928a.h"
#include "machine/i8255.h"
#include "machine/segacrpt_device.h"
#include "speaker.h"


class sg1000a_state : public driver_device
{
public:
	sg1000a_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_decrypted_opcodes(*this, "decrypted_opcodes") { }

	DECLARE_WRITE8_MEMBER(sg1000a_coin_counter_w);
	DECLARE_DRIVER_INIT(sg1000a);
	required_device<cpu_device> m_maincpu;
	optional_shared_ptr<uint8_t> m_decrypted_opcodes;
};


/*************************************
 *
 *  CPU memory handlers
 *
 *************************************/

static ADDRESS_MAP_START( program_map, AS_PROGRAM, 8, sg1000a_state )
	AM_RANGE(0x0000, 0xbfff) AM_ROM
	AM_RANGE(0xc000, 0xc3ff) AM_RAM AM_MIRROR(0x400)
ADDRESS_MAP_END

static ADDRESS_MAP_START( decrypted_opcodes_map, AS_OPCODES, 8, sg1000a_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM AM_SHARE("decrypted_opcodes")
	AM_RANGE(0x8000, 0xbfff) AM_ROM AM_REGION("maincpu", 0x8000)
ADDRESS_MAP_END

static ADDRESS_MAP_START( io_map, AS_IO, 8, sg1000a_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x7f, 0x7f) AM_DEVWRITE("snsnd", sn76489a_device, write)
	AM_RANGE(0xbe, 0xbe) AM_DEVREADWRITE("tms9928a", tms9928a_device, vram_read, vram_write)
	AM_RANGE(0xbf, 0xbf) AM_DEVREADWRITE("tms9928a", tms9928a_device, register_read, register_write)
	AM_RANGE(0xdc, 0xdf) AM_DEVREADWRITE("ppi8255", i8255_device, read, write)
ADDRESS_MAP_END

static ADDRESS_MAP_START( sderby2_io_map, AS_IO, 8, sg1000a_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x7f, 0x7f) AM_DEVWRITE("snsnd", sn76489a_device, write)
	AM_RANGE(0xbe, 0xbe) AM_DEVREADWRITE("tms9928a", tms9928a_device, vram_read, vram_write)
	AM_RANGE(0xbf, 0xbf) AM_DEVREADWRITE("tms9928a", tms9928a_device, register_read, register_write)
	// AM_RANGE(0xc0, 0xc1) NEC D8251AC UART
	AM_RANGE(0xc8, 0xcb) AM_DEVREADWRITE("ppi8255", i8255_device, read, write) // NEC D8255AC-2
ADDRESS_MAP_END

/*************************************
 *
 *  Generic Port definitions
 *
 *************************************/

static INPUT_PORTS_START( sg1000 )
	PORT_START("P1")
	PORT_BIT ( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP )
	PORT_BIT ( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN )
	PORT_BIT ( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT )
	PORT_BIT ( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT )
	PORT_BIT ( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT ( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT ( 0x40, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT ( 0x80, IP_ACTIVE_LOW, IPT_START2 )

	PORT_START("P2")
	PORT_BIT ( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_PLAYER(2)
	PORT_BIT ( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_PLAYER(2)
	PORT_BIT ( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_PLAYER(2)
	PORT_BIT ( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(2)
	PORT_BIT ( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT ( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT ( 0x40, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT ( 0x80, IP_ACTIVE_LOW, IPT_COIN1 )

	PORT_START("DSW")
	PORT_BIT ( 0x0f, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Coinage ) ) PORT_DIPLOCATION("DSW:1,2")
	PORT_DIPSETTING(    0x00, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x30, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x20, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x10, DEF_STR( 1C_3C ) )
	PORT_DIPNAME( 0x40, 0x00, DEF_STR( Demo_Sounds ) ) PORT_DIPLOCATION("DSW:3")
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Language ) ) PORT_DIPLOCATION("DSW:4")
	PORT_DIPSETTING(    0x00, DEF_STR( Japanese ) )
	PORT_DIPSETTING(    0x80, DEF_STR( English ) )
INPUT_PORTS_END

/*************************************
 *
 *  Port definitions
 *
 *************************************/

static INPUT_PORTS_START( chwrestl )
	PORT_INCLUDE( sg1000 )

	PORT_MODIFY("P1")
	PORT_BIT ( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_PLAYER(2)
	PORT_BIT ( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_PLAYER(2)
	PORT_BIT ( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_PLAYER(2)
	PORT_BIT ( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(2)
	PORT_BIT ( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT ( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)

	PORT_MODIFY("P2")
	PORT_BIT ( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP )
	PORT_BIT ( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN )
	PORT_BIT ( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT )
	PORT_BIT ( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT )
	PORT_BIT ( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT ( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 )
INPUT_PORTS_END

static INPUT_PORTS_START( chboxing )
	PORT_INCLUDE( sg1000 )

	PORT_MODIFY("DSW")
	PORT_DIPNAME( 0x80, 0x00, DEF_STR( Language ) ) PORT_DIPLOCATION("DSW:4")
	PORT_DIPSETTING(    0x00, DEF_STR( English ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Japanese ) )
INPUT_PORTS_END

static INPUT_PORTS_START( dokidoki )
	PORT_INCLUDE( sg1000 )

	PORT_MODIFY("DSW")
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unused ) ) PORT_DIPLOCATION("DSW:4")
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
INPUT_PORTS_END

static INPUT_PORTS_START( sderby2s )
	PORT_INCLUDE( sg1000 )

	PORT_MODIFY("DSW")
	PORT_DIPNAME( 0x01, 0x00, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW:1")
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW:2")
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW:3")
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW:4")
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW:5")
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW:6")
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW:7")
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW:8")
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
INPUT_PORTS_END


WRITE8_MEMBER(sg1000a_state::sg1000a_coin_counter_w)
{
	machine().bookkeeping().coin_counter_w(0, data & 0x01);
}

/*************************************
 *
 *  Machine drivers
 *
 *************************************/

static MACHINE_CONFIG_START( sg1000a )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, XTAL_3_579545MHz)
	MCFG_CPU_PROGRAM_MAP(program_map)
	MCFG_CPU_IO_MAP(io_map)

	MCFG_DEVICE_ADD("ppi8255", I8255, 0)
	MCFG_I8255_IN_PORTA_CB(IOPORT("P1"))
	MCFG_I8255_IN_PORTB_CB(IOPORT("P2"))
	MCFG_I8255_IN_PORTC_CB(IOPORT("DSW"))
	MCFG_I8255_OUT_PORTC_CB(WRITE8(sg1000a_state, sg1000a_coin_counter_w))

	/* video hardware */
	MCFG_DEVICE_ADD( "tms9928a", TMS9928A, XTAL_10_738635MHz / 2 )
	MCFG_TMS9928A_VRAM_SIZE(0x4000)
	MCFG_TMS9928A_OUT_INT_LINE_CB(INPUTLINE("maincpu", INPUT_LINE_IRQ0))

	MCFG_TMS9928A_SCREEN_ADD_NTSC( "screen" )
	MCFG_SCREEN_UPDATE_DEVICE( "tms9928a", tms9928a_device, screen_update )

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("snsnd", SN76489A, XTAL_3_579545MHz)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( sg1000ax, sg1000a )
	MCFG_CPU_REPLACE("maincpu", SEGA_315_5033, XTAL_3_579545MHz)
	MCFG_CPU_PROGRAM_MAP(program_map)
	MCFG_CPU_IO_MAP(io_map)
	MCFG_CPU_DECRYPTED_OPCODES_MAP(decrypted_opcodes_map)
	MCFG_SEGACRPT_SET_DECRYPTED_TAG(":decrypted_opcodes")
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( sderby2s, sg1000a )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_CLOCK(XTAL_10_738635MHz / 3)
	MCFG_CPU_IO_MAP(sderby2_io_map)

	// Actually uses a Sega 315-5066 chip, which is a TMS9918 and SN76489 in the same package but with RGB output
MACHINE_CONFIG_END

/*************************************
 *
 *  ROM definitions
 *
 *************************************/

ROM_START( chwrestl )
	ROM_REGION( 2*0x10000, "maincpu", 0 )
	ROM_LOAD( "5732",   0x0000, 0x4000, CRC(a4e44370) SHA1(a9dbf60e77327dd2bec6816f3142b42ad9ca4d09) ) /* encrypted */
	ROM_LOAD( "5733",   0x4000, 0x4000, CRC(4f493538) SHA1(467862fe9337497e3cdebb29bf28f6cfe3066ccd) ) /* encrypted */
	ROM_LOAD( "5734",   0x8000, 0x4000, CRC(d99b6301) SHA1(5e762ed45cde08d5223828c6b1d3569b2240462c) )
ROM_END

ROM_START( chboxing )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "cb6105.bin", 0x0000, 0x4000, CRC(43516f2e) SHA1(e3a9bbe914b5bfdcd1f85ca5fae922c4cae3c106) )
	ROM_LOAD( "cb6106.bin", 0x4000, 0x4000, CRC(65e2c750) SHA1(843466b8d6baebb4d5e434fbdafe3ae8fed03475) )
	ROM_LOAD( "cb6107.bin", 0x8000, 0x2000, CRC(c2f8e522) SHA1(932276e7ad33aa9efbb4cd10bc3071d88cb082cb) )
ROM_END

ROM_START( dokidoki )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "epr-7356.ic1",   0x0000, 0x4000, CRC(95658c31) SHA1(f7b5638ab1b8b244b189317d954eb37b51923791) )
	ROM_LOAD( "epr-7357.ic2",   0x4000, 0x4000, CRC(e8dbad85) SHA1(9f13dafacee370d6e4720d8e27cf889053e79eb3) )
	ROM_LOAD( "epr-7358.ic3",   0x8000, 0x4000, CRC(c6f26b0b) SHA1(3753e05b6e77159832dbe88562ba7a818120d1a3) )
ROM_END

ROM_START( sderby2s )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "epr-6450d.ic10",   0x0000, 0x4000, CRC(e56986d3) SHA1(a2dbdc95128cc94a1492e080aeea402f2d4b89fe) )
	ROM_LOAD( "epr-6504d.ic11",   0x4000, 0x4000, CRC(7bb364b9) SHA1(9f93572b6d999422d93ad5f7a251b4695565651f) )
ROM_END

/*************************************
 *
 *  Driver initialization
 *
 *************************************/

DRIVER_INIT_MEMBER(sg1000a_state,sg1000a)
{
}



/*************************************
 *
 *  Game drivers
 *
 *************************************/

GAME( 1984, chboxing, 0, sg1000a,  chboxing, sg1000a_state, sg1000a,  ROT0, "Sega", "Champion Boxing",                  0 )
GAME( 1985, chwrestl, 0, sg1000ax, chwrestl, sg1000a_state, sg1000a,  ROT0, "Sega", "Champion Pro Wrestling",           0 )
GAME( 1985, dokidoki, 0, sg1000a,  dokidoki, sg1000a_state, sg1000a,  ROT0, "Sega", "Doki Doki Penguin Land",           0 )
GAME( 1985, sderby2s, 0, sderby2s, sderby2s, sg1000a_state, sg1000a,  ROT0, "Sega", "Super Derby II (Satellite board)", MACHINE_NOT_WORKING ) // inputs aren't hooked up, probably needs to be connected to the main board anyway
