// license:BSD-3-Clause
// copyright-holders:superctr
/*
 Yamaha TG100 AWM Tone Generator
 Skeleton written by superctr

 Roms dumped by vampirefrog
 Service manual scanned by bmos

 CPU: Hitachi HD6435208A00P (H8/520)
  - 20MHz clock
  - Mode bits 0,1 high, 2 low
  - 32kb RAM (1x HM65256 PSRAM)
 Sound generator: Yamaha YMW258-F (GEW8, identical to MultiPCM?)
  - 9.4MHz clock
  - 28 voices polyphony
  - 2MB sample ROM containing 140 12-bit PCM samples
 Effect DSP: Yamaha YM3413
  - clocked by sound generator
  - Effect memory: 64kb (2x HM65256 PSRAM)
 LCD:
  - 1x16 characters

 Other ICs:
  HG62E11R54FS (XK462A00) Gate array (LCD control, glue logic)

*/

#include "emu.h"

#include "cpu/h8/h83002.h"
#include "sound/multipcm.h"

#include "screen.h"
#include "speaker.h"

class tg100_state : public driver_device
{
public:
	tg100_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_ymw258(*this, "ymw258")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<multipcm_device> m_ymw258;
};

/* all memory accesses are decoded by the gate array... */
static ADDRESS_MAP_START( tg100_map, AS_PROGRAM, 16, tg100_state )
	AM_RANGE(0x00000000, 0x0007ffff) AM_RAM /* gate array stuff */
	AM_RANGE(0x00080000, 0x0009ffff) AM_ROM AM_REGION("prgrom", 0x00000)
ADDRESS_MAP_END

static ADDRESS_MAP_START( tg100_io_map, AS_IO, 8, tg100_state )
//  ADDRESS_MAP_GLOBAL_MASK(0xff)
ADDRESS_MAP_END

static INPUT_PORTS_START( tg100 )
INPUT_PORTS_END

static ADDRESS_MAP_START( ymw258_map, 0, 8, tg100_state )
	AM_RANGE(0x000000, 0x1fffff) AM_ROM
ADDRESS_MAP_END

static MACHINE_CONFIG_START( tg100 )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu",  H83002, XTAL_20MHz) /* TODO: correct CPU type (H8/520) */
	MCFG_CPU_PROGRAM_MAP( tg100_map )
	MCFG_CPU_IO_MAP( tg100_io_map )

	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")

	MCFG_SOUND_ADD("ymw258", MULTIPCM, 9400000)
	MCFG_DEVICE_ADDRESS_MAP(0, ymw258_map)
	MCFG_SOUND_ROUTE(0, "lspeaker", 1.0)
	MCFG_SOUND_ROUTE(1, "rspeaker", 1.0)

MACHINE_CONFIG_END

ROM_START( tg100 )

	ROM_REGION(0x20000, "prgrom", 0) // H8/520 memory bus is 8 bit actually
	ROM_LOAD16_WORD_SWAP( "xk731c0.ic4", 0x00000, 0x20000, CRC(8fb6139c) SHA1(483103a2ffc63a90a2086c597baa2b2745c3a1c2) )

	ROM_REGION(0x200000, "ymw258", 0)
	ROM_LOAD( "xk992a0.ic6", 0x000000, 0x200000, CRC(01dc6954) SHA1(32ec77a46f4d005538c735f56ad48fa7243c63be) )

ROM_END

//    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT     CLASS           INIT  COMPANY   FULLNAME       FLAGS
CONS( 1991, tg100,    0,      0,      tg100,    tg100,    tg100_state,    0,    "Yamaha", "TG100",      MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
