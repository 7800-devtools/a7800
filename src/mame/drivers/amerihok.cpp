// license:BSD-3-Clause
// copyright-holders:David Haywood
/*

Ameri-Hockey?

U3 -  27C512
U8 -  27C020
U9 -  27C020
U10- 27C020

12 MHz crystal

Processor is a ROMless MCU from the Z8 family.

*/

#include "emu.h"
#include "cpu/z8/z8.h"
#include "sound/okim6376.h"
#include "speaker.h"


class amerihok_state : public driver_device
{
public:
	amerihok_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
	{ }

	virtual void machine_start() override;
	virtual void machine_reset() override;

	required_device<cpu_device> m_maincpu;
};

static ADDRESS_MAP_START(amerihok_map, AS_PROGRAM, 8, amerihok_state)
	AM_RANGE(0x0000, 0xffff) AM_ROM
ADDRESS_MAP_END

static INPUT_PORTS_START( amerihok )
INPUT_PORTS_END



void amerihok_state::machine_start()
{
}

void amerihok_state::machine_reset()
{
}


static MACHINE_CONFIG_START( amerihok )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z8681, XTAL_12MHz) // type guessed
	MCFG_CPU_PROGRAM_MAP(amerihok_map)
//  MCFG_CPU_VBLANK_INT_DRIVER("screen", amerihok_state,  irq4_line_hold)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("oki", OKIM6376, 1000000) // 64-pin surface mount
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END



ROM_START( amerihok )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "air-h-u3", 0x00000, 0x10000, CRC(f43eaa25) SHA1(b73e3f6db9fe277dab3fd9d1161f3b71b5805048) )

	ROM_REGION( 0xc0000, "oki", 0 )
	ROM_LOAD( "air-h-u8", 0x00000, 0x40000, CRC(17a84f88) SHA1(33a5a66b1e7c8bf79c99e442c62d8ce0c7d1c22c) )
	ROM_LOAD( "air-h-u9", 0x40000, 0x40000, CRC(be01ca4a) SHA1(87513a5c547633d5a3f09e931bd7ec78bcaa94dc) )
	ROM_LOAD( "airh-u10", 0x80000, 0x40000, CRC(71ee6421) SHA1(10131fc7c009158308c4a8bb2b037101622c07a1) )
ROM_END

GAME( 199?, amerihok,  0,    amerihok, amerihok, driver_device,  0, ROT0, "Ameri", "Ameri-Hockey", MACHINE_IS_SKELETON_MECHANICAL )
