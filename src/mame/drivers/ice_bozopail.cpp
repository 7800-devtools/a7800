// license:BSD-3-Clause
// copyright-holders:David Haywood
/*

Bozo Pail toss by ICE  (ice_tbd notes say Innovative Creations in Entertainment - same company?)

Devices are 27c080

U9 is version 2.07


PCB uses a 68HC11A1P for a processor/security......

could be related to (or the same thing as - our name could be incorrect)
http://www.highwaygames.com/arcade-machines/bozo-s-grand-prize-game-6751/


*/

#include "emu.h"
#include "cpu/mc68hc11/mc68hc11.h"
#include "speaker.h"

class ice_bozopail_state : public driver_device
{
public:
	ice_bozopail_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
	{ }

	virtual void machine_start() override;
	virtual void machine_reset() override;

	required_device<cpu_device> m_maincpu;
};

static ADDRESS_MAP_START( ice_bozo_map, AS_PROGRAM, 8, ice_bozopail_state )
	AM_RANGE(0xe000, 0xffff) AM_ROM AM_REGION("maincpu", 0x1fe000)
ADDRESS_MAP_END

static INPUT_PORTS_START( ice_bozo )
INPUT_PORTS_END



void ice_bozopail_state::machine_start()
{
}

void ice_bozopail_state::machine_reset()
{
}


static MACHINE_CONFIG_START( ice_bozo )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", MC68HC11, 8000000) // unknown clock
	MCFG_CPU_PROGRAM_MAP(ice_bozo_map)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
MACHINE_CONFIG_END



ROM_START( ice_bozo )
	ROM_REGION( 0x200000, "maincpu", 0 ) // mostly sound data, some code
	ROM_LOAD( "ICE-BOZO.U18", 0x000000, 0x100000, CRC(00500a8b) SHA1(50b8a784ae61510a08cafbfb8529ec2a8ac1bf06) )
	ROM_LOAD( "ICE-BOZO.U9",  0x100000, 0x100000, CRC(26fd9d60) SHA1(41fe8d42db1eb16b413bd5a0f16bf0d081c3cc97) )
ROM_END

GAME( 1997?, ice_bozo,  0,    ice_bozo, ice_bozo, ice_bozopail_state,  0, ROT0, "Innovative Creations in Entertainment", "Bozo's Pail Toss (v2.07)", MACHINE_IS_SKELETON_MECHANICAL )
