// license:BSD-3-Clause
// copyright-holders:David Haywood
/*

clown roll down z8 = 2732a
clown roll down z9 = 2732a

can't find any info on this?

*/

#include "emu.h"
#include "speaker.h"
#include "cpu/m6800/m6800.h"
#include "machine/6821pia.h"


class clowndwn_state : public driver_device
{
public:
	clowndwn_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
	{ }

	virtual void machine_start() override;
	virtual void machine_reset() override;

	required_device<cpu_device> m_maincpu;
};

static ADDRESS_MAP_START(clowndwn_map, AS_PROGRAM, 8, clowndwn_state)
	AM_RANGE(0x0000, 0x07ff) AM_RAM
	AM_RANGE(0x4100, 0x4103) AM_DEVREADWRITE("pia0", pia6821_device, read, write)
	AM_RANGE(0x4200, 0x4203) AM_DEVREADWRITE("pia1", pia6821_device, read, write)
	AM_RANGE(0x4400, 0x4403) AM_DEVREADWRITE("pia2", pia6821_device, read, write)
	AM_RANGE(0x4800, 0x4803) AM_DEVREADWRITE("pia3", pia6821_device, read, write)
	AM_RANGE(0x5000, 0x5003) AM_DEVREADWRITE("pia4", pia6821_device, read, write)
	AM_RANGE(0xe000, 0xffff) AM_ROM AM_REGION("maincpu", 0)
ADDRESS_MAP_END

static INPUT_PORTS_START( clowndwn )
INPUT_PORTS_END



void clowndwn_state::machine_start()
{
}

void clowndwn_state::machine_reset()
{
}


static MACHINE_CONFIG_START( clowndwn )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M6802, 8000000) // unknown type and clock
	MCFG_CPU_PROGRAM_MAP(clowndwn_map)

	MCFG_DEVICE_ADD("pia0", PIA6821, 0)
	MCFG_DEVICE_ADD("pia1", PIA6821, 0)
	MCFG_DEVICE_ADD("pia2", PIA6821, 0)
	MCFG_DEVICE_ADD("pia3", PIA6821, 0)
	MCFG_DEVICE_ADD("pia4", PIA6821, 0)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
MACHINE_CONFIG_END


// has
// COPYRIGHT 1982, 1983, 1984, 1985, and 1987 by ELWOOD ELECTRONICS CO., INC
// in Z9

ROM_START( clowndwn )
	ROM_REGION( 0x2000, "maincpu", 0 )
	ROM_LOAD( "CLWNROLL.Z8", 0x0000, 0x1000, CRC(ec655745) SHA1(e38de904f30530f8971eb4a9d7796da345bf81ad) )
	ROM_LOAD( "CLWNROLL.Z9", 0x1000, 0x1000, CRC(aeef885e) SHA1(bc6805b638625a347e1288a927ce30e030afe9e3) )
ROM_END

GAME( 1987, clowndwn,  0,    clowndwn, clowndwn, clowndwn_state,  0, ROT0, "Elwood Electronics", "Clown Roll Down (Elwood)", MACHINE_IS_SKELETON_MECHANICAL )
