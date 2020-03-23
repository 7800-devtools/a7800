// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/*************************************************************************

    drivers/digijet.cpp

    Skeleton driver for the Volkswagen Digijet series of automotive ECUs

    The Digijet Engine Control Unit (ECU) was used in Volkswagen vehicles
    from the early 1980s.

    Currently, the only dump is from a 1985 Volkswagen Vanagon (USA CA).

**************************************************************************/

/*
    TODO:

    - Everything
*/

#include "emu.h"
#include "cpu/mcs48/mcs48.h"

#define I8049_TAG   "i8049"

class digijet_state : public driver_device
{
public:
	digijet_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, I8049_TAG)
	{
	}

	required_device<cpu_device> m_maincpu;

	virtual void machine_start() override { }
	virtual void machine_reset() override { }
};

static ADDRESS_MAP_START( io_map, AS_IO, 8, digijet_state )
ADDRESS_MAP_END

static INPUT_PORTS_START( digijet )
INPUT_PORTS_END

static MACHINE_CONFIG_START( digijet )
	/* basic machine hardware */
	MCFG_CPU_ADD(I8049_TAG, I8049, XTAL_11MHz)
	MCFG_CPU_IO_MAP(io_map)
MACHINE_CONFIG_END

ROM_START( digijet )
	ROM_REGION( 0x800, I8049_TAG, 0 )
	ROM_LOAD( "vanagon_85_usa_ca.bin", 0x000, 0x800, CRC(2ed7c4c5) SHA1(ae48d8892b44fe76b48bcefd293c15cd47af3fba) ) // Volkswagen Vanagon, 1985, USA, California
ROM_END

//    YEAR  NAME      PARENT    COMPAT    MACHINE   INPUT     STATE        INIT  COMPANY       FULLNAME   FLAGS
CONS( 1985, digijet,  0,        0,        digijet,  digijet,  digijet_state,  0, "Volkswagen", "Digijet", MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW )
