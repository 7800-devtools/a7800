// license:GPL2+
// copyright-holders:Felipe Sanches
/******************************************************************

 Alesis QS-series keyboards

 Currently this skeleton covers only the Alesis QS-7
 unit, but other keyboards in this series have similar
 hardware caracteristics.

 Author: Felipe Correa da Silva Sanches <juca@members.fsf.org>

*******************************************************************/

#include "emu.h"
#include "cpu/h8/h83048.h"
//#include "sound/alesis_qs.h"

class qs_state : public driver_device
{
public:
	qs_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_maincpu(*this, "maincpu")
	{ }

protected:
	required_device<cpu_device> m_maincpu;
};

/* Input ports */
static INPUT_PORTS_START( qs7 )
//        PORT_START("COL1")
//        PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("?") PORT_CODE(KEYCODE_)
//        PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("?") PORT_CODE(KEYCODE_)
//        PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("?") PORT_CODE(KEYCODE_)
//        PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("?") PORT_CODE(KEYCODE_)
//        PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("?") PORT_CODE(KEYCODE_)
//        PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("?") PORT_CODE(KEYCODE_)
//        PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("?") PORT_CODE(KEYCODE_)
//        PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("?") PORT_CODE(KEYCODE_)
INPUT_PORTS_END

static ADDRESS_MAP_START( qs7_prog_map, AS_PROGRAM, 16, qs_state )
	//ADDRESS_MAP_GLOBAL_MASK(0x3ffff)
	AM_RANGE(0x00000, 0x3ffff) AM_ROM
ADDRESS_MAP_END

static MACHINE_CONFIG_START( qs7 )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", H83048, XTAL_10MHz) /* FIX-ME! Actual CPU is H8/510 and XTAL value is a guess */
	MCFG_CPU_PROGRAM_MAP(qs7_prog_map)

		//MCFG_ALESIS_KEYSCAN_ASIC_ADD("keyscan")

		/* Alesis Sound Generator ASIC */
		//MCFG_ALESIS_SG_ASIC_ADD("sndgen")

		/* Alesis Sound Effects Processor ASIC */
		//MCFG_ALESIS_FX_ASIC_ADD("sfx")

	/* video hardware */
		//TODO: add LCD display controller here

	/* sound hardware */
	//MCFG_SPEAKER_STANDARD_STEREO("stereo")
	//MCFG_ALESIS_QS_SERIES_ADD("sound", SND_CLOCK)
	//MCFG_SOUND_ROUTE(ALL_OUTPUTS, "stereo", 1.0)

		/* Interfaces */
		//MCFG_PCMCIA_ADD("pcmcia")
		//MIDI
		//RS232
MACHINE_CONFIG_END

ROM_START( alesqs7 )
	ROM_REGION( 0x80000, "maincpu", 0 )
	ROM_LOAD( "2-31-0069_q7_v1.02_alesis_sp_09_12_96_cs_dbcc.u18", 0x00000, 0x80000, CRC(6e5404cb) SHA1(f00598b66ab7a83b16105cbb73e09c66ce3493a7) )

//  ROM_REGION( 0x200000, "sound", 0 ) /* Samples ROMs (2Mbyte each) */
//  ROM_LOAD( "?.u?", 0x00000, 0x200000, NO_DUMP )
//  ROM_LOAD( "?.u?", 0x00000, 0x200000, NO_DUMP )
//  ROM_LOAD( "?.u?", 0x00000, 0x200000, NO_DUMP )
//  ROM_LOAD( "?.u?", 0x00000, 0x200000, NO_DUMP )
//  ROM_LOAD( "?.u?", 0x00000, 0x200000, NO_DUMP )
//  ROM_LOAD( "?.u?", 0x00000, 0x200000, NO_DUMP )
//  ROM_LOAD( "?.u?", 0x00000, 0x200000, NO_DUMP )
//  ROM_LOAD( "?.u?", 0x00000, 0x200000, NO_DUMP )
ROM_END

//    YEAR  NAME     PARENT  COMPAT  MACHINE  INPUT  INIT          COMPANY   FULLNAME                       FLAGS
COMP( 1996, alesqs7, 0,      0,      qs7,     qs7,   qs_state, 0,  "Alesis", "Alesis QS7 musical keyboard", MACHINE_IS_SKELETON )
