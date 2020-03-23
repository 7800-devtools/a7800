// license:BSD-3-Clause
// copyright-holders:David Haywood
/***************************************************************************

 Pot of Gold (c)200? U. S. Games
 it appears this is just the name of a series of machines with different
 software themes?

 board has the following etched

     US GAMES
   MADE IN USA
 P/N: 34010  REV C-1

 potgoldu: not sure this is a good dump.. one rom is much bigger than the others
 and doesn't seem to pair with the ROM I'd expect it to pair with...

 I'm just tagging the whole thing as BAD_DUMP for now.

***************************************************************************/

#include "emu.h"
#include "cpu/tms34010/tms34010.h"
#include "screen.h"


class potgold_state : public driver_device
{
public:
	potgold_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu") { }

		required_device<cpu_device> m_maincpu;

	virtual void machine_reset() override;
	virtual void video_start() override;

	TMS340X0_SCANLINE_RGB32_CB_MEMBER(scanline_update);
};


#define CPU_CLOCK           XTAL_40MHz
#define VIDEO_CLOCK         (22118400) // ?
#define SOUND_CLOCK         (3579645)





void potgold_state::video_start()
{
}

TMS340X0_SCANLINE_RGB32_CB_MEMBER(potgold_state::scanline_update)
{
}


void potgold_state::machine_reset()
{
}


static ADDRESS_MAP_START( potgold_map, AS_PROGRAM, 16, potgold_state )
	AM_RANGE(0xff000000, 0xffffffff) AM_ROM AM_REGION("user1", 0)
ADDRESS_MAP_END


static INPUT_PORTS_START( potgold )
INPUT_PORTS_END


static MACHINE_CONFIG_START( potgold )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", TMS34010, XTAL_40MHz)
	MCFG_CPU_PROGRAM_MAP(potgold_map)
	MCFG_TMS340X0_HALT_ON_RESET(false) /* halt on reset */
	MCFG_TMS340X0_PIXEL_CLOCK(VIDEO_CLOCK/2) /* pixel clock */
	MCFG_TMS340X0_PIXELS_PER_CLOCK(1) /* pixels per clock */
	MCFG_TMS340X0_SCANLINE_RGB32_CB(potgold_state, scanline_update)  /* scanline callback (rgb32) */

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_RAW_PARAMS(VIDEO_CLOCK/2, 444, 0, 320, 233, 0, 200)
	MCFG_SCREEN_UPDATE_DEVICE("maincpu", tms34010_device, tms340x0_rgb32)

	/* sound hardware */
	/* YM2413 */
MACHINE_CONFIG_END

ROM_START( potgoldu )
	ROM_REGION16_LE( 0x400000, "user1", 0 ) /* 34010 code */

	// these two are definitely a pair
	ROM_LOAD16_BYTE( "400x.u5",  0x180000, 0x20000, BAD_DUMP CRC(4949300b) SHA1(edf5e3de8561258ceb8fc0ab0291859d2cf7c21b) )
	ROM_LOAD16_BYTE( "400x.u9",  0x180001, 0x20000, BAD_DUMP CRC(80e1ab14) SHA1(36595446d73dc5bf5c7f47b9385e5fdc84cce195) )

	// these two.. don't match up
	ROM_LOAD16_BYTE( "400x.u4",  0x180000, 0x20000, BAD_DUMP CRC(66d6697c) SHA1(6e1072cce70b56b8bf186cbb0f3dcc970ef6ca39) ) // the end of this seems to fit in as you'd expect, before u3
	ROM_LOAD16_BYTE( "400x.u8",  0x180001, 0x80000, BAD_DUMP CRC(0496bc92) SHA1(ae80b3de856ae60de29e2d7e05c6ed5fb37232a9) ) // the start of this resembles u9.. the rest doesn't seem to match up with anything? there are odd bytes of text strings etc.

	// these two are definitely a pair
	ROM_LOAD16_BYTE( "400x.u3",  0x1c0000, 0x20000, BAD_DUMP CRC(c0894db0) SHA1(d68321949250bfe0f14bd5ef8d115ba4b3786b8b) )
	ROM_LOAD16_BYTE( "400x.u7",  0x1c0001, 0x20000, BAD_DUMP CRC(0953ecf7) SHA1(91cbe5d9aff171902dc3eb43a308a7a833c8fb71) )
ROM_END

ROM_START( potgoldu580 ) // TMS34010FNL-40 + MC68H705 + YMF704C + ADV476KP35 RAMDAC + SC28L198A1A UART + EPM7192SQC160-10 CPLD
	ROM_REGION16_LE( 0x400000, "user1", 0 ) /* 34010 code */

	ROM_LOAD16_BYTE( "POG_580F.U4", 0x180000, 0x80000, CRC(087704d2) SHA1(915c0c57d014d04d5016099915b754e7592cbb0d) )
	ROM_LOAD16_BYTE( "POG_580F.U7", 0x180001, 0x80000, CRC(4b76499b) SHA1(3d377107a201607d63f802f54771ae562b60ae27) )

	ROM_LOAD16_BYTE( "POG_580F.U5", 0x280000, 0x80000, CRC(64c3b488) SHA1(30564feee544f7b4d1d48c68dbfcd6ae0ae1b220) )
	ROM_LOAD16_BYTE( "POG_580F.U8", 0x280001, 0x80000, CRC(cca108a4) SHA1(edd46df79bd8835ca61b5d48277de4a70a83e2a0) )

	// Dumper's note: Not included is the "Security" chip needed to run the game. However from what I can tell the chip only collates the bins, dumps them to ram, and keeps settings.
	ROM_REGION( 0x2000, "mcu", 0 )  /* 68H705 (68hc705??) microcontroller */
	ROM_LOAD( "mc68h705",     0x0000, 0x2000, NO_DUMP ) // not sure which type of mcu this is, size is a guess
ROM_END

GAME( 200?, potgoldu,    0,        potgold,   potgold, potgold_state,   0, ROT0, "U.S. Games Inc.",  "Pot O' Gold (U.S. Games, v400x?)", MACHINE_IS_SKELETON )
GAME( 200?, potgoldu580, potgoldu, potgold,   potgold, potgold_state,   0, ROT0, "U.S. Games Inc.",  "Pot O' Gold (U.S. Games, v580F)",  MACHINE_IS_SKELETON )
