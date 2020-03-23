// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*
    otomedius.cpp: Konami Otomedius (and maybe related Konami PC-based stuff)

    Skeleton by R. Belmont

    Hardware for Otomedius:
        - Intel Socket 478 Celeron CPU, 2.5 GHz, S-Spec "SL6ZY"
          More info: http://www.cpu-world.com/sspec/SL/SL6ZY.html
        - Intel 82865G northbridge
        - Intel 82801EB southbridge / "ICH5" Super I/O
        - 512MB of system RAM
        - ATI-branded Radeon 9600XT AGP video card with 128 MB of VRAM
        - Konami protection dongle marked "GEGGG JA-B"
*/

#include "emu.h"
#include "cpu/i386/i386.h"
#include "screen.h"

class konami_pc_state : public driver_device
{
public:
	konami_pc_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu")
	{ }

	required_device<cpu_device> m_maincpu;

	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_konami_pc(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
};

void konami_pc_state::video_start()
{
}

uint32_t konami_pc_state::screen_update_konami_pc(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	return 0;
}

static ADDRESS_MAP_START( konami_pc_map, AS_PROGRAM, 32, konami_pc_state )
	AM_RANGE(0x00000000, 0x0009ffff) AM_RAM
	AM_RANGE(0x000f0000, 0x000fffff) AM_ROM AM_REGION("maincpu", 0x70000)
	AM_RANGE(0xfff80000, 0xffffffff) AM_ROM AM_REGION("maincpu", 0)
ADDRESS_MAP_END

static INPUT_PORTS_START( konami_pc )
INPUT_PORTS_END


void konami_pc_state::machine_start()
{
}

void konami_pc_state::machine_reset()
{
}

static MACHINE_CONFIG_START( konami_pc )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", PENTIUM3, 100000000) // not correct, but why bother?
	MCFG_CPU_PROGRAM_MAP(konami_pc_map)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(640, 480)
	MCFG_SCREEN_VISIBLE_AREA(0, 640-1, 0, 480-1)
	MCFG_SCREEN_UPDATE_DRIVER(konami_pc_state, screen_update_konami_pc)
MACHINE_CONFIG_END

/***************************************************************************

  Game drivers

***************************************************************************/

ROM_START( otomedius )
	ROM_REGION( 0x80000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD( "sst49fl004b.u18", 0x000000, 0x080000, CRC(bb9f4e3e) SHA1(95b393a38a5eded3204debfe7a88cc7ea15adf9a) )

	ROM_REGION( 0x10000, "vbios", 0 )   // video card BIOS
	ROM_LOAD( "ati.9600xt.128.samsung.031113.rom", 0x000000, 0x00d000, CRC(020ec211) SHA1(3860c980106f00e5259ecd8d4cd2f9b3fca2428a) )

	DISK_REGION( "ide:0:hdd:image" ) // Seagate ST340015A 40GB PATA drive
	DISK_IMAGE( "otomedius", 0, SHA1(9283f8b7cd747be7b8e7321953adbf6cbe926f25) )
ROM_END

GAME( 2007, otomedius,  0,   konami_pc, konami_pc, konami_pc_state,  0, ROT0, "Konami", "Otomedius (ver GGG:J:A:A:2008041801)",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
