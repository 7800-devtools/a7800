// license:BSD-3-Clause
// copyright-holders:R. Belmont
/***************************************************************************

 GI Classic / GI Classic EX
 (c) 1995, 1998 Konami
 Preliminary driver by R. Belmont

 GI Classic EX Main PCB:
 Main CPU: 68000
 Video: 056832 + 058143 (GX tilemaps)
 Video: 055673(x2) + 053246 (GX sprites)
 Video: 055555 (Mixer)
 053252 (x2) (CRTC?)

 GI Classic EX Satellite PCB:
 Main CPU: 68000-12
 Video: 056832 / 058143 (GX tilemaps)
 Video: 000907 LCD Controller

 WANTED: main PCB and any other PCBs for GI Classic EX, plus any and all
 PCBs for other games also believed to be on this h/w:
 - GI-Classic (1995)
 - GI-Classic Special (1996)
 - GI-Classic WINDS (1996)
 - GI-Classic WINDS EX (1998)

 Other "GI" games, list from http://www.konami.jp/am/g1/
 - GI-LEADING SIRE (1999)
 - GI-LEADING SIRE Ver. 2 (2000)
 - GI-LEADING SIRE Ver. 3 (2001)
 - GI-WINNING SIRE (2002)
 - GI-TURFWILD (2003)
 - GI-WINNING SIRE Ver. 2 (2003)
 - GI-TURFWILD 2 (2004)
 - GI-TURFWILD 3 (2005)
 - GI-HORSEPARK (2005)
 - GI-HORSEPARK EX (2006)
 - GI-HORSEPARK EX STD (2006)
 - GI-HORSEPARK GX STD (2009)
 - GI-HORSEPARK GX (2009)
 - GI-Turf TV (2010)

***************************************************************************/

#include "emu.h"
#include "cpu/m68000/m68000.h"
#include "machine/k053252.h"
#include "video/k055555.h"
#include "video/k054156_k054157_k056832.h"
#include "video/k053246_k053247_k055673.h"
#include "video/konami_helper.h"
#include "screen.h"
#include "speaker.h"

class giclassic_state : public driver_device
{
public:
	giclassic_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_k056832(*this, "k056832"),
		m_palette(*this, "palette")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<k056832_device> m_k056832;
	required_device<palette_device> m_palette;

	DECLARE_PALETTE_INIT(giclassic);

	INTERRUPT_GEN_MEMBER(giclassic_interrupt);

	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_giclassic(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	K056832_CB_MEMBER(tile_callback);

	DECLARE_WRITE16_MEMBER(control_w);
	DECLARE_READ16_MEMBER(vrom_r);

private:
	uint8_t m_control;
};

// --------------------------------------------------------------------------------------------------------------
// Client portion
// --------------------------------------------------------------------------------------------------------------

K056832_CB_MEMBER(giclassic_state::tile_callback)
{
	*color = (*color & 0xf);
}

void giclassic_state::video_start()
{
}

uint32_t giclassic_state::screen_update_giclassic(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(0, cliprect);
	screen.priority().fill(0, cliprect);

	m_k056832->tilemap_draw(screen, bitmap, cliprect, 3, 0, 1);
	m_k056832->tilemap_draw(screen, bitmap, cliprect, 2, 0, 2);
	m_k056832->tilemap_draw(screen, bitmap, cliprect, 1, 0, 4);
//  m_k056832->tilemap_draw(screen, bitmap, cliprect, 0, 0, 8);

	return 0;
}

INTERRUPT_GEN_MEMBER(giclassic_state::giclassic_interrupt)
{
	if (m_control & 2)
	{
		m_maincpu->set_input_line(M68K_IRQ_1, HOLD_LINE);
		m_maincpu->set_input_line(M68K_IRQ_3, HOLD_LINE);
	}
}

WRITE16_MEMBER(giclassic_state::control_w)
{
	// bits:
	// 0 = ?
	// 1 = IRQ enable
	// 2 = ?
	// 3 = extra VROM readback bank
	// 4 = screen on?

	m_control = data & 0xff;
}

READ16_MEMBER(giclassic_state::vrom_r)
{
	if (m_control & 8)
	{
		return m_k056832->piratesh_rom_r(space, offset + 0x1000);
	}

	return m_k056832->piratesh_rom_r(space, offset);
}

static ADDRESS_MAP_START( satellite_main, AS_PROGRAM, 16, giclassic_state )
	AM_RANGE(0x000000, 0x07ffff) AM_ROM AM_REGION("maincpu", 0)
	AM_RANGE(0x100000, 0x103fff) AM_RAM
	AM_RANGE(0x200000, 0x200fff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x800000, 0x801fff) AM_RAM AM_DEVREADWRITE("k056832", k056832_device, ram_word_r, ram_word_w)
	AM_RANGE(0x900000, 0x90003f) AM_DEVREADWRITE("k056832", k056832_device, word_r, word_w)
	AM_RANGE(0xb00000, 0xb01fff) AM_READ(vrom_r)
	AM_RANGE(0xc00000, 0xc00001) AM_WRITE(control_w)
	AM_RANGE(0xd00000, 0xd0003f) AM_RAM // these must read/write or 26S (LCD controller) fails
	AM_RANGE(0xe00000, 0xe0001f) AM_DEVWRITE8("k056832", k056832_device, b_w, 0xff00)
	AM_RANGE(0xf00000, 0xf00001) AM_NOP AM_WRITENOP // watchdog reset
ADDRESS_MAP_END

static INPUT_PORTS_START( giclassic )
INPUT_PORTS_END

void giclassic_state::machine_start()
{
}

void giclassic_state::machine_reset()
{
}

// --------------------------------------------------------------------------------------------------------------
// Server portion
// --------------------------------------------------------------------------------------------------------------

class giclassicsvr_state : public driver_device
{
public:
	giclassicsvr_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_k056832(*this, "k056832"),
		m_k055673(*this, "k055673"),
		m_palette(*this, "palette")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<k056832_device> m_k056832;
	required_device<k055673_device> m_k055673;
	required_device<palette_device> m_palette;

	INTERRUPT_GEN_MEMBER(giclassicsvr_interrupt);

	virtual void machine_start() override;
	virtual void machine_reset() override;
	uint32_t screen_update_giclassicsvr(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	K056832_CB_MEMBER(tile_callback);
	K055673_CB_MEMBER(sprite_callback);

	DECLARE_WRITE16_MEMBER(control_w);
	DECLARE_READ16_MEMBER(control_r);

private:
	uint16 m_control;
};

WRITE16_MEMBER(giclassicsvr_state::control_w)
{
	m_control = data;
}

READ16_MEMBER(giclassicsvr_state::control_r)
{
	return m_control;
}

INTERRUPT_GEN_MEMBER(giclassicsvr_state::giclassicsvr_interrupt)
{
	//if (m_control & 2)
	{
		m_maincpu->set_input_line(M68K_IRQ_4, HOLD_LINE);
		m_maincpu->set_input_line(M68K_IRQ_5, HOLD_LINE);
	}
}

K056832_CB_MEMBER(giclassicsvr_state::tile_callback)
{
}

K055673_CB_MEMBER(giclassicsvr_state::sprite_callback)
{
	int c = *color;

	*color = (c & 0x001f);
	//int pri = (c >> 5) & 7;
	// .... .... ...x xxxx - Color
	// .... .... xxx. .... - Priority?
	// .... ..x. .... .... - ?
	// ..x. .... .... .... - ?

	*priority_mask = 0;

	// 0 - Sprites over everything
	// f0 -
	// f0 cc -
	// f0 cc aa -

	// 1111 0000
	// 1100 1100
	// 1010 1010
}


uint32_t giclassicsvr_state::screen_update_giclassicsvr(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(0, cliprect);
	screen.priority().fill(0, cliprect);

	m_k056832->tilemap_draw(screen, bitmap, cliprect, 3, 0, 1);
	m_k056832->tilemap_draw(screen, bitmap, cliprect, 2, 0, 2);
	m_k056832->tilemap_draw(screen, bitmap, cliprect, 1, 0, 4);
	m_k056832->tilemap_draw(screen, bitmap, cliprect, 0, 0, 8);

	return 0;
}

static ADDRESS_MAP_START( server_main, AS_PROGRAM, 16, giclassicsvr_state )
	AM_RANGE(0x000000, 0x07ffff) AM_ROM AM_REGION("maincpu", 0)
	AM_RANGE(0x080000, 0x08ffff) AM_RAM
	AM_RANGE(0x090000, 0x093fff) AM_RAM
	AM_RANGE(0x100000, 0x107fff) AM_RAM AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x180000, 0x183fff) AM_RAM
	AM_RANGE(0x280000, 0x281fff) AM_RAM AM_DEVREADWRITE("k056832", k056832_device, ram_word_r, ram_word_w)
	AM_RANGE(0x300000, 0x300007) AM_DEVWRITE("k055673", k055673_device, k053246_word_w) // SPRITES
	AM_RANGE(0x300060, 0x30006f) AM_DEVREAD("k055673", k055673_device, k055673_ps_rom_word_r) // SPRITES
	AM_RANGE(0x308000, 0x30803f) AM_DEVREADWRITE("k056832", k056832_device, word_r, word_w)
	AM_RANGE(0x320000, 0x32001f) AM_DEVREADWRITE8("k053252a", k053252_device, read, write, 0x00ff) // CRTC 1
	AM_RANGE(0x320000, 0x32001f) AM_DEVREADWRITE8("k053252b", k053252_device, read, write, 0xff00) // CRTC 2
	AM_RANGE(0x380000, 0x380001) AM_WRITENOP    // watchdog reset
	AM_RANGE(0x398000, 0x398001) AM_READWRITE(control_r, control_w)
	AM_RANGE(0x400000, 0x41ffff) AM_RAM
ADDRESS_MAP_END

static INPUT_PORTS_START( giclassvr )
INPUT_PORTS_END

void giclassicsvr_state::machine_start()
{
}

void giclassicsvr_state::machine_reset()
{
}

static MACHINE_CONFIG_START( giclassic )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M68000, XTAL_20MHz / 2) // PCB is marked "68000 12 MHz", but only visible osc is 20 MHz
	MCFG_CPU_PROGRAM_MAP(satellite_main)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", giclassic_state, giclassic_interrupt)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(59.62)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(600, 384)
	MCFG_SCREEN_VISIBLE_AREA(0, 599, 0, 383)
	MCFG_SCREEN_UPDATE_DRIVER(giclassic_state, screen_update_giclassic)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 256)
	MCFG_PALETTE_ENABLE_SHADOWS()
	MCFG_PALETTE_FORMAT(xxxxBBBBGGGGRRRR)

	MCFG_DEVICE_ADD("k056832", K056832, 0)
	MCFG_K056832_CB(giclassic_state, tile_callback)
	MCFG_K056832_CONFIG("gfx1", K056832_BPP_4PIRATESH, 1, 0, "none")
	MCFG_K056832_PALETTE("palette")
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( giclassvr )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M68000, XTAL_16MHz) // unknown speed
	MCFG_CPU_PROGRAM_MAP(server_main)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", giclassicsvr_state, giclassicsvr_interrupt)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(59.62)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(600, 384)
	MCFG_SCREEN_VISIBLE_AREA(0, 599, 0, 383)
	MCFG_SCREEN_UPDATE_DRIVER(giclassicsvr_state, screen_update_giclassicsvr)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 16384)
	MCFG_PALETTE_ENABLE_SHADOWS()
	MCFG_PALETTE_FORMAT(xxxxBBBBGGGGRRRR)

	MCFG_DEVICE_ADD("k056832", K056832, 0)
	MCFG_K056832_CB(giclassicsvr_state, tile_callback)
	MCFG_K056832_CONFIG("gfx1", K056832_BPP_4PIRATESH, 0, 0, "none")
	MCFG_K056832_PALETTE("palette")

	MCFG_DEVICE_ADD("k055673", K055673, 0)
	MCFG_K055673_CB(giclassicsvr_state, sprite_callback)
	MCFG_K055673_CONFIG("gfx2", K055673_LAYOUT_PS, -60, 24)
	MCFG_K055673_PALETTE("palette")

	MCFG_DEVICE_ADD("k053252a", K053252, XTAL_32MHz/4)
	MCFG_K053252_OFFSETS(40, 16) // TODO

	MCFG_DEVICE_ADD("k053252b", K053252, XTAL_32MHz/4)
	MCFG_K053252_OFFSETS(40, 16) // TODO
MACHINE_CONFIG_END

ROM_START( giclasex )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* main program */
	ROM_LOAD16_WORD_SWAP( "gsgu760ae01.12t", 0x000000, 0x080000, CRC(f0f9c118) SHA1(1753d53946bc0703d329e4a09c452713b260da75) )

	ROM_REGION( 0x100000, "gfx1", 0 )   /* tilemaps */
	ROM_LOAD( "gsgu760ae03.14c", 0x000000, 0x080000, CRC(1663d327) SHA1(98c1a9653d38f4918f78b3a11af0c29c658201f5) )
	ROM_LOAD( "gsgu760ae02.14e", 0x080000, 0x080000, CRC(2b9fe163) SHA1(f60190a9689a70d6c5bb14fb46b7ac2267cf0969) )
ROM_END

ROM_START( giclassvr )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* main program */
	ROM_LOAD16_WORD_SWAP( "gsgu_760_fd01.34e.bin", 0x000000, 0x080000, CRC(da89c1d7) SHA1(551d050a9b6e54fbf98e966eb37924b644037893) )

	ROM_REGION( 0x100000, "gfx1", 0 )   /* tilemaps */
	ROM_LOAD( "gsgu_760_ad04.25q", 0x080000, 0x080000, CRC(71a45742) SHA1(fbddd54f5fb236662f7cc7e9b350723bc5404f72) )
	ROM_LOAD( "gsgu_760_ad05.25r", 0x000000, 0x080000, CRC(44221eec) SHA1(966452e606e828b536ed11cbdd626a2fe3165199) )

	ROM_REGION( 0x100000, "gfx2", 0 )   /* tilemaps */
	ROM_LOAD32_WORD( "gsgu_760_ad02.34j", 0x000000, 0x080000, CRC(6d33c720) SHA1(35da3e1f0133a76480d2078fae89ea87b841ffc7) )
	ROM_LOAD32_WORD( "gsgu_760_ad02.34k", 0x000002, 0x080000, CRC(8057a417) SHA1(82d4a1d84729e9f0a8aff4c219a19601b89caf15) )
ROM_END

GAME( 1998, giclasex,  0, giclassic, giclassic, giclassic_state,    0, 0, "Konami", "GI-Classic EX (satellite terminal)", MACHINE_NOT_WORKING|MACHINE_NO_SOUND_HW)
GAME( 1998, giclassvr, 0, giclassvr, giclassvr, giclassicsvr_state, 0, 0, "Konami", "GI-Classic EX (server)",             MACHINE_NOT_WORKING|MACHINE_NO_SOUND_HW)
