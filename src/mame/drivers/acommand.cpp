// license:BSD-3-Clause
// copyright-holders:Angelo Salese
/*******************************************************************************************

Alien Command (c) 1993 Jaleco

driver by Angelo Salese

Actually same HW as the Cisco Heat ones.

TODO:
-Understand what "devices" area needs to make this working.It's likely that the upper switches
 controls the UFO's and the lower switches the astronauts.
-Back tilemap paging is likely to be incorrect.
-3D Artworks for the UFO's,Astronauts etc.
-Merge to the Cisco Heat driver.

Notes:
-The real HW is a redemption machine with two guns, similar to the "Cosmo Gang the Video"
(Namco) redemption version.

m68k irq table vectors
lev 1 : 0x64 : 0000 04f0 - rte
lev 2 : 0x68 : 0000 044a - vblank
lev 3 : 0x6c : 0000 0484 - "dynamic color change" (?)
lev 4 : 0x70 : 0000 04f0 - rte
lev 5 : 0x74 : 0000 04f0 - rte
lev 6 : 0x78 : 0000 04f0 - rte
lev 7 : 0x7c : 0000 04f0 - rte

===========================================================================================

Jaleco Alien Command
Redemption Video Game with Guns

2/7/99

Hardware Specs: 68000 at 12Mhz and OKI6295

JALMR17  BIN       524,288  02-07-99  1:17a JALMR17.BIN
JALCF2   BIN     1,048,576  02-07-99  1:10a JALCF2.BIN
JALCF3   BIN       131,072  02-07-99  1:12a JALCF3.BIN
JALCF4   BIN       131,072  02-07-99  1:13a JALCF4.BIN
JALCF5   BIN       524,288  02-07-99  1:15a JALCF5.BIN
JALCF6   BIN       131,072  02-07-99  1:14a JALCF6.BIN
JALGP1   BIN       524,288  02-07-99  1:21a JALGP1.BIN
JALGP2   BIN       524,288  02-07-99  1:24a JALGP2.BIN
JALGP3   BIN       524,288  02-07-99  1:20a JALGP3.BIN
JALGP4   BIN       524,288  02-07-99  1:23a JALGP4.BIN
JALGP5   BIN       524,288  02-07-99  1:19a JALGP5.BIN
JALGP6   BIN       524,288  02-07-99  1:23a JALGP6.BIN
JALGP7   BIN       524,288  02-07-99  1:19a JALGP7.BIN
JALGP8   BIN       524,288  02-07-99  1:22a JALGP8.BIN
JALMR14  BIN       524,288  02-07-99  1:17a JALMR14.BIN
JALCF1   BIN     1,048,576  02-07-99  1:11a JALCF1.BIN


*******************************************************************************************/

#include "emu.h"
#include "cpu/m68000/m68000.h"
#include "sound/okim6295.h"
#include "video/ms1_tmap.h"
#include "screen.h"
#include "speaker.h"

#include "acommand.lh"


class acommand_state : public driver_device
{
public:
	acommand_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_spriteram(*this, "spriteram"),
		m_maincpu(*this, "maincpu"),
		m_oki1(*this, "oki1"),
		m_oki2(*this, "oki2"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_bgtmap(*this, "bgtmap"),
		m_txtmap(*this, "txtmap") { }

	required_shared_ptr<uint16_t> m_spriteram;
	std::unique_ptr<uint16_t[]> m_ac_vregs;
	uint16_t m_7seg0;
	uint16_t m_7seg1;
	uint16_t m_ufo_lane[5];
	uint8_t m_boss_door;
	DECLARE_WRITE8_MEMBER(oki_bank_w);
	DECLARE_WRITE16_MEMBER(output_7seg0_w);
	DECLARE_WRITE16_MEMBER(output_7seg1_w);
	DECLARE_WRITE16_MEMBER(output_lamps_w);

	DECLARE_READ16_MEMBER(ext_devices_0_r);
	DECLARE_WRITE16_MEMBER(ext_devices_0_w);
	DECLARE_READ16_MEMBER(ext_devices_1_r);
	DECLARE_WRITE16_MEMBER(ext_devices_1_w);
	DECLARE_WRITE16_MEMBER(ext_devices_2_w);

	DECLARE_WRITE16_MEMBER(ac_unk2_w);
	TILEMAP_MAPPER_MEMBER(bg_scan);
	virtual void video_start() override;
	uint32_t screen_update_acommand(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_DEVICE_CALLBACK_MEMBER(acommand_scanline);
	void draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect, int priority, int pri_mask);
	required_device<cpu_device> m_maincpu;
	required_device<okim6295_device> m_oki1;
	required_device<okim6295_device> m_oki2;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<megasys1_tilemap_device> m_bgtmap;
	required_device<megasys1_tilemap_device> m_txtmap;
};



void acommand_state::draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect, int priority, int pri_mask)
{
	uint16_t *spriteram16 = m_spriteram;
	int offs;

	for (offs = 0;offs < m_spriteram.bytes()/2;offs += 8)
	{
		if (!(spriteram16[offs+0] & 0x1000))
		{
			int sx = (spriteram16[offs+3] & 0x0ff);
			int code = spriteram16[offs+6];
			int color = spriteram16[offs+7];
			int w = (spriteram16[offs+0] & 0x0f);
			int h = ((spriteram16[offs+0] & 0xf0) >> 4);
			int sy = (spriteram16[offs+4] & 0x0ff) - ((h+1)*0x10);
/**/        int pri = spriteram16[offs+5];
/**/        int flipy = ((spriteram16[offs+1] & 0x2000) >> 13);
			// "this is the boss" uses flip x
			int flipx = ((spriteram16[offs+1] & 0x1000) >> 12);

			int xx,yy,x;
			int delta = 16;

			flipx ^= flip_screen();
			flipy ^= flip_screen();

			if ((pri&pri_mask)!=priority) continue;

			if (flip_screen())
			{
				sx = 368 - sx;
				sy = 240 - sy;
				delta = -16;
			}

			yy = h;
			do
			{
				x = flipx ? sx + w*16 : sx;
				xx = w;
				do
				{
					m_gfxdecode->gfx(0)->transpen(bitmap,cliprect,
							code,
							color,
							flipx, flipy,
							((x + 16) & 0x1ff) - 16,sy & 0x1ff,15);

					code++;
					if(flipx)
						x -= delta;
					else
						x += delta;
				} while (--xx >= 0);

				sy += delta;
			} while (--yy >= 0);
		}
	}
}


void acommand_state::video_start()
{
	m_ac_vregs = std::make_unique<uint16_t[]>(0x80/2);
}


uint32_t acommand_state::screen_update_acommand(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	// reference has black pen background, as weird it might sound
	bitmap.fill(m_palette->black_pen(), cliprect);

	m_bgtmap->draw(screen, bitmap, cliprect, 0, 0);
	draw_sprites(bitmap,cliprect,0,0);
	m_txtmap->draw(screen, bitmap, cliprect, 0, 0);

	return 0;
}


/******************************************************************************************/

/*This is always zero ATM*/
WRITE16_MEMBER(acommand_state::ac_unk2_w)
{
	if(data)
		popmessage("UNK-2 enabled %04x",data);
}

/*************************************
 *
 * I/O
 *
 ************************************/

WRITE8_MEMBER(acommand_state::oki_bank_w)
{
	m_oki1->set_rom_bank(data & 0x3);
	m_oki2->set_rom_bank((data & 0x30) >> 4);
}


/*                                    0    1    2    3    4    5    6    7    8    9    a    b    c    d    e    f*/
static const uint8_t led_fill[0x10] = { 0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x00,0x00,0x00,0x00,0x00,0x00};

WRITE16_MEMBER(acommand_state::output_7seg0_w)
{
	COMBINE_DATA(&m_7seg0);

	// nybble 0,1,2: left 7segs, nybble 3: right 7seg 1st digit
	for (int i = 0; i < 4; i++)
		output().set_digit_value(i, led_fill[m_7seg0 >> (i*4) & 0xf]);
}

WRITE16_MEMBER(acommand_state::output_7seg1_w)
{
	COMBINE_DATA(&m_7seg1);

	// nybble 0,1: right 7seg 2nd,3rd digit
	for (int i = 0; i < 2; i++)
		output().set_digit_value(i+4, led_fill[m_7seg1 >> (i*4) & 0xf]);

	// other: ?
}

/*
    "Upper switch / Under Switch"
    xx-x ---- xx-x xx-x
    -x-- ---- ---- ---- Catch Switch - 3
    --x- ---- ---- ---- Lower Switch - 3
    ---x ---- ---- ---- Upper Switch - 3
    ---- -x-- ---- ---- Catch Switch - 2
    ---- --x- ---- ---- Lower Switch - 2
    ---- ---x ---- ---- Upper Switch - 2
    ---- ---- -x-- ---- Catch Switch - 1
    ---- ---- --x- ---- Lower Switch - 1 (active high)
    ---- ---- ---x ---- Upper Switch - 1 (active low)
    ---- ---- ---- --xx Boss Door - Motor
    state of UFO lanes:
    0x0
    0x1
    0x2
    0x3
    0x4
    0x5 ufo lane limit switch
    0x6
    0x7 astronaut switch or jamming
    0x8
    0x9 ufo lane switch or motor
    0xa astronaut switch or jamming
    0xb ufo lane switch or motor
    0xc ""
    0xd ufo lane limit switch
    0xe astronaut switch or jamming
    0xf ""
*/
READ16_MEMBER(acommand_state::ext_devices_0_r)
{
	return 0xfffc | m_boss_door;
}

WRITE16_MEMBER(acommand_state::ext_devices_0_w)
{
	printf("%04x EXT 0\n",data);
	m_boss_door = data & 3;
	m_ufo_lane[0] = (data >> 8) & 0x1f;
}

/*
    ---- ---- --x- ---- Lower Switch - 5
    ---- ---- ---x ---- Upper Switch - 5
    ---- ---- ---- --x- Lower Switch - 4 (active high)
    ---- ---- ---- ---x Upper Switch - 4 (active low)
*/
READ16_MEMBER(acommand_state::ext_devices_1_r)
{
	return 0xffff;
}

WRITE16_MEMBER(acommand_state::ext_devices_1_w)
{
	//printf("%04x EXT 1\n",data);
	m_ufo_lane[1] = (data >> 0) & 0x1f;
	m_ufo_lane[2] = (data >> 8) & 0x1f;
}

WRITE16_MEMBER(acommand_state::ext_devices_2_w)
{
	//printf("%04x EXT 2\n",data);
	m_ufo_lane[3] = (data >> 0) & 0x1f;
	m_ufo_lane[4] = (data >> 8) & 0x1f;
}

WRITE16_MEMBER(acommand_state::output_lamps_w)
{
	machine().bookkeeping().coin_counter_w(0, data & 0x40);
	machine().bookkeeping().coin_counter_w(1, data & 0x80);

	// --xx --xx lamps
}

static ADDRESS_MAP_START( acommand_map, AS_PROGRAM, 16, acommand_state )
	AM_RANGE(0x000000, 0x03ffff) AM_ROM
	AM_RANGE(0x082000, 0x082005) AM_DEVWRITE("bgtmap", megasys1_tilemap_device, scroll_w)
	AM_RANGE(0x082100, 0x082105) AM_DEVWRITE("txtmap", megasys1_tilemap_device, scroll_w)
	AM_RANGE(0x082208, 0x082209) AM_WRITE(ac_unk2_w)
	AM_RANGE(0x0a0000, 0x0a3fff) AM_RAM_DEVWRITE("bgtmap", megasys1_tilemap_device, write) AM_SHARE("bgtmap")
	AM_RANGE(0x0b0000, 0x0b3fff) AM_RAM_DEVWRITE("txtmap", megasys1_tilemap_device, write) AM_SHARE("txtmap")
	AM_RANGE(0x0b8000, 0x0bffff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x0f0000, 0x0f7fff) AM_RAM
	AM_RANGE(0x0f8000, 0x0f8fff) AM_RAM AM_SHARE("spriteram")
	AM_RANGE(0x0f9000, 0x0fffff) AM_RAM

	AM_RANGE(0x100000, 0x100001) AM_WRITE8(oki_bank_w,0x00ff)
	AM_RANGE(0x100008, 0x100009) AM_READ_PORT("IN0") AM_WRITE(output_lamps_w)
	AM_RANGE(0x100014, 0x100017) AM_DEVREADWRITE8("oki1", okim6295_device, read, write, 0x00ff)
	AM_RANGE(0x100018, 0x10001b) AM_DEVREADWRITE8("oki2", okim6295_device, read, write, 0x00ff)

	AM_RANGE(0x100040, 0x100041) AM_READWRITE(ext_devices_0_r,ext_devices_0_w)
	AM_RANGE(0x100044, 0x100045) AM_READWRITE(ext_devices_1_r,ext_devices_1_w)
	AM_RANGE(0x100048, 0x100049) AM_WRITE(ext_devices_2_w)

	AM_RANGE(0x100050, 0x100051) AM_WRITE(output_7seg0_w)
	AM_RANGE(0x100054, 0x100055) AM_WRITE(output_7seg1_w)
	AM_RANGE(0x10005c, 0x10005d) AM_READ_PORT("DSW")

ADDRESS_MAP_END

static INPUT_PORTS_START( acommand )
	PORT_START("IN0")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_SERVICE_NO_TOGGLE( 0x0020, IP_ACTIVE_LOW )
	PORT_DIPNAME( 0x0040, 0x0040, "IN0" )
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0080, 0x0080, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0080, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_DIPNAME( 0x0200, 0x0200, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0200, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_DIPNAME( 0x0800, 0x0800, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0800, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x1000, 0x1000, "Ticket Dispenser - 1" )
	PORT_DIPSETTING(      0x1000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x2000, 0x2000, "Ticket Dispenser - 2")
	PORT_DIPSETTING(      0x2000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x4000, 0x4000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x4000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x8000, 0x8000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x8000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )

	PORT_START("DSW")
	PORT_DIPNAME( 0x0001, 0x0001, "IN2" )
	PORT_DIPSETTING(      0x0001, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0002, 0x0002, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0002, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0004, 0x0004, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0004, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0008, 0x0008, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0008, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0010, 0x0010, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0010, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0020, 0x0020, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0020, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0040, 0x0040, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0080, 0x0080, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0080, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0100, 0x0100, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0100, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0200, 0x0200, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0200, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0400, 0x0400, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0400, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0800, 0x0800, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0800, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x1000, 0x1000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x1000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x2000, 0x2000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x2000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x4000, 0x4000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x4000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x8000, 0x8000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x8000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
INPUT_PORTS_END

static const gfx_layout tilelayout =
{
	16,16,
	RGN_FRAC(1,1),
	4,
	{ 0, 1, 2, 3 },
	{ 0*4, 1*4, 2*4, 3*4, 4*4, 5*4, 6*4, 7*4,
			16*32+0*4, 16*32+1*4, 16*32+2*4, 16*32+3*4, 16*32+4*4, 16*32+5*4, 16*32+6*4, 16*32+7*4 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32,
			8*32, 9*32, 10*32, 11*32, 12*32, 13*32, 14*32, 15*32 },
	32*32
};

static GFXDECODE_START( acommand )
	GFXDECODE_ENTRY( "gfx3", 0, tilelayout, 0x1800, 256 )
GFXDECODE_END

TIMER_DEVICE_CALLBACK_MEMBER(acommand_state::acommand_scanline)
{
	int scanline = param;

	if(scanline == 240) // vblank-out irq
		m_maincpu->set_input_line(2, HOLD_LINE);

	if(scanline == 0) // vblank-in irq? (update palette and layers)
		m_maincpu->set_input_line(3, HOLD_LINE);
}

static MACHINE_CONFIG_START( acommand )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu",M68000,12000000)
	MCFG_CPU_PROGRAM_MAP(acommand_map)
	MCFG_TIMER_DRIVER_ADD_SCANLINE("scantimer", acommand_state, acommand_scanline, "screen", 0, 1)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(acommand_state, screen_update_acommand)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", acommand)
	MCFG_PALETTE_ADD("palette", 0x4000)
	MCFG_PALETTE_FORMAT(RRRRGGGGBBBBRGBx)

	MCFG_MEGASYS1_TILEMAP_ADD("bgtmap", "palette", 0x0f00)
	MCFG_MEGASYS1_TILEMAP_ADD("txtmap", "palette", 0x2700)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")

	MCFG_OKIM6295_ADD("oki1", 2112000, PIN7_HIGH) // clock frequency & pin 7 not verified
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 1.0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 1.0)

	MCFG_OKIM6295_ADD("oki2", 2112000, PIN7_HIGH) // clock frequency & pin 7 not verified
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 1.0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 1.0)
MACHINE_CONFIG_END

/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( acommand )
	ROM_REGION( 0x040000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "jalcf3.bin",   0x000000, 0x020000, CRC(f031abf7) SHA1(e381742fd6a6df4ddae42ddb3a074a55dc550b3c) )
	ROM_LOAD16_BYTE( "jalcf4.bin",   0x000001, 0x020000, CRC(dd0c0540) SHA1(3e788fcb30ae725bd0ec9b57424e3946db1e946f) )

	ROM_REGION( 0x20000, "txtmap", 0 ) /* BG0 */
	ROM_LOAD( "jalcf6.bin",   0x000000, 0x020000, CRC(442173d6) SHA1(56c02bc2761967040127977ecabe844fc45e2218) )

	ROM_REGION( 0x080000, "bgtmap", 0 ) /* BG1 */
	ROM_LOAD( "jalcf5.bin",   0x000000, 0x080000, CRC(ff0be97f) SHA1(5ccab778318dec30849d7b7f25091d4aab8bde32) )

	ROM_REGION( 0x400000, "gfx3", 0 ) /* SPR */
	ROM_LOAD16_BYTE( "jalgp1.bin",   0x000000, 0x080000, CRC(c4aeeae2) SHA1(ee0d3dd93a604f8e1a96b55c4a1cd001d49f1157) )
	ROM_LOAD16_BYTE( "jalgp2.bin",   0x000001, 0x080000, CRC(f0e4e80e) SHA1(08252ef8b5e309cce2d4654410142f4ae9e3ef22) )
	ROM_LOAD16_BYTE( "jalgp3.bin",   0x100000, 0x080000, CRC(7acebd83) SHA1(64be95186d62003b637fcdf45a9c0b7aab182116) )
	ROM_LOAD16_BYTE( "jalgp4.bin",   0x100001, 0x080000, CRC(6a6b72f3) SHA1(3ba359b1a89eb3f6664ed83d93f79d7f895d4222) )
	ROM_LOAD16_BYTE( "jalgp5.bin",   0x200000, 0x080000, CRC(65ab751d) SHA1(f2cb8701eb8c3567a1d03248e6918c5a7b5df939) )
	ROM_LOAD16_BYTE( "jalgp6.bin",   0x200001, 0x080000, CRC(24e3ab23) SHA1(d1431688e1518ba52935f6ab44b815975bec4c27) )
	ROM_LOAD16_BYTE( "jalgp7.bin",   0x300000, 0x080000, CRC(44b71098) SHA1(a6ec2573f9a266d4f8f315f6e99b12525011f512) )
	ROM_LOAD16_BYTE( "jalgp8.bin",   0x300001, 0x080000, CRC(ce0b7838) SHA1(46e34971cb62565a3948d8c0a18086648c32e13b) )

	ROM_REGION( 0x100000, "oki1", 0 )   /* M6295 samples */
	ROM_LOAD( "jalcf2.bin",   0x000000, 0x100000, CRC(b982fd97) SHA1(35ee5b1b9be762ccfefda24d73e329ceea876deb) )

	ROM_REGION( 0x100000, "oki2", 0 )   /* M6295 samples */
	ROM_LOAD( "jalcf1.bin",   0x000000, 0x100000, CRC(24af21d3) SHA1(f68ab81a6c833b57ae9eef916a1c8578f3d893dd) )

	ROM_REGION( 0x100000, "user1", 0 ) /* ? these two below are identical*/
	ROM_LOAD( "jalmr14.bin",   0x000000, 0x080000, CRC(9d428fb7) SHA1(02f72938d73db932bd217620a175a05215f6016a) )
	ROM_LOAD( "jalmr17.bin",   0x080000, 0x080000, CRC(9d428fb7) SHA1(02f72938d73db932bd217620a175a05215f6016a) )
ROM_END

GAMEL( 1994, acommand,  0,      acommand,  acommand, acommand_state,  0, ROT0, "Jaleco", "Alien Command" , MACHINE_NOT_WORKING | MACHINE_MECHANICAL, layout_acommand )
