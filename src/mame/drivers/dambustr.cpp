// license:GPL-2.0+
// copyright-holders:Norbert Kehrer
/***************************************************************************

Dambusters
(c) 1981 South West Research Ltd. (Bristol, UK)

Reverse-engineering and MAME Driver by Norbert Kehrer (August 2006)

NOTE:  Eventually to be merged into GALAXIAN.C

2008-08
Dip locations verified with manual


Stephh's notes (based on the games Z80 code and some tests) :

1) 'dambustr'

  - This seems to be a bugfixed version as only the 3 "fire" buttons are
    tested while entering the initials (code at 0x1e59).
  - The "Initials Reset" button only affects the initials, not the scores !
  - The "Disable Background Collision" Dip Switch only prevents you to lose
    "lives" if you touch the background : you can still be hit by enemies'
    bullets and be stalled if speed is too low.
  - There is a setting which is only available in the "test mode", it's the
    maximum game time. If you don't set it, the game considers it's illimited
    time (full doesn't decrease). But once you set it to a value, it is not
    possible to turn the value back to 0 even by reseting the game.
  - If you go too far without dying, background becomes complete garbage,
    but it then appears again correctly. Is there a bad dumped ROM or is
    emulation bugged somewhere ? Verification against real PCB is needed !

2) 'dambustra'

  - There is sort of bug at 0x1e59 : you can't enter a letter in the
    initials screen while pressing one of the system buttons (SERVICE,
    "Initials Reset", START1 or START2), and even COIN2.
    This is because 8 bytes are checked instead of 3 in 'dambustr'.
  - Same as 'dambustr' otherwise.

3) 'dambustruk'

  - This set is based on 'dambustr' as there is no initials bug (code at 0x1e0f).
  - The differences with 'dambustr' are :
      * coinage for 2nd slot
      * currency and price (but it's still 2 credits to start a game)
      * score is divided by 10

***************************************************************************/


#include "emu.h"
#include "includes/galaxold.h"
#include "audio/galaxian.h"

#include "cpu/z80/z80.h"
#include "machine/watchdog.h"
#include "speaker.h"


class dambustr_state : public galaxold_state
{
public:
	dambustr_state(const machine_config &mconfig, device_type type, const char *tag)
		: galaxold_state(mconfig, type, tag),
		m_custom(*this, "cust") { }

	required_device<galaxian_sound_device> m_custom;

	int m_noise_data;
	DECLARE_WRITE8_MEMBER(dambustr_noise_enable_w);
	DECLARE_DRIVER_INIT(dambustr);
};



/* FIXME: Really needed? - Should be handled by either interface */
WRITE8_MEMBER(dambustr_state::dambustr_noise_enable_w)
{
	if (data != m_noise_data) {
		m_noise_data = data;
		m_custom->noise_enable_w(space, offset, data);
	}
}


static ADDRESS_MAP_START( dambustr_map, AS_PROGRAM, 8, dambustr_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM

	AM_RANGE(0x8000, 0x8000) AM_WRITE(dambustr_bg_color_w)
	AM_RANGE(0x8001, 0x8001) AM_WRITE(dambustr_bg_split_line_w)

	AM_RANGE(0xc000, 0xc7ff) AM_RAM

	AM_RANGE(0xd000, 0xd3ff) AM_RAM_WRITE(galaxold_videoram_w) AM_SHARE("videoram")
	AM_RANGE(0xd400, 0xd7ff) AM_READ(galaxold_videoram_r)
	AM_RANGE(0xd800, 0xd83f) AM_RAM_WRITE(galaxold_attributesram_w) AM_SHARE("attributesram")
	AM_RANGE(0xd840, 0xd85f) AM_RAM AM_SHARE("spriteram")
	AM_RANGE(0xd860, 0xd87f) AM_RAM AM_SHARE("bulletsram")

	AM_RANGE(0xd880, 0xd8ff) AM_RAM

	AM_RANGE(0xe000, 0xe000) AM_READ_PORT("IN0")
	AM_RANGE(0xe002, 0xe003) AM_WRITE(galaxold_coin_counter_w)
	AM_RANGE(0xe004, 0xe007) AM_DEVWRITE("cust", galaxian_sound_device, lfo_freq_w)

	AM_RANGE(0xe800, 0xefff) AM_READ_PORT("IN1")
	AM_RANGE(0xe800, 0xe802) AM_DEVWRITE("cust", galaxian_sound_device, background_enable_w)
	AM_RANGE(0xe803, 0xe803) AM_WRITE(dambustr_noise_enable_w)
	AM_RANGE(0xe804, 0xe804) AM_DEVWRITE("cust", galaxian_sound_device, fire_enable_w) // probably louder than normal shot
	AM_RANGE(0xe805, 0xe805) AM_DEVWRITE("cust", galaxian_sound_device, fire_enable_w) // normal shot (like Galaxian)
	AM_RANGE(0xe806, 0xe807) AM_DEVWRITE("cust", galaxian_sound_device, vol_w)

	AM_RANGE(0xf000, 0xf7ff) AM_READ_PORT("DSW")
	AM_RANGE(0xf001, 0xf001) AM_WRITE(galaxold_nmi_enable_w)
	AM_RANGE(0xf004, 0xf004) AM_WRITE(galaxold_stars_enable_w)
	AM_RANGE(0xf006, 0xf006) AM_WRITE(galaxold_flip_screen_x_w)
	AM_RANGE(0xf007, 0xf007) AM_WRITE(galaxold_flip_screen_y_w)

	AM_RANGE(0xf800, 0xf800) AM_DEVWRITE("cust", galaxian_sound_device, pitch_w)
	AM_RANGE(0xf800, 0xffff) AM_DEVREAD("watchdog", watchdog_timer_device, reset_r)
ADDRESS_MAP_END


/* verified from Z80 code */
static INPUT_PORTS_START( dambustr )
	PORT_START("IN0")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_COIN2 )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_COIN1 )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_START2 )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_START1 )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("Initials Reset") PORT_CODE(KEYCODE_0)
	PORT_SERVICE_NO_TOGGLE( 0x40, IP_ACTIVE_HIGH )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_BUTTON3 )

	PORT_START("IN1")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON2 )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_BUTTON1 )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_4WAY
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT )  PORT_4WAY
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN )  PORT_4WAY
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP )    PORT_4WAY
	PORT_DIPNAME( 0x40, 0x40, "Coin Counters" ) PORT_DIPLOCATION("SW:!1")
	PORT_DIPSETTING(    0x00, "1" )
	PORT_DIPSETTING(    0x40, "2" )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Coinage ) ) PORT_DIPLOCATION("SW:!2")
	PORT_DIPSETTING(    0x80, DEF_STR( 1C_1C ) )   PORT_CONDITION("IN1", 0x40, EQUALS, 0x00)
	PORT_DIPSETTING(    0x00, DEF_STR( 1C_2C ) )   PORT_CONDITION("IN1", 0x40, EQUALS, 0x00)
	PORT_DIPSETTING(    0x80, "A 1/1  B 1/2" )     PORT_CONDITION("IN1", 0x40, EQUALS, 0x40)
	PORT_DIPSETTING(    0x00, "A 1/2  B 1/6" )     PORT_CONDITION("IN1", 0x40, EQUALS, 0x40)

	PORT_START("DSW")
	PORT_DIPNAME( 0x03, 0x00, DEF_STR( Lives ) ) PORT_DIPLOCATION("SW:!3,!4")
	PORT_DIPSETTING(    0x00, "3" )
	PORT_DIPSETTING(    0x02, "4" )
	PORT_DIPSETTING(    0x01, "5" )
	PORT_DIPSETTING(    0x03, "6" )
	PORT_DIPNAME( 0x04, 0x00, "Disable Background Collision (Cheat)" ) PORT_DIPLOCATION("SW:!5") /* see notes - manual says must be OFF */
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, "Union Jack Flag" ) PORT_DIPLOCATION("SW:!6")
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_BIT( 0xf0, IP_ACTIVE_HIGH, IPT_UNUSED )
INPUT_PORTS_END

/* verified from Z80 code */
static INPUT_PORTS_START( dambustruk )
	PORT_INCLUDE(dambustr)

	PORT_MODIFY("IN1")
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Coinage ) ) PORT_DIPLOCATION("SW:!2")
	PORT_DIPSETTING(    0x80, DEF_STR( 1C_1C ) )   PORT_CONDITION("IN1", 0x40, EQUALS, 0x00)
	PORT_DIPSETTING(    0x00, DEF_STR( 1C_2C ) )   PORT_CONDITION("IN1", 0x40, EQUALS, 0x00)
	PORT_DIPSETTING(    0x80, "A 1/1  B 1/6" )     PORT_CONDITION("IN1", 0x40, EQUALS, 0x40)
	PORT_DIPSETTING(    0x00, "A 1/2  B 1/12" )    PORT_CONDITION("IN1", 0x40, EQUALS, 0x40)
INPUT_PORTS_END


static const gfx_layout dambustr_charlayout =
{
	8,8,
	RGN_FRAC(1,2),
	2,
	{ RGN_FRAC(0,2), RGN_FRAC(1,2) },
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	8*8
};


static const gfx_layout dambustr_spritelayout =
{
	16,16,
	RGN_FRAC(1,2),
	2,
	{ RGN_FRAC(0,2), RGN_FRAC(1,2) },
	{ 0, 1, 2, 3, 4, 5, 6, 7,
			8*8+0, 8*8+1, 8*8+2, 8*8+3, 8*8+4, 8*8+5, 8*8+6, 8*8+7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8,
			16*8, 17*8, 18*8, 19*8, 20*8, 21*8, 22*8, 23*8 },
	32*8
};


static GFXDECODE_START( dambustr )
	GFXDECODE_ENTRY( "gfx1", 0x0000, dambustr_charlayout,   0, 8 )
	GFXDECODE_ENTRY( "gfx1", 0x0000, dambustr_spritelayout, 0, 8 )
GFXDECODE_END


DRIVER_INIT_MEMBER(dambustr_state,dambustr)
{
	int i, j, tmp;
	int tmpram[16];
	uint8_t *rom = memregion("maincpu")->base();
	uint8_t *usr = memregion("user1")->base();
	uint8_t *gfx = memregion("gfx1")->base();

	// Bit swap addresses
	for(i=0; i<4096*4; i++) {
		rom[i] = usr[BITSWAP16(i,15,14,13,12, 4,10,9,8,7,6,5,3,11,2,1,0)];
	};

	// Swap program ROMs
	for(i=0; i<0x1000; i++) {
		tmp = rom[0x5000+i];
		rom[0x5000+i] = rom[0x6000+i];
		rom[0x6000+i] = rom[0x1000+i];
		rom[0x1000+i] = tmp;
	};

	// Bit swap in $1000-$1fff and $4000-$5fff
	for(i=0; i<0x1000; i++) {
		rom[0x1000+i] = BITSWAP8(rom[0x1000+i],7,6,5,1,3,2,4,0);
		rom[0x4000+i] = BITSWAP8(rom[0x4000+i],7,6,5,1,3,2,4,0);
		rom[0x5000+i] = BITSWAP8(rom[0x5000+i],7,6,5,1,3,2,4,0);
	};

	// Swap graphics ROMs
	for(i=0;i<0x4000;i+=16) {
		for(j=0; j<16; j++)
			tmpram[j] = gfx[i+j];
		for(j=0; j<8; j++) {
			gfx[i+j] = tmpram[j*2];
			gfx[i+j+8] = tmpram[j*2+1];
		};
	};
}



static MACHINE_CONFIG_START( dambustr )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, 18432000/6)    /* 3.072 MHz */
	MCFG_CPU_PROGRAM_MAP(dambustr_map)

	MCFG_MACHINE_RESET_OVERRIDE(dambustr_state,galaxold)

	MCFG_DEVICE_ADD("7474_9m_1", TTL7474, 0)
	MCFG_7474_OUTPUT_CB(WRITELINE(dambustr_state,galaxold_7474_9m_1_callback))

	MCFG_DEVICE_ADD("7474_9m_2", TTL7474, 0)
	MCFG_7474_COMP_OUTPUT_CB(WRITELINE(dambustr_state,galaxold_7474_9m_2_q_callback))

	MCFG_TIMER_DRIVER_ADD("int_timer", dambustr_state, galaxold_interrupt_timer)

	MCFG_WATCHDOG_ADD("watchdog")

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(16000.0/132/2)
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(dambustr_state, screen_update_dambustr)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", dambustr)
	MCFG_PALETTE_ADD("palette", 32+2+64+8)      /* 32 for the characters, 2 for the bullets, 64 for the stars, 8 for the background */

	MCFG_PALETTE_INIT_OWNER(dambustr_state,dambustr)
	MCFG_VIDEO_START_OVERRIDE(dambustr_state,dambustr)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")

	MCFG_FRAGMENT_ADD(galaxian_audio)
MACHINE_CONFIG_END


ROM_START( dambustr )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "db8a.pr8",   0x4000, 0x1000, CRC(fd041ff4) SHA1(8d27da7bf0c655633711b960cbc23950c8a371ae) )
	ROM_LOAD( "db6a.pr6",   0x5000, 0x1000, CRC(448db54b) SHA1(c9afbf02bf4d4ac2972ab7ac6adfa4e951ae79c2) )
	ROM_LOAD( "db7a.pr7",   0x6000, 0x1000, CRC(675b1f5e) SHA1(6a386212a640fb467b6956a4dc5a68476af1cf97) )
	ROM_LOAD( "db5a.pr5",   0x7000, 0x1000, CRC(75659ecc) SHA1(b61254fb12f3999607abd88d1cc649dcfbf0384c) )

	ROM_REGION( 0x10000,"user1",0)
	ROM_LOAD( "db11a.pr11",  0x0000, 0x1000, CRC(427bd3fb) SHA1(cdbaef4040fa2e0598a086e320d51ecb26a591dd) )
	ROM_LOAD( "db9a.pr9",    0x1000, 0x1000, CRC(57164563) SHA1(8471d0660f39511d0afa3cdd63a1e84b0ea80fd0) )
	ROM_LOAD( "db10a.pr10",  0x2000, 0x1000, CRC(075b9c5e) SHA1(ff6ce873897004c0e796813725e260df85a520f9) )
	ROM_LOAD( "db12a.pr12",  0x3000, 0x1000, CRC(ed01a68b) SHA1(9dd37c2a25865717a7acdd7e2a3bef26a4cef3d9) )

	ROM_REGION( 0x4000, "gfx1", 0 )
	ROM_LOAD( "db3a.pr3",   0x0000, 0x1000, CRC(9e9a9710) SHA1(a9f67a05a2882b9f6f3378cc73e90539de4b8ca4) )
	ROM_LOAD( "db1a.pr1",   0x1000, 0x1000, CRC(4cb964cd) SHA1(1c90b14deb201a64b8ed4378b022e9e4574aed94) )
	ROM_LOAD( "db4a.pr3",   0x2000, 0x1000, CRC(d9d2df33) SHA1(97057fe33c146898755b556558ff707b9f4551ec) )
	ROM_LOAD( "db2a.pr2",   0x3000, 0x1000, CRC(0a0a6af5) SHA1(ecd2a6696ce9154f030c830ccb45690787881a73) )

	ROM_REGION( 0x0020, "proms", 0 )
	ROM_LOAD( "mi-7603-5.ic4", 0x0000, 0x0020, CRC(f131f92f) SHA1(8b0f623d2ea09b5612dde0f330c5e473a7d72e06) ) /* near DB5 - DB8 */

	ROM_REGION( 0x0020, "unk_prom", 0 ) //timing?
	ROM_LOAD( "mi-7603-5.ic3", 0x0000, 0x0020, CRC(e2a54c47) SHA1(1e08f8e3d0ae0efb2d178ab11ec2bddaeb6d7478) ) /* near DB1 - DB4 */
ROM_END


ROM_START( dambustra )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "db8a.pr8",   0x4000, 0x1000, CRC(fd041ff4) SHA1(8d27da7bf0c655633711b960cbc23950c8a371ae) )
	ROM_LOAD( "db6.pr6",    0x5000, 0x1000, CRC(56d301a9) SHA1(a0839767af822ab1b8df1a7d0767e72b494974c6) ) /* This single rom had a yellow label, while all the rest were white */
	ROM_LOAD( "db7a.pr7",   0x6000, 0x1000, CRC(675b1f5e) SHA1(6a386212a640fb467b6956a4dc5a68476af1cf97) )
	ROM_LOAD( "db5a.pr5",   0x7000, 0x1000, CRC(75659ecc) SHA1(b61254fb12f3999607abd88d1cc649dcfbf0384c) )

	ROM_REGION( 0x10000,"user1",0)
	ROM_LOAD( "db11.pr11",   0x0000, 0x1000, CRC(427bd3fb) SHA1(cdbaef4040fa2e0598a086e320d51ecb26a591dd) )
	ROM_LOAD( "db9a.pr9",    0x1000, 0x1000, CRC(57164563) SHA1(8471d0660f39511d0afa3cdd63a1e84b0ea80fd0) )
	ROM_LOAD( "db10a.pr10",  0x2000, 0x1000, CRC(075b9c5e) SHA1(ff6ce873897004c0e796813725e260df85a520f9) ) /* Had a hand drawn "PLUS" symbol on the rom label */
	ROM_LOAD( "db12a.pr12",  0x3000, 0x1000, CRC(ed01a68b) SHA1(9dd37c2a25865717a7acdd7e2a3bef26a4cef3d9) )

	ROM_REGION( 0x4000, "gfx1", 0 )
	ROM_LOAD( "db3a.rom",   0x0000, 0x1000, CRC(2347e26e) SHA1(85909c6cca7a5249d28668291a28e24d5b90293b) ) /* All roms, except for 6 & 11, had a hand drawn "A" on the label */
	ROM_LOAD( "db1a.pr1",   0x1000, 0x1000, CRC(4cb964cd) SHA1(1c90b14deb201a64b8ed4378b022e9e4574aed94) )
	ROM_LOAD( "db4a.pr3",   0x2000, 0x1000, CRC(d9d2df33) SHA1(97057fe33c146898755b556558ff707b9f4551ec) )
	ROM_LOAD( "db2a.pr2",   0x3000, 0x1000, CRC(0a0a6af5) SHA1(ecd2a6696ce9154f030c830ccb45690787881a73) )

	ROM_REGION( 0x0020, "proms", 0 )
	ROM_LOAD( "mi-7603-5.ic4", 0x0000, 0x0020, CRC(f131f92f) SHA1(8b0f623d2ea09b5612dde0f330c5e473a7d72e06) ) /* near DB5 - DB8 */

	ROM_REGION( 0x0020, "unk_prom", 0 ) //timing?
	ROM_LOAD( "mi-7603-5.ic3", 0x0000, 0x0020, CRC(e2a54c47) SHA1(1e08f8e3d0ae0efb2d178ab11ec2bddaeb6d7478) ) /* near DB1 - DB4 */
ROM_END


ROM_START( dambustruk )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "db8.pr8",    0x4000, 0x1000, CRC(fd041ff4) SHA1(8d27da7bf0c655633711b960cbc23950c8a371ae) )
	ROM_LOAD( "db6p.bin",   0x5000, 0x1000, CRC(35dcee01) SHA1(2c23c727d9b38322a6d0548dfe6a2a254f3530af) )
	ROM_LOAD( "db7.pr7",    0x6000, 0x1000, CRC(675b1f5e) SHA1(6a386212a640fb467b6956a4dc5a68476af1cf97) )
	ROM_LOAD( "db5.pr5",    0x7000, 0x1000, CRC(75659ecc) SHA1(b61254fb12f3999607abd88d1cc649dcfbf0384c) )

	ROM_REGION( 0x10000,"user1",0)
	ROM_LOAD( "db11.bin",   0x0000, 0x1000, CRC(9e6b34fe) SHA1(5cf47f5a5280ac53490240df220edf6178e87f4f) )
	ROM_LOAD( "db9.pr9",    0x1000, 0x1000, CRC(57164563) SHA1(8471d0660f39511d0afa3cdd63a1e84b0ea80fd0) )
	ROM_LOAD( "db10p.bin",  0x2000, 0x1000, CRC(c129c57b) SHA1(c25abd7ee97b71941d9fa6acd0d92c116f1ff408) )
	ROM_LOAD( "db12.bin",   0x3000, 0x1000, CRC(ea4c65f5) SHA1(cb761e0543cacd6b437c6e88615f97df83245a34) )

	ROM_REGION( 0x4000, "gfx1", 0 )
	ROM_LOAD( "db3.pr3",   0x0000, 0x1000, CRC(9e9a9710) SHA1(a9f67a05a2882b9f6f3378cc73e90539de4b8ca4) )
	ROM_LOAD( "db1ap.pr1", 0x1000, 0x1000, CRC(4cb964cd) SHA1(1c90b14deb201a64b8ed4378b022e9e4574aed94) )
	ROM_LOAD( "db4.pr3",   0x2000, 0x1000, CRC(d9d2df33) SHA1(97057fe33c146898755b556558ff707b9f4551ec) )
	ROM_LOAD( "db2.pr2",   0x3000, 0x1000, CRC(0a0a6af5) SHA1(ecd2a6696ce9154f030c830ccb45690787881a73) )

	ROM_REGION( 0x0020, "proms", 0 )
	ROM_LOAD( "mi-7603-5.ic4", 0x0000, 0x0020, CRC(f131f92f) SHA1(8b0f623d2ea09b5612dde0f330c5e473a7d72e06) ) /* near DB5 - DB8 */

	ROM_REGION( 0x0020, "unk_prom", 0 ) //timing?
	ROM_LOAD( "mi-7603-5.ic3", 0x0000, 0x0020, CRC(e2a54c47) SHA1(1e08f8e3d0ae0efb2d178ab11ec2bddaeb6d7478) ) /* near DB1 - DB4 */
ROM_END


GAME( 1981, dambustr,   0,        dambustr, dambustr,   dambustr_state, dambustr, ROT90, "South West Research", "Dambusters (US, set 1)", 0 )
GAME( 1981, dambustra,  dambustr, dambustr, dambustr,   dambustr_state, dambustr, ROT90, "South West Research", "Dambusters (US, set 2)", 0 )
GAME( 1981, dambustruk, dambustr, dambustr, dambustruk, dambustr_state, dambustr, ROT90, "South West Research", "Dambusters (UK)",        0 )
