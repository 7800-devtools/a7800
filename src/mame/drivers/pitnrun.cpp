// license:BSD-3-Clause
// copyright-holders:Tomasz Slanina, Pierpaolo Prazzoli
/****************************************************
   Pit&Run - Taito 1984

 driver by  Tomasz Slanina and  Pierpaolo Prazzoli

 hardware is very similar to suprridr.cpp, thepit.cpp, timelimt.cpp

TODO:

 - analog sound
   writes to $a8xx triggering analog sound :
    $a800 - drivers are getting into the cars
    $a801 - collisions
    $a802 - same as above
    $a803 - slide on water
    $a804 - accelerate
    $a807 - analog sound reset


-----------------------------------------------------
$8101 B - course
$8102 B - trial
$8492 B - fuel
$84f6 B - lap
$84c1 W - time
-----------------------------------------------------

N4200374A

K1000232A
            A11_17     2128  PR9
           (68705P5)         PR10
                             PR11
     SW1                     PR12
                        Z80
                                      clr.1
                        PR8           clr.2

               PR6                    clr.3
               PR7
                              2114
                              2114

K1000231A

    2114 2114
    PR13
                Z80

          8910 8910
   5MHz

K1000233A

  2125      2125        2128
  2125      2125        2128
  2125      2125        PR4
  2125      2125        PR5
  2125      2125

                             2114
     PR1                     2114
     PR2
     PR3
*/

#include "emu.h"
#include "includes/pitnrun.h"

#include "cpu/m6805/m68705.h"
#include "cpu/z80/z80.h"

#include "machine/gen_latch.h"
#include "machine/watchdog.h"

#include "sound/ay8910.h"

#include "screen.h"
#include "speaker.h"


INTERRUPT_GEN_MEMBER(pitnrun_state::nmi_source)
{
	if (m_nmi) device.execute().set_input_line(INPUT_LINE_NMI, PULSE_LINE);
}

WRITE8_MEMBER(pitnrun_state::nmi_enable_w)
{
	m_nmi = data & 1;
}

WRITE8_MEMBER(pitnrun_state::hflip_w)
{
	flip_screen_x_set(data);
}

WRITE8_MEMBER(pitnrun_state::vflip_w)
{
	flip_screen_y_set(data);
}

static ADDRESS_MAP_START( pitnrun_map, AS_PROGRAM, 8, pitnrun_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0x87ff) AM_RAM
	AM_RANGE(0x8800, 0x8fff) AM_RAM_WRITE(videoram_w) AM_SHARE("videoram")
	AM_RANGE(0x9000, 0x9fff) AM_RAM_WRITE(videoram2_w) AM_SHARE("videoram2")
	AM_RANGE(0xa000, 0xa0ff) AM_RAM AM_SHARE("spriteram")
	AM_RANGE(0xa800, 0xa800) AM_READ_PORT("SYSTEM")
	AM_RANGE(0xa800, 0xa807) AM_WRITENOP /* Analog Sound */
	AM_RANGE(0xb000, 0xb000) AM_READ_PORT("DSW") AM_WRITE(nmi_enable_w)
	AM_RANGE(0xb001, 0xb001) AM_WRITE(color_select_w)
	AM_RANGE(0xb004, 0xb004) AM_WRITENOP/* COLOR SEL 2 - not used ?*/
	AM_RANGE(0xb005, 0xb005) AM_WRITE(char_bank_select)
	AM_RANGE(0xb006, 0xb006) AM_WRITE(hflip_w)
	AM_RANGE(0xb007, 0xb007) AM_WRITE(vflip_w)
	AM_RANGE(0xb800, 0xb800) AM_READ_PORT("INPUTS") AM_DEVWRITE("soundlatch", generic_latch_8_device, write)
	AM_RANGE(0xc800, 0xc801) AM_WRITE(scroll_w)
	AM_RANGE(0xc802, 0xc802) AM_WRITE(scroll_y_w)
	//AM_RANGE(0xc804, 0xc804) AM_WRITE(mcu_data_w)
	AM_RANGE(0xc805, 0xc805) AM_WRITE(h_heed_w)
	AM_RANGE(0xc806, 0xc806) AM_WRITE(v_heed_w)
	AM_RANGE(0xc807, 0xc807) AM_WRITE(ha_w)
	//AM_RANGE(0xd000, 0xd000) AM_READ(mcu_data_r)
	//AM_RANGE(0xd800, 0xd800) AM_READ(mcu_status_r)
	AM_RANGE(0xf000, 0xf000) AM_DEVREAD("watchdog", watchdog_timer_device, reset_r)
ADDRESS_MAP_END

static ADDRESS_MAP_START( pitnrun_map_mcu, AS_PROGRAM, 8, pitnrun_state )
	AM_IMPORT_FROM(pitnrun_map)
	AM_RANGE(0xc804, 0xc804) AM_WRITE(mcu_data_w)
	AM_RANGE(0xd000, 0xd000) AM_READ(mcu_data_r)
	AM_RANGE(0xd800, 0xd800) AM_READ(mcu_status_r)
ADDRESS_MAP_END

static ADDRESS_MAP_START( pitnrun_sound_map, AS_PROGRAM, 8, pitnrun_state )
	AM_RANGE(0x0000, 0x2fff) AM_ROM
	AM_RANGE(0x3800, 0x3bff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( pitnrun_sound_io_map, AS_IO, 8, pitnrun_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x00) AM_DEVWRITE("soundlatch", generic_latch_8_device, clear_w)
	AM_RANGE(0x8c, 0x8d) AM_DEVWRITE("ay2", ay8910_device, address_data_w)
	AM_RANGE(0x8e, 0x8f) AM_DEVWRITE("ay1", ay8910_device, address_data_w)
	AM_RANGE(0x8f, 0x8f) AM_DEVREAD("ay1", ay8910_device, data_r)
	AM_RANGE(0x90, 0x96) AM_WRITENOP
	AM_RANGE(0x97, 0x97) AM_WRITENOP
	AM_RANGE(0x98, 0x98) AM_WRITENOP
ADDRESS_MAP_END


static INPUT_PORTS_START( pitnrun )
	PORT_START("SYSTEM")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_COIN1  )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_SERVICE1 )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_START1 )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_START2 )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNKNOWN )

	PORT_START("INPUTS")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT ) PORT_4WAY
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_4WAY
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN ) PORT_4WAY
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP ) PORT_4WAY
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNKNOWN )

	PORT_START("DSW")
	PORT_DIPNAME( 0x07, 0x01, DEF_STR( Coinage ) )  PORT_DIPLOCATION("DSW:1,2,3")
	PORT_DIPSETTING(    0x00, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x01, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x02, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x03, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(    0x05, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(    0x06, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(    0x07, DEF_STR( 1C_7C ) )
	PORT_DIPUNUSED_DIPLOC( 0x08, 0x00, "DSW:4" )
	PORT_DIPUNUSED_DIPLOC( 0x10, 0x00, "DSW:5" )
	PORT_DIPNAME( 0x20, 0x00, "Gasoline Count" )    PORT_DIPLOCATION("DSW:6")
	PORT_DIPSETTING(    0x00, "10 Up or 10 Down" )
	PORT_DIPSETTING(    0x20, "20 Up or 20 Down" )
	PORT_DIPNAME( 0x40, 0x00, DEF_STR( Cabinet ) )  PORT_DIPLOCATION("DSW:7")
	PORT_DIPSETTING(    0x00, DEF_STR( Upright ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Cocktail ) )
	PORT_DIPNAME( 0x80, 0x00, "No Hit (Cheat)")     PORT_DIPLOCATION("DSW:8")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x80, DEF_STR( On ) )       // also enables bootup test
INPUT_PORTS_END

static INPUT_PORTS_START( jumpkun )
	PORT_START("SYSTEM")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_COIN1  )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_SERVICE1 )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_START1 )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_START2 )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNKNOWN )

	PORT_START("INPUTS")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT ) PORT_4WAY
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_4WAY
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN ) PORT_4WAY
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP ) PORT_4WAY
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_BUTTON2 )
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_UNKNOWN )

	PORT_START("DSW")
	PORT_DIPNAME( 0x07, 0x01, DEF_STR( Coinage ) ) PORT_DIPLOCATION("DSW:1,2,3")
	PORT_DIPSETTING(    0x00, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x01, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x02, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x03, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(    0x05, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(    0x06, DEF_STR( 1C_6C ) )
	PORT_DIPSETTING(    0x07, DEF_STR( 1C_7C ) )
	PORT_DIPNAME( 0x08, 0x00, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW:4")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, DEF_STR( Lives ) ) PORT_DIPLOCATION("DSW:5")
	PORT_DIPSETTING(    0x00, "2" )
	PORT_DIPSETTING(    0x10, "4" )
	PORT_DIPNAME( 0x20, 0x00, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW:6")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x20, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x00, DEF_STR( Cabinet ) )  PORT_DIPLOCATION("DSW:7")
	PORT_DIPSETTING(    0x00, DEF_STR( Upright ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Cocktail ) )
	PORT_DIPNAME( 0x80, 0x00, "Invincibility (Cheat)") PORT_DIPLOCATION("DSW:8")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x80, DEF_STR( On ) )
INPUT_PORTS_END


static const gfx_layout spritelayout =
{
	16,16,
	RGN_FRAC(1,3),
	3,
	{ 0,RGN_FRAC(1,3), RGN_FRAC(2,3) },
	{ 0, 1, 2, 3, 4, 5, 6, 7,
		8*8+0, 8*8+1, 8*8+2, 8*8+3, 8*8+4, 8*8+5, 8*8+6, 8*8+7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8,
		16*8, 17*8, 18*8, 19*8, 20*8, 21*8, 22*8, 23*8 },
	32*8
};

static const gfx_layout charlayout =
{
	8,8,
	RGN_FRAC(1,2),
	4,
	{RGN_FRAC(1,2),RGN_FRAC(1,2)+4,0,4},
	{ STEP4(0,1), STEP4(8,1) },
	{ STEP8(0,8*2) },
	8*8*2
};

static GFXDECODE_START( pitnrun )
	GFXDECODE_ENTRY( "gfx3", 0, charlayout,   64, 2 )
	GFXDECODE_ENTRY( "gfx2", 0, charlayout,   32, 2 )
	GFXDECODE_ENTRY( "gfx1", 0, spritelayout,  0, 4 )
GFXDECODE_END

static MACHINE_CONFIG_START( pitnrun )
	MCFG_CPU_ADD("maincpu", Z80,XTAL_18_432MHz/6)       /* verified on pcb */
	MCFG_CPU_PROGRAM_MAP(pitnrun_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", pitnrun_state,  nmi_source)

	MCFG_CPU_ADD("audiocpu", Z80, XTAL_5MHz/2)          /* verified on pcb */
	MCFG_CPU_PROGRAM_MAP(pitnrun_sound_map)
	MCFG_CPU_IO_MAP(pitnrun_sound_io_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", pitnrun_state,  irq0_line_hold)

	MCFG_WATCHDOG_ADD("watchdog")

	MCFG_QUANTUM_TIME(attotime::from_hz(6000))

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(256, 256)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(pitnrun_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", pitnrun)
	MCFG_PALETTE_ADD("palette", 32*3)
	MCFG_PALETTE_INIT_OWNER(pitnrun_state, pitnrun)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")

	MCFG_SOUND_ADD("ay1", AY8910, XTAL_18_432MHz/12)    /* verified on pcb */
	MCFG_AY8910_PORT_A_READ_CB(DEVREAD8("soundlatch", generic_latch_8_device, read))
	MCFG_AY8910_PORT_B_READ_CB(DEVREAD8("soundlatch", generic_latch_8_device, read))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)

	MCFG_SOUND_ADD("ay2", AY8910, XTAL_18_432MHz/12)    /* verified on pcb */
	MCFG_AY8910_PORT_A_READ_CB(DEVREAD8("soundlatch", generic_latch_8_device, read))
	MCFG_AY8910_PORT_B_READ_CB(DEVREAD8("soundlatch", generic_latch_8_device, read))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( pitnrun_mcu, pitnrun )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(pitnrun_map_mcu)

	MCFG_CPU_ADD("mcu", M68705P5, XTAL_18_432MHz/6)     /* verified on pcb */
	MCFG_M68705_PORTA_R_CB(READ8(pitnrun_state, m68705_portA_r))
	MCFG_M68705_PORTB_R_CB(READ8(pitnrun_state, m68705_portB_r))
	MCFG_M68705_PORTC_R_CB(READ8(pitnrun_state, m68705_portC_r))
	MCFG_M68705_PORTA_W_CB(WRITE8(pitnrun_state, m68705_portA_w))
	MCFG_M68705_PORTB_W_CB(WRITE8(pitnrun_state, m68705_portB_w))
MACHINE_CONFIG_END

ROM_START( pitnrun )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "pr12", 0x0000, 0x2000, CRC(587a7b85) SHA1(f200ff9b706e13760a23e0187c6bffe496af0087) )
	ROM_LOAD( "pr11", 0x2000, 0x2000, CRC(270cd6dd) SHA1(ad42562e18aa30319fc55c201e5507e8734a5b4d) )
	ROM_LOAD( "pr10", 0x4000, 0x2000, CRC(65d92d89) SHA1(4030ccdb4d84e69c256e95431ee5a18cffeae5c0) )
	ROM_LOAD( "pr9",  0x6000, 0x2000, CRC(3155286d) SHA1(45af8cb81d70f2e30b52bbc7abd9f8d15231735f) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "pr13", 0x0000, 0x1000, CRC(fc8fd05c) SHA1(f40cc9c6fff6bda8411f4d638a0f5c5915aa3746) )

	ROM_REGION( 0x0800, "mcu", 0 )
	ROM_LOAD( "a11_17.3a", 0x0000, 0x0800, CRC(e7d5d6e1) SHA1(c1131d6fcc36926e287be26090a3c89f22feaa35) )

	ROM_REGION( 0x6000, "gfx1", 0 )
	ROM_LOAD( "pr1", 0x0000, 0x2000, CRC(c3b3131e) SHA1(ed0463e7eef452d7fbdcb031f9477825e9780943) )
	ROM_LOAD( "pr2", 0x2000, 0x2000, CRC(2fa1682a) SHA1(9daefb525fd69f0d9a45ff27e89865545e177a5a) )
	ROM_LOAD( "pr3", 0x4000, 0x2000, CRC(e678fe39) SHA1(134e36fd30bf3cf5884732f3455ca4d9dab6b665) )

	ROM_REGION( 0x4000, "gfx2", 0 )
	ROM_LOAD( "pr4", 0x0000, 0x2000, CRC(fbae3504) SHA1(ce799dfd653462c0814e7530f3f8a686ab0ad7f4) )
	ROM_LOAD( "pr5", 0x2000, 0x2000, CRC(c9177180) SHA1(98c8f8f586b78b88dba254bd662642ee27f9b131) )

	ROM_REGION( 0x2000, "gfx3", 0 )
	ROM_LOAD( "pr6", 0x0000, 0x1000, CRC(c53cb897) SHA1(81a73e6031b52fa45ec507ff4264b14474ef42a2) )
	ROM_LOAD( "pr7", 0x1000, 0x1000, CRC(7cdf9a55) SHA1(404dface7e09186e486945981e39063929599efc) )

	ROM_REGION( 0x2000, "spot", 0 )
	ROM_LOAD( "pr8", 0x0000, 0x2000, CRC(8e346d10) SHA1(1362ce4362c2d28c48fbd8a33da0cec5ef8e321f) )

	ROM_REGION( 0x0060, "proms", 0 )
	ROM_LOAD( "clr.1",  0x0000, 0x0020, CRC(643012f4) SHA1(4a0c9766b9da456e39ce379ad62d695bf82413b0) )
	ROM_LOAD( "clr.2",  0x0020, 0x0020, CRC(50705f02) SHA1(a3d348678fd66f37c7a0d29af88f40740918b8d3) )
	ROM_LOAD( "clr.3",  0x0040, 0x0020, CRC(25e70e5e) SHA1(fdb9c69e9568a725dd0e3ac25835270fb4f49280) )
ROM_END

ROM_START( pitnruna )
	ROM_REGION( 0x010000, "maincpu", 0 )
	ROM_LOAD( "pr_12-1.5d", 0x0000, 0x2000, CRC(2539aec3) SHA1(5ee87cf2379a6b6218f0c1f79374edafe5413616) )
	ROM_LOAD( "pr_11-1.5c", 0x2000, 0x2000, CRC(818a49f8) SHA1(0a4c77055529967595984277f11dc1cd1eec4dae) )
	ROM_LOAD( "pr_10-1.5b", 0x4000, 0x2000, CRC(69b3a864) SHA1(3d29e1f71f1a94650839696c3070d5739360bee0) )
	ROM_LOAD( "pr_9-1.5a",  0x6000, 0x2000, CRC(ba0c4093) SHA1(0273e4bd09b9eebff490fdac27e6ae9b54bb3cd9) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "pr-13", 0x0000, 0x1000, CRC(32a18d3b) SHA1(fcff1c13183b64ede0865dd04eee5182029bebdf) )

	ROM_REGION( 0x0800, "mcu", 0 )
	ROM_LOAD( "a11_17.3a", 0x0000, 0x0800, CRC(e7d5d6e1) SHA1(c1131d6fcc36926e287be26090a3c89f22feaa35) )

	ROM_REGION( 0x06000, "gfx1", 0 )
	ROM_LOAD( "pr-1.1k", 0x0000, 0x2000, CRC(c3b3131e) SHA1(ed0463e7eef452d7fbdcb031f9477825e9780943) )
	ROM_LOAD( "pr-2.1m", 0x2000, 0x2000, CRC(2fa1682a) SHA1(9daefb525fd69f0d9a45ff27e89865545e177a5a) )
	ROM_LOAD( "pr-3.1n", 0x4000, 0x2000, CRC(e678fe39) SHA1(134e36fd30bf3cf5884732f3455ca4d9dab6b665) )

	ROM_REGION( 0x4000, "gfx2", 0 )
	ROM_LOAD( "pr-4.6d", 0x0000, 0x2000, CRC(fbae3504) SHA1(ce799dfd653462c0814e7530f3f8a686ab0ad7f4) )
	ROM_LOAD( "pr-5.6f", 0x2000, 0x2000, CRC(c9177180) SHA1(98c8f8f586b78b88dba254bd662642ee27f9b131) )

	ROM_REGION( 0x2000, "gfx3", 0 )
	ROM_LOAD( "pr-6.3m", 0x0000, 0x1000, CRC(c53cb897) SHA1(81a73e6031b52fa45ec507ff4264b14474ef42a2) )
	ROM_LOAD( "pr-7.3p", 0x1000, 0x1000, CRC(7cdf9a55) SHA1(404dface7e09186e486945981e39063929599efc) )

	ROM_REGION( 0x2000, "spot", 0 )
	ROM_LOAD( "pr-8.4j", 0x0000, 0x2000, CRC(8e346d10) SHA1(1362ce4362c2d28c48fbd8a33da0cec5ef8e321f) )

	ROM_REGION( 0x0060, "proms", 0 )
	ROM_LOAD( "clr.1",  0x0000, 0x0020, CRC(643012f4) SHA1(4a0c9766b9da456e39ce379ad62d695bf82413b0) )
	ROM_LOAD( "clr.2",  0x0020, 0x0020, CRC(50705f02) SHA1(a3d348678fd66f37c7a0d29af88f40740918b8d3) )
	ROM_LOAD( "clr.3",  0x0040, 0x0020, CRC(25e70e5e) SHA1(fdb9c69e9568a725dd0e3ac25835270fb4f49280) )
ROM_END

ROM_START( jumpkun )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "PR1.5D.2764", 0x00000, 0x02000, CRC(b0eabe9f) SHA1(e662f3946efe72b0bbf6c6934201163f765bb7aa) )
	ROM_LOAD( "PR2.5C.2764", 0x02000, 0x02000, CRC(d9240413) SHA1(f4d0491e125f1fe435b200b38fa125889784af0a) )
	ROM_LOAD( "PR3.5B.2764", 0x04000, 0x02000, CRC(105e3fec) SHA1(06ea902e6647fc37a603146324e3d0a067e1f649) )
	ROM_LOAD( "PR4.5A.2764", 0x06000, 0x02000, CRC(3a17ca88) SHA1(00516798d546098831e75547664c8fdaa2bbf050) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "SND1.2732", 0x00000, 0x01000, CRC(1290f316) SHA1(13e393860c1f7d1f97343b9f936c60adb7641efc) )
	ROM_LOAD( "SND2.2732", 0x01000, 0x01000, CRC(ec5e4489) SHA1(fc94fe798a1925e8e3dd15161648e9a960969fc4) )

	ROM_REGION( 0x0800, "mcu", ROMREGION_ERASE00 )
	// not populated

	ROM_REGION( 0x6000, "gfx1", 0 )
	ROM_LOAD( "OBJ1.1K.2764", 0x00000, 0x02000, CRC(8929abfd) SHA1(978994af5816c20a8cd520263d04d1cc1e4df576) )
	ROM_LOAD( "OBJ2.1M.2764", 0x02000, 0x02000, CRC(c7bf5819) SHA1(15d8e1dd1c0911785237e9063a75a42a2dc1bd50) )
	ROM_LOAD( "OBJ3.1N.2764", 0x04000, 0x02000, CRC(5eeec986) SHA1(e58a0b98b90a1dd3971ed305100337aa2e5ec450) )

	ROM_REGION( 0x4000, "gfx2", 0 )
	ROM_LOAD( "CHR1.6D.2764", 0x00000, 0x02000, CRC(3c93d4ee) SHA1(003121c49bccbb95efb137e6d92d26eea1957fbd)  )
	ROM_LOAD( "CHR2.6F.2764", 0x02000, 0x02000, CRC(154fad33) SHA1(7eddc794bd547053f185bb79a8220907bab13d85)  )

	ROM_REGION( 0x2000, "gfx3", 0 )
	ROM_LOAD( "BSC2.3M.2764", 0x00000, 0x01000, CRC(25445f17) SHA1(b1ada95d8f02623bb4a4562d2d278a882414e57e) )
	ROM_LOAD( "BSC1.3P.2764", 0x01000, 0x01000, CRC(39ca2c37) SHA1(b8c71f443a0faf54df03ac5aca46ddd34c42d3a0) )

	ROM_REGION( 0x2000, "spot", ROMREGION_ERASE00 )
	// not populated

	ROM_REGION( 0x0060, "proms", 0 )
	ROM_LOAD( "8H.82S123.bin", 0x0000, 0x0020, CRC(e54a6fe6) SHA1(c51da2cbf54b7abff7b0cdf0d6846c375b71edcd) )
	ROM_LOAD( "8L.82S123.bin", 0x0020, 0x0020, CRC(624830d5) SHA1(793b2f770ccef6c03bf3ecbf4debcc0531f62da1) )
	ROM_LOAD( "8J.82S123.bin", 0x0040, 0x0020, CRC(223a6990) SHA1(06e16de037c2c7ad5733390859fa7ec1ab1e2f69) )
ROM_END

GAME( 1984, pitnrun,  0,       pitnrun_mcu, pitnrun, pitnrun_state, 0, ROT90, "Taito Corporation", "Pit & Run - F-1 Race (set 1)", MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE )
GAME( 1984, pitnruna, pitnrun, pitnrun_mcu, pitnrun, pitnrun_state, 0, ROT90, "Taito Corporation", "Pit & Run - F-1 Race (set 2)", MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE )
GAME( 1984, jumpkun,  0,       pitnrun,     jumpkun, pitnrun_state, 0, ROT90, "Kaneko",            "Jump Kun (prototype)",         MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE ) // no copyright message
