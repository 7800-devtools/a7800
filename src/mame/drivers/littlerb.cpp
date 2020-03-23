// license:BSD-3-Clause
// copyright-holders:Pierpaolo Prazzoli, David Haywood
/* Little Robin */

/* driver by
Pierpaolo Prazzoli
David Haywood
*/


/*

Notes:

How big are the actual framebuffers?  Are both also double buffered?

Sound pitch is directly correlated with irqs, scanline timings and pixel clock,
so it's surely not 100% correct. Sound sample playbacks looks fine at current time tho.

------



Dip sw.1
--------
             | Coin 1 | Coin 2  |
              1  2  3   4  5  6   7  8   Coin  Play
---------------------------------------------------
 Coins        -  -  -   -  -  -           1     4
              +  -  -   +  -  -           1     3
              -  +  -   -  +  -           1     2
              +  +  -   +  +  -           1     1
              -  -  +   -  -  +           2     1
              +  -  +   +  -  +           3     1
              -  +  +   -  +  +           4     1
              +  +  +   +  +  +           5     1
 Player                           -  -    2
                                  +  -    3
                                  -  +    4
                                  +  +    5


Dip sw.2
--------          1  2  3  4  5  6  7  8
-----------------------------------------------------------
 Demo Sound       -                        Yes
                  +                        No
 Mode                -                     Test Mode
                     +                     Game Mode
 Difficulty             -  -  -            0 (Easy)
                        +  -  -            1
                        -  +  -            2 (Normal)
                        +  +  -            3
                        -  -  +            4
                        +  -  +            5
                        -  +  +            6
                        +  +  +            7 (Hardest)
 Bonus                           -  -      Every 150000
                                 +  -      Every 200000
                                 -  +      Every 300000
                                 +  +      No Bonus


*/

#include "emu.h"
#include "cpu/m68000/m68000.h"
#include "machine/inder_vid.h"
#include "sound/dac.h"
#include "sound/volt_reg.h"
#include "speaker.h"

#define littlerb_printf logerror
#define littlerb_alt_printf logerror

class littlerb_state : public driver_device
{
public:
	littlerb_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_maincpu(*this, "maincpu"),
			m_indervid(*this, "inder_vid"),
			m_ldac(*this, "ldac"),
			m_rdac(*this, "rdac"),
			m_soundframe(0)
	{
	}

	required_device<cpu_device> m_maincpu;
	required_device<inder_vid_device> m_indervid;

	required_device<dac_byte_interface> m_ldac;
	required_device<dac_byte_interface> m_rdac;
	uint8_t m_sound_index_l,m_sound_index_r;
	uint16_t m_sound_pointer_l,m_sound_pointer_r;
	int m_soundframe;

	DECLARE_CUSTOM_INPUT_MEMBER(littlerb_frame_step_r);
	DECLARE_WRITE16_MEMBER(littlerb_l_sound_w);
	DECLARE_WRITE16_MEMBER(littlerb_r_sound_w);
	uint8_t sound_data_shift();

	TIMER_DEVICE_CALLBACK_MEMBER(littlerb_sound_step_cb);
	TIMER_DEVICE_CALLBACK_MEMBER(littlerb_sound_cb);

	DECLARE_DRIVER_INIT(littlerb);
};


/* could be slightly different (timing wise, directly related to the irqs), but certainly they smoked some bad pot for this messy way ... */
uint8_t littlerb_state::sound_data_shift()
{
	return ((m_soundframe % 16) == 0) ? 8 : 0;
}

/* l is SFX, r is BGM (they doesn't seem to share the same data ROM) */
WRITE16_MEMBER(littlerb_state::littlerb_l_sound_w)
{
	m_sound_index_l = (data >> sound_data_shift()) & 0xff;
	m_sound_pointer_l = 0;
	//popmessage("%04x %04x",m_sound_index_l,m_sound_index_r);
}

WRITE16_MEMBER(littlerb_state::littlerb_r_sound_w)
{
	m_sound_index_r = (data >> sound_data_shift()) & 0xff;
	m_sound_pointer_r = 0;
	//popmessage("%04x %04x",m_sound_index_l,m_sound_index_r);
}

static ADDRESS_MAP_START( littlerb_main, AS_PROGRAM, 16, littlerb_state )
	AM_RANGE(0x000008, 0x000017) AM_WRITENOP
	AM_RANGE(0x000020, 0x00002f) AM_WRITENOP
	AM_RANGE(0x000070, 0x000073) AM_WRITENOP
	AM_RANGE(0x060004, 0x060007) AM_WRITENOP
	AM_RANGE(0x000000, 0x0fffff) AM_ROM
	AM_RANGE(0x200000, 0x203fff) AM_RAM // main ram?

	AM_RANGE(0x700000, 0x700007) AM_DEVREADWRITE("inder_vid:tms", tms34010_device, host_r, host_w)

	AM_RANGE(0x740000, 0x740001) AM_WRITE(littlerb_l_sound_w)
	AM_RANGE(0x760000, 0x760001) AM_WRITE(littlerb_r_sound_w)
	AM_RANGE(0x780000, 0x780001) AM_WRITENOP // generic outputs
	AM_RANGE(0x7c0000, 0x7c0001) AM_READ_PORT("DSW")
	AM_RANGE(0x7e0000, 0x7e0001) AM_READ_PORT("P1")
	AM_RANGE(0x7e0002, 0x7e0003) AM_READ_PORT("P2")
ADDRESS_MAP_END

/* guess according to DASM code and checking the gameplay speed, could be different */
CUSTOM_INPUT_MEMBER(littlerb_state::littlerb_frame_step_r)
{
	uint32_t ret = m_soundframe;

	return (ret) & 7;
}

static INPUT_PORTS_START( littlerb )
	PORT_START("DSW")   /* 16bit */
	PORT_DIPNAME( 0x0003, 0x0001, DEF_STR( Lives ) )    PORT_DIPLOCATION("SW1:8,7")
	PORT_DIPSETTING(      0x0003, "2" )
	PORT_DIPSETTING(      0x0001, "3" )
	PORT_DIPSETTING(      0x0002, "4" )
	PORT_DIPSETTING(      0x0000, "5" )
	PORT_DIPNAME( 0x001c, 0x0004, DEF_STR( Coin_B ) )   PORT_DIPLOCATION("SW1:6,5,4")
	PORT_DIPSETTING(      0x0000, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(      0x0010, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(      0x0008, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(      0x0018, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(      0x0004, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(      0x0014, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(      0x000c, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(      0x001c, DEF_STR( 1C_4C ) )
	PORT_DIPNAME( 0x00e0, 0x0020, DEF_STR( Coin_A ) )   PORT_DIPLOCATION("SW1:3,2,1")
	PORT_DIPSETTING(      0x0000, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(      0x0080, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(      0x0040, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(      0x00c0, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(      0x0020, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(      0x00a0, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(      0x0060, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(      0x00e0, DEF_STR( 1C_4C ) )
	PORT_DIPNAME( 0x0100, 0x0100, DEF_STR( Unknown ) )  PORT_DIPLOCATION("SW2:8")
	PORT_DIPSETTING(      0x0100, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0600, 0x0600, DEF_STR( Bonus_Life ) )   PORT_DIPLOCATION("SW2:7,6")
	PORT_DIPSETTING(      0x0600, "Every 150,000" )
	PORT_DIPSETTING(      0x0200, "Every 200,000" )
	PORT_DIPSETTING(      0x0400, "Every 300,000" )
	PORT_DIPSETTING(      0x0000, DEF_STR( None ) )
	PORT_DIPNAME( 0x3800, 0x2800, DEF_STR( Difficulty ) )   PORT_DIPLOCATION("SW2:5,4,3")
	PORT_DIPSETTING(      0x3800, DEF_STR( Easy ) )
	PORT_DIPSETTING(      0x1800, DEF_STR( Medium_Easy ) )
	PORT_DIPSETTING(      0x2800, DEF_STR( Normal ) )
	PORT_DIPSETTING(      0x0800, DEF_STR( Medium_Hard ) )
	PORT_DIPSETTING(      0x3000, DEF_STR( Hard ) )
	PORT_DIPSETTING(      0x1000, DEF_STR( Harder ) )
	PORT_DIPSETTING(      0x2000, DEF_STR( Very_Hard ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( Hardest ) )
	PORT_SERVICE_DIPLOC(  0x4000, IP_ACTIVE_LOW, "SW2:2" )
	PORT_DIPNAME( 0x8000, 0x8000, DEF_STR( Demo_Sounds ) )  PORT_DIPLOCATION("SW2:1")
	PORT_DIPSETTING(      0x0000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x8000, DEF_STR( On ) )

	PORT_START("P1")    /* 16bit */
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(1)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(1)
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_DIPNAME( 0x1000, 0x1000, "???"  )
	PORT_DIPSETTING(      0x1000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_BIT( 0xe000, IP_ACTIVE_HIGH, IPT_SPECIAL ) PORT_CUSTOM_MEMBER(DEVICE_SELF, littlerb_state,littlerb_frame_step_r, nullptr)

	PORT_START("P2")    /* 16bit */
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(2)
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0xff00, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END


TIMER_DEVICE_CALLBACK_MEMBER(littlerb_state::littlerb_sound_cb)
{
	uint8_t *sample_rom = memregion("samples")->base();

	m_ldac->write(sample_rom[m_sound_pointer_l | (m_sound_index_l << 10) | 0x40000]);
	m_rdac->write(sample_rom[m_sound_pointer_r | (m_sound_index_r << 10) | 0x00000]);
	m_sound_pointer_l++;
	m_sound_pointer_l&=0x3ff;
	m_sound_pointer_r++;
	m_sound_pointer_r&=0x3ff;
}

TIMER_DEVICE_CALLBACK_MEMBER(littlerb_state::littlerb_sound_step_cb)
{
	m_soundframe++;
}

static MACHINE_CONFIG_START( littlerb )
	MCFG_CPU_ADD("maincpu", M68000, XTAL_16MHz/2) // 10MHz rated part, near 16Mhz XTAL
	MCFG_CPU_PROGRAM_MAP(littlerb_main)

	MCFG_INDER_VIDEO_ADD("inder_vid") // XTAL_40MHz

	// TODO: not accurate - driven by XTAL_6MHz?
	MCFG_TIMER_DRIVER_ADD_PERIODIC("step_timer", littlerb_state, littlerb_sound_step_cb,  attotime::from_hz(7500/150))
	MCFG_TIMER_DRIVER_ADD_PERIODIC("sound_timer", littlerb_state, littlerb_sound_cb,  attotime::from_hz(7500))

	MCFG_SPEAKER_STANDARD_STEREO("lspeaker","rspeaker")

	MCFG_SOUND_ADD("ldac", DAC_8BIT_R2R, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 0.5) // unknown DAC
	MCFG_SOUND_ADD("rdac", DAC_8BIT_R2R, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 0.5) // unknown DAC
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "ldac", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "ldac", -1.0, DAC_VREF_NEG_INPUT)
	MCFG_SOUND_ROUTE_EX(0, "rdac", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "rdac", -1.0, DAC_VREF_NEG_INPUT)
MACHINE_CONFIG_END

ROM_START( littlerb )
	ROM_REGION( 0x100000, "maincpu", 0 ) /* 68000 Code */
	ROM_LOAD16_BYTE( "roma.u53", 0x00001, 0x80000, CRC(172fbc13) SHA1(cd165ca0d0546e2634cf182dc98004cbfb02cf9f) )
	ROM_LOAD16_BYTE( "romb.u29", 0x00000, 0x80000, CRC(b2fb1d61) SHA1(9a9d7176c241928d07af651e5f7f21d4f019701d) )

	ROM_REGION( 0x80000, "samples", 0 ) /* sound samples */
	ROM_LOAD( "romc.u26", 0x40000, 0x40000, CRC(f193c5b6) SHA1(95548a40e2b5064c558b36cabbf507d23678b1b2) )
	ROM_LOAD( "romd.u32", 0x00000, 0x40000, CRC(d6b81583) SHA1(b7a63d18a41ccac4d3db9211de0b0cdbc914317a) )
ROM_END

DRIVER_INIT_MEMBER(littlerb_state,littlerb)
{
	/* various scenes flicker to the point of graphics being invisible (eg. the map screen at the very start of a game)
	   unless you overclock the TMS34010 to 120%, possible timing bug in the core? this is a hack */
	m_indervid->subdevice<cpu_device>("tms")->set_clock_scale(1.2f);
}

GAME( 1994, littlerb, 0, littlerb, littlerb, littlerb_state, littlerb, ROT0, "TCH", "Little Robin", MACHINE_IMPERFECT_GRAPHICS|MACHINE_IMPERFECT_SOUND )
