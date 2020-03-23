// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria
/***************************************************************************

Shanghai

driver by Nicola Salmoria

TODO:
- games are currently too fast (especially noticeable with kothello screen transitions), either irqs actually
  fires every two frames or a HD63484 SR bit isn't behaving correctly;
- minor glitch with gfx copy on shanghai stage info panel (garbage on right);

* kothello

Notes: If you use the key labeled as 'Service Coin' you can start the game
with a single 'coin' no matter the Coinage Setting, but the credit is not
displayed.

***************************************************************************/

#include "emu.h"
#include "audio/seibu.h"

#include "cpu/nec/nec.h"
#include "sound/2203intf.h"
#include "video/hd63484.h"
#include "screen.h"
#include "speaker.h"


class shanghai_state : public driver_device
{
public:
	shanghai_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu") { }

	required_device<cpu_device> m_maincpu;

	DECLARE_WRITE16_MEMBER(shanghai_coin_w);

	DECLARE_PALETTE_INIT(shanghai);

	INTERRUPT_GEN_MEMBER(interrupt);
};


PALETTE_INIT_MEMBER(shanghai_state,shanghai)
{
	int i;


	for (i = 0;i < palette.entries();i++)
	{
		int bit0,bit1,bit2,r,g,b;


		/* red component */
		bit0 = (i >> 2) & 0x01;
		bit1 = (i >> 3) & 0x01;
		bit2 = (i >> 4) & 0x01;
		r = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;
		/* green component */
		bit0 = (i >> 5) & 0x01;
		bit1 = (i >> 6) & 0x01;
		bit2 = (i >> 7) & 0x01;
		g = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;
		/* blue component */
		bit0 = 0;
		bit1 = (i >> 0) & 0x01;
		bit2 = (i >> 1) & 0x01;
		b = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;

		palette.set_pen_color(i,rgb_t(r,g,b));
	}
}

INTERRUPT_GEN_MEMBER(shanghai_state::interrupt)
{
	device.execute().set_input_line_and_vector(0,HOLD_LINE,0x80);
}

WRITE16_MEMBER(shanghai_state::shanghai_coin_w)
{
	if (ACCESSING_BITS_0_7)
	{
		machine().bookkeeping().coin_counter_w(0,data & 1);
		machine().bookkeeping().coin_counter_w(1,data & 2);
	}
}

static ADDRESS_MAP_START( shanghai_map, AS_PROGRAM, 16, shanghai_state )
	AM_RANGE(0x00000, 0x03fff) AM_RAM
	AM_RANGE(0x80000, 0xfffff) AM_ROM
ADDRESS_MAP_END


static ADDRESS_MAP_START( shangha2_map, AS_PROGRAM, 16, shanghai_state )
	AM_RANGE(0x00000, 0x03fff) AM_RAM
	AM_RANGE(0x04000, 0x041ff) AM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x80000, 0xfffff) AM_ROM
ADDRESS_MAP_END


static ADDRESS_MAP_START( shanghai_portmap, AS_IO, 16, shanghai_state )
	AM_RANGE(0x00, 0x01) AM_DEVREADWRITE("hd63484", hd63484_device, status_r, address_w)
	AM_RANGE(0x02, 0x03) AM_DEVREADWRITE("hd63484", hd63484_device, data_r, data_w)
	AM_RANGE(0x20, 0x23) AM_DEVREADWRITE8("ymsnd", ym2203_device, read, write, 0x00ff)
	AM_RANGE(0x40, 0x41) AM_READ_PORT("P1")
	AM_RANGE(0x44, 0x45) AM_READ_PORT("P2")
	AM_RANGE(0x48, 0x49) AM_READ_PORT("SYSTEM")
	AM_RANGE(0x4c, 0x4d) AM_WRITE(shanghai_coin_w)
ADDRESS_MAP_END


static ADDRESS_MAP_START( shangha2_portmap, AS_IO, 16, shanghai_state )
	AM_RANGE(0x00, 0x01) AM_READ_PORT("P1")
	AM_RANGE(0x10, 0x11) AM_READ_PORT("P2")
	AM_RANGE(0x20, 0x21) AM_READ_PORT("SYSTEM")
	AM_RANGE(0x30, 0x31) AM_DEVREADWRITE("hd63484", hd63484_device, status_r, address_w)
	AM_RANGE(0x32, 0x33) AM_DEVREADWRITE("hd63484", hd63484_device, data_r, data_w)
	AM_RANGE(0x40, 0x43) AM_DEVREADWRITE8("ymsnd", ym2203_device, read, write, 0x00ff)
	AM_RANGE(0x50, 0x51) AM_WRITE(shanghai_coin_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( kothello_map, AS_PROGRAM, 16, shanghai_state )
	AM_RANGE(0x00000, 0x07fff) AM_RAM
	AM_RANGE(0x08010, 0x08011) AM_DEVREADWRITE("hd63484", hd63484_device, status_r, address_w)
	AM_RANGE(0x08012, 0x08013) AM_DEVREADWRITE("hd63484", hd63484_device, data_r, data_w)
	AM_RANGE(0x09010, 0x09011) AM_READ_PORT("P1")
	AM_RANGE(0x09012, 0x09013) AM_READ_PORT("P2")
	AM_RANGE(0x09014, 0x09015) AM_READ_PORT("SYSTEM")
	AM_RANGE(0x09016, 0x0901f) AM_WRITENOP // 0x9016 is set to 0 at the boot
	AM_RANGE(0x0a000, 0x0a1ff) AM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x0b010, 0x0b01f) AM_DEVREADWRITE("seibu_sound", seibu_sound_device, main_word_r, main_word_w)
	AM_RANGE(0x80000, 0xfffff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( kothello_sound_map, AS_PROGRAM, 8, shanghai_state )
	AM_RANGE(0x0000, 0x1fff) AM_ROM
	AM_RANGE(0x2000, 0x27ff) AM_RAM
	AM_RANGE(0x4000, 0x4000) AM_DEVWRITE("seibu_sound", seibu_sound_device, pending_w)
	AM_RANGE(0x4001, 0x4001) AM_DEVWRITE("seibu_sound", seibu_sound_device, irq_clear_w)
	AM_RANGE(0x4002, 0x4002) AM_DEVWRITE("seibu_sound", seibu_sound_device, rst10_ack_w)
	AM_RANGE(0x4003, 0x4003) AM_DEVWRITE("seibu_sound", seibu_sound_device, rst18_ack_w)
	AM_RANGE(0x4005, 0x4006) AM_DEVWRITE("adpcm", seibu_adpcm_device, adr_w)
	AM_RANGE(0x4007, 0x4007) AM_DEVWRITE("seibu_sound", seibu_sound_device, bank_w)
	AM_RANGE(0x4008, 0x4009) AM_DEVREADWRITE("seibu_sound", seibu_sound_device, ym_r, ym_w)
	AM_RANGE(0x4010, 0x4011) AM_DEVREAD("seibu_sound", seibu_sound_device, soundlatch_r)
	AM_RANGE(0x4012, 0x4012) AM_DEVREAD("seibu_sound", seibu_sound_device, main_data_pending_r)
	AM_RANGE(0x4013, 0x4013) AM_READ_PORT("COIN")
	AM_RANGE(0x4018, 0x4019) AM_DEVWRITE("seibu_sound", seibu_sound_device, main_data_w)
	AM_RANGE(0x401a, 0x401a) AM_DEVWRITE("adpcm", seibu_adpcm_device, ctl_w)
	AM_RANGE(0x401b, 0x401b) AM_DEVWRITE("seibu_sound", seibu_sound_device, coin_w)
	AM_RANGE(0x8000, 0xffff) AM_ROMBANK("seibu_bank1")
ADDRESS_MAP_END

static INPUT_PORTS_START( kothello )
	SEIBU_COIN_INPUTS   /* coin inputs read through sound cpu */

	PORT_START("P1")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0xff00, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("P2")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0xff00, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("SYSTEM")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0xff00, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("DSW1")
	PORT_DIPNAME( 0x07, 0x07, DEF_STR( Coin_A ) ) PORT_DIPLOCATION("SW1:1,2,3")
	PORT_DIPSETTING(    0x00, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x02, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x06, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x07, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x03, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x05, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x01, DEF_STR( 1C_4C ) )
	PORT_DIPNAME( 0x38, 0x38, DEF_STR( Coin_B ) ) PORT_DIPLOCATION("SW1:4,5,6")
	PORT_DIPSETTING(    0x00, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x20, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x10, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x30, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x38, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x18, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x28, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 1C_4C ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unused ) ) PORT_DIPLOCATION("SW1:7")
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unused ) ) PORT_DIPLOCATION("SW1:8")
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x03, 0x01, "Move Timer (Versus)" ) PORT_DIPLOCATION("SW2:1,2")
	PORT_DIPSETTING(    0x03, "20 Seconds" )
	PORT_DIPSETTING(    0x01, "25 Seconds" )
	PORT_DIPSETTING(    0x02, "30 Seconds" )
	PORT_DIPSETTING(    0x00, "35 Seconds" )
	PORT_DIPNAME( 0x0c, 0x04, "Move Timer (Puzzle)" ) PORT_DIPLOCATION("SW2:3,4")
	PORT_DIPSETTING(    0x0c, "30 Seconds" )
	PORT_DIPSETTING(    0x04, "35 Seconds" )
	PORT_DIPSETTING(    0x08, "40 Seconds" )
	PORT_DIPSETTING(    0x00, "45 Seconds" )
	PORT_DIPNAME( 0x30, 0x10, DEF_STR( Difficulty ) ) PORT_DIPLOCATION("SW2:5,6")
	PORT_DIPSETTING(    0x30, DEF_STR( Easy ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Medium_Easy ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Medium_Hard ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Hard ) )
	PORT_DIPNAME( 0xc0, 0x40, DEF_STR( Unknown ) ) PORT_DIPLOCATION("SW2:7,8") // alleged to be number of losses to end tsume mode
	PORT_DIPSETTING(    0xc0, "1" )
	PORT_DIPSETTING(    0x40, "2" )
	PORT_DIPSETTING(    0x80, "3" )
	PORT_DIPSETTING(    0x00, "4" )
INPUT_PORTS_END


static INPUT_PORTS_START( shanghai )
	PORT_START("P1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON3 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("P2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_COCKTAIL
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_COCKTAIL
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_COCKTAIL
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("SYSTEM")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_COIN3 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("DSW1")
	PORT_SERVICE_DIPLOC( 0x01, IP_ACTIVE_LOW, "SW1:8" )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Allow_Continue ) )   PORT_DIPLOCATION("SW1:7")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x1c, 0x1c, DEF_STR( Coin_B ) )       PORT_DIPLOCATION("SW1:6,5,4")
	PORT_DIPSETTING(    0x00, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x0c, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x1c, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x18, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x14, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x10, DEF_STR( 1C_4C ) )
	PORT_DIPNAME( 0xe0, 0xe0, DEF_STR( Coin_A ) )       PORT_DIPLOCATION("SW1:3,2,1")
	PORT_DIPSETTING(    0x00, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x20, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x40, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x60, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0xe0, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0xc0, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0xa0, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x80, DEF_STR( 1C_4C ) )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x01, 0x01, "Confirmation" )      PORT_DIPLOCATION("SW2:8")
	PORT_DIPSETTING(    0x01, DEF_STR( No ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x02, 0x02, "Help" )          PORT_DIPLOCATION("SW2:7")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x02, DEF_STR( On ) )
	PORT_DIPNAME( 0x0c, 0x08, "2 Players Move Time" )   PORT_DIPLOCATION("SW2:6,5")
	PORT_DIPSETTING(    0x0c, "8" )
	PORT_DIPSETTING(    0x08, "10" )
	PORT_DIPSETTING(    0x04, "12" )
	PORT_DIPSETTING(    0x00, "14" )
	PORT_DIPNAME( 0x30, 0x20, "Bonus Time for Making Pair" )    PORT_DIPLOCATION("SW2:4,3")
	PORT_DIPSETTING(    0x30, "3" )
	PORT_DIPSETTING(    0x20, "4" )
	PORT_DIPSETTING(    0x10, "5" )
	PORT_DIPSETTING(    0x00, "6" )
	PORT_DIPNAME( 0xc0, 0x40, "Start Time" )        PORT_DIPLOCATION("SW2:2,1")
	PORT_DIPSETTING(    0xc0, "30" )
	PORT_DIPSETTING(    0x80, "60" )
	PORT_DIPSETTING(    0x40, "90" )
	PORT_DIPSETTING(    0x00, "120" )
INPUT_PORTS_END

static INPUT_PORTS_START( shangha2 )
	PORT_START("P1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON3 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("P2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_COCKTAIL
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_COCKTAIL
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_COCKTAIL
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_COCKTAIL
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("SYSTEM")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_COIN3 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("DSW1")
	PORT_SERVICE_DIPLOC( 0x01, IP_ACTIVE_LOW, "SW2:8" )
	PORT_DIPNAME( 0x06, 0x06, DEF_STR( Difficulty ) )   PORT_DIPLOCATION("SW2:7,6")
	PORT_DIPSETTING(    0x06, DEF_STR( Easy ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Normal ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Hard ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Hardest ) )
	PORT_DIPNAME( 0x08, 0x00, "2 Players Move Time" )   PORT_DIPLOCATION("SW2:5")
	PORT_DIPSETTING(    0x08, "8" )
	PORT_DIPSETTING(    0x00, "10" )
	PORT_DIPNAME( 0x30, 0x20, "Bonus Time for Making Pair" )    PORT_DIPLOCATION("SW2:4,3")
	PORT_DIPSETTING(    0x30, "3" )
	PORT_DIPSETTING(    0x20, "4" )
	PORT_DIPSETTING(    0x10, "5" )
	PORT_DIPSETTING(    0x00, "6" )
	PORT_DIPNAME( 0xc0, 0x40, "Start Time" )        PORT_DIPLOCATION("SW2:2,1")
	PORT_DIPSETTING(    0xc0, "30" )
	PORT_DIPSETTING(    0x80, "60" )
	PORT_DIPSETTING(    0x40, "90" )
	PORT_DIPSETTING(    0x00, "120" )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x03, 0x03, "Mystery Tiles" )     PORT_DIPLOCATION("SW1:8,7")
	PORT_DIPSETTING(    0x03, "0" )
	PORT_DIPSETTING(    0x02, "4" )
	PORT_DIPSETTING(    0x01, "6" )
	PORT_DIPSETTING(    0x00, "8" )
	PORT_DIPNAME( 0x1c, 0x1c, DEF_STR( Coin_B ) )       PORT_DIPLOCATION("SW1:6,5,4")
	PORT_DIPSETTING(    0x00, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x0c, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x1c, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x18, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x14, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x10, DEF_STR( 1C_4C ) )
	PORT_DIPNAME( 0xe0, 0xe0, DEF_STR( Coin_A ) )       PORT_DIPLOCATION("SW1:3,2,1")
	PORT_DIPSETTING(    0x00, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x20, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x40, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x60, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0xe0, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0xc0, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0xa0, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x80, DEF_STR( 1C_4C ) )
INPUT_PORTS_END

static ADDRESS_MAP_START( hd63484_map, 0, 16, shanghai_state )
	AM_RANGE(0x00000, 0x3ffff) AM_RAM
ADDRESS_MAP_END

static MACHINE_CONFIG_START( shanghai )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", V30, XTAL_16MHz/2) /* NEC D70116C-8 */
	MCFG_CPU_PROGRAM_MAP(shanghai_map)
	MCFG_CPU_IO_MAP(shanghai_portmap)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", shanghai_state,  interrupt)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(57)
	//MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500))
	MCFG_SCREEN_SIZE(384, 280)
	MCFG_SCREEN_VISIBLE_AREA(0, 384-1, 0, 280-1)
	MCFG_SCREEN_UPDATE_DEVICE("hd63484", hd63484_device, update_screen)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 256)
	MCFG_PALETTE_FORMAT(xxxxBBBBGGGGRRRR)
	MCFG_PALETTE_INIT_OWNER(shanghai_state,shanghai)

	MCFG_HD63484_ADD("hd63484", 0, hd63484_map)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("ymsnd", YM2203, XTAL_16MHz/4)
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("DSW1"))
	MCFG_AY8910_PORT_B_READ_CB(IOPORT("DSW2"))
	MCFG_SOUND_ROUTE(0, "mono", 0.15)
	MCFG_SOUND_ROUTE(1, "mono", 0.15)
	MCFG_SOUND_ROUTE(2, "mono", 0.15)
	MCFG_SOUND_ROUTE(3, "mono", 0.80)
MACHINE_CONFIG_END


static MACHINE_CONFIG_START( shangha2 )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", V30, XTAL_16MHz/2) /* ? */
	MCFG_CPU_PROGRAM_MAP(shangha2_map)
	MCFG_CPU_IO_MAP(shangha2_portmap)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", shanghai_state,  interrupt)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(57)
	//MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500))
	MCFG_SCREEN_SIZE(384, 280)
	MCFG_SCREEN_VISIBLE_AREA(0, 384-1, 0, 280-1)
	MCFG_SCREEN_UPDATE_DEVICE("hd63484", hd63484_device, update_screen)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 256)
	MCFG_PALETTE_FORMAT(xxxxBBBBGGGGRRRR)

	MCFG_HD63484_ADD("hd63484", 0, hd63484_map)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("ymsnd", YM2203, XTAL_16MHz/4)
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("DSW1"))
	MCFG_AY8910_PORT_B_READ_CB(IOPORT("DSW2"))
	MCFG_SOUND_ROUTE(0, "mono", 0.15)
	MCFG_SOUND_ROUTE(1, "mono", 0.15)
	MCFG_SOUND_ROUTE(2, "mono", 0.15)
	MCFG_SOUND_ROUTE(3, "mono", 0.80)
MACHINE_CONFIG_END


static MACHINE_CONFIG_START( kothello )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", V30, XTAL_16MHz)
	MCFG_CPU_PROGRAM_MAP(kothello_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", shanghai_state,  interrupt)

	MCFG_CPU_ADD("audiocpu", Z80, XTAL_16MHz/4)
	MCFG_CPU_PROGRAM_MAP(kothello_sound_map)

	MCFG_QUANTUM_TIME(attotime::from_hz(12000))

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(57)
	//MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500))
	MCFG_SCREEN_SIZE(384, 280)
	MCFG_SCREEN_VISIBLE_AREA(0, 384-1, 0, 280-1)
	MCFG_SCREEN_UPDATE_DEVICE("hd63484", hd63484_device, update_screen)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 256)
	MCFG_PALETTE_FORMAT(xxxxBBBBGGGGRRRR)

	MCFG_HD63484_ADD("hd63484", 0, hd63484_map)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	/* same as standard seibu ym2203, but also reads "DSW" */
	MCFG_SOUND_ADD("ymsnd", YM2203, XTAL_16MHz/4)
	MCFG_YM2203_IRQ_HANDLER(DEVWRITELINE("seibu_sound", seibu_sound_device, fm_irqhandler))
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("DSW1"))
	MCFG_AY8910_PORT_B_READ_CB(IOPORT("DSW2"))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.15)

	MCFG_DEVICE_ADD("seibu_sound", SEIBU_SOUND, 0)
	MCFG_SEIBU_SOUND_CPU("audiocpu")
	MCFG_SEIBU_SOUND_ROMBANK("seibu_bank1")
	MCFG_SEIBU_SOUND_YM_READ_CB(DEVREAD8("ymsnd", ym2203_device, read))
	MCFG_SEIBU_SOUND_YM_WRITE_CB(DEVWRITE8("ymsnd", ym2203_device, write))

	MCFG_SOUND_ADD("adpcm", SEIBU_ADPCM, 8000) // actually MSM5205
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.80)
MACHINE_CONFIG_END

/***************************************************************************

  Game driver(s)

***************************************************************************/

/*

Shanghai
Sunsoft, (c) 1988
original copyright (c) 1988 Activision, Inc.
Arcade system designed by Sun Electronics (c) 1988

PCB Layout

SHG-01-B
+------------------------------------------+
| DSW2 DSW1       16MHz                    |
|  YM2203C      D70116C-8       D4364 D4364|
|                               IC12* IC13*|
|                               IC21  IC22 |
|       YM3014B                 IC27  IC28 |
|J                              IC36  IC37 |
|A                                         |
|M                       HD63484P8         |
|M  18MHz       MB81464             MB81464|
|A              MB81464             MB81464|
|         PAL   MB81464             MB81464|
|               MB81464             MB81464|
|               MB81464             MB81464|
|               MB81464             MB81464|
|               MB81464             MB81464|
|               MB81464             MB81464|
+------------------------------------------+

  CPU: NEC D70116C-8 V30
Sound: YM2203C + YM3014B DAC
Video: HD63484P8

Ram:
Fujitsu MB81464-12 64K x 4bit DRAM
NEC D4364C-15L 8K x 8bit SRAM

IC12 & IC13 unpopulated

*/

ROM_START( shanghai )
	ROM_REGION( 0x100000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "shg-22a.ic22", 0xa0001, 0x10000, CRC(e0a085be) SHA1(e281043f97c4cd34a33eb1ec7154abbe67a9aa03) )
	ROM_LOAD16_BYTE( "shg-21a.ic21", 0xa0000, 0x10000, CRC(4ab06d32) SHA1(02667d1270b101386b947d5b9bfe64052e498041) )
	ROM_LOAD16_BYTE( "shg-28a.ic28", 0xc0001, 0x10000, CRC(983ec112) SHA1(110e120e35815d055d6108a7603e83d2d990c666) )
	ROM_LOAD16_BYTE( "shg-27a.ic27", 0xc0000, 0x10000, CRC(41af0945) SHA1(dfc4638a17f716ccc8e59f275571d6dc1093a745) )
	ROM_LOAD16_BYTE( "shg-37b.ic37", 0xe0001, 0x10000, CRC(ead3d66c) SHA1(f9be9a4773ea6c9ba931f7aa8c79121caacc231c) ) /* Single byte difference from IC37 below  0xD58C == 0x01 */
	ROM_LOAD16_BYTE( "shg-36b.ic36", 0xe0000, 0x10000, CRC(a1d6af96) SHA1(01c4c22bf03b3d260fffcbc6dfc5f2dd2bcba14a) )
ROM_END

ROM_START( shanghaij )
	ROM_REGION( 0x100000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "shg-22a.ic22", 0xa0001, 0x10000, CRC(e0a085be) SHA1(e281043f97c4cd34a33eb1ec7154abbe67a9aa03) )
	ROM_LOAD16_BYTE( "shg-21a.ic21", 0xa0000, 0x10000, CRC(4ab06d32) SHA1(02667d1270b101386b947d5b9bfe64052e498041) )
	ROM_LOAD16_BYTE( "shg-28a.ic28", 0xc0001, 0x10000, CRC(983ec112) SHA1(110e120e35815d055d6108a7603e83d2d990c666) )
	ROM_LOAD16_BYTE( "shg-27a.ic27", 0xc0000, 0x10000, CRC(41af0945) SHA1(dfc4638a17f716ccc8e59f275571d6dc1093a745) )
	ROM_LOAD16_BYTE( "shg-37b.ic37", 0xe0001, 0x10000, CRC(3f192da0) SHA1(e70d5da5d702e9bf9ac6b77df62bcf51894aadcf) ) /*  0xD58C == 0x00 */
	ROM_LOAD16_BYTE( "shg-36b.ic36", 0xe0000, 0x10000, CRC(a1d6af96) SHA1(01c4c22bf03b3d260fffcbc6dfc5f2dd2bcba14a) )
ROM_END

ROM_START( shangha2 )
	ROM_REGION( 0x100000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "sht-27j", 0x80001, 0x20000, CRC(969cbf00) SHA1(350025f4e39c7d89cb72e46b52fb467e3e9056f4) )
	ROM_LOAD16_BYTE( "sht-26j", 0x80000, 0x20000, CRC(4bf01ab4) SHA1(6928374db080212a371991ee98cd563e158907f0) )
	ROM_LOAD16_BYTE( "sht-31j", 0xc0001, 0x20000, CRC(312e3b9d) SHA1(f15f76a087d4972aa72145eced8d1fb15329b359) )
	ROM_LOAD16_BYTE( "sht-30j", 0xc0000, 0x20000, CRC(2861a894) SHA1(6da99d15f41e900735f8943f2710487817f98579) )
ROM_END

ROM_START( shangha2a ) // content is the same, just different ROM sizes
	ROM_REGION( 0x100000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "3.bin", 0x80001, 0x10000, CRC(93aacccb) SHA1(8b29b9b24cf268a4376b7f653c19d6f46d698552) )
	ROM_LOAD16_BYTE( "1.bin", 0x80000, 0x10000, CRC(0fb2d8ee) SHA1(fee8074d8116f551c634f088b8121d48a9b4a008) )
	ROM_LOAD16_BYTE( "7.bin", 0xa0001, 0x10000, CRC(f9e06880) SHA1(7840b6672cc02fd70f478a5c9f11cfc26ddfca52) )
	ROM_LOAD16_BYTE( "5.bin", 0xa0000, 0x10000, CRC(06ada73c) SHA1(13ee91b94489096f03afc05fdd3d4c65a87a6628) )
	ROM_LOAD16_BYTE( "4.bin", 0xc0001, 0x10000, CRC(b4d82724) SHA1(84496b7ad43817c307227bdab4f58a19484519bb) )
	ROM_LOAD16_BYTE( "2.bin", 0xc0000, 0x10000, CRC(97a25fdb) SHA1(43f065b737e5c4bd44c02ab1d0d6fa34aea8d139) )

	ROM_LOAD16_BYTE( "8.bin", 0xf0001, 0x08000, CRC(21c41557) SHA1(967c97a6b35407a5b32938c88bf7e719a1489b6b) )
	ROM_LOAD16_BYTE( "6.bin", 0xf0000, 0x08000, CRC(14250057) SHA1(15af554099c977e3c753d758080805581a9e4c50) )
ROM_END


/*

Kyuukyoku no Othello
Success Corp. 1990

PCB Layout

SSS8906
|------------------------------------------|
| YM3014 DSW2  DSW1           4464    4464 |
|                             4464    4464 |
|              YM2203         4464    4464 |
|J      M5205  Z80            4464    4464 |
|A             6116           4464    4464 |
|M             PAL            4464    4464 |
|M                            4464    4464 |
|A  ROM6                      4464    4464 |
|              ROM5           HD63484      |
| YM3931                             898-3 |
|                  S1S6091                 |
|                                          |
|                                          |
| PAL                                      |
| PAL              ROM3  ROM4              |
|                                    PAL   |
|                  ROM2  ROM1              |
| CXQ70116                                 |
| D71011           6264  6264              |
|16MHz     PAL     6264  6264              |
|------------------------------------------|
Notes:
      Z80 clock     : 4.000MHz
      CXQ70116 clock: 16.000MHz
      YM3931 clock  : 4.000MHz
      YM2203 clock  : 4.000MHz
      M5205 clock   : 444598Hz; sample rate = M5205 clock / 48
      HD63484 clock : pin50- 4.000MHz, pin52- 2.000MHz
      VSync         : 57Hz

      HD63484  : CRT Controller
      CXQ70116 : Compatible with NEC V30 (DIP40)
      D71011   : ? (DIP18) 710xx is series of common NEC ICs; timers, counters, parallel interface, interrupt controllers etc
      898-3    : BI 898-3-R 22  8920  (?, DIP16, tied to hd63484)
      YM3931   : Also printed 'SEI0100BU' (SDIP64)
      S1S6091  : Custom QFP80 (Graphics controller?)
      4464     : 64K x4 DRAM
      6264       8K x8 SRAM
      6116     : 2K x8 SRAM

*/


ROM_START( kothello )
	ROM_REGION( 0x100000, "maincpu", 0 )
	ROM_LOAD16_BYTE( "rom1.3e", 0x80001, 0x20000, CRC(8601dcfa) SHA1(e7ffc6da0bfb5cec5a543a2a5223b235c3428eb3) )
	ROM_LOAD16_BYTE( "rom2.5e", 0x80000, 0x20000, CRC(68f6b7a3) SHA1(9f7e217e07bc79b1e95551cd0fe107294bf5889f) )
	ROM_LOAD16_BYTE( "rom3.3f", 0xc0001, 0x20000, CRC(2f3dacd1) SHA1(35bfdc1f377b87a80c3abbb48f9f0b52108fbfc0) )
	ROM_LOAD16_BYTE( "rom4.5f", 0xc0000, 0x20000, CRC(ee8bbea7) SHA1(35dfa7aa89cecba6482b18a5233511bacc4bf331) )

	ROM_REGION( 0x20000, "audiocpu", 0 )
	ROM_LOAD( "rom5.5l",   0x00000, 0x02000, CRC(7eb6e697) SHA1(4476e13f9a9e04472581f2c069760f53b33d5672))
	ROM_CONTINUE(          0x10000, 0x0e000 )

	ROM_REGION( 0x10000, "adpcm", 0 )
	ROM_LOAD( "rom6.7m",   0x00000, 0x10000, CRC(4ab1335d) SHA1(3a803e8a7e9b0c2a26ee23e7ac9c89c70cf2504b))
ROM_END



GAME( 1988, shanghai,  0,        shanghai, shanghai, shanghai_state, 0, ROT0, "Sunsoft", "Shanghai (World)",           MACHINE_IMPERFECT_GRAPHICS | MACHINE_SUPPORTS_SAVE )
GAME( 1988, shanghaij, shanghai, shanghai, shanghai, shanghai_state, 0, ROT0, "Sunsoft", "Shanghai (Japan)",           MACHINE_IMPERFECT_GRAPHICS | MACHINE_SUPPORTS_SAVE )
GAME( 1989, shangha2,  0,        shangha2, shangha2, shanghai_state, 0, ROT0, "Sunsoft", "Shanghai II (Japan, set 1)", MACHINE_SUPPORTS_SAVE )
GAME( 1989, shangha2a, shangha2, shangha2, shangha2, shanghai_state, 0, ROT0, "Sunsoft", "Shanghai II (Japan, set 2)", MACHINE_SUPPORTS_SAVE )
GAME( 1990, kothello,  0,        kothello, kothello, shanghai_state, 0, ROT0, "Success", "Kyuukyoku no Othello",       MACHINE_IMPERFECT_GRAPHICS | MACHINE_SUPPORTS_SAVE )
