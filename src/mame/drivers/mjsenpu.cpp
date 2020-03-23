// license:BSD-3-Clause
// copyright-holders:David Haywood
/********************************************************************

 PCB is marked 'Ver 1.8 B/D-' on 2 of the edges

 Custom chip marked
 ORIENTAL SOFT
  SPR800F1
    0011E

 inputs need finishing off

 -- Test Mode Note --

 The test mode for this game is very buggy, this is not a MAME bug
 the same behavior has been observed on the original PCB.

 One example of this is the dipswitch viewer, which should show 3
 rows of 8 1/0 values, one for each switch. ie

 00000000
 00000000
 00000000

 However..

 Instead of properly representing each of the dips, the 1st switch in
 each bank ends up turning on/off the entire row display (for rows 2/3
 it shifts row 1 by one pixel)

 This then means the 2nd switch changes the digit in the 1st position
 so

 10000000

 the 8th switch changes the 7th digit so

 10000010

 and there's no way at all to change the last digit.

*********************************************************************/

#include "emu.h"
#include "cpu/e132xs/e132xs.h"
#include "machine/nvram.h"
#include "machine/ticket.h"
#include "sound/okim6295.h"
#include "screen.h"
#include "speaker.h"

class mjsenpu_state : public driver_device
{
public:
	mjsenpu_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_maincpu(*this, "maincpu"),
			m_oki(*this, "oki"),
			m_hopper(*this, "hopper"),
			m_mainram(*this, "mainram"),
	//      m_vram(*this, "vram"),
			m_palette(*this, "palette")
	{
	}

	/* devices */
	required_device<e132xt_device> m_maincpu;
	required_device<okim6295_device> m_oki;
	required_device<ticket_dispenser_device> m_hopper;

	required_shared_ptr<uint32_t> m_mainram;
//  required_shared_ptr<uint32_t> m_vram;
	uint8_t m_pal[0x200];
	uint32_t m_vram0[0x20000 / 4];
	uint32_t m_vram1[0x20000 / 4];
	uint8_t m_control;
	uint8_t m_mux;

	DECLARE_READ8_MEMBER(palette_low_r);
	DECLARE_READ8_MEMBER(palette_high_r);
	DECLARE_WRITE8_MEMBER(palette_low_w);
	DECLARE_WRITE8_MEMBER(palette_high_w);
	void set_palette(int offset);

	DECLARE_WRITE8_MEMBER(control_w);
	DECLARE_WRITE8_MEMBER(mux_w);

	DECLARE_READ32_MEMBER(muxed_inputs_r);

	DECLARE_READ32_MEMBER(mjsenpu_speedup_r);

	DECLARE_READ32_MEMBER(vram_r);
	DECLARE_WRITE32_MEMBER(vram_w);

	DECLARE_DRIVER_INIT(mjsenpu);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_mjsenpu(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	required_device<palette_device> m_palette;
};


READ8_MEMBER(mjsenpu_state::palette_low_r)
{
	return m_pal[(offset * 2) + 0];
}


READ8_MEMBER(mjsenpu_state::palette_high_r)
{
	return m_pal[(offset * 2) + 1];
}

void mjsenpu_state::set_palette(int offset)
{
	uint16_t paldata = (m_pal[(offset * 2) + 0] << 8) | (m_pal[(offset * 2) + 1]);
	m_palette->set_pen_color(offset, pal5bit(paldata >> 0), pal5bit(paldata >> 5), pal6bit(paldata >> 10));
}

WRITE8_MEMBER(mjsenpu_state::palette_low_w)
{
	m_pal[(offset * 2)+0] = data;
	set_palette(offset);
}

WRITE8_MEMBER(mjsenpu_state::palette_high_w)
{
	m_pal[(offset * 2)+1] = data;
	set_palette(offset);
}


READ32_MEMBER(mjsenpu_state::vram_r)
{
	if (m_control & 0x01) return m_vram1[offset];
	else return m_vram0[offset];
}

WRITE32_MEMBER(mjsenpu_state::vram_w)
{
	if (m_control & 0x01) COMBINE_DATA(&m_vram1[offset]);
	else COMBINE_DATA(&m_vram0[offset]);
}

WRITE8_MEMBER(mjsenpu_state::control_w)
{
	// bit 0x80 is always set? (sometimes disabled during screen transitions briefly, could be display enable?)

	// bit 0x40 not used?
	// bit 0x20 not used?

	// bit 0x10 is the M6295 bank, samples <26 are the same in both banks and so bank switch isn't written for them, not even in sound test.
	m_oki->set_rom_bank((data&0x10)>>4);

	// bits 0x08 is used in the alt payout / hopper mode (see dipswitches)

	// 0x04 seem to be hopper/ticket related? different ones get used depending on the dips
	m_hopper->write(space, 0, data & 0x04);

	// bit 0x02 could be coin counter?
	machine().bookkeeping().coin_counter_w(0, data & 0x02 );

	// bit 0x01 alternates frequently, using as video buffer, but that's a complete guess
	m_control = data;

//  if (data &~0x9e)
//      printf("control_w %02x\n", data);
}

WRITE8_MEMBER(mjsenpu_state::mux_w)
{
	if ((data != 0x80) &&
		(data != 0x9e) &&
		(data != 0x9d) &&
		(data != 0x9b) &&
		(data != 0x97) &&
		(data != 0x8f))
			printf("mux_w %02x\n", data);

	m_mux = data;
}

READ32_MEMBER(mjsenpu_state::muxed_inputs_r)
{
	switch (m_mux)
	{
	case 0x80: // not read
		break;

	case 0x9e:
		return ioport("MUX_9E")->read();

	case 0x9d:
		return ioport("MUX_9D")->read();

	case 0x9b:
		return ioport("MUX_9B")->read();

	case 0x97:
		return ioport("MUX_97")->read();

	case 0x8f:
		return ioport("MUX_8F")->read();
	}

	logerror("muxed_inputs_r with %02x\n", m_mux);

	return 0x00000000;// 0xffffffff;
}

static ADDRESS_MAP_START( mjsenpu_32bit_map, AS_PROGRAM, 32, mjsenpu_state )
	AM_RANGE(0x00000000, 0x001fffff) AM_RAM AM_SHARE("mainram")
	AM_RANGE(0x40000000, 0x401fffff) AM_ROM AM_REGION("user2",0) // main game rom

	AM_RANGE(0x80000000, 0x8001ffff) AM_READWRITE(vram_r,vram_w)

	AM_RANGE(0xffc00000, 0xffc000ff) AM_READWRITE8(palette_low_r, palette_low_w, 0xffffffff)
	AM_RANGE(0xffd00000, 0xffd000ff) AM_READWRITE8(palette_high_r, palette_high_w, 0xffffffff)

	AM_RANGE(0xffe00000, 0xffe007ff) AM_RAM AM_SHARE("nvram")

	AM_RANGE(0xfff80000, 0xffffffff) AM_ROM AM_REGION("user1",0) // boot rom
ADDRESS_MAP_END


static ADDRESS_MAP_START( mjsenpu_io, AS_IO, 32, mjsenpu_state )
	AM_RANGE(0x4000, 0x4003)  AM_READ(muxed_inputs_r)
	AM_RANGE(0x4010, 0x4013)  AM_READ_PORT("IN1")

	AM_RANGE(0x4020, 0x4023)  AM_WRITE8( control_w, 0x000000ff)

	AM_RANGE(0x4030, 0x4033)  AM_READ_PORT("DSW1")
	AM_RANGE(0x4040, 0x4043)  AM_READ_PORT("DSW2")
	AM_RANGE(0x4050, 0x4053)  AM_READ_PORT("DSW3")

	AM_RANGE(0x4060, 0x4063)  AM_WRITE8( mux_w, 0x000000ff)

	AM_RANGE(0x4070, 0x4073)  AM_DEVREADWRITE8("oki", okim6295_device, read, write, 0x000000ff)
ADDRESS_MAP_END

static INPUT_PORTS_START( mjsenpu )

	PORT_START("MUX_8F") // in joystick mode?
	PORT_BIT( 0x00000001, IP_ACTIVE_LOW, IPT_START1 ) PORT_CONDITION("DSW3",0x08,EQUALS,0x00)
	PORT_BIT( 0x00000002, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_CONDITION("DSW3",0x08,EQUALS,0x00)
	PORT_BIT( 0x00000004, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_CONDITION("DSW3",0x08,EQUALS,0x00)
	PORT_BIT( 0x00000008, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_CONDITION("DSW3",0x08,EQUALS,0x00)
	PORT_BIT( 0x00000010, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_CONDITION("DSW3",0x08,EQUALS,0x00)
	PORT_BIT( 0x00000020, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_CONDITION("DSW3",0x08,EQUALS,0x00)
	PORT_BIT( 0x00000040, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_CONDITION("DSW3",0x08,EQUALS,0x00)
	PORT_BIT( 0x00000080, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_CONDITION("DSW3",0x08,EQUALS,0x00)
	PORT_BIT( 0x000000ff, IP_ACTIVE_LOW, IPT_UNUSED ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0xffffff00, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("MUX_9E")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_MAHJONG_A        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_MAHJONG_E        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_MAHJONG_I        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_MAHJONG_M        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_MAHJONG_KAN      ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_START1           ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x0000003f, IP_ACTIVE_LOW, IPT_UNUSED ) PORT_CONDITION("DSW3",0x08,EQUALS,0x00)
	PORT_BIT( 0xffffffc0, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("MUX_9D")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_MAHJONG_B        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_MAHJONG_F        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_MAHJONG_J        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_MAHJONG_N        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_MAHJONG_REACH    ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_MAHJONG_BET      ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x0000003f, IP_ACTIVE_LOW, IPT_UNUSED ) PORT_CONDITION("DSW3",0x08,EQUALS,0x00)
	PORT_BIT( 0xffffffc0, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("MUX_9B")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_MAHJONG_C        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_MAHJONG_G        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_MAHJONG_K        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_MAHJONG_CHI      ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_MAHJONG_RON      ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN          ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x0000003f, IP_ACTIVE_LOW, IPT_UNUSED ) PORT_CONDITION("DSW3",0x08,EQUALS,0x00)
	PORT_BIT( 0xffffffc0, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("MUX_97")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_MAHJONG_D        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_MAHJONG_H        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_MAHJONG_L        ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_MAHJONG_PON      ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNKNOWN          ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN          ) PORT_CONDITION("DSW3",0x08,EQUALS,0x08)
	PORT_BIT( 0x0000003f, IP_ACTIVE_LOW, IPT_UNUSED ) PORT_CONDITION("DSW3",0x08,EQUALS,0x00)
	PORT_BIT( 0xffffffc0, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("IN1")
	PORT_BIT(                 0x00000001, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT(                 0x00000002, IP_ACTIVE_LOW, IPT_GAMBLE_PAYOUT )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_SPECIAL ) PORT_READ_LINE_DEVICE_MEMBER("hopper", ticket_dispenser_device, line_r) // might be coin out
	PORT_BIT(                 0x00000008, IP_ACTIVE_LOW, IPT_GAMBLE_BOOK )
	PORT_SERVICE_NO_TOGGLE(   0x00000010, IP_ACTIVE_LOW )
	PORT_BIT(                 0x00000020, IP_ACTIVE_LOW, IPT_MEMORY_RESET ) // clears stats in bookkeeping
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_MAHJONG_BET ) PORT_CONDITION("DSW3", 0x08,EQUALS,0x00)
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNUSED ) PORT_CONDITION("DSW3", 0x08, EQUALS, 0x08)
	PORT_DIPNAME( 0x00000080, 0x00000080, DEF_STR( Unknown ) ) // unused??
	PORT_DIPSETTING(          0x00000080, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_BIT( 0xffffff00, IP_ACTIVE_LOW, IPT_UNUSED )




	PORT_START("DSW1")
	PORT_DIPNAME( 0x00000003, 0x00000003, DEF_STR( Coin_A ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(          0x00000003, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(          0x00000002, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(          0x00000001, DEF_STR( 1C_3C ) )
	PORT_DIPNAME( 0x0000000c, 0x0000000c, "Note Value" ) // used if DSW3 bit 0x02 is changed
	PORT_DIPSETTING(          0x00000000, "100" )
	PORT_DIPSETTING(          0x00000004, "50" )
	PORT_DIPSETTING(          0x00000008, "10" )
	PORT_DIPSETTING(          0x0000000c, "5" )
	PORT_DIPNAME( 0x00000030, 0x00000030, "Ratio 2" )
	PORT_DIPSETTING(          0x00000000, "1:10" )
	PORT_DIPSETTING(          0x00000010, "1:5" )
	PORT_DIPSETTING(          0x00000020, "1:2" )
	PORT_DIPSETTING(          0x00000030, "1:1" )
	PORT_DIPNAME( 0x000000c0, 0x000000c0, "Percentage 1" )
	PORT_DIPSETTING(          0x00000000, "96" )
	PORT_DIPSETTING(          0x00000040, "92" )
	PORT_DIPSETTING(          0x00000080, "88" )
	PORT_DIPSETTING(          0x000000c0, "84" )
	PORT_BIT( 0xffffff00, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x00000003, 0x00000003, "Value 2" )
	PORT_DIPSETTING(          0x00000000, "5" )
	PORT_DIPSETTING(          0x00000001, "3" )
	PORT_DIPSETTING(          0x00000002, "2" )
	PORT_DIPSETTING(          0x00000003, "1" )
	PORT_DIPNAME( 0x00000004, 0x00000004, "Value 3" )
	PORT_DIPSETTING(          0x00000004, "10" )
	PORT_DIPSETTING(          0x00000000, "20" )
	PORT_DIPNAME( 0x00000008, 0x00000000, DEF_STR( Demo_Sounds )  )
	PORT_DIPSETTING(          0x00000008, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x00000010, 0x00000010, DEF_STR( Flip_Screen ) )
	PORT_DIPSETTING(          0x00000010, DEF_STR( Off ) )
	PORT_DIPSETTING(          0x00000000, DEF_STR( On ) )
	PORT_DIPNAME( 0x000000e0, 0x000000e0, "Percentage 2" )
	PORT_DIPSETTING(          0x00000000, "60" )
	PORT_DIPSETTING(          0x00000020, "65" )
	PORT_DIPSETTING(          0x00000040, "70" )
	PORT_DIPSETTING(          0x00000060, "75" )
	PORT_DIPSETTING(          0x00000080, "80" )
	PORT_DIPSETTING(          0x000000a0, "85" )
	PORT_DIPSETTING(          0x000000c0, "90" )
	PORT_DIPSETTING(          0x000000e0, "95" )
	PORT_BIT( 0xffffff00, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("DSW3")
	PORT_DIPNAME( 0x00000001, 0x00000001, "Credit Limit" )
	PORT_DIPSETTING(          0x00000001, "100" )
	PORT_DIPSETTING(          0x00000000, "500" )
	PORT_DIPNAME( 0x00000002, 0x00000002, "Coin Type?" ) // uses different coinage
	PORT_DIPSETTING(          0x00000002, "Coins?" )
	PORT_DIPSETTING(          0x00000000, "Notes?" )
	PORT_DIPNAME( 0x00000004, 0x00000004, "Hopper Type?" )
	PORT_DIPSETTING(          0x00000004, "Normal?" ) // pressing Pay Out button activates hopper on bit 0x04 and pays out
	PORT_DIPSETTING(          0x00000000, "Other?" ) // pressing Pay Out activates something on bit 0x08, prints KEY OUT and quickly resets the game
	PORT_DIPNAME( 0x00000008, 0x00000008, "Control Type" )
	PORT_DIPSETTING(          0x00000008, "Mahjong Panel" )
	PORT_DIPSETTING(          0x00000000, "Joystick" )
	PORT_DIPNAME( 0x00000010, 0x00000010, "Symbol 5" )
	PORT_DIPSETTING(          0x00000010, "0" )
	PORT_DIPSETTING(          0x00000000, "1" )
	PORT_DIPNAME( 0x00000060, 0x00000060, "Percentage 3" )
	PORT_DIPSETTING(          0x00000000, "92" )
	PORT_DIPSETTING(          0x00000020, "88" )
	PORT_DIPSETTING(          0x00000040, "84" )
	PORT_DIPSETTING(          0x00000060, "80" )
	PORT_DIPNAME( 0x00000080, 0x00000080, "Symbol 6" )
	PORT_DIPSETTING(          0x00000080, "0" )
	PORT_DIPSETTING(          0x00000000, "1" )
	PORT_BIT( 0xffffff00, IP_ACTIVE_LOW, IPT_UNUSED )

INPUT_PORTS_END


void mjsenpu_state::video_start()
{
}



uint32_t mjsenpu_state::screen_update_mjsenpu(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	int x,y,count;
	int color;

	uint32_t* vram;

	if (m_control & 0x01) vram = m_vram0;
	else  vram = m_vram1;



	count = 0;
	for (y=0;y < 256;y++)
	{
		for (x=0;x < 512/4;x++)
		{
			color = vram[count] & 0x000000ff;
			bitmap.pix16(y, x*4 + 2) = color;

			color = (vram[count] & 0x0000ff00) >> 8;
			bitmap.pix16(y, x*4 + 3) = color;

			color = (vram[count] & 0x00ff0000) >> 16;
			bitmap.pix16(y, x*4 + 0) = color;

			color = (vram[count] & 0xff000000) >> 24;
			bitmap.pix16(y, x*4 + 1) = color;

			count++;
		}
	}
	return 0;
}


void mjsenpu_state::machine_start()
{
	save_item(NAME(m_pal));
	save_item(NAME(m_vram0));
	save_item(NAME(m_vram1));
	save_item(NAME(m_control));
	save_item(NAME(m_mux));
}

void mjsenpu_state::machine_reset()
{
}

/*
following clocks are on the PCB

22.1184
27.000
1.0000000

*/

static MACHINE_CONFIG_START( mjsenpu )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", E132XT, 27000000*2) /* ?? Mhz */
	MCFG_CPU_PROGRAM_MAP(mjsenpu_32bit_map)
	MCFG_CPU_IO_MAP(mjsenpu_io)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", mjsenpu_state,  irq0_line_hold)

	MCFG_NVRAM_ADD_1FILL("nvram")

	// more likely coins out?
	MCFG_TICKET_DISPENSER_ADD("hopper", attotime::from_msec(50), TICKET_MOTOR_ACTIVE_LOW, TICKET_STATUS_ACTIVE_HIGH)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(512, 256)
	MCFG_SCREEN_VISIBLE_AREA(0, 512-1, 0, 256-16-1)
	MCFG_SCREEN_UPDATE_DRIVER(mjsenpu_state, screen_update_mjsenpu)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 0x100)

	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_OKIM6295_ADD("oki", 1000000, PIN7_HIGH) /* 1 Mhz? */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.00)
MACHINE_CONFIG_END


ROM_START( mjsenpu )
	ROM_REGION32_BE( 0x80000, "user1", 0 ) /* Hyperstone CPU Code */
	ROM_LOAD( "U1", 0x000000, 0x080000, CRC(ebfb1079) SHA1(9d676c635d5ee464df5730518399e141ebc515ed) )

	ROM_REGION32_BE( 0x200000, "user2", 0 ) /* Hyperstone CPU Code */
	ROM_LOAD16_WORD_SWAP( "U13", 0x000000, 0x200000, CRC(a803c5a5) SHA1(61c7386a1bb6224b788de01293697d0e896839a8) )

	ROM_REGION( 0x080000, "oki", 0 )
	ROM_LOAD( "SU2", 0x000000, 0x080000, CRC(848045d5) SHA1(4d32e1a5bd0937069dd8d50dfd8b63d4a45e40e6) )
ROM_END



READ32_MEMBER(mjsenpu_state::mjsenpu_speedup_r)
{
	int pc = m_maincpu->pc();

	if (pc == 0xadb8)
	{
		space.device().execute().spin_until_interrupt();
	}
	else
	{
	//  printf("%08x\n", pc);
	}

	return m_mainram[0x23468/4];
}




DRIVER_INIT_MEMBER(mjsenpu_state,mjsenpu)
{
/*
0000ADAE: LDHU.D L42, L38, $0
0000ADB2: LDW.A 0, L39, $23468
0000ADB8: ADDI L38, $1
0000ADBA: STHU.D L42, L38, $0
0000ADBE: CMPI L39, $0
0000ADC0: BNE $adae

   (loops for 744256 instructions)
*/
	// not especially effective, might be wrong.
	m_maincpu->space(AS_PROGRAM).install_read_handler(0x23468, 0x2346b, read32_delegate(FUNC(mjsenpu_state::mjsenpu_speedup_r), this));
}


GAME( 2002, mjsenpu, 0, mjsenpu, mjsenpu, mjsenpu_state, mjsenpu, ROT0, "Oriental Soft", "Mahjong Senpu", 0 )
