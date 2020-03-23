// license:BSD-3-Clause
// copyright-holders:Mirko Buffoni, Roberto Fresca
/***************************************************************************

  Super Poker (IGS)
  Driver by Mirko Buffoni
  Additional work by Roberto Fresca.

****************************************************************************

  NOTES:

  - Very similar to IGS009 driver, but without the reels stuff.
    Maybe both drivers can be merged at some point.

  - The Super Poker US/UA sets look like they lack of PPI 8255 devices...

****************************************************************************

  TODO:

  - Understand how to reset NVRAM
  - Map DSW (Operator mode doesn't help)
  - Map Leds and Coin counters
  - 3super8 randomly crashes
  - 3super8 doesn't have the 8x32 tilemap, change the video emulation accordingly

***************************************************************************/

#include "emu.h"
#include "cpu/z180/z180.h"
#include "cpu/z80/z80.h"
#include "machine/i8255.h"
#include "sound/ym2413.h"
#include "sound/okim6295.h"
#include "machine/nvram.h"
#include "screen.h"
#include "speaker.h"


class spoker_state : public driver_device
{
public:
	spoker_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_bg_tile_ram(*this, "bg_tile_ram"),
		m_fg_tile_ram(*this, "fg_tile_ram"),
		m_fg_color_ram(*this, "fg_color_ram") { }

	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;

	required_shared_ptr<uint8_t> m_bg_tile_ram;
	tilemap_t *m_bg_tilemap;

	required_shared_ptr<uint8_t> m_fg_tile_ram;
	required_shared_ptr<uint8_t> m_fg_color_ram;
	tilemap_t *m_fg_tilemap;

	// common
	int m_nmi_ack;
	uint8_t m_out[3];

	// spk116it and spk115it specific
	int m_video_enable;
	int m_hopper;
	uint8_t m_igs_magic[2];

	// common
	DECLARE_WRITE8_MEMBER(bg_tile_w);
	DECLARE_WRITE8_MEMBER(fg_tile_w);
	DECLARE_WRITE8_MEMBER(fg_color_w);
	DECLARE_WRITE8_MEMBER(nmi_and_coins_w);
	DECLARE_WRITE8_MEMBER(leds_w);

	// spk116it and spk115it specific
	DECLARE_WRITE8_MEMBER(video_and_leds_w);
	DECLARE_WRITE8_MEMBER(magic_w);
	DECLARE_READ8_MEMBER(magic_r);

	DECLARE_CUSTOM_INPUT_MEMBER(hopper_r);

	DECLARE_DRIVER_INIT(spkleftover);
	DECLARE_DRIVER_INIT(spk116it);
	DECLARE_DRIVER_INIT(3super8);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;

	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
};


/***************************************************************************
                                Video Hardware
***************************************************************************/

WRITE8_MEMBER(spoker_state::bg_tile_w)
{
	m_bg_tile_ram[offset] = data;
	m_bg_tilemap->mark_tile_dirty(offset);
}

TILE_GET_INFO_MEMBER(spoker_state::get_bg_tile_info)
{
	int code = m_bg_tile_ram[tile_index];
	SET_TILE_INFO_MEMBER(1 + (tile_index & 3), code & 0xff, 0, 0);
}

TILE_GET_INFO_MEMBER(spoker_state::get_fg_tile_info)
{
	int code = m_fg_tile_ram[tile_index] | (m_fg_color_ram[tile_index] << 8);
	SET_TILE_INFO_MEMBER(0, code, (4*(code >> 14)+3), 0);
}

WRITE8_MEMBER(spoker_state::fg_tile_w)
{
	m_fg_tile_ram[offset] = data;
	m_fg_tilemap->mark_tile_dirty(offset);
}

WRITE8_MEMBER(spoker_state::fg_color_w)
{
	m_fg_color_ram[offset] = data;
	m_fg_tilemap->mark_tile_dirty(offset);
}

void spoker_state::video_start()
{
	m_bg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(spoker_state::get_bg_tile_info),this), TILEMAP_SCAN_ROWS, 8,  32, 128, 8);
	m_fg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(spoker_state::get_fg_tile_info),this), TILEMAP_SCAN_ROWS, 8,  8,  128, 32);
	m_fg_tilemap->set_transparent_pen(0);
}

uint32_t spoker_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(m_palette->black_pen(), cliprect);
	m_bg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	m_fg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	return 0;
}


/***************************************************************************
                               Misc Handlers
***************************************************************************/

CUSTOM_INPUT_MEMBER(spoker_state::hopper_r)
{
	if (m_hopper) return !(m_screen->frame_number()%10);
	return machine().input().code_pressed(KEYCODE_H);
}

static void show_out(running_machine &machine,  uint8_t *out)
{
#ifdef MAME_DEBUG
	machine.popmessage("%02x %02x %02x", out[0], out[1], out[2]);
#endif
}

WRITE8_MEMBER(spoker_state::nmi_and_coins_w)
{
	if ((data) & (0x22))
	{
		logerror("PC %06X: nmi_and_coins = %02x\n",space.device().safe_pc(),data);
//      popmessage("%02x",data);
	}

	machine().bookkeeping().coin_counter_w(0, data & 0x01);   // coin_a
	machine().bookkeeping().coin_counter_w(1, data & 0x04);   // coin_c
	machine().bookkeeping().coin_counter_w(2, data & 0x08);   // key in
	machine().bookkeeping().coin_counter_w(3, data & 0x10);   // coin out mech

	output().set_led_value(6, data & 0x40);   // led for coin out / hopper active

	if(((m_nmi_ack & 0x80) == 0) && data & 0x80)
		m_maincpu->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);

	m_nmi_ack = data & 0x80;     // nmi acknowledge, 0 -> 1

	m_out[0] = data;
	show_out(machine(), m_out);
}

WRITE8_MEMBER(spoker_state::video_and_leds_w)
{
	output().set_led_value(4, data & 0x01); // start?
	output().set_led_value(5, data & 0x04); // l_bet?

	m_video_enable = data & 0x40;
	m_hopper = (~data)& 0x80;

	m_out[1] = data;
	show_out(machine(), m_out);
}

WRITE8_MEMBER(spoker_state::leds_w)
{
	output().set_led_value(0, data & 0x01);  // stop_1
	output().set_led_value(1, data & 0x02);  // stop_2
	output().set_led_value(2, data & 0x04);  // stop_3
	output().set_led_value(3, data & 0x08);  // stop
	// data & 0x10?

	m_out[2] = data;
	show_out(machine(), m_out);
}

WRITE8_MEMBER(spoker_state::magic_w)
{
	m_igs_magic[offset] = data;

	if (offset == 0)
		return;

	switch(m_igs_magic[0])
	{
		case 0x01:
			break;

		default:
//          popmessage("magic %x <- %04x",igs_magic[0],data);
			logerror("%06x: warning, writing to igs_magic %02x = %02x\n", space.device().safe_pc(), m_igs_magic[0], data);
	}
}

READ8_MEMBER(spoker_state::magic_r)
{
	switch(m_igs_magic[0])
	{
		case 0x00:
			if ( !(m_igs_magic[1] & 0x01) ) return ioport("DSW1")->read();
			if ( !(m_igs_magic[1] & 0x02) ) return ioport("DSW2")->read();
			if ( !(m_igs_magic[1] & 0x04) ) return ioport("DSW3")->read();
			if ( !(m_igs_magic[1] & 0x08) ) return ioport("DSW4")->read();
			if ( !(m_igs_magic[1] & 0x10) ) return ioport("DSW5")->read();
			logerror("%06x: warning, reading dsw with igs_magic[1] = %02x\n", space.device().safe_pc(), m_igs_magic[1]);
			break;

		default:
			logerror("%06x: warning, reading with igs_magic = %02x\n", space.device().safe_pc(), m_igs_magic[0]);
	}

	return 0;
}


/***************************************************************************
                                Memory Maps
***************************************************************************/

static ADDRESS_MAP_START( spoker_map, AS_PROGRAM, 8, spoker_state )
	AM_RANGE(0x00000, 0x0f3ff) AM_ROM
	AM_RANGE(0x0f400, 0x0ffff) AM_RAM AM_SHARE("nvram")
ADDRESS_MAP_END

static ADDRESS_MAP_START( spoker_portmap, AS_IO, 8, spoker_state )
	AM_RANGE(0x0000, 0x003f) AM_RAM // Z180 internal regs
	AM_RANGE(0x2000, 0x23ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x2400, 0x27ff) AM_RAM_DEVWRITE("palette", palette_device, write_ext) AM_SHARE("palette_ext")
	AM_RANGE(0x3000, 0x33ff) AM_RAM_WRITE(bg_tile_w ) AM_SHARE("bg_tile_ram")
	AM_RANGE(0x5000, 0x5fff) AM_RAM_WRITE(fg_tile_w )  AM_SHARE("fg_tile_ram")
	AM_RANGE(0x6480, 0x6483) AM_DEVREADWRITE("ppi8255_0", i8255_device, read, write)    /* NMI and coins (w), service (r), coins (r) */
	AM_RANGE(0x6490, 0x6493) AM_DEVREADWRITE("ppi8255_1", i8255_device, read, write)    /* buttons 1 (r), video and leds (w), leds (w) */
	AM_RANGE(0x64a0, 0x64a0) AM_READ_PORT( "BUTTONS2" )
	AM_RANGE(0x64b0, 0x64b1) AM_DEVWRITE("ymsnd", ym2413_device, write)
	AM_RANGE(0x64c0, 0x64c0) AM_DEVREADWRITE("oki", okim6295_device, read, write)
	AM_RANGE(0x64d0, 0x64d1) AM_READWRITE(magic_r, magic_w )    // DSW1-5
	AM_RANGE(0x7000, 0x7fff) AM_RAM_WRITE(fg_color_w ) AM_SHARE("fg_color_ram")
ADDRESS_MAP_END

static ADDRESS_MAP_START( 3super8_portmap, AS_IO, 8, spoker_state )
//  AM_RANGE(0x1000, 0x1fff) AM_WRITENOP
	AM_RANGE(0x2000, 0x27ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE(0x2800, 0x2fff) AM_RAM_DEVWRITE("palette", palette_device, write_ext) AM_SHARE("palette_ext")
	AM_RANGE(0x3000, 0x33ff) AM_RAM_WRITE(bg_tile_w ) AM_SHARE("bg_tile_ram")
	AM_RANGE(0x4000, 0x4000) AM_READ_PORT( "DSW1" )
	AM_RANGE(0x4001, 0x4001) AM_READ_PORT( "DSW2" )
	AM_RANGE(0x4002, 0x4002) AM_READ_PORT( "DSW3" )
	AM_RANGE(0x4003, 0x4003) AM_READ_PORT( "DSW4" )
	AM_RANGE(0x4004, 0x4004) AM_READ_PORT( "DSW5" )
//  AM_RANGE(0x4000, 0x40ff) AM_WRITENOP
	AM_RANGE(0x5000, 0x5fff) AM_RAM_WRITE(fg_tile_w )  AM_SHARE("fg_tile_ram")

//  The following one (0x6480) should be output. At beginning of code, there is a PPI initialization
//  setting O-I-I for ports ABC. Except these routines are just a leftover from the original game,
//  and now the I/O is handled by a CPLD or FPGA (as seen in goldstar and gp98).
	AM_RANGE(0x6480, 0x6480) AM_READ_PORT( "IN0" )
	AM_RANGE(0x6490, 0x6490) AM_READ_PORT( "IN1" )

	AM_RANGE(0x6491, 0x6491) AM_DEVREADWRITE("oki", okim6295_device, read, write)
	AM_RANGE(0x64a0, 0x64a0) AM_READ_PORT( "IN2" )
	AM_RANGE(0x64b0, 0x64b0) AM_WRITE(leds_w )
	AM_RANGE(0x64c0, 0x64c0) AM_READNOP //irq ack?
	AM_RANGE(0x64f0, 0x64f0) AM_WRITE(nmi_and_coins_w )
	AM_RANGE(0x7000, 0x7fff) AM_RAM_WRITE(fg_color_w ) AM_SHARE("fg_color_ram")
ADDRESS_MAP_END


/***************************************************************************
                                Input Ports
***************************************************************************/

static INPUT_PORTS_START( spoker )
	PORT_START("DSW1")
	PORT_DIPNAME( 0x01, 0x00, DEF_STR( Demo_Sounds ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPUNKNOWN( 0x02, 0x02 )
	PORT_DIPNAME( 0x1c, 0x1c, DEF_STR( Coinage ) )
	PORT_DIPSETTING(    0x1c, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x18, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x14, DEF_STR( 1C_4C ) )
	PORT_DIPSETTING(    0x10, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(    0x0c, "1 Coin/10 Credits" )
	PORT_DIPSETTING(    0x08, "1 Coin/20 Credits" )
	PORT_DIPSETTING(    0x04, "1 Coin/40 Credits" )
	PORT_DIPSETTING(    0x00, "1 Coin/50 Credits" )
	PORT_DIPNAME( 0x20, 0x20, "Card Type" )
	PORT_DIPSETTING(    0x20, "Cards" )
	PORT_DIPSETTING(    0x00, "Numbers" )
	PORT_DIPUNKNOWN( 0x40, 0x40 )
	PORT_DIPUNKNOWN( 0x80, 0x80 )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x03, 0x00, "Min Bet" )
	PORT_DIPSETTING(    0x03, "1" )
	PORT_DIPSETTING(    0x02, "5" )
	PORT_DIPSETTING(    0x01, "10" )
	PORT_DIPSETTING(    0x00, "20" )
	PORT_DIPNAME( 0x1c, 0x1c, "Key In/Out" )
	PORT_DIPSETTING(    0x1c, "10 Credits" )
	PORT_DIPSETTING(    0x18, "20 Credits" )
	PORT_DIPSETTING(    0x14, "40 Credits" )
	PORT_DIPSETTING(    0x10, "50 Credits" )
	PORT_DIPSETTING(    0x0c, "100 Credits" )
	PORT_DIPSETTING(    0x08, "200 Credits" )
	PORT_DIPSETTING(    0x04, "250 Credits" )
	PORT_DIPSETTING(    0x00, "500 Credits" )
	PORT_DIPUNKNOWN( 0x20, 0x20 )
	PORT_DIPUNKNOWN( 0x40, 0x40 )
	PORT_DIPUNKNOWN( 0x80, 0x80 )

	PORT_START("DSW3")
	PORT_DIPUNKNOWN( 0x01, 0x01 )
	PORT_DIPUNKNOWN( 0x02, 0x02 )
	PORT_DIPUNKNOWN( 0x04, 0x04 )
	PORT_DIPUNKNOWN( 0x08, 0x08 )
	PORT_DIPNAME( 0x10, 0x10, "Credit Limit" )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, "On (2000)" )
	PORT_DIPUNKNOWN( 0x20, 0x20 )
	PORT_DIPUNKNOWN( 0x40, 0x40 )
	PORT_DIPUNKNOWN( 0x80, 0x80 )

	PORT_START("DSW4")
	PORT_DIPNAME( 0x07, 0x07, "Max Bet" )
	PORT_DIPSETTING(    0x07, "1" )
	PORT_DIPSETTING(    0x06, "2" )
	PORT_DIPSETTING(    0x05, "5" )
	PORT_DIPSETTING(    0x04, "10" )
	PORT_DIPSETTING(    0x03, "20" )
	PORT_DIPSETTING(    0x02, "40" )
	PORT_DIPSETTING(    0x01, "50" )
	PORT_DIPSETTING(    0x00, "100" )
	PORT_DIPUNKNOWN( 0x08, 0x08 )
	PORT_DIPUNKNOWN( 0x10, 0x10 )
	PORT_DIPUNKNOWN( 0x20, 0x20 )
	PORT_DIPUNKNOWN( 0x40, 0x40 )
	PORT_DIPUNKNOWN( 0x80, 0x80 )

	PORT_START("DSW5")
	PORT_DIPUNKNOWN( 0x01, 0x01 )
	PORT_DIPUNKNOWN( 0x02, 0x02 )
	PORT_DIPUNKNOWN( 0x04, 0x04 )
	PORT_DIPUNKNOWN( 0x08, 0x08 )
	PORT_DIPUNKNOWN( 0x10, 0x10 )
	PORT_DIPUNKNOWN( 0x20, 0x20 )
	PORT_DIPUNKNOWN( 0x40, 0x40 )
	PORT_DIPUNKNOWN( 0x80, 0x80 )

	PORT_START("SERVICE")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNKNOWN  )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_SERVICE1 ) PORT_NAME("Memory Clear") // stats, memory
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_SPECIAL  ) PORT_CUSTOM_MEMBER(DEVICE_SELF,spoker_state,hopper_r, nullptr) PORT_NAME("HPSW")   // hopper sensor
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN  )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_GAMBLE_PAYOUT )
	PORT_SERVICE_NO_TOGGLE( 0x20, IP_ACTIVE_LOW )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_GAMBLE_BOOK ) PORT_NAME("Statistics")
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN  )

	PORT_START("COINS")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_COIN1   )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_COIN2   )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_GAMBLE_KEYIN )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_GAMBLE_KEYOUT ) PORT_NAME("Key Down")
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("BUTTONS1")
	PORT_BIT( 0xff, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("BUTTONS2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_POKER_HOLD1 ) PORT_NAME("Hold 1 / High / Low")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_POKER_HOLD5 ) PORT_NAME("Hold 5 / Bet")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_POKER_HOLD4 ) PORT_NAME("Hold 4 / Take")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_POKER_HOLD3 ) PORT_NAME("Hold 3 / W-Up")
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_POKER_HOLD2 ) PORT_NAME("Hold 2 / Red / Black")
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END

static INPUT_PORTS_START( 3super8 )
	PORT_START("DSW1")
	PORT_DIPUNKNOWN( 0x01, 0x01 )
	PORT_DIPUNKNOWN( 0x02, 0x02 )
	PORT_DIPUNKNOWN( 0x04, 0x04 )
	PORT_DIPUNKNOWN( 0x08, 0x08 )
	PORT_DIPUNKNOWN( 0x10, 0x10 )
	PORT_DIPUNKNOWN( 0x20, 0x20 )
	PORT_DIPUNKNOWN( 0x40, 0x40 )
	PORT_DIPUNKNOWN( 0x80, 0x80 )

	PORT_START("DSW2")
	PORT_DIPUNKNOWN( 0x01, 0x01 )
	PORT_DIPUNKNOWN( 0x02, 0x02 )
	PORT_DIPUNKNOWN( 0x04, 0x04 )
	PORT_DIPUNKNOWN( 0x08, 0x08 )
	PORT_DIPUNKNOWN( 0x10, 0x10 )
	PORT_DIPUNKNOWN( 0x20, 0x20 )
	PORT_DIPUNKNOWN( 0x40, 0x40 )
	PORT_DIPUNKNOWN( 0x80, 0x80 )

	PORT_START("DSW3")
	PORT_DIPUNKNOWN( 0x01, 0x01 )
	PORT_DIPUNKNOWN( 0x02, 0x02 )
	PORT_DIPUNKNOWN( 0x04, 0x04 )
	PORT_DIPUNKNOWN( 0x08, 0x08 )
	PORT_DIPUNKNOWN( 0x10, 0x10 )
	PORT_DIPUNKNOWN( 0x20, 0x20 )
	PORT_DIPUNKNOWN( 0x40, 0x40 )
	PORT_DIPUNKNOWN( 0x80, 0x80 )

	PORT_START("DSW4")
	PORT_DIPUNKNOWN( 0xff, 0xff )

	PORT_START("DSW5")
	PORT_DIPUNKNOWN( 0xff, 0xff )

	PORT_START("IN0")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNKNOWN  )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_SPECIAL  ) PORT_CUSTOM_MEMBER(DEVICE_SELF,spoker_state,hopper_r, nullptr) PORT_NAME("HPSW")   // hopper sensor
	PORT_SERVICE( 0x04, IP_ACTIVE_LOW )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_GAMBLE_BOOK ) PORT_NAME("Statistics")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_COIN1   )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_COIN2   )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_GAMBLE_KEYOUT ) PORT_NAME("Key Down")
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_GAMBLE_PAYOUT )

	PORT_START("IN1")
	PORT_DIPNAME( 0x01, 0x01, "IN1" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("IN2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_POKER_HOLD1 ) PORT_NAME("Hold 1 / High / Low")
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_POKER_HOLD5 ) PORT_NAME("Hold 5 / Bet")
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_POKER_HOLD4 ) PORT_NAME("Hold 4 / Take")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_POKER_HOLD3 ) PORT_NAME("Hold 3 / W-Up")
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_POKER_HOLD2 ) PORT_NAME("Hold 2 / Red / Black")
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN  )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN  )
INPUT_PORTS_END


/***************************************************************************
                     Graphics Layout & Graphics Decode
***************************************************************************/

static const gfx_layout layout_8x8x6 =
{
	8, 8,
	RGN_FRAC(1, 3),
	6,
	{ RGN_FRAC(0,3)+8, RGN_FRAC(0,3)+0,
		RGN_FRAC(1,3)+8, RGN_FRAC(1,3)+0,
		RGN_FRAC(2,3)+8, RGN_FRAC(2,3)+0 },
	{ STEP8(0,1) },
	{ STEP8(0,2*8) },
	8*8*2
};

static const gfx_layout layout_8x32x6 =
{
	8, 32,
	RGN_FRAC(1, 3),
	6,
	{ RGN_FRAC(0,3)+8, RGN_FRAC(0,3)+0,
		RGN_FRAC(1,3)+8, RGN_FRAC(1,3)+0,
		RGN_FRAC(2,3)+8, RGN_FRAC(2,3)+0 },
	{ STEP8(0,1) },
	{ STEP32(0,2*8) },
	8*32*2
};

static const gfx_layout layout3s8_8x8x6 =
{
	8,8,
	RGN_FRAC(1,3),
	6,
	{ RGN_FRAC(0,3)+2*8, RGN_FRAC(0,3)+0, RGN_FRAC(1,3)+2*8, RGN_FRAC(1,3)+0,RGN_FRAC(2,3)+2*8, RGN_FRAC(2,3)+0 },
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	{ 0*8, 1*8, 4*8, 5*8, 8*8, 9*8, 12*8, 13*8 },
	16*8
};

static GFXDECODE_START( spoker )
	GFXDECODE_ENTRY( "gfx1", 0x00000, layout_8x8x6,  0, 16 )
	GFXDECODE_ENTRY( "gfx2", 0x04000, layout_8x32x6, 0, 16 )
	GFXDECODE_ENTRY( "gfx2", 0x08000, layout_8x32x6, 0, 16 )
	GFXDECODE_ENTRY( "gfx2", 0x0c000, layout_8x32x6, 0, 16 )
	GFXDECODE_ENTRY( "gfx2", 0x00000, layout_8x32x6, 0, 16 )
GFXDECODE_END

static GFXDECODE_START( 3super8 )
	GFXDECODE_ENTRY( "gfx1", 0x00000, layout3s8_8x8x6, 0, 16 )
	GFXDECODE_ENTRY( "gfx2", 0x04000, layout_8x32x6,   0, 16 )
	GFXDECODE_ENTRY( "gfx2", 0x08000, layout_8x32x6,   0, 16 )
	GFXDECODE_ENTRY( "gfx2", 0x0c000, layout_8x32x6,   0, 16 )
	GFXDECODE_ENTRY( "gfx2", 0x00000, layout_8x32x6,   0, 16 )
GFXDECODE_END


/***************************************************************************
                           Machine Start & Reset
***************************************************************************/

void spoker_state::machine_start()
{
	save_item(NAME(m_nmi_ack));
	save_item(NAME(m_out));
	save_item(NAME(m_video_enable));
	save_item(NAME(m_hopper));
	save_item(NAME(m_igs_magic));
}

void spoker_state::machine_reset()
{
	m_nmi_ack = 0;
	m_hopper = 0;
	m_video_enable = 1;
}


/***************************************************************************
                              Machine Drivers
***************************************************************************/

static MACHINE_CONFIG_START( spoker )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z180, XTAL_12MHz / 2)   /* HD64180RP8, 8 MHz? */
	MCFG_CPU_PROGRAM_MAP(spoker_map)
	MCFG_CPU_IO_MAP(spoker_portmap)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", spoker_state, nmi_line_assert)

	MCFG_NVRAM_ADD_0FILL("nvram")

	MCFG_DEVICE_ADD("ppi8255_0", I8255A, 0)  // Control 0x8b --> A:out; B:input; C:input.
	MCFG_I8255_OUT_PORTA_CB(WRITE8(spoker_state, nmi_and_coins_w))
	MCFG_I8255_IN_PORTB_CB(IOPORT("SERVICE"))
	MCFG_I8255_IN_PORTC_CB(IOPORT("COINS"))

	MCFG_DEVICE_ADD("ppi8255_1", I8255A, 0)  // Control 0x90 --> A:input; B:out; C:out.
	MCFG_I8255_IN_PORTA_CB(IOPORT("BUTTONS1"))
	MCFG_I8255_OUT_PORTB_CB(WRITE8(spoker_state, video_and_leds_w))
	MCFG_I8255_OUT_PORTC_CB(WRITE8(spoker_state, leds_w))

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(512, 256)
	MCFG_SCREEN_VISIBLE_AREA(0, 512-1, 0, 256-16-1)
	MCFG_SCREEN_UPDATE_DRIVER(spoker_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", spoker)
	MCFG_PALETTE_ADD("palette", 0x400)
	MCFG_PALETTE_FORMAT(xBBBBBGGGGGRRRRR)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("ymsnd", YM2413, XTAL_3_579545MHz)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.4)

	MCFG_OKIM6295_ADD("oki", XTAL_12MHz / 12, PIN7_HIGH)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( 3super8, spoker )

	MCFG_CPU_REPLACE("maincpu", Z80, XTAL_24MHz / 4)    /* z840006, 24/4 MHz? */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(spoker_map)
	MCFG_CPU_IO_MAP(3super8_portmap)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", spoker_state, nmi_line_assert)
	MCFG_CPU_PERIODIC_INT_DRIVER(spoker_state, irq0_line_hold, 120) // this signal comes from the PIC

	MCFG_DEVICE_REMOVE("ppi8255_0")
	MCFG_DEVICE_REMOVE("ppi8255_1")

	MCFG_GFXDECODE_MODIFY("gfxdecode", 3super8)

	MCFG_DEVICE_REMOVE("ymsnd")
MACHINE_CONFIG_END


/***************************************************************************
                                ROMs load
***************************************************************************/

/* Super Poker (IGS)
   US & UA versions.

   Original IGS boards
   IGS PCB-0308-01

   HD64180RP8 (u25)
   IGS026a (u10)
   IGS001a (u30)
   IGS002  (u20)
   U3567   (u42)

   1x 12.288 MHz Xtal. (next to HD64180RP8)

   Programs are encrypted.
   See the last 0x1000 of each one
   to see the patterns...
*/
ROM_START( spk306us )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "27c512_v306_us.u27",   0x0000, 0x10000, CRC(a6c6359e) SHA1(768c53e6c9a4d453e5342a169932fcc30b10fd04) )

	ROM_REGION( 0xc0000, "gfx1", 0 )
	ROM_LOAD( "1_mx28f2000p_v306_us.u33",  0x80000, 0x40000, BAD_DUMP CRC(7ae9b639) SHA1(eb29acf94e96b5a8446dab1e46675766da0538f9) )
	ROM_LOAD( "2_mx28f2000p_v306_us.u32",  0x40000, 0x40000, BAD_DUMP CRC(3a9fc765) SHA1(10ccacf4da189f41b1c0fdc8d943b24ac3464e17) )
	ROM_LOAD( "3_mx28f2000p_v306_us.u31",  0x00000, 0x40000, BAD_DUMP CRC(71f6ea7a) SHA1(f91735d79af153cbbbe82312ba2af789b89c43dd) )

	ROM_REGION( 0x30000, "gfx2", 0 )
	ROM_FILL(                              0x0000, 0x30000, 0xff ) /* filling the whole bank */

	ROM_REGION( 0x40000, "oki", 0 ) /* 4-bit adpcm samples */
	ROM_LOAD( "mx28f2000p_v306_ussp.u34",   0x0000, 0x40000, BAD_DUMP CRC(33e6089d) SHA1(cd1ad01e92c18bbeab3fe3ea9152f8b0a3eb1b29) )
ROM_END

ROM_START( spk205us )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "27c512_v205_us.u27",   0x0000, 0x10000, CRC(4b743c73) SHA1(bd05db13e27dc441e0483b883b770365b2702254) )

	ROM_REGION( 0xc0000, "gfx1", 0 )
	ROM_LOAD( "1_mx28f2000p_v205_us.u33",  0x80000, 0x40000, CRC(7ae9b639) SHA1(eb29acf94e96b5a8446dab1e46675766da0538f9) )
	ROM_LOAD( "2_mx28f2000p_v205_us.u32",  0x40000, 0x40000, CRC(3a9fc765) SHA1(10ccacf4da189f41b1c0fdc8d943b24ac3464e17) )
	ROM_LOAD( "3_mx28f2000p_v205_us.u31",  0x00000, 0x40000, CRC(71f6ea7a) SHA1(f91735d79af153cbbbe82312ba2af789b89c43dd) )

	ROM_REGION( 0x30000, "gfx2", 0 )
	ROM_FILL(                              0x0000, 0x30000, 0xff ) /* filling the whole bank */

	ROM_REGION( 0x40000, "oki", 0 ) /* 4-bit adpcm samples */
	ROM_LOAD( "mx28f2000p_v205_ussp.u34",   0x0000, 0x40000, CRC(33e6089d) SHA1(cd1ad01e92c18bbeab3fe3ea9152f8b0a3eb1b29) )
ROM_END

ROM_START( spk203us )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "27c512_v203_us.u27",   0x0000, 0x10000, CRC(41328b3d) SHA1(2a4cf0cfdb09e72dabbaf09901cff222847c195a) )

	ROM_REGION( 0xc0000, "gfx1", 0 )
	ROM_LOAD( "1_mx28f2000p_v203_us.u33",  0x80000, 0x40000, CRC(b309e9cf) SHA1(2e3f81c9c654c859c0fd4c0953302c9283e7a4d8) )
	ROM_LOAD( "2_mx28f2000p_v203_us.u32",  0x40000, 0x40000, CRC(05048307) SHA1(38d5ba5522a60ae4f34731ea7bd3e2c16683125d) )
	ROM_LOAD( "3_mx28f2000p_v203_us.u31",  0x00000, 0x40000, CRC(beae217b) SHA1(9bfa69954c42ada88bedb7cedaceff841cb88a58) )

	ROM_REGION( 0x30000, "gfx2", 0 )
	ROM_FILL(                              0x0000, 0x30000, 0xff ) /* filling the whole bank */

	ROM_REGION( 0x40000, "oki", 0 ) /* 4-bit adpcm samples */
	ROM_LOAD( "mx28f2000p_v203_ussp.u34",   0x0000, 0x40000, CRC(33e6089d) SHA1(cd1ad01e92c18bbeab3fe3ea9152f8b0a3eb1b29) )
ROM_END

ROM_START( spk200ua )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "27c512_v200_ua.u27",   0x0000, 0x10000, CRC(f4572b88) SHA1(b1f845b5340639eee1464acb8a40241868a21070) )

	ROM_REGION( 0xc0000, "gfx1", 0 )
	ROM_LOAD( "1_mx28f2000p_v200_ua.u33",  0x80000, 0x40000, CRC(b309e9cf) SHA1(2e3f81c9c654c859c0fd4c0953302c9283e7a4d8) )
	ROM_LOAD( "2_mx28f2000p_v200_ua.u32",  0x40000, 0x40000, CRC(05048307) SHA1(38d5ba5522a60ae4f34731ea7bd3e2c16683125d) )
	ROM_LOAD( "3_mx28f2000p_v200_ua.u31",  0x00000, 0x40000, CRC(beae217b) SHA1(9bfa69954c42ada88bedb7cedaceff841cb88a58) )

	ROM_REGION( 0x30000, "gfx2", 0 )
	ROM_FILL(                              0x0000, 0x30000, 0xff ) /* filling the whole bank */

	ROM_REGION( 0x40000, "oki", 0 ) /* 4-bit adpcm samples */
	ROM_LOAD( "mx28f2000p_v200_uasp.u34",   0x0000, 0x40000, CRC(33e6089d) SHA1(cd1ad01e92c18bbeab3fe3ea9152f8b0a3eb1b29) )
ROM_END

ROM_START( spk102ua )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "27c512_v102_ua.u27",   0x0000, 0x10000, CRC(ec5e9f6d) SHA1(5d7a86f8faef7a4b7a9dde040b00b987ffb09479) )

	ROM_REGION( 0xc0000, "gfx1", 0 )
	ROM_LOAD( "1_mx28f2000p_v102_ua.u33",  0x80000, 0x40000, CRC(b309e9cf) SHA1(2e3f81c9c654c859c0fd4c0953302c9283e7a4d8) )
	ROM_LOAD( "2_mx28f2000p_v102_ua.u32",  0x40000, 0x40000, CRC(05048307) SHA1(38d5ba5522a60ae4f34731ea7bd3e2c16683125d) )
	ROM_LOAD( "3_mx28f2000p_v102_ua.u31",  0x00000, 0x40000, CRC(beae217b) SHA1(9bfa69954c42ada88bedb7cedaceff841cb88a58) )

	ROM_REGION( 0x30000, "gfx2", 0 )
	ROM_FILL(                              0x0000, 0x30000, 0xff ) /* filling the whole bank */

	ROM_REGION( 0x40000, "oki", 0 ) /* 4-bit adpcm samples */
	ROM_LOAD( "mx28f2000p_v102_uasp.u34",   0x0000, 0x40000, CRC(33e6089d) SHA1(cd1ad01e92c18bbeab3fe3ea9152f8b0a3eb1b29) )
ROM_END

/*
   Super Poker
   Italian sets...
*/
ROM_START( spk116it )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "v.bin",   0x0000, 0x10000, CRC(e44e943a)  SHA1(78e32d07e2be9a452be10735641cbcf269068c55) )

	ROM_REGION( 0xc0000, "gfx1", 0 )
	ROM_LOAD( "6_v116it.bin",  0x80000, 0x40000, CRC(55b54b11) SHA1(decf27d40ec842374af02c93d761375690be83a3) )
	ROM_LOAD( "5_v116it.bin",  0x40000, 0x40000, CRC(163f5b64) SHA1(5d3a5c2a64691ee9e2bb3a7c283aa9efa53fb35e) )
	ROM_LOAD( "4_v116it.bin",  0x00000, 0x40000, CRC(ec2c6ac3) SHA1(e0a38da26202d2b9a481060fe5b88a38e284201e) )

	ROM_REGION( 0x30000, "gfx2", 0 )
	ROM_LOAD( "3_v116it.bin",  0x20000, 0x10000, CRC(5f18b012) SHA1(c9a96237eaf3138f136bbaffb29dde0ef568ce73) )
	ROM_LOAD( "2_v116it.bin",  0x10000, 0x10000, CRC(50fc3505) SHA1(ca1e4ee7e0bb59c3bd67727f65054a48000ae7fe) )
	ROM_LOAD( "1_v116it.bin",  0x00000, 0x10000, CRC(28ce630a) SHA1(9b597073d33841e7db2c68bbe9f30b734d7f7b41) )

	ROM_REGION( 0x40000, "oki", 0 ) /* expansion rom - contains backgrounds and pictures charmaps */
	ROM_LOAD( "7_v116it.bin",   0x0000, 0x40000, CRC(67789f1c) SHA1(1bef621b4d6399f76020c6310e2e1c2f861679de) )
ROM_END

ROM_START( spk116itmx )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "v_v116it_mx.bin",  0x0000, 0x10000, CRC(f84eefe3) SHA1(9b3acde4f8f5f635155ed132c10d11bf609da2dd) )

	ROM_REGION( 0xc0000, "gfx1", 0 )
	ROM_LOAD( "6_v116itmx.bin",  0x80000, 0x40000, CRC(55b54b11) SHA1(decf27d40ec842374af02c93d761375690be83a3) )
	ROM_LOAD( "5_v116itmx.bin",  0x40000, 0x40000, CRC(163f5b64) SHA1(5d3a5c2a64691ee9e2bb3a7c283aa9efa53fb35e) )
	ROM_LOAD( "4_v116itmx.bin",  0x00000, 0x40000, CRC(ec2c6ac3) SHA1(e0a38da26202d2b9a481060fe5b88a38e284201e) )

	ROM_REGION( 0x30000, "gfx2", 0 )
	ROM_LOAD( "3_v116itmx.bin",  0x20000, 0x10000, CRC(5f18b012) SHA1(c9a96237eaf3138f136bbaffb29dde0ef568ce73) )
	ROM_LOAD( "2_v116itmx.bin",  0x10000, 0x10000, CRC(50fc3505) SHA1(ca1e4ee7e0bb59c3bd67727f65054a48000ae7fe) )
	ROM_LOAD( "1_v116itmx.bin",  0x00000, 0x10000, CRC(28ce630a) SHA1(9b597073d33841e7db2c68bbe9f30b734d7f7b41) )

	ROM_REGION( 0x40000, "oki", 0 ) /* expansion rom - contains backgrounds and pictures charmaps */
	ROM_LOAD( "7_v116itmx.bin",   0x0000, 0x40000, CRC(67789f1c) SHA1(1bef621b4d6399f76020c6310e2e1c2f861679de) )
ROM_END

ROM_START( spk115it )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "v_v115it.bin",   0x0000, 0x10000, CRC(df52997b) SHA1(72a76e84aeedfdebd4c6cb47809117a28b5d3892) ) // sldh

	ROM_REGION( 0xc0000, "gfx1", 0 )
	ROM_LOAD( "6_v115it.bin",  0x80000, 0x40000, CRC(f9b027f8) SHA1(c4686a4024062482f9864e0445087e32899fc775) ) // sldh
	ROM_LOAD( "5_v115it.bin",  0x40000, 0x40000, CRC(baca51b6) SHA1(c97322c814729332378b6304a79062fea385ca97) ) // sldh
	ROM_LOAD( "4_v115it.bin",  0x00000, 0x40000, CRC(1172c790) SHA1(43f1d019ecae5c605722e3fe77ae2f022b01260b) ) // sldh

	ROM_REGION( 0x30000, "gfx2", 0 )
	ROM_LOAD( "3_v115it.bin",  0x20000, 0x10000, CRC(5f18b012) SHA1(c9a96237eaf3138f136bbaffb29dde0ef568ce73) )
	ROM_LOAD( "2_v115it.bin",  0x10000, 0x10000, CRC(50fc3505) SHA1(ca1e4ee7e0bb59c3bd67727f65054a48000ae7fe) )
	ROM_LOAD( "1_v115it.bin",  0x00000, 0x10000, CRC(28ce630a) SHA1(9b597073d33841e7db2c68bbe9f30b734d7f7b41) )

	ROM_REGION( 0x40000, "oki", 0 ) /* expansion rom - contains backgrounds and pictures charmaps */
	ROM_LOAD( "7_v115it.bin",   0x0000, 0x40000, CRC(67789f1c) SHA1(1bef621b4d6399f76020c6310e2e1c2f861679de) )
ROM_END

ROM_START( spk114it )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "v_v114it.bin",  0x0000, 0x10000, CRC(471f5d97) SHA1(94781fb7bd8e9633fafca72de41113c3f21fbf4e) )

	ROM_REGION( 0xc0000, "gfx1", 0 )
	ROM_LOAD( "6_v114it.bin",  0x80000, 0x40000, CRC(466cec74) SHA1(6e4fe56a55f1e230eb01266c67cc82a87f6aaec6) )
	ROM_LOAD( "5_v114it.bin",  0x40000, 0x40000, CRC(ae32a620) SHA1(d5238628269050e21da31d628b1d80b0f9b2d251) )
	ROM_LOAD( "4_v114it.bin",  0x00000, 0x40000, CRC(c2105b1c) SHA1(cd2a930e9c15d1fdcc02ce87c109b5b4107430fa) )

	ROM_REGION( 0x30000, "gfx2", 0 )
	ROM_FILL(                  0x0000, 0x30000, 0xff ) /* filling the whole bank */

	ROM_REGION( 0x40000, "oki", 0 ) /* ADPCM samples */
	ROM_LOAD( "7_v114it.bin",  0x0000, 0x40000, CRC(67789f1c) SHA1(1bef621b4d6399f76020c6310e2e1c2f861679de) )
ROM_END


/*

Produttore  ?Italy?
N.revisione
CPU

1x 24mhz osc
2x fpga
1x z840006
1x PIC16c65a-20/p
1x 6295 oki

ROMs

Note

4x 8 dipswitch
1x 4 dispwitch

*/

// all gfx / sound roms are bad.  they're definitely meant to have different data
//  in each half, and maybe even be twice the size.
//  in all cases the first half is missing (the sample table in the samples rom for example)
//1.bin                                           1ST AND 2ND HALF IDENTICAL
//2.bin                                           1ST AND 2ND HALF IDENTICAL
//3.bin                                           1ST AND 2ND HALF IDENTICAL
//sound.bin                                       1ST AND 2ND HALF IDENTICAL

ROM_START( 3super8 )
	ROM_REGION( 0x20000, "maincpu", 0 )
	ROM_LOAD( "prgrom.bin", 0x00000, 0x20000, CRC(37c85dfe) SHA1(56bd2fb859b17dda1e675a385b6bcd6867ecceb0)  )

	ROM_REGION( 0x1000, "pic", 0 )
	ROM_LOAD( "pic16c65a-20-p", 0x0000, 0x1000, NO_DUMP )

	ROM_REGION( 0xc0000, "gfx1", 0 )
	ROM_LOAD( "1.bin", 0x00000, 0x40000, BAD_DUMP CRC(d9d3e21e) SHA1(2f3f07ca427d9f56f0ff143d15d95cbf15255e33) ) // sldh
	ROM_LOAD( "2.bin", 0x40000, 0x40000, BAD_DUMP CRC(fbb50ab1) SHA1(50a7ef9219c38d59117c510fe6d53fb3ba1fa456) ) // sldh
	ROM_LOAD( "3.bin", 0x80000, 0x40000, BAD_DUMP CRC(545aa4e6) SHA1(3348d4b692900c9e9cd4a52b20922a84e596cd35) ) // sldh
	ROM_FILL( 0x00000 ,0x20000, 0x00 )
	ROM_FILL( 0x40000 ,0x20000, 0x00 )
	ROM_FILL( 0x80000 ,0x20000, 0x00 )

	ROM_REGION( 0x30000, "gfx2", ROMREGION_ERASE00 )

	ROM_REGION( 0xc0000, "rep_gfx", 0 ) //not real, taken from spk116it
	ROM_LOAD( "4.bin",  0x00000, 0x40000, BAD_DUMP CRC(ec2c6ac3) SHA1(e0a38da26202d2b9a481060fe5b88a38e284201e) )
	ROM_LOAD( "5.bin",  0x40000, 0x40000, BAD_DUMP CRC(163f5b64) SHA1(5d3a5c2a64691ee9e2bb3a7c283aa9efa53fb35e) )
	ROM_LOAD( "6.bin",  0x80000, 0x40000, BAD_DUMP CRC(55b54b11) SHA1(decf27d40ec842374af02c93d761375690be83a3) )

	ROM_REGION( 0x40000, "oki", 0 )
	ROM_LOAD( "sound.bin", 0x00000, 0x40000, BAD_DUMP CRC(230b31c3) SHA1(38c107325d3a4e9781912078b1317dc9ba3e1ced) )
ROM_END


/***************************************************************************
                              Driver Init
***************************************************************************/

DRIVER_INIT_MEMBER(spoker_state, spkleftover)
{
/*  The last 4K have the scheme/table for the whole encryption.
    Maybe a leftover...
*/
	int A, B;
	uint8_t *rom = memregion("maincpu")->base();

	for (A = 0; A < 0x10000; A++)
	{
		B = ((A & 0x0fff) | 0xf000);
		rom[A] = rom[A] ^ rom[B];
	}
}

DRIVER_INIT_MEMBER(spoker_state, spk116it)
{
	int A;
	uint8_t *rom = memregion("maincpu")->base();

	for (A = 0; A < 0x10000; A++)
	{
		rom[A] ^= 0x02;
		if ((A & 0x0208) == 0x0208) rom[A] ^= 0x20;
		if ((A & 0x0228) == 0x0008) rom[A] ^= 0x20;
		if ((A & 0x04A0) == 0x04A0) rom[A] ^= 0x02;
		if ((A & 0x1208) == 0x1208) rom[A] ^= 0x01;
	}
}

DRIVER_INIT_MEMBER(spoker_state, 3super8)
{
	uint8_t *ROM = memregion("maincpu")->base();
	int i;

	/* Decryption is probably done using one macrocell/output on an address decoding pal which we do not have a dump of */
	/* The encryption is quite awful actually, especially since the program rom is entirely blank/0xFF but encrypted on its second half, exposing the entire function in plaintext */
	/* Input: A6, A7, A8, A9, A11; Output: D5 XOR */
	/* function: (A6&A8)&((!A7&A11)|(A9&!A11)); */
	/* nor-reduced: !(!(!(!A6|!A8))|!(!(A7|!A11)|!(!A9|A11))); */
	for(i=0;i<0x20000;i++)
	{
		uint8_t a6, a7, a8, a9, a11, d5 = 0;
		a6 = BIT(i,6); a7 = BIT(i,7); a8 = BIT(i,8); a9 = BIT(i,9); a11 = BIT(i,11);
		d5 = (a6 & a8) & ((~a7 & a11) | (a9 & ~a11));
		ROM[i] ^= d5*0x20;
	}

	/* cheesy hack: take gfx roms from spk116it and rearrange them for this game needs */
	{
		uint8_t *src = memregion("rep_gfx")->base();
		uint8_t *dst = memregion("gfx1")->base();
		uint8_t x;

		for(x=0;x<3;x++)
		{
			for(i=0;i<0x20000;i+=4)
			{
				dst[i+0+x*0x40000] = src[i+0+x*0x40000];
				dst[i+1+x*0x40000] = src[i+2+x*0x40000];
				dst[i+2+x*0x40000] = src[i+1+x*0x40000];
				dst[i+3+x*0x40000] = src[i+3+x*0x40000];
			}
		}
	}
}


/***************************************************************************
                              Game Drivers
***************************************************************************/

/*    YEAR   NAME        PARENT    MACHINE  INPUT    STATE          INIT         ROT     COMPANY      FULLNAME                   FLAGS  */
GAME( 1996,  spk306us,   0,        spoker,  spoker,  spoker_state,  spkleftover, ROT0,  "IGS",       "Super Poker (v306US)",     MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )  // needs proper machine driver
GAME( 1996,  spk205us,   spk306us, spoker,  spoker,  spoker_state,  spkleftover, ROT0,  "IGS",       "Super Poker (v205US)",     MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )  // needs proper machine driver
GAME( 1996,  spk203us,   spk306us, spoker,  spoker,  spoker_state,  spkleftover, ROT0,  "IGS",       "Super Poker (v203US)",     MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )  // needs proper machine driver
GAME( 1996,  spk200ua,   spk306us, spoker,  spoker,  spoker_state,  spkleftover, ROT0,  "IGS",       "Super Poker (v200UA)",     MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )  // needs proper machine driver
GAME( 1993?, spk116it,   spk306us, spoker,  spoker,  spoker_state,  spk116it,    ROT0,  "IGS",       "Super Poker (v116IT)",     MACHINE_SUPPORTS_SAVE )
GAME( 1993?, spk116itmx, spk306us, spoker,  spoker,  spoker_state,  0,           ROT0,  "IGS",       "Super Poker (v116IT-MX)",  MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )  // needs proper machine driver
GAME( 1993?, spk115it,   spk306us, spoker,  spoker,  spoker_state,  spk116it,    ROT0,  "IGS",       "Super Poker (v115IT)",     MACHINE_SUPPORTS_SAVE )
GAME( 1993?, spk114it,   spk306us, spoker,  spoker,  spoker_state,  0,           ROT0,  "IGS",       "Super Poker (v114IT)",     MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )  // needs proper machine driver
GAME( 1996,  spk102ua,   spk306us, spoker,  spoker,  spoker_state,  spkleftover, ROT0,  "IGS",       "Super Poker (v102UA)",     MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )  // needs proper machine driver
GAME( 1993?, 3super8,    0,        3super8, 3super8, spoker_state,  3super8,     ROT0,  "<unknown>", "3 Super 8 (Italy)",        MACHINE_NOT_WORKING | MACHINE_IMPERFECT_GRAPHICS | MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE ) //roms are badly dumped
