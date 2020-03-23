// license:BSD-3-Clause
// copyright-holders:Angelo Salese
/***************************************************************************

    Last Bank (c) 1994 Excellent System

    driver by Angelo Salese

    Uses a TC0091LVC, a variant of the one used on Taito L HW

***************************************************************************/

#include "emu.h"
#include "cpu/z80/z80.h"
#include "sound/2203intf.h"
#include "sound/es8712.h"
#include "sound/okim6295.h"
#include "machine/nvram.h"
#include "machine/tc009xlvc.h"
#include "screen.h"
#include "speaker.h"


#define MASTER_CLOCK XTAL_14_31818MHz

class lastbank_state : public driver_device
{
public:
	lastbank_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_vdp(*this, "tc0091lvc"),
		m_oki(*this, "oki"),
		m_essnd(*this, "essnd")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<tc0091lvc_device> m_vdp;
	required_device<okim6295_device> m_oki;
	required_device<es8712_device> m_essnd;

	virtual void video_start() override;
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE_LINE_MEMBER(screen_vblank);

	uint8_t m_ram_bank[4];
	uint8_t m_rom_bank;
	uint8_t m_irq_vector[3];
	uint8_t m_irq_enable;
	uint8_t m_mux_data;
	uint8_t m_soundlatch[2];
	uint8_t m_sound_flags;

	DECLARE_READ8_MEMBER(lastbank_rom_r);

	DECLARE_READ8_MEMBER(lastbank_ram_0_r);
	DECLARE_READ8_MEMBER(lastbank_ram_1_r);
	DECLARE_READ8_MEMBER(lastbank_ram_2_r);
	DECLARE_READ8_MEMBER(lastbank_ram_3_r);
	DECLARE_WRITE8_MEMBER(lastbank_ram_0_w);
	DECLARE_WRITE8_MEMBER(lastbank_ram_1_w);
	DECLARE_WRITE8_MEMBER(lastbank_ram_2_w);
	DECLARE_WRITE8_MEMBER(lastbank_ram_3_w);

	DECLARE_WRITE8_MEMBER(output_w);

	DECLARE_READ8_MEMBER(mux_0_r);
	DECLARE_WRITE8_MEMBER(mux_w);
	DECLARE_WRITE8_MEMBER(soundlatch_w);

	DECLARE_READ8_MEMBER(soundlatch1_r);
	DECLARE_READ8_MEMBER(soundlatch2_r);
	DECLARE_WRITE8_MEMBER(sound_flags_w);
	DECLARE_CUSTOM_INPUT_MEMBER(sound_status_r);

	DECLARE_READ8_MEMBER(lastbank_rom_bank_r);
	DECLARE_WRITE8_MEMBER(lastbank_rom_bank_w);
	DECLARE_READ8_MEMBER(lastbank_ram_bank_r);
	DECLARE_WRITE8_MEMBER(lastbank_ram_bank_w);
	DECLARE_READ8_MEMBER(lastbank_irq_vector_r);
	DECLARE_WRITE8_MEMBER(lastbank_irq_vector_w);
	DECLARE_READ8_MEMBER(lastbank_irq_enable_r);
	DECLARE_WRITE8_MEMBER(lastbank_irq_enable_w);

	uint8_t ram_bank_r(uint16_t offset, uint8_t bank_num);
	void ram_bank_w(uint16_t offset, uint8_t data, uint8_t bank_num);
	TIMER_DEVICE_CALLBACK_MEMBER(lastbank_irq_scanline);
};

void lastbank_state::video_start()
{
}

uint32_t lastbank_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(0, cliprect);

	m_vdp->screen_update(screen, bitmap, cliprect);

	return 0;
}

WRITE_LINE_MEMBER(lastbank_state::screen_vblank)
{
	if (state)
	{
		m_vdp->screen_eof();
	}
}

READ8_MEMBER(lastbank_state::lastbank_rom_r)
{
	uint8_t *ROM = memregion("maincpu")->base();

	return ROM[offset + m_rom_bank * 0x2000];
}

READ8_MEMBER(lastbank_state::lastbank_rom_bank_r)
{
	return m_rom_bank;
}

WRITE8_MEMBER(lastbank_state::lastbank_rom_bank_w)
{
	m_rom_bank = data;
}

READ8_MEMBER(lastbank_state::lastbank_irq_vector_r)
{
	return m_irq_vector[offset];
}

WRITE8_MEMBER(lastbank_state::lastbank_irq_vector_w)
{
	m_irq_vector[offset] = data;
}

READ8_MEMBER(lastbank_state::lastbank_irq_enable_r)
{
	return m_irq_enable;
}

WRITE8_MEMBER(lastbank_state::lastbank_irq_enable_w)
{
	m_irq_enable = data;
}

READ8_MEMBER(lastbank_state::lastbank_ram_bank_r)
{
	return m_ram_bank[offset];
}

WRITE8_MEMBER(lastbank_state::lastbank_ram_bank_w)
{
	m_ram_bank[offset] = data;
}

uint8_t lastbank_state::ram_bank_r(uint16_t offset, uint8_t bank_num)
{
	address_space &vdp_space = machine().device<tc0091lvc_device>("tc0091lvc")->space();
	return vdp_space.read_byte(offset + (m_ram_bank[bank_num]) * 0x1000);;
}

void lastbank_state::ram_bank_w(uint16_t offset, uint8_t data, uint8_t bank_num)
{
	address_space &vdp_space = machine().device<tc0091lvc_device>("tc0091lvc")->space();
	vdp_space.write_byte(offset + (m_ram_bank[bank_num]) * 0x1000,data);;
}

READ8_MEMBER(lastbank_state::lastbank_ram_0_r) { return ram_bank_r(offset, 0); }
READ8_MEMBER(lastbank_state::lastbank_ram_1_r) { return ram_bank_r(offset, 1); }
READ8_MEMBER(lastbank_state::lastbank_ram_2_r) { return ram_bank_r(offset, 2); }
READ8_MEMBER(lastbank_state::lastbank_ram_3_r) { return ram_bank_r(offset, 3); }
WRITE8_MEMBER(lastbank_state::lastbank_ram_0_w) { ram_bank_w(offset, data, 0); }
WRITE8_MEMBER(lastbank_state::lastbank_ram_1_w) { ram_bank_w(offset, data, 1); }
WRITE8_MEMBER(lastbank_state::lastbank_ram_2_w) { ram_bank_w(offset, data, 2); }
WRITE8_MEMBER(lastbank_state::lastbank_ram_3_w) { ram_bank_w(offset, data, 3); }



READ8_MEMBER(lastbank_state::mux_0_r)
{
	const char *const keynames[2][5] = {
		{"P1_KEY0", "P1_KEY1", "P1_KEY2", "P1_KEY3", "P1_KEY4"},
		{"P2_KEY0", "P2_KEY1", "P2_KEY2", "P2_KEY3", "P2_KEY4"} };
	uint8_t res;
	int i;

	res = 0xff;

	for(i=0;i<5;i++)
	{
		if(m_mux_data & 1 << i)
			res = ioport(keynames[0][i])->read();
	}

	return res;
}

WRITE8_MEMBER(lastbank_state::output_w)
{
	switch (offset)
	{
		case 0:
		case 1:
			//logerror("%s: Writing %02x to A80%x\n", machine().describe_context(), data, offset);
			break;

		case 2:
			machine().bookkeeping().coin_counter_w(0, BIT(data, 0)); // coin 1
			machine().bookkeeping().coin_counter_w(1, BIT(data, 2)); // coin 2
			machine().bookkeeping().coin_counter_w(2, BIT(data, 3)); // coin 3
			machine().bookkeeping().coin_counter_w(3, BIT(data, 1)); // key in
			break;
	}
}

WRITE8_MEMBER(lastbank_state::mux_w)
{
	m_mux_data = data;
}

WRITE8_MEMBER(lastbank_state::soundlatch_w)
{
	m_soundlatch[offset] = data;
}

READ8_MEMBER(lastbank_state::soundlatch1_r)
{
	return m_soundlatch[0];
}

READ8_MEMBER(lastbank_state::soundlatch2_r)
{
	return m_soundlatch[1];
}

WRITE8_MEMBER(lastbank_state::sound_flags_w)
{
	m_sound_flags = data;
	if (!BIT(data, 4))
		m_essnd->reset();
	if (!BIT(data, 5))
		m_oki->reset();
}

CUSTOM_INPUT_MEMBER(lastbank_state::sound_status_r)
{
	return BIT(m_sound_flags, 0) << 1 | BIT(m_sound_flags, 1);
}

static ADDRESS_MAP_START( tc0091lvc_map, AS_PROGRAM, 8, lastbank_state )
	AM_RANGE(0x0000, 0x5fff) AM_ROM
	AM_RANGE(0x6000, 0x7fff) AM_READ(lastbank_rom_r)

	AM_RANGE(0x8000, 0x9fff) AM_RAM AM_SHARE("nvram")

	AM_RANGE(0xc000, 0xcfff) AM_READWRITE(lastbank_ram_0_r,lastbank_ram_0_w)
	AM_RANGE(0xd000, 0xdfff) AM_READWRITE(lastbank_ram_1_r,lastbank_ram_1_w)
	AM_RANGE(0xe000, 0xefff) AM_READWRITE(lastbank_ram_2_r,lastbank_ram_2_w)
	AM_RANGE(0xf000, 0xfdff) AM_READWRITE(lastbank_ram_3_r,lastbank_ram_3_w)

	AM_RANGE(0xfe00, 0xfeff) AM_DEVREADWRITE("tc0091lvc", tc0091lvc_device, vregs_r, vregs_w)
	AM_RANGE(0xff00, 0xff02) AM_READWRITE(lastbank_irq_vector_r, lastbank_irq_vector_w)
	AM_RANGE(0xff03, 0xff03) AM_READWRITE(lastbank_irq_enable_r, lastbank_irq_enable_w)
	AM_RANGE(0xff04, 0xff07) AM_READWRITE(lastbank_ram_bank_r, lastbank_ram_bank_w)
	AM_RANGE(0xff08, 0xff08) AM_READWRITE(lastbank_rom_bank_r, lastbank_rom_bank_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( lastbank_map, AS_PROGRAM, 8, lastbank_state )
	AM_IMPORT_FROM( tc0091lvc_map )
	AM_RANGE(0xa000, 0xa00d) AM_NOP // MSM62X42B or equivalent probably read from here
	AM_RANGE(0xa800, 0xa800) AM_READ_PORT("COINS")
	AM_RANGE(0xa800, 0xa802) AM_WRITE(output_w)
	AM_RANGE(0xa803, 0xa803) AM_WRITE(mux_w) // mux for $a808 / $a80c
	AM_RANGE(0xa804, 0xa804) AM_READ_PORT("SPECIAL")
	AM_RANGE(0xa805, 0xa806) AM_WRITE(soundlatch_w)
	AM_RANGE(0xa807, 0xa807) AM_WRITENOP // hopper?
	AM_RANGE(0xa808, 0xa808) AM_READ(mux_0_r)
	AM_RANGE(0xa80c, 0xa80c) AM_READ(mux_0_r)
	AM_RANGE(0xa81c, 0xa81c) AM_READ_PORT("DSW0")
	AM_RANGE(0xa81d, 0xa81d) AM_READ_PORT("DSW1")
	AM_RANGE(0xa81e, 0xa81e) AM_READ_PORT("DSW2")
	AM_RANGE(0xa81f, 0xa81f) AM_READ_PORT("DSW3")
ADDRESS_MAP_END

static ADDRESS_MAP_START( lastbank_audio_map, AS_PROGRAM, 8, lastbank_state )
	AM_RANGE(0x0000, 0xbfff) AM_ROM
	AM_RANGE(0xc000, 0xdfff) AM_RAM
	AM_RANGE(0xe000, 0xe7ff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( lastbank_audio_io, AS_IO, 8, lastbank_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x06) AM_DEVREADWRITE("essnd", es8712_device, read, write)
	AM_RANGE(0x40, 0x40) AM_DEVREADWRITE("oki", okim6295_device, read, write)
	AM_RANGE(0x80, 0x80) AM_READ(soundlatch1_r) AM_WRITE(sound_flags_w)
	AM_RANGE(0xc0, 0xc0) AM_READ(soundlatch2_r)
ADDRESS_MAP_END

static INPUT_PORTS_START( lastbank )
	PORT_START("COINS")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_GAMBLE_KEYOUT ) PORT_CODE(KEYCODE_M)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_GAMBLE_PAYOUT )
	PORT_SERVICE_NO_TOGGLE( 0x04, IP_ACTIVE_LOW )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_GAMBLE_BOOK )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_COIN3 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_GAMBLE_KEYIN ) PORT_CODE(KEYCODE_N)
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_COIN1 )

	PORT_START("SPECIAL")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_MEMORY_RESET )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_GAMBLE_SERVICE )
	PORT_DIPNAME( 0x04, 0x04, "Hopper Count" )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, "Hopper Empty" )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_BIT( 0x30, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0xc0, IP_ACTIVE_HIGH, IPT_SPECIAL ) PORT_CUSTOM_MEMBER(DEVICE_SELF, lastbank_state, sound_status_r, nullptr)

	PORT_START("P1_KEY0")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("5-6") PORT_CODE(KEYCODE_B)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("3-4") PORT_CODE(KEYCODE_G)
	PORT_BIT( 0x0c, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("4-6") PORT_CODE(KEYCODE_V)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("4-5") PORT_CODE(KEYCODE_C)
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("P1_KEY1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_GAMBLE_LOW ) PORT_NAME("Small") PORT_CODE(KEYCODE_Y)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("1-4") PORT_CODE(KEYCODE_E)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("FF") PORT_CODE(KEYCODE_MINUS)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("2-4") PORT_CODE(KEYCODE_S)
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("P1_KEY2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_GAMBLE_HIGH ) PORT_NAME("Big") PORT_CODE(KEYCODE_U)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("Cancel") PORT_CODE(KEYCODE_SPACE)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("1-2") PORT_CODE(KEYCODE_Q)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("3-5") PORT_CODE(KEYCODE_Z)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("2-6") PORT_CODE(KEYCODE_F)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("1-6") PORT_CODE(KEYCODE_T)
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("P1_KEY3")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_GAMBLE_D_UP )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("1-5") PORT_CODE(KEYCODE_R)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_OTHER ) PORT_NAME("Payout 2") PORT_CODE(KEYCODE_O)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("2-5") PORT_CODE(KEYCODE_D)
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("P1_KEY4")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_GAMBLE_TAKE )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_GAMBLE_BET ) PORT_NAME("Auto Bet") PORT_CODE(KEYCODE_2)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("1-3") PORT_CODE(KEYCODE_W)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("3-6") PORT_CODE(KEYCODE_X)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("2-3") PORT_CODE(KEYCODE_A)
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("DSW0")
	PORT_DIPNAME( 0x01, 0x01, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW1:1")
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW1:2")
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW1:3")
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW1:4")
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW1:5")
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW1:6")
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW1:7")
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW1:8")
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("DSW1")
	PORT_DIPNAME( 0x01, 0x01, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW2:1")
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW2:2")
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW2:3")
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW2:4")
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW2:5")
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW2:6")
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Demo_Sounds ) ) PORT_DIPLOCATION("DSW2:7")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x40, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW2:8")
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x0f, 0x0f, DEF_STR( Coin_A ) ) PORT_DIPLOCATION("DSW3:1,2,3,4")
	PORT_DIPSETTING(    0x07, "1 Coin /100 Credits" )
	PORT_DIPSETTING(    0x08, "1 Coin /50 Credits" )
	PORT_DIPSETTING(    0x09, "1 Coin /25 Credits" )
	PORT_DIPSETTING(    0x0a, "1 Coin /20 Credits" )
	PORT_DIPSETTING(    0x0b, "1 Coin /10 Credits" )
	PORT_DIPSETTING(    0x0c, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(    0x0d, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x01, "5 Coins /2 Credits" )
	PORT_DIPSETTING(    0x0e, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x05, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(    0x0f, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x06, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x03, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x02, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x00, "10 Coins /1 Credit" )
	PORT_DIPNAME( 0xf0, 0xf0, "Coin C" ) PORT_DIPLOCATION("DSW3:5,6,7,8")
	PORT_DIPSETTING(    0x70, "1 Coin /100 Credits" )
	PORT_DIPSETTING(    0x80, "1 Coin /50 Credits" )
	PORT_DIPSETTING(    0x90, "1 Coin /25 Credits" )
	PORT_DIPSETTING(    0xa0, "1 Coin /20 Credits" )
	PORT_DIPSETTING(    0xb0, "1 Coin /10 Credits" )
	PORT_DIPSETTING(    0xc0, DEF_STR( 1C_5C ) )
	PORT_DIPSETTING(    0xd0, DEF_STR( 1C_3C ) )
	PORT_DIPSETTING(    0x10, "5 Coins /2 Credits" )
	PORT_DIPSETTING(    0xe0, DEF_STR( 1C_2C ) )
	PORT_DIPSETTING(    0x50, DEF_STR( 2C_3C ) )
	PORT_DIPSETTING(    0xf0, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x60, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x40, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x30, DEF_STR( 4C_1C ) )
	PORT_DIPSETTING(    0x20, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x00, "10 Coins /1 Credit" )

	PORT_START("DSW3")
	PORT_DIPNAME( 0x01, 0x01, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW4:1")
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW4:2")
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW4:3")
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x38, 0x38, DEF_STR( Coin_B ) ) PORT_DIPLOCATION("DSW4:4,5,6")
	PORT_DIPSETTING(    0x30, "1 Coin /50 Credits" )
	PORT_DIPSETTING(    0x38, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x28, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x20, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x18, DEF_STR( 5C_1C ) )
	PORT_DIPSETTING(    0x10, DEF_STR( 6C_1C ) )
	PORT_DIPSETTING(    0x08, "9 Coins /1 Credit" )
	PORT_DIPSETTING(    0x00, "10 Coins /1 Credit" )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW4:7")
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) ) PORT_DIPLOCATION("DSW4:8")
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
INPUT_PORTS_END

static const gfx_layout bg2_layout =
{
	8, 8,
	RGN_FRAC(1,1),
	4,
	{ 8, 12, 0, 4 },
	{ 3, 2, 1, 0, 19, 18, 17, 16 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32 },
	8*8*4
};

#define O 8*8*4
#define O2 2*O
static const gfx_layout sp2_layout =
{
	16, 16,
	RGN_FRAC(1,1),
	4,
	{ 8, 12, 0, 4 },
	{ 3, 2, 1, 0, 19, 18, 17, 16, O+3, O+2, O+1, O+0, O+19, O+18, O+17, O+16 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32, O2+0*32, O2+1*32, O2+2*32, O2+3*32, O2+4*32, O2+5*32, O2+6*32, O2+7*32 },
	8*8*4*4
};
#undef O
#undef O2


static GFXDECODE_START( lastbank )
	GFXDECODE_ENTRY( "gfx1",        0, bg2_layout, 0, 16 )
	GFXDECODE_ENTRY( "gfx1",        0, sp2_layout, 0, 16 )
GFXDECODE_END

TIMER_DEVICE_CALLBACK_MEMBER(lastbank_state::lastbank_irq_scanline)
{
	int scanline = param;

	if (scanline == 240 && (m_irq_enable & 4))
	{
		m_maincpu->set_input_line_and_vector(0, HOLD_LINE, m_irq_vector[2]);
	}

	if (scanline == 0 && (m_irq_enable & 2))
	{
		m_maincpu->set_input_line_and_vector(0, HOLD_LINE, m_irq_vector[1]);
	}
}

static MACHINE_CONFIG_START( lastbank )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu",Z80,MASTER_CLOCK/4) //!!! TC0091LVC !!!
	MCFG_CPU_PROGRAM_MAP(lastbank_map)
	MCFG_TIMER_DRIVER_ADD_SCANLINE("scantimer", lastbank_state, lastbank_irq_scanline, "screen", 0, 1)

	MCFG_NVRAM_ADD_0FILL("nvram")

	MCFG_CPU_ADD("audiocpu",Z80,MASTER_CLOCK/4)
	MCFG_CPU_PROGRAM_MAP(lastbank_audio_map)
	MCFG_CPU_IO_MAP(lastbank_audio_io)
	// yes, we have no interrupts

	MCFG_QUANTUM_PERFECT_CPU("maincpu")

	//MCFG_MACHINE_START_OVERRIDE(lastbank_state,lastbank)
	//MCFG_MACHINE_RESET_OVERRIDE(lastbank_state,lastbank)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500))
	MCFG_SCREEN_SIZE(64*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 40*8-1, 2*8, 30*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(lastbank_state, screen_update)
	MCFG_SCREEN_VBLANK_CALLBACK(WRITELINE(lastbank_state, screen_vblank))
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", lastbank )
	MCFG_PALETTE_ADD("palette", 0x100)

	MCFG_DEVICE_ADD("tc0091lvc", TC0091LVC, 0)
	MCFG_TC0091LVC_GFXDECODE("gfxdecode")

//  MCFG_VIDEO_START_OVERRIDE(lastbank_state,lastbank)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_OKIM6295_ADD("oki", 1000000, PIN7_HIGH)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.75)

	MCFG_ES8712_ADD("essnd", 12000)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)

	// A RTC-62421 is present on the Last Bank PCB. However, the code
	// that tries to read from it is broken and nonfunctional. The RTC
	// is also absent from some other games on the same hardware.
MACHINE_CONFIG_END

/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( lastbank )
	ROM_REGION( 0x40000, "maincpu", 0 )
	ROM_LOAD( "3.u9", 0x00000, 0x40000, CRC(f430e1f0) SHA1(dd5b697f5c2250d98911f4c7d3e7d4cc16b0b40f) )

	ROM_REGION( 0x40000, "audiocpu", 0 )
	ROM_LOAD( "8.u48", 0x00000, 0x10000, CRC(3a7bfe10) SHA1(7dc543e11d3c0b9872fcc622339ade25383a1eb3) )

	ROM_REGION( 0x120000, "gfx1", 0 )
	ROM_LOAD( "u11",   0x000000, 0x100000, CRC(2588d82d) SHA1(426f6821862d54123e53410e2776586ddf6b21e7) )
	ROM_LOAD( "5.u10", 0x100000, 0x020000, CRC(51f3c5a7) SHA1(73d4c8817fe96d75be32c43e816e93c52b5d2b27) )

	ROM_REGION( 0x40000, "oki", 0 )
	ROM_LOAD( "6.u55", 0x00000, 0x40000, CRC(9e78e234) SHA1(031f93e4bc338d0257fa673da7ce656bb1cda5fb) )

	ROM_REGION( 0x80000, "essnd", 0 ) /* Samples */
	ROM_LOAD( "7.u60", 0x00000, 0x80000, CRC(41be7146) SHA1(00f1c0d5809efccf888e27518a2a5876c4b633d8) )
ROM_END

GAME( 1994, lastbank,  0,   lastbank, lastbank, lastbank_state,  0, ROT0, "Excellent System", "Last Bank (v1.16)", 0 )
