// license:BSD-3-Clause
// copyright-holders:Luca Elia
/***************************************************************************

Quiz Punch II (C)1989 Space Computer

Driver by Luca Elia

- It uses an unknown DIP40 device for protection, that supplies
  the address to jump to (same as mosaic.cpp) and handles the EEPROM

PCB Layout
----------

|---------------------------------------------|
|U1    U26        6116                   32MHz|
|                             U120 U119       |
|U2                                  U118 U117|
|      U27        6116                        |
|                                             |
|                                             |
|      U28        6116        Z80             |
|                         U2A     93C46   U111|
|                                      6264   |
|      U29                    *           8MHz|
|  U20            6116                        |
|                                6116         |
|  U21 U30                                    |
|                 6116                        |
|  U22  6116                           DSW1(8)|
|VOL    YM2203 Z80                            |
|    YM3014     MAHJONG28                     |
|---------------------------------------------|
Notes:
      * - Unknown DIP40 chip. +5V on pin 17, GND on pin 22
          pins 4,3,2 of 93C46 tied to unknown chip on pins 23,24,25
      All clocks unknown, PCB not working
      Possibly Z80's @ 4MHz and YM2203 @ 2MHz
      PCB marked 'Ducksan Trading Co. Ltd. Made In Korea'

***************************************************************************/
/***************************************************************************

Quiz Punch (C)1988 Space Computer
Ducksan 1989

88-01-14-0775
|--------------------------------------------------|
|VOL  MC1455                        02.U2   01.U1  |
|UPC1241                                           |
|LM358  05.U22 04.U21  03.U20                      |
|YM3014B  6116   10.U30 09.U29 08.U28 07.U27 06.U26|
|YM2203  Z80A                                      |
|                                                  |
|J              6116   6116   6116   6116   6116   |
|A                                                 |
|M                                                 |
|M                       |------------|            |
|A                       |            |            |
|                        |  EPOXY     |            |
|                        |  MODULE    |            |
|                        |            |            |
|    6116       6116     |            |            |
|                        |------------|    14.U120 |
|                               GM76C88    13.U119 |
|                                          12.U118 |
|                                          11.U117 |
|   DSW(8)                   8MHz 15.U111     32MHz|
|--------------------------------------------------|
Notes:
       Z80A - clock 4.000MHz (8/2)
     YM2203 - clock 4.000MHz (8/2)
     VSync - 59.3148Hz
     HSync - 15.2526kHz

     Epoxy Module contains:
      Z80B (input clock 4.000MHz)
      68705P5 MCU
      HY 93C46 EEPROM (upside down)
      4 logic chips

***************************************************************************/

#include "emu.h"

#include "cpu/m6805/m68705.h"
#include "cpu/z80/z80.h"

#include "machine/eepromser.h"
#include "machine/gen_latch.h"

#include "sound/2203intf.h"

#include "screen.h"
#include "speaker.h"


// very preliminary quizpun2 protection simulation

#define VERBOSE_PROTECTION_LOG 0

enum prot_state { STATE_IDLE = 0, STATE_ADDR_R, STATE_ROM_R, STATE_EEPROM_R, STATE_EEPROM_W };
struct prot_t {
	prot_state state;
	int wait_param;
	int param;
	int cmd;
	int addr;
};

class quizpun2_state : public driver_device
{
public:
	quizpun2_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_palette(*this, "palette"),
		m_soundlatch(*this, "soundlatch"),
		m_eeprom(*this, "eeprom"),
		m_fg_ram(*this, "fg_ram"),
		m_bg_ram(*this, "bg_ram")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<gfxdecode_device> m_gfxdecode;
	required_device<palette_device> m_palette;
	required_device<generic_latch_8_device> m_soundlatch;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_shared_ptr<uint8_t> m_fg_ram;
	required_shared_ptr<uint8_t> m_bg_ram;

	tilemap_t *m_bg_tmap;
	tilemap_t *m_fg_tmap;
	uint8_t m_scroll;

	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	TILE_GET_INFO_MEMBER(get_fg_tile_info);
	DECLARE_WRITE8_MEMBER(bg_ram_w);
	DECLARE_WRITE8_MEMBER(fg_ram_w);
	DECLARE_WRITE8_MEMBER(scroll_w);
	DECLARE_WRITE8_MEMBER(rombank_w);
	DECLARE_WRITE8_MEMBER(irq_ack);
	DECLARE_WRITE8_MEMBER(soundlatch_w);

	virtual void machine_reset() override;
	virtual void machine_start() override;
	virtual void video_start() override;
	uint32_t screen_update_quizpun2(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	// quizpun2
	void log_protection( address_space &space, const char *warning );
	struct prot_t m_prot;
	DECLARE_READ8_MEMBER(quizpun2_protection_r);
	DECLARE_WRITE8_MEMBER(quizpun2_protection_w);

	// quizpun
	uint8_t m_port_a, m_port_b;
	bool m_quizpun_pending;
	bool m_quizpun_written;
	bool m_quizpun_repeat;

	DECLARE_READ8_MEMBER(quizpun_68705_port_a_r);
	DECLARE_WRITE8_MEMBER(quizpun_68705_port_a_w);

	DECLARE_READ8_MEMBER(quizpun_68705_port_b_r);
	DECLARE_WRITE8_MEMBER(quizpun_68705_port_b_w);

	DECLARE_READ8_MEMBER(quizpun_68705_port_c_r);
	DECLARE_WRITE8_MEMBER(quizpun_68705_port_c_w);

	DECLARE_READ8_MEMBER(quizpun_protection_r);
	DECLARE_WRITE8_MEMBER(quizpun_protection_w);
};

/***************************************************************************
                                Video Hardware
***************************************************************************/

TILE_GET_INFO_MEMBER(quizpun2_state::get_bg_tile_info)
{
	uint16_t code = m_bg_ram[ tile_index * 2 ] + m_bg_ram[ tile_index * 2 + 1 ] * 256;
	SET_TILE_INFO_MEMBER(0, code, code >> 12, TILE_FLIPXY((code & 0x800) >> 11));
}

TILE_GET_INFO_MEMBER(quizpun2_state::get_fg_tile_info)
{
	uint16_t code  = m_fg_ram[ tile_index * 4 ]/* + m_fg_ram[ tile_index * 4 + 1 ] * 256*/;
	uint8_t  color = m_fg_ram[ tile_index * 4 + 2 ];
	SET_TILE_INFO_MEMBER(1, code, color / 2, 0);
}

WRITE8_MEMBER(quizpun2_state::bg_ram_w)
{
	m_bg_ram[offset] = data;
	m_bg_tmap->mark_tile_dirty(offset/2);
}

WRITE8_MEMBER(quizpun2_state::fg_ram_w)
{
	m_fg_ram[offset] = data;
	m_fg_tmap->mark_tile_dirty(offset/4);
}

WRITE8_MEMBER(quizpun2_state::scroll_w)
{
	m_scroll = data;
}

void quizpun2_state::video_start()
{
	m_bg_tmap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(quizpun2_state::get_bg_tile_info),this), TILEMAP_SCAN_ROWS,16,16,0x20,0x40);
	m_fg_tmap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(quizpun2_state::get_fg_tile_info),this), TILEMAP_SCAN_ROWS,16,16,0x20,0x40);

	m_bg_tmap->set_transparent_pen(0);
	m_fg_tmap->set_transparent_pen(0);
}

uint32_t quizpun2_state::screen_update_quizpun2(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	int layers_ctrl = -1;

#ifdef MAME_DEBUG
	if (machine().input().code_pressed(KEYCODE_Z))
	{
		int msk = 0;
		if (machine().input().code_pressed(KEYCODE_Q))  msk |= 1;
		if (machine().input().code_pressed(KEYCODE_W))  msk |= 2;
		if (msk != 0) layers_ctrl &= msk;
	}
#endif

	int bg_scroll = (m_scroll & 0x3) >> 0;
	int fg_scroll = (m_scroll & 0xc) >> 2;

	m_bg_tmap->set_scrolly(0, bg_scroll * 0x100);
	m_fg_tmap->set_scrolly(0, fg_scroll * 0x100);

	if (layers_ctrl & 1)    m_bg_tmap->draw(screen, bitmap, cliprect, TILEMAP_DRAW_OPAQUE, 0);
	else                    bitmap.fill(m_palette->black_pen(), cliprect);

	if (layers_ctrl & 2)    m_fg_tmap->draw(screen, bitmap, cliprect, 0, 0);

//  popmessage("BG: %x FG: %x", bg_scroll, fg_scroll);

	return 0;
}

/***************************************************************************
                         Quizpun2 Protection Simulation

    ROM checksum:   write 0x80 | (0x00-0x7f), write 0, read 2 bytes
    Read address:   write 0x80 | param1 & 0x07f (0x00), write param2 & 0x7f, read 2 bytes
    Read EEPROM:    write 0x20 | (0x00-0x0f), write 0, read 8 bytes
    Write EEPROM:   write 0x00 | (0x00-0x0f), write 0, write 8 bytes

***************************************************************************/

void quizpun2_state::machine_start()
{
	uint8_t *ROM = memregion("maincpu")->base();
	membank("bank1")->configure_entries(0, 0x20, &ROM[0x10000], 0x2000);
}

void quizpun2_state::machine_reset()
{
	membank("bank1")->set_entry(0);

	// quizpun2
	m_prot.state = STATE_IDLE;
	m_prot.wait_param = 0;
	m_prot.param = 0;
	m_prot.cmd = 0;
	m_prot.addr = 0;

	// quizpun
	m_port_a = m_port_b = 0;
	m_quizpun_pending = m_quizpun_written = m_quizpun_repeat = false;
}

void quizpun2_state::log_protection( address_space &space, const char *warning )
{
	logerror("%04x: protection - %s (state %x, wait %x, param %02x, cmd %02x, addr %02x)\n", space.device().safe_pc(), warning,
		m_prot.state,
		m_prot.wait_param,
		m_prot.param,
		m_prot.cmd,
		m_prot.addr
	);
}

READ8_MEMBER(quizpun2_state::quizpun2_protection_r)
{
	struct prot_t &prot = m_prot;
	uint8_t ret;

	switch ( prot.state )
	{
		case STATE_ROM_R:       // Checksum of MCU addresses 0-ff (0x8e9c^0xffff expected)
			if      (prot.addr == 0xfe) ret = 0x8e ^ 0xff;
			else if (prot.addr == 0xff) ret = 0x9c ^ 0xff;
			else                        ret = 0x00;
			break;

		case STATE_ADDR_R:      // Address to jump to (big endian!)
			switch ( prot.param )
			{
				case 0x19:  // Print
					ret = 0x0b95 >> ((prot.addr & 1) ? 0 : 8);
					break;

				case 0x44:  // Clear screen?
					ret = 0x1bd9 >> ((prot.addr & 1) ? 0 : 8);  // needed, but should also clear the screen
					break;

				case 0x45:  // Backup RAM check
					ret = 0x2242 >> ((prot.addr & 1) ? 0 : 8);
					break;

				default:
					log_protection(space, "unknown address");
					ret = 0x2e59 >> ((prot.addr & 1) ? 0 : 8);  // return the address of: XOR A, RET
			}
			break;

		case STATE_EEPROM_R:        // EEPROM read
		{
			uint8_t *eeprom = memregion("eeprom")->base();
			ret = eeprom[prot.addr];
			break;
		}

		default:
			log_protection(space, "unknown read");
			ret = 0x00;
	}

#if VERBOSE_PROTECTION_LOG
	log_protection(space, "info READ");
#endif

	prot.addr++;

	return ret;
}

WRITE8_MEMBER(quizpun2_state::quizpun2_protection_w)
{
	struct prot_t &prot = m_prot;

	switch ( prot.state )
	{
		case STATE_EEPROM_W:
		{
			uint8_t *eeprom = memregion("eeprom")->base();
			eeprom[prot.addr] = data;
			prot.addr++;
			if ((prot.addr % 8) == 0)
				prot.state = STATE_IDLE;
			break;
		}

		default:
			if (prot.wait_param)
			{
				prot.param = data;
				prot.wait_param = 0;

				// change state:

				if (prot.cmd & 0x80)
				{
					if (prot.param == 0x00)
					{
						prot.state = STATE_ROM_R;
						prot.addr = (prot.cmd & 0x7f) * 2;
					}
					else if (prot.cmd == 0x80)
					{
						prot.state = STATE_ADDR_R;
						prot.addr = 0;
					}
					else
						log_protection(space, "unknown command");
				}
				else if (prot.cmd >= 0x00 && prot.cmd <= 0x0f)
				{
					prot.state = STATE_EEPROM_W;
					prot.addr = (prot.cmd & 0x0f) * 8;
				}
				else if (prot.cmd >= 0x20 && prot.cmd <= 0x2f)
				{
					prot.state = STATE_EEPROM_R;
					prot.addr = (prot.cmd & 0x0f) * 8;
				}
				else
				{
					prot.state = STATE_IDLE;
					log_protection(space, "unknown command");
				}
			}
			else
			{
				prot.cmd = data;
				prot.wait_param = 1;
			}
			break;
	}

#if VERBOSE_PROTECTION_LOG
	log_protection(space, "info WRITE");
#endif
}

/***************************************************************************
                            Memory Maps - Main CPU
***************************************************************************/

WRITE8_MEMBER(quizpun2_state::rombank_w)
{
	membank("bank1")->set_entry(data & 0x1f);
}

WRITE8_MEMBER(quizpun2_state::irq_ack)
{
	m_maincpu->set_input_line(INPUT_LINE_IRQ0, CLEAR_LINE);
}

WRITE8_MEMBER(quizpun2_state::soundlatch_w)
{
	m_soundlatch->write(space, 0, data ^ 0x80);
	m_audiocpu->set_input_line(INPUT_LINE_NMI, PULSE_LINE);
}

static ADDRESS_MAP_START( quizpun2_map, AS_PROGRAM, 8, quizpun2_state )
	AM_RANGE( 0x0000, 0x7fff ) AM_ROM
	AM_RANGE( 0x8000, 0x9fff ) AM_ROMBANK("bank1")

	AM_RANGE( 0xa000, 0xbfff ) AM_RAM_WRITE(fg_ram_w) AM_SHARE("fg_ram")
	AM_RANGE( 0xc000, 0xcfff ) AM_RAM_WRITE(bg_ram_w) AM_SHARE("bg_ram")

	AM_RANGE( 0xd000, 0xd3ff ) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_RANGE( 0xe000, 0xffff ) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( common_io_map, AS_IO, 8, quizpun2_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE( 0x40, 0x40 ) AM_WRITE(irq_ack)
	AM_RANGE( 0x50, 0x50 ) AM_WRITE(soundlatch_w)
	AM_RANGE( 0x60, 0x60 ) AM_WRITE(rombank_w)
	AM_RANGE( 0x70, 0x70 ) AM_WRITE(scroll_w)
	AM_RANGE( 0x80, 0x80 ) AM_READ_PORT( "DSW" )
	AM_RANGE( 0x90, 0x90 ) AM_READ_PORT( "IN0" )
	AM_RANGE( 0xa0, 0xa0 ) AM_READ_PORT( "IN1" )
ADDRESS_MAP_END

static ADDRESS_MAP_START( quizpun2_io_map, AS_IO, 8, quizpun2_state )
	AM_IMPORT_FROM( common_io_map )
	AM_RANGE( 0xe0, 0xe0 ) AM_READWRITE(quizpun2_protection_r, quizpun2_protection_w)
ADDRESS_MAP_END

// quizpun

READ8_MEMBER(quizpun2_state::quizpun_protection_r)
{
//  logerror("%s: port A read %02x\n", machine().describe_context(), m_port_a);

	/*
	   Upon reading this port the main cpu is stalled until the mcu provides the value to read
	   and explicitly un-stalls the z80. Is this possible under the current MAME architecture?

	   ** ghastly hack **

	   The first read stalls the main cpu and triggers the mcu, it returns an incorrect value.
	   It also decrements the main cpu PC back to the start of the read instruction.
	   When the mcu un-stalls the Z80, the read happens again, returning the correct mcu-provided value this time
	*/
	if (m_quizpun_repeat)
	{
		m_quizpun_repeat = false;
	}
	else
	{
		m_quizpun_pending = true;
		m_quizpun_written = false;
		m_maincpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
		m_maincpu->yield();

		m_maincpu->set_state_int(Z80_PC, m_maincpu->state_int(Z80_PC) - 2);
		m_quizpun_repeat = true;
	}

	return m_port_a;
}

WRITE8_MEMBER(quizpun2_state::quizpun_protection_w)
{
//  logerror("%s: port A write %02x\n", machine().describe_context(), data);
	m_port_a = data;
	m_quizpun_pending = true;
	m_quizpun_written = true;
	m_maincpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
	m_maincpu->yield();
}

static ADDRESS_MAP_START( quizpun_io_map, AS_IO, 8, quizpun2_state )
	AM_IMPORT_FROM( common_io_map )
	AM_RANGE( 0xe0, 0xe0 ) AM_READWRITE(quizpun_protection_r, quizpun_protection_w )
ADDRESS_MAP_END

/***************************************************************************
                            Memory Maps - MCU
***************************************************************************/

// Port A - I/O with main cpu (data)

READ8_MEMBER(quizpun2_state::quizpun_68705_port_a_r)
{
//  logerror("%s: port A read %02x\n", machine().describe_context(), m_port_a);
	return m_port_a;
}

WRITE8_MEMBER(quizpun2_state::quizpun_68705_port_a_w)
{
//  logerror("%s: port A write %02x\n", machine().describe_context(), data);
	m_port_a = data;
}

// Port B - I/O with main cpu (status)

READ8_MEMBER(quizpun2_state::quizpun_68705_port_b_r)
{
	// bit 3: 0 = pending
	// bit 1: 0 = main cpu has written
	// bit 0: 0 = main cpu is reading

	uint8_t const ret =
			0xf4 |
			( m_quizpun_pending                        ? 0 : (1 << 3)) |
			((m_quizpun_pending &&  m_quizpun_written) ? 0 : (1 << 1)) |
			((m_quizpun_pending && !m_quizpun_written) ? 0 : (1 << 0));

//  logerror("%s: port B read %02x\n", machine().describe_context(), ret);
	return ret;
}

WRITE8_MEMBER(quizpun2_state::quizpun_68705_port_b_w)
{
//  logerror("%s: port B write %02x\n", machine().describe_context(), data);

	// bit 2: 0->1 run main cpu

	if (!BIT(m_port_b, 2) && BIT(data, 2))
	{
		m_quizpun_pending = false;
		m_maincpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
	}
	m_port_b = data;
}

// Port C - EEPROM

READ8_MEMBER(quizpun2_state::quizpun_68705_port_c_r)
{
	uint8_t const ret = 0xf7 | (m_eeprom->do_read() ? 0x08 : 0x00);
//  logerror("%s: port C read %02x\n", machine().describe_context(), ret);
	return ret;
}

WRITE8_MEMBER(quizpun2_state::quizpun_68705_port_c_w)
{
	// latch the bit
	m_eeprom->di_write(BIT(data, 2));

	// reset line asserted: reset.
	m_eeprom->cs_write(BIT(data, 1) ? ASSERT_LINE : CLEAR_LINE);

	// clock line asserted: write latch or select next bit to read
	m_eeprom->clk_write(BIT(data, 0) ? ASSERT_LINE : CLEAR_LINE);

//  logerror("%s: port C write %02x\n", machine().describe_context(), data);
}

/***************************************************************************
                            Memory Maps - Sound CPU
***************************************************************************/

static ADDRESS_MAP_START( quizpun2_sound_map, AS_PROGRAM, 8, quizpun2_state )
	AM_RANGE( 0x0000, 0xf7ff ) AM_ROM
	AM_RANGE( 0xf800, 0xffff ) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( quizpun2_sound_io_map, AS_IO, 8, quizpun2_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE( 0x00, 0x00 ) AM_WRITENOP  // IRQ end
	AM_RANGE( 0x20, 0x20 ) AM_WRITENOP  // NMI end
	AM_RANGE( 0x40, 0x40 ) AM_DEVREAD("soundlatch", generic_latch_8_device, read)
	AM_RANGE( 0x60, 0x61 ) AM_DEVREADWRITE("ymsnd", ym2203_device, read, write)
ADDRESS_MAP_END

/***************************************************************************
                                Input Ports
***************************************************************************/

static INPUT_PORTS_START( quizpun2 )
	PORT_START("DSW")
	PORT_SERVICE( 0x01, IP_ACTIVE_LOW )
	PORT_DIPUNKNOWN( 0x02, 0x02 )
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Coinage ) )
	PORT_DIPSETTING(    0x00, DEF_STR( 3C_1C ) )
	PORT_DIPSETTING(    0x04, DEF_STR( 2C_1C ) )
	PORT_DIPSETTING(    0x0c, DEF_STR( 1C_1C ) )
	PORT_DIPSETTING(    0x08, DEF_STR( 1C_2C ) )
	PORT_DIPNAME( 0x30, 0x30, "Play Time" )
	PORT_DIPSETTING(    0x30, "6" )
	PORT_DIPSETTING(    0x20, "7" )
	PORT_DIPSETTING(    0x10, "8" )
	PORT_DIPSETTING(    0x00, "9" )
	PORT_DIPNAME( 0xc0, 0xc0, DEF_STR( Lives ) )
	PORT_DIPSETTING(    0xc0, "2" )
	PORT_DIPSETTING(    0x80, "3" )
	PORT_DIPSETTING(    0x40, "4" )
	PORT_DIPSETTING(    0x00, "5" )

	PORT_START("IN0") // port $90
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(2)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(2)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1)
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(1)
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_PLAYER(1)

	PORT_START("IN1") // port $a0
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_COIN1   )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_START2  )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_START1  )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )
INPUT_PORTS_END

/***************************************************************************
                                Graphics Layout
***************************************************************************/

static const gfx_layout layout_16x16x4 =
{
	16, 16,
	RGN_FRAC(1, 1),
	4,
	{ STEP4(0,1) },
	{ 4*1,4*0, 4*3,4*2, 4*5,4*4, 4*7,4*6, 4*9,4*8, 4*11,4*10, 4*13,4*12, 4*15,4*14 },
	{ STEP16(0,16*4) },
	16*16*4
};

static const gfx_layout layout_16x16x1 =
{
	16, 16,
	RGN_FRAC(1, 1),
	1,
	{ 0 },
	{ STEP8(7*1,-1),STEP8(15*1,-1) },
	{ STEP16(0,16*1) },
	16*16*1
};

static GFXDECODE_START( quizpun2 )
	GFXDECODE_ENTRY( "bg",  0, layout_16x16x4,     0, 256/16 )
	GFXDECODE_ENTRY( "fg",  0, layout_16x16x1, 0x100, 256/2  )
	GFXDECODE_ENTRY( "fg2", 0, layout_16x16x1, 0x100, 256/2  )
GFXDECODE_END

/***************************************************************************
                                Machine Drivers
***************************************************************************/

static MACHINE_CONFIG_START( quizpun2 )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, XTAL_8MHz / 2) // 4 MHz
	MCFG_CPU_PROGRAM_MAP(quizpun2_map)
	MCFG_CPU_IO_MAP(quizpun2_io_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", quizpun2_state,  irq0_line_hold)

	MCFG_CPU_ADD("audiocpu", Z80, XTAL_8MHz / 2)    // 4 MHz
	MCFG_CPU_PROGRAM_MAP(quizpun2_sound_map)
	MCFG_CPU_IO_MAP(quizpun2_sound_io_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", quizpun2_state,  irq0_line_hold)
	// NMI generated by main CPU

	MCFG_EEPROM_SERIAL_93C46_ADD("eeprom")

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(384, 256)
	MCFG_SCREEN_VISIBLE_AREA(0, 384-1, 0, 256-1)
	MCFG_SCREEN_UPDATE_DRIVER(quizpun2_state, screen_update_quizpun2)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", quizpun2)
	MCFG_PALETTE_ADD("palette", 0x200)
	MCFG_PALETTE_FORMAT(xRRRRRGGGGGBBBBB)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")

	MCFG_SOUND_ADD("ymsnd", YM2203, XTAL_8MHz / 2) // 4 MHz
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( quizpun, quizpun2 )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_IO_MAP(quizpun_io_map)

	MCFG_CPU_ADD("mcu", M68705P5, XTAL_4MHz) // xtal is 4MHz, divided by 4 internally
	MCFG_M68705_PORTA_R_CB(READ8(quizpun2_state, quizpun_68705_port_a_r))
	MCFG_M68705_PORTB_R_CB(READ8(quizpun2_state, quizpun_68705_port_b_r))
	MCFG_M68705_PORTC_R_CB(READ8(quizpun2_state, quizpun_68705_port_c_r))
	MCFG_M68705_PORTA_W_CB(WRITE8(quizpun2_state, quizpun_68705_port_a_w))
	MCFG_M68705_PORTB_W_CB(WRITE8(quizpun2_state, quizpun_68705_port_b_w))
	MCFG_M68705_PORTC_W_CB(WRITE8(quizpun2_state, quizpun_68705_port_c_w))
MACHINE_CONFIG_END

/***************************************************************************
                                ROMs Loading
***************************************************************************/

ROM_START( quizpun2 )
	ROM_REGION( 0x50000, "maincpu", 0 )
	ROM_LOAD( "u111", 0x00000, 0x08000, CRC(14bdaffc) SHA1(7fb5988ea565d7cbe3c8e2cdb9402d3cf81507d7) )
	ROM_LOAD( "u117", 0x10000, 0x10000, CRC(e9d1d05e) SHA1(c24104e023d12db8c9199d3e18750414aa511e40) )
	ROM_LOAD( "u118", 0x20000, 0x10000, CRC(1f232707) SHA1(3f5f44611f25c556521333f15daf3e2128cc1538) BAD_DUMP ) // fails rom check
	ROM_LOAD( "u119", 0x30000, 0x10000, CRC(c73b82f7) SHA1(d5c683440e9db46dd5859b519b3f32da80352626) )
	ROM_LOAD( "u120", 0x40000, 0x10000, CRC(700648b8) SHA1(dfa824166dfe7361d7c2ab0d8aa1ada882916cb9) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "u22", 0x00000, 0x10000, CRC(f40768b5) SHA1(4410f71850357ec1d10a3a114bb540966e72781b) )

	ROM_REGION( 0x1000, "mcu", 0 )
	ROM_LOAD( "mcu.bin", 0x0000, 0x1000, NO_DUMP )

	ROM_REGION( 0x40000, "bg", 0 )    // 16x16x8
	ROM_LOAD( "u21", 0x00000, 0x10000, CRC(8ac86759) SHA1(2eac9ceee4462ce905aa08ff4f5a6215e0b6672f) )
	ROM_LOAD( "u20", 0x10000, 0x10000, CRC(67640a46) SHA1(5b33850afbb89db9ce9044a578423bfe3a55420d) )
	ROM_LOAD( "u29", 0x20000, 0x10000, CRC(cd8ff05b) SHA1(25e5be914fe49ff96a3c04de0c0e266a79068930) )
	ROM_LOAD( "u30", 0x30000, 0x10000, CRC(8612b443) SHA1(1033a378b21023eca471f43309d49461494b5ea1) )

	ROM_REGION( 0x6000, "fg", 0 ) // 16x16x1
	ROM_LOAD( "u26", 0x1000, 0x1000, CRC(151de8af) SHA1(2159ab030043e69d63cc9fbbc772f5bae8ab3f9d) )
	ROM_CONTINUE(    0x0000, 0x1000 )
	ROM_LOAD( "u27", 0x3000, 0x1000, CRC(2afdafea) SHA1(4c116a1e8a91f2e309646063139763b837e24bc7) )
	ROM_CONTINUE(    0x2000, 0x1000 )
	ROM_LOAD( "u28", 0x5000, 0x1000, CRC(c8bd85ad) SHA1(e7f0882f669edea1bb4634c263872f63da6a3290) ) // 1ST HALF = xx00
	ROM_CONTINUE(    0x4000, 0x1000 )

	ROM_REGION( 0x20000, "fg2", 0 )    // 16x16x1
	ROM_LOAD( "u1", 0x00000, 0x10000, CRC(58506040) SHA1(9d8bed2585e8f188a20270fccd9cfbdb91e48599) )
	ROM_LOAD( "u2", 0x10000, 0x10000, CRC(9294a19c) SHA1(cd7109262e5f68b946c84aa390108bcc47ee1300) )

	ROM_REGION16_BE( 0x80, "eeprom", 0 ) // EEPROM (tied to the unknown DIP40)
	ROM_LOAD( "93c46", 0x00, 0x80, CRC(4d244cc8) SHA1(6593d5b7ac1ebb77fee4648ad1d3d9b59a25fdc8) BAD_DUMP ) // backup ram error

	ROM_REGION( 0x2000, "unknown", 0 )
	ROM_LOAD( "u2a", 0x0000, 0x2000, CRC(13afc2bd) SHA1(0d9c8813525dfc7a844e72d2cf84261db3d10a23) ) // 111xxxxxxxxxx = 0xFF
ROM_END

ROM_START( quizpun )
	ROM_REGION( 0x50000, "maincpu", 0 )
	ROM_LOAD( "15.u111", 0x00000, 0x08000, CRC(0ffe42d9) SHA1(f91e499800923d185a5d3514fc4c50e5c86378bf) )
	ROM_LOAD( "11.u117", 0x10000, 0x10000, CRC(13541476) SHA1(5e81e4143fbc8fa68c2c7d54792a432e97964d7f) )
	ROM_LOAD( "12.u118", 0x20000, 0x10000, CRC(678b57c1) SHA1(83869e5b6fe528c0b072f7d97338febc31db9f8b) )
	ROM_LOAD( "13.u119", 0x30000, 0x10000, CRC(9c0ee0de) SHA1(14b148f3ca951a5a9010b4d253e3ba7d35708403) )
	ROM_LOAD( "14.u120", 0x40000, 0x10000, CRC(21c11262) SHA1(e50678fafdf775a49ef96f8837b124824a2d1ca2) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "05.u22", 0x00000, 0x10000, CRC(515f337e) SHA1(21b2cca95b5da934fd8139892c2ee2c623d51a4e) )

	ROM_REGION( 0x800, "mcu", 0 )
	ROM_LOAD( "68705p5.bin", 0x000, 0x800, CRC(2e52bc67) SHA1(13ad4aee88c53c75c7cc1f31a149ba0234447f42) ) // in epoxy block

	ROM_REGION( 0x40000, "bg", 0 )    // 16x16x8
	ROM_LOAD( "04.u21", 0x00000, 0x10000, CRC(fa8d64f4) SHA1(71badabf8f34f246dec83323a1cddbe74deb91bd) )
	ROM_LOAD( "03.u20", 0x10000, 0x10000, CRC(8dda8167) SHA1(42838cf6866fb1d59c5bb3b477053aac448e7760) )
	ROM_LOAD( "09.u29", 0x20000, 0x10000, CRC(b9f28569) SHA1(1395cd226d314ee57385eed25f28b68607bfda53) )
	ROM_LOAD( "10.u30", 0x30000, 0x10000, CRC(db5762c0) SHA1(606dc4a3e6b8034f063f11dcf0a2b1db59838f4c) )

	ROM_REGION( 0xc000, "fg", 0 ) // 16x16x1
	ROM_LOAD( "06.u26", 0x1000, 0x1000, CRC(6d071b6d) SHA1(19565c8d768eeecd4119677915cc06f3ea18a47a) )
	ROM_CONTINUE(       0x0000, 0x1000 )
	ROM_LOAD( "07.u27", 0x3000, 0x1000, CRC(0f8b516e) SHA1(8bfabfd0bd28a1c7ddd01586fe9757b241feb59b) ) // FIXED BITS (xxxxxxx00xxxxxxx), BADADDR --xxxxxxxxxxxxx
	ROM_CONTINUE(       0x2000, 0x1000 )
	ROM_CONTINUE(       0x6000, 0x6000 ) // ??
	ROM_LOAD( "08.u28", 0x5000, 0x1000, CRC(51c0c5cb) SHA1(0c7bfc9b6b3ce0cdd5c0e36df2b4d90f9cff7fae) ) // FIXED BITS (0xxxxxx000000000), 111xxxxxxxxx1 = 0x00
	ROM_CONTINUE(       0x4000, 0x1000 )

	ROM_REGION( 0x20000, "fg2", 0 )    // 16x16x1
	ROM_LOAD( "01.u1", 0x00000, 0x10000, CRC(58506040) SHA1(9d8bed2585e8f188a20270fccd9cfbdb91e48599) )
	ROM_LOAD( "02.u2", 0x10000, 0x10000, CRC(9294a19c) SHA1(cd7109262e5f68b946c84aa390108bcc47ee1300) )

	ROM_REGION16_BE( 0x80, "eeprom", 0 )
	ROM_LOAD( "93c46eeprom.bin", 0, 0x80, CRC(4d244cc8) SHA1(6593d5b7ac1ebb77fee4648ad1d3d9b59a25fdc8) BAD_DUMP ) // backup ram error
ROM_END

GAME( 1988, quizpun,  0, quizpun,  quizpun2, quizpun2_state, 0, ROT270, "Space Computer", "Quiz Punch",    MACHINE_NOT_WORKING | MACHINE_IMPERFECT_GRAPHICS | MACHINE_IMPERFECT_SOUND )
GAME( 1989, quizpun2, 0, quizpun2, quizpun2, quizpun2_state, 0, ROT270, "Space Computer", "Quiz Punch II", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_GRAPHICS | MACHINE_IMPERFECT_SOUND | MACHINE_UNEMULATED_PROTECTION )
