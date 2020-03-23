// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/***************************************************************************

        VTech PreComputer 1000 / 2000

        04/12/2009 Skeleton driver.

        TODO:
        - fix MisterX LCD
        - MisterX bankswitch
        - dump the chargen

****************************************************************************/


#include "emu.h"

#include "cpu/z80/z80.h"
#include "sound/beep.h"
#include "video/hd44780.h"
#include "video/sed1520.h"

#include "bus/generic/slot.h"
#include "bus/generic/carts.h"

#include "rendlay.h"
#include "screen.h"
#include "softlist.h"
#include "speaker.h"

#include "gl3000s.lh"


class pc2000_state : public driver_device
{
public:
	pc2000_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_lcdc(*this, "hd44780")
		, m_beep(*this, "beeper")
		, m_cart(*this, "cartslot")
		, m_bank0(*this, "bank0")
		, m_bank1(*this, "bank1")
		, m_bank2(*this, "bank2")
	{ }

	required_device<cpu_device> m_maincpu;
	optional_device<hd44780_device> m_lcdc;
	required_device<beep_device> m_beep;
	required_device<generic_slot_device> m_cart;
	optional_memory_bank m_bank0;
	required_memory_bank m_bank1;
	optional_memory_bank m_bank2;

	uint8_t m_mux_data;
	uint8_t m_beep_state;

	virtual void machine_start() override;
	virtual void machine_reset() override;

	DECLARE_READ8_MEMBER( key_matrix_r );
	DECLARE_WRITE8_MEMBER( key_matrix_w );
	DECLARE_WRITE8_MEMBER( rombank0_w );
	DECLARE_WRITE8_MEMBER( rombank1_w );
	DECLARE_WRITE8_MEMBER( rombank2_w );
	DECLARE_READ8_MEMBER( beep_r );
	DECLARE_WRITE8_MEMBER( beep_w );
	DECLARE_PALETTE_INIT(pc2000);
	DECLARE_DEVICE_IMAGE_LOAD_MEMBER(pc2000_cart);
};

class gl3000s_state : public pc2000_state
{
public:
	gl3000s_state(const machine_config &mconfig, device_type type, const char *tag)
		: pc2000_state(mconfig, type, tag),
			m_lcdc_r(*this, "sed1520_r"),
			m_lcdc_l(*this, "sed1520_l")
		{ }

	required_device<sed1520_device> m_lcdc_r;
	required_device<sed1520_device> m_lcdc_l;

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
};

class gl4004_state : public pc2000_state
{
public:
	gl4004_state(const machine_config &mconfig, device_type type, const char *tag)
		: pc2000_state(mconfig, type, tag)
		{ }

	virtual void machine_start() override;
	HD44780_PIXEL_UPDATE(gl4000_pixel_update);
};

class pc1000_state : public pc2000_state
{
public:
	pc1000_state(const machine_config &mconfig, device_type type, const char *tag)
		: pc2000_state(mconfig, type, tag)
		{ }


	virtual void machine_start() override;
	virtual void machine_reset() override;

	DECLARE_READ8_MEMBER( kb_r );
	DECLARE_READ8_MEMBER( lcdc_data_r );
	DECLARE_WRITE8_MEMBER( lcdc_data_w );
	DECLARE_READ8_MEMBER( lcdc_control_r );
	DECLARE_WRITE8_MEMBER( lcdc_control_w );
	HD44780_PIXEL_UPDATE(pc1000_pixel_update);
};


/* TODO: put a breakpoint at 1625 and test the inputs, writes at dce4 are the scancode values */
READ8_MEMBER( pc2000_state::key_matrix_r )
{
	static const char *const bitnames[2][8] =
	{
		{"IN0", "IN1", "IN2", "IN3", "IN4", "IN5", "IN6", "IN7"},
		{"IN8", "IN9", "INA", "INB", "INC", "IND", "INE", "INF"}
	};

	uint8_t data = 0xff;

	for (int line=0; line<8; line++)
		if (m_mux_data & (1<<line))
			data &= ioport(bitnames[offset][line])->read();

	return data;
}

WRITE8_MEMBER( pc2000_state::key_matrix_w )
{
	m_mux_data = data;
}

WRITE8_MEMBER( pc2000_state::rombank0_w )
{
	m_bank0->set_entry(data & 0x1f);
}

WRITE8_MEMBER( pc2000_state::rombank1_w )
{
	m_bank1->set_entry(data & 0x1f);
}

WRITE8_MEMBER( pc2000_state::rombank2_w )
{
	if (data & 0x80)
		m_bank2->set_entry(data & 0x8f);   //cartridge
	else
		m_bank2->set_entry(data & 0x1f);
}

READ8_MEMBER( pc2000_state::beep_r )
{
	return m_beep_state;
}

WRITE8_MEMBER( pc2000_state::beep_w )
{
	m_beep->set_state(BIT(data, 3));
	m_beep_state = data;
}

static ADDRESS_MAP_START(pc2000_mem, AS_PROGRAM, 8, pc2000_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x3fff) AM_ROMBANK("bank0")
	AM_RANGE(0x4000, 0x7fff) AM_ROMBANK("bank1")
	AM_RANGE(0x8000, 0xbfff) AM_ROMBANK("bank2")    //0x8000 - 0xbfff tests a cartridge, header is 0x55 0xaa 0x59 0x45, if it succeeds a jump at 0x8004 occurs
	AM_RANGE(0xc000, 0xdfff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( pc2000_io , AS_IO, 8, pc2000_state)
	ADDRESS_MAP_UNMAP_HIGH
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x00) AM_WRITE(rombank0_w)
	AM_RANGE(0x01, 0x01) AM_WRITE(rombank1_w)
	AM_RANGE(0x03, 0x03) AM_WRITE(rombank2_w)
	AM_RANGE(0x0a, 0x0b) AM_DEVREADWRITE("hd44780", hd44780_device, read, write)
	AM_RANGE(0x10, 0x11) AM_READWRITE(key_matrix_r, key_matrix_w)
	AM_RANGE(0x12, 0x12) AM_READWRITE(beep_r, beep_w)
ADDRESS_MAP_END


uint32_t gl3000s_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(0);
	m_lcdc_l->screen_update(screen, bitmap, cliprect);
	m_lcdc_r->screen_update(screen, bitmap, cliprect);
	return 0;
}

int gl3000s_sed1520_screen_update(device_t &device, bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t *vram, int start_line, int adc, int start_x)
{
	for (int y=0; y<2; y++)
	{
		int row_pos = 0;
		for (int x=0; x<61; x++)
		{
			int addr = (y + (start_line >> 3)) * 80 + row_pos;
			for (int yi=0; yi<8; yi++)
			{
				int px = start_x - (adc ? (80 - x) : x);
				int py = 8 + y*8 + yi;

				if (cliprect.contains(px, py))
					bitmap.pix16(py, px) = (vram[addr % 0x140] >> yi) & 1;
			}

			row_pos++;
		}
	}
	return 0;
}

SED1520_UPDATE_CB(gl3000s_screen_update_right)
{
	return gl3000s_sed1520_screen_update(device, bitmap, cliprect, vram, start_line, adc, 119);
}

SED1520_UPDATE_CB(gl3000s_screen_update_left)
{
	uint8_t sec[5];
	uint8_t points[2][5];
	memset(sec, 0, sizeof(sec));
	memset(points, 0, sizeof(points));

	for (int y=0; y<2; y++)
		for (int x=59; x<85; x++)
		{
			uint8_t data = vram[(y*0x50 + x) % 0x140];
			int32_t dpos = (x - 74) / 2;
			if (dpos < 0)
			{
				dpos = 0;
			}

			for (int yi=0; yi<8; yi++)
			{
				int state = (data >> yi) & 1;

				if (y == 0 && (x == 74 || x == 76 || x == 78) && yi == 7)         sec[dpos] |= (state << 0);
				else if (y == 0 && (x == 74 || x == 76 || x == 78) && yi == 0)    sec[dpos] |= (state << 2);
				else if (y == 0 && (x == 75 || x == 77 || x == 79) && yi == 7)    sec[dpos] |= (state << 5);
				else if (y == 0 && (x == 75 || x == 77 || x == 79) && yi == 0)    sec[dpos] |= (state << 4);
				else if (y == 0 && (x == 75 || x == 77 || x == 79) && yi == 1)    sec[dpos] |= (state << 3);
				else if (y == 1 && (x == 74 || x == 76 || x == 78) && yi == 7)    sec[dpos] |= (state << 1);
				else if (y == 1 && (x == 75 || x == 77 || x == 79) && yi == 7)    sec[dpos] |= (state << 6);

				else if ((x == 74 || x == 76 || x == 78) && yi == 3)    points[y][dpos] |= (state << 0);
				else if ((x == 74 || x == 76 || x == 78) && yi == 4)    points[y][dpos] |= (state << 1);
				else if ((x == 74 || x == 76 || x == 78) && yi == 5)    points[y][dpos] |= (state << 2);
				else if ((x == 75 || x == 77 || x == 79) && yi == 3)    points[y][dpos] |= (state << 5);
				else if ((x == 75 || x == 77 || x == 79) && yi == 4)    points[y][dpos] |= (state << 6);
				else if ((x == 75 || x == 77 || x == 79) && yi == 5)    points[y][dpos] |= (state << 4);
				else if ((x == 75 || x == 77 || x == 79) && yi == 6)    points[y][dpos] |= (state << 3);

				else if (y == 1 && x >= 65 && x <= 68 && yi == 7)       device.machine().output().set_indexed_value("LEV", x - 64, state);
				else if (x >= 59  && x <= 60 && yi == 7)                device.machine().output().set_indexed_value("TRY", x - 58 + (y ? 0 : 1), state);
				else if (y == 1 && x >= 61 && x <= 64 && yi == 7)       device.machine().output().set_indexed_value("TICK", x - 59, state);
				else if (y == 0 && x >= 61 && x <= 64 && yi == 7)       device.machine().output().set_indexed_value("TICK", 62 - x + (x >= 63 ? 8 : 0), state);

				else if (x < 74 && yi < 7)
				{
					int cx = x - 59;
					bitmap.pix16(yi, (y ? 0 : 89) + (16 - (cx + cx / 5))) = state;
				}
			}
		}

	for(int i=0; i < 3; i++)
	{
		device.machine().output().set_indexed_value("TIME", i, sec[i]);
		device.machine().output().set_indexed_value("P1", i, points[1][i]);
		device.machine().output().set_indexed_value("P2", i, points[0][i]);
	}

	return gl3000s_sed1520_screen_update(device, bitmap, cliprect, vram, start_line, adc, 58);
}


static ADDRESS_MAP_START( gl3000s_io , AS_IO, 8, gl3000s_state)
	ADDRESS_MAP_UNMAP_HIGH
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x01, 0x01) AM_WRITE(rombank1_w)
	AM_RANGE(0x03, 0x03) AM_WRITE(rombank2_w)
	AM_RANGE(0x08, 0x09) AM_DEVREADWRITE("sed1520_r", sed1520_device, read, write)
	AM_RANGE(0x0a, 0x0b) AM_DEVREADWRITE("sed1520_l", sed1520_device, read, write)
	AM_RANGE(0x10, 0x11) AM_READWRITE(key_matrix_r, key_matrix_w)
ADDRESS_MAP_END

READ8_MEMBER( pc1000_state::kb_r )
{
	static const char *const bitnames[9] =
	{
		"IN0", "IN1", "IN2", "IN3", "IN4", "IN5", "IN6", "IN7", "IN8"
	};

	uint8_t data = 0xff;

	for (int line=0; line<9; line++)
		if (!(offset & (1<<line)))
			data &= ioport(bitnames[line])->read();

	return data;
}

READ8_MEMBER( pc1000_state::lcdc_data_r )
{
	//logerror("lcdc data r\n");
	return m_lcdc->data_read(space, 0)>>4;
}

WRITE8_MEMBER( pc1000_state::lcdc_data_w )
{
	//popmessage("%s", (char*)m_maincpu->space(AS_PROGRAM).get_read_ptr(0x4290));

	//logerror("lcdc data w %x\n", data);
	m_lcdc->data_write(space, 0, data<<4);
}

READ8_MEMBER( pc1000_state::lcdc_control_r )
{
	//logerror("lcdc control r\n");
	return m_lcdc->control_read(space, 0)>>4;
}

WRITE8_MEMBER( pc1000_state::lcdc_control_w )
{
	//logerror("lcdc control w %x\n", data);
	m_lcdc->control_write(space, 0, data<<4);
}

HD44780_PIXEL_UPDATE(pc1000_state::pc1000_pixel_update)
{
	uint8_t layout[] = { 0x00, 0x4f, 0x4e, 0x4d, 0x4c, 0x4b, 0x4a, 0x49, 0x48, 0x47, 0x40, 0x3f, 0x3e, 0x3d, 0x3c, 0x3b, 0x3a, 0x39, 0x38, 0x37 };
	//uint8_t layout[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49 };

	for(int i=0; i<20; i++)
		if (pos == layout[i])
		{
			bitmap.pix16(line*9 + y, i*6 + x) = state;
			break;
		}
}

static ADDRESS_MAP_START(pc1000_mem, AS_PROGRAM, 8, pc1000_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x3fff) AM_ROM AM_REGION("bios", 0x00000)
	AM_RANGE(0x4000, 0x47ff) AM_RAM
	AM_RANGE(0x8000, 0xbfff) AM_DEVREAD("cartslot", generic_slot_device, read_rom)    //0x8000 - 0xbfff tests a cartridge, header is 0x55 0xaa 0x33, if it succeeds a jump at 0x8010 occurs
	AM_RANGE(0xc000, 0xffff) AM_ROMBANK("bank1")
ADDRESS_MAP_END

static ADDRESS_MAP_START( pc1000_io , AS_IO, 8, pc1000_state)
	AM_RANGE(0x0000, 0x01ff) AM_READ(kb_r)
	AM_RANGE(0x4000, 0x4000) AM_MIRROR(0xfe) AM_READWRITE(lcdc_control_r, lcdc_control_w)
	AM_RANGE(0x4100, 0x4100) AM_MIRROR(0xfe) AM_READWRITE(lcdc_data_r, lcdc_data_w)
ADDRESS_MAP_END

/* Input ports */
static INPUT_PORTS_START( pc2000 )
	PORT_START("IN0")
	PORT_DIPNAME( 0x01, 0x01, "IN0" ) //0x83
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

	PORT_START("IN1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Caps Lock")  PORT_CODE(KEYCODE_CAPSLOCK)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("1")          PORT_CODE(KEYCODE_1) PORT_CHAR('1')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Q")          PORT_CODE(KEYCODE_Q) PORT_CHAR('Q')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A")          PORT_CODE(KEYCODE_A) PORT_CHAR('A')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Z")          PORT_CODE(KEYCODE_Z) PORT_CHAR('Z')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("\\")         PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('\\')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Space")      PORT_CODE(KEYCODE_SPACE) PORT_CHAR(' ')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("IN2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("2")  PORT_CODE(KEYCODE_2) PORT_CHAR('2')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("3")  PORT_CODE(KEYCODE_3) PORT_CHAR('3')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E")  PORT_CODE(KEYCODE_E) PORT_CHAR('E')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("S")  PORT_CODE(KEYCODE_S) PORT_CHAR('S')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D")  PORT_CODE(KEYCODE_D) PORT_CHAR('D')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("X")  PORT_CODE(KEYCODE_X) PORT_CHAR('X')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C")  PORT_CODE(KEYCODE_C) PORT_CHAR('C')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("W")  PORT_CODE(KEYCODE_W) PORT_CHAR('W')

	PORT_START("IN3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("4")  PORT_CODE(KEYCODE_4) PORT_CHAR('4')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("5")  PORT_CODE(KEYCODE_5) PORT_CHAR('5')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("T")  PORT_CODE(KEYCODE_T) PORT_CHAR('T')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F")  PORT_CODE(KEYCODE_F) PORT_CHAR('F')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G")  PORT_CODE(KEYCODE_G) PORT_CHAR('G')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("V")  PORT_CODE(KEYCODE_V) PORT_CHAR('V')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B")  PORT_CODE(KEYCODE_B) PORT_CHAR('B')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("R")  PORT_CODE(KEYCODE_R) PORT_CHAR('R')

	PORT_START("IN4")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("6")  PORT_CODE(KEYCODE_6) PORT_CHAR('6')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("7")  PORT_CODE(KEYCODE_7) PORT_CHAR('7')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("U")  PORT_CODE(KEYCODE_U) PORT_CHAR('U')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("H")  PORT_CODE(KEYCODE_H) PORT_CHAR('H')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J")  PORT_CODE(KEYCODE_J) PORT_CHAR('J')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("N")  PORT_CODE(KEYCODE_N) PORT_CHAR('N')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("M")  PORT_CODE(KEYCODE_M) PORT_CHAR('M')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Y")  PORT_CODE(KEYCODE_Y) PORT_CHAR('Y')

	PORT_START("IN5")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("8")  PORT_CODE(KEYCODE_8) PORT_CHAR('8')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("9")  PORT_CODE(KEYCODE_9) PORT_CHAR('9')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("O")  PORT_CODE(KEYCODE_O) PORT_CHAR('O')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("K")  PORT_CODE(KEYCODE_K) PORT_CHAR('K')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("L")  PORT_CODE(KEYCODE_L) PORT_CHAR('L')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(",")  PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(".")  PORT_CODE(KEYCODE_STOP) PORT_CHAR('.')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("I")  PORT_CODE(KEYCODE_I) PORT_CHAR('I')

	PORT_START("IN6")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("0")  PORT_CODE(KEYCODE_0) PORT_CHAR('0')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("HOME")   PORT_CODE(KEYCODE_HOME)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("=")  PORT_CODE(KEYCODE_EQUALS)   PORT_CHAR('=') PORT_CHAR('+')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(";")  PORT_CODE(KEYCODE_COLON) PORT_CHAR(';')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("\'") PORT_CODE(KEYCODE_QUOTE) PORT_CHAR('\'')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("/")  PORT_CODE(KEYCODE_SLASH) PORT_CHAR('/')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("P")  PORT_CODE(KEYCODE_P) PORT_CHAR('P')

	PORT_START("IN7")
	PORT_BIT(0x03, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("LEFT")   PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("RIGHT")  PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("ENTER")  PORT_CODE(KEYCODE_ENTER)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DEL")    PORT_CODE(KEYCODE_DEL)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("ESC")    PORT_CODE(KEYCODE_ESC)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("IN8")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Scramble")           PORT_CODE(KEYCODE_F1)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Guess Rork")         PORT_CODE(KEYCODE_F2)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Missing Letter")     PORT_CODE(KEYCODE_F3)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Letter Hunt")        PORT_CODE(KEYCODE_F4)
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("IN9")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Letter Zapper")      PORT_CODE(KEYCODE_F5)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Letter Switch")      PORT_CODE(KEYCODE_F6)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Sentence Jumble")    PORT_CODE(KEYCODE_F7)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Grammar Challenge")  PORT_CODE(KEYCODE_F8)
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("INA")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Plurals")            PORT_CODE(KEYCODE_F9)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Past Tense")         PORT_CODE(KEYCODE_F10)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Synonyms")           PORT_CODE(KEYCODE_F11)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Antonyms")           PORT_CODE(KEYCODE_F12)
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("INB")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Basic Math")         PORT_CODE(KEYCODE_0_PAD)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Advanced Math")      PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Math Riddles")       PORT_CODE(KEYCODE_2_PAD)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Number Challenge")   PORT_CODE(KEYCODE_3_PAD)
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("INC")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Ecology")            PORT_CODE(KEYCODE_4_PAD)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("History")            PORT_CODE(KEYCODE_5_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Geography")          PORT_CODE(KEYCODE_6_PAD)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Super Power")        PORT_CODE(KEYCODE_7_PAD)
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("IND")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Typing Game")        PORT_CODE(KEYCODE_8_PAD)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Alpha Jumble")       PORT_CODE(KEYCODE_9_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Word Flash")         PORT_CODE(KEYCODE_PGUP)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Sign Search")        PORT_CODE(KEYCODE_PGDN)
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("INE")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("PC2000 Basic")       PORT_CODE(KEYCODE_OPENBRACE)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Spell Checker")      PORT_CODE(KEYCODE_CLOSEBRACE)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Calculator")         PORT_CODE(KEYCODE_RSHIFT)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Load Cartridge")     PORT_CODE(KEYCODE_RCONTROL)
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("INF")
	PORT_DIPNAME( 0x01, 0x01, "INF" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(DEF_STR( Level_Select )) PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Num Players")        PORT_CODE(KEYCODE_LCONTROL)
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END

static INPUT_PORTS_START( pc1000 )
	PORT_START("IN0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("SCHREIBMASCHINENKURS") PORT_CODE(KEYCODE_F1)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("COMPUTER-UBUNGEN") PORT_CODE(KEYCODE_F2)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("ALLGEMEINWISSEN") PORT_CODE(KEYCODE_F3)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("MATHE")      PORT_CODE(KEYCODE_F7)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("GESCHICHTE") PORT_CODE(KEYCODE_F4)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("GEOGRAPHIE") PORT_CODE(KEYCODE_F5)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("NATURWISSENSCHAFTEN") PORT_CODE(KEYCODE_F6)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("SPIELE")     PORT_CODE(KEYCODE_F8)

	PORT_START("IN1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("KASSETTE")   PORT_CODE(KEYCODE_F9)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("AUS")        PORT_CODE(KEYCODE_F12)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Left")       PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Right")      PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("?")          PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("?")          PORT_CODE(KEYCODE_2_PAD)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("?")          PORT_CODE(KEYCODE_3_PAD)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("ELEKRONIK-RECHNER")   PORT_CODE(KEYCODE_F10)

	PORT_START("IN2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("0")  PORT_CODE(KEYCODE_0) PORT_CHAR('0')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("?")  PORT_CODE(KEYCODE_4_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("\xc3\xb6")  PORT_CODE(KEYCODE_OPENBRACE)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("\xc3\xa4")  PORT_CODE(KEYCODE_CLOSEBRACE)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("\xc3\xbc")  PORT_CODE(KEYCODE_QUOTE)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("-")  PORT_CODE(KEYCODE_MINUS) PORT_CHAR('-')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Shift")  PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("P")  PORT_CODE(KEYCODE_P) PORT_CHAR('P')

	PORT_START("IN3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("8")  PORT_CODE(KEYCODE_8) PORT_CHAR('8')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("9")  PORT_CODE(KEYCODE_9) PORT_CHAR('9')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("O")  PORT_CODE(KEYCODE_O) PORT_CHAR('O')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("K")  PORT_CODE(KEYCODE_K) PORT_CHAR('K')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("L")  PORT_CODE(KEYCODE_L) PORT_CHAR('L')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(",")  PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(".")  PORT_CODE(KEYCODE_STOP) PORT_CHAR('.')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("I")  PORT_CODE(KEYCODE_I) PORT_CHAR('I')

	PORT_START("IN4")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("6")  PORT_CODE(KEYCODE_6) PORT_CHAR('6')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("7")  PORT_CODE(KEYCODE_7) PORT_CHAR('7')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("U")  PORT_CODE(KEYCODE_U) PORT_CHAR('U')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("H")  PORT_CODE(KEYCODE_H) PORT_CHAR('H')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J")  PORT_CODE(KEYCODE_J) PORT_CHAR('J')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("N")  PORT_CODE(KEYCODE_N) PORT_CHAR('N')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("M")  PORT_CODE(KEYCODE_M) PORT_CHAR('M')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Z")  PORT_CODE(KEYCODE_Z) PORT_CHAR('Z')

	PORT_START("IN5")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("4")  PORT_CODE(KEYCODE_4) PORT_CHAR('4')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("5")  PORT_CODE(KEYCODE_5) PORT_CHAR('5')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("T")  PORT_CODE(KEYCODE_T) PORT_CHAR('T')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F")  PORT_CODE(KEYCODE_F) PORT_CHAR('F')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G")  PORT_CODE(KEYCODE_G) PORT_CHAR('G')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("V")  PORT_CODE(KEYCODE_V) PORT_CHAR('V')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B")  PORT_CODE(KEYCODE_B) PORT_CHAR('B')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("R")  PORT_CODE(KEYCODE_R) PORT_CHAR('R')

	PORT_START("IN6")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("2")  PORT_CODE(KEYCODE_2) PORT_CHAR('2')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("3")  PORT_CODE(KEYCODE_3) PORT_CHAR('3')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E")  PORT_CODE(KEYCODE_E) PORT_CHAR('E')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("S")  PORT_CODE(KEYCODE_S) PORT_CHAR('S')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D")  PORT_CODE(KEYCODE_D) PORT_CHAR('D')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("X")  PORT_CODE(KEYCODE_X) PORT_CHAR('X')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C")  PORT_CODE(KEYCODE_C) PORT_CHAR('C')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("W")  PORT_CODE(KEYCODE_W) PORT_CHAR('W')

	PORT_START("IN7")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Caps Lock")  PORT_CODE(KEYCODE_CAPSLOCK)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("1")  PORT_CODE(KEYCODE_1) PORT_CHAR('1')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Q")  PORT_CODE(KEYCODE_Q) PORT_CHAR('Q')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A")  PORT_CODE(KEYCODE_A) PORT_CHAR('A')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Y")  PORT_CODE(KEYCODE_Y) PORT_CHAR('Y')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DEL") PORT_CODE(KEYCODE_DEL)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Space") PORT_CODE(KEYCODE_SPACE) PORT_CHAR(' ')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_5_PAD)

	PORT_START("IN8")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_6_PAD)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_7_PAD)
	PORT_BIT(0xfc, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END

static INPUT_PORTS_START( gl3000s )
	PORT_START("IN0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_1) PORT_CHAR('1')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_ESC)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_E) PORT_CHAR('E')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_CAPSLOCK)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Einstellen Antwort")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT)

	PORT_START("IN1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_2) PORT_CHAR('2')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q) PORT_CHAR('Q')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_R) PORT_CHAR('R')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_A) PORT_CHAR('A')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_X) PORT_CHAR('X')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Hilfe")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_SPACE)  PORT_CHAR(' ')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_M) PORT_CHAR('M')

	PORT_START("IN2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_3) PORT_CHAR('3')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_W) PORT_CHAR('W')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_T) PORT_CHAR('T')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_S) PORT_CHAR('S')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y) PORT_CHAR('Y')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Druck Symbol")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS) PORT_CHAR('-')

	PORT_START("IN3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_4) PORT_CHAR('4')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z) PORT_CHAR('Z')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_D) PORT_CHAR('D')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_K) PORT_CHAR('K')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH) PORT_NAME("\xc3\x84")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_UP)

	PORT_START("IN4")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_5) PORT_CHAR('5')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_U) PORT_CHAR('U')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_F) PORT_CHAR('F')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_J) PORT_CHAR('J')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_C) PORT_CHAR('C')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_INSERT)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_N) PORT_CHAR('N')

	PORT_START("IN5")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_6) PORT_CHAR('6')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS)  PORT_NAME("\xc3\x9f")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_P) PORT_CHAR('P')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE) PORT_CHAR('\'')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE)  PORT_NAME("\xc3\x96")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP) PORT_CHAR('.')

	PORT_START("IN6")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_7) PORT_CHAR('7')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_0) PORT_CHAR('0')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_O) PORT_CHAR('O')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE)  PORT_NAME("\xc3\x9c")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_L) PORT_CHAR('L')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_PLUS_PAD) PORT_CHAR('+')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_LALT) PORT_CODE(KEYCODE_RALT)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',')

	PORT_START("IN7")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_8) PORT_CHAR('8')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_9) PORT_CHAR('9')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_I) PORT_CHAR('I')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_G) PORT_CHAR('G')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_H) PORT_CHAR('H')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_V) PORT_CHAR('V')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER) PORT_CHAR(13)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_B) PORT_CHAR('B')

	PORT_START("IN8")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Off")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("BASIC")              PORT_CODE(KEYCODE_DEL_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Logische Folgen")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Bruch / Dezimalrechnen")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Addition")
	PORT_BIT(0xe0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("IN9")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Textverarbeitung")   PORT_CODE(KEYCODE_ENTER_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Buchstabenschl" "\xc3\xbc" "ssel")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Bruch / Prozentrechnen") PORT_CODE(KEYCODE_DEL)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Subtraktion")        PORT_CODE(KEYCODE_HOME)
	PORT_BIT(0xe1, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("INA")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Taschenrechner") PORT_CODE(KEYCODE_ASTERISK)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Gleitpfeile")    PORT_CODE(KEYCODE_MINUS_PAD)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Algebra")        PORT_CODE(KEYCODE_SLASH_PAD)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Multiplikation") PORT_CODE(KEYCODE_9_PAD)
	PORT_BIT(0xe1, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("INB")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("ZusatzKassette") PORT_CODE(KEYCODE_8_PAD)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Stufe")          PORT_CODE(KEYCODE_7_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Spieler")        PORT_CODE(KEYCODE_6_PAD)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Zeichensuche")   PORT_CODE(KEYCODE_5_PAD)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Division")       PORT_CODE(KEYCODE_4_PAD)
	PORT_BIT(0xe0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("INC")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Pr" "\xc3\xa4" "teritum")   PORT_CODE(KEYCODE_3_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Synonyme")       PORT_CODE(KEYCODE_2_PAD)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Tipp Dich Fit")  PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Super Schlau-Quiz")  PORT_CODE(KEYCODE_0_PAD)
	PORT_BIT(0xe1, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("IND")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Silbentrennung")     PORT_CODE(KEYCODE_F12)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Knifflige Grammatik")  PORT_CODE(KEYCODE_F11)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Textaufgaben")       PORT_CODE(KEYCODE_F10)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Geographie-Quiz")    PORT_CODE(KEYCODE_F9)
	PORT_BIT(0xe1, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("INE")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Wortr" "\xc3\xa4" "tsel")       PORT_CODE(KEYCODE_F8)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Verdrehte S" "\xc3\xa4" "tze")  PORT_CODE(KEYCODE_F7)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Blitz-S" "\xc3\xa4" "tze")      PORT_CODE(KEYCODE_F6)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Geschichte-Quiz")    PORT_CODE(KEYCODE_F5)
	PORT_BIT(0xe1, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("INF")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Verr" "\xc3\xbc" "ckte R" "\xc3\xa4" "tsel")  PORT_CODE(KEYCODE_F4)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Wortarten")                  PORT_CODE(KEYCODE_F3)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Wortschlange")               PORT_CODE(KEYCODE_F2)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Naturwissenschaften-Quiz")   PORT_CODE(KEYCODE_F1)
	PORT_BIT(0xe1, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END

void pc2000_state::machine_start()
{
	std::string region_tag;
	uint8_t *bios = memregion("bios")->base();
	memory_region *cart_region = memregion(region_tag.assign(m_cart->tag()).append(GENERIC_ROM_REGION_TAG).c_str());
	uint8_t *cart = (cart_region != nullptr) ? cart_region->base() : memregion("bios")->base();

	m_bank0->configure_entries(0, 0x10, bios, 0x4000);
	m_bank1->configure_entries(0, 0x10, bios, 0x4000);
	m_bank2->configure_entries(0, 0x10, bios, 0x4000);
	m_bank2->configure_entries(0x80, 0x10, cart, 0x4000);
}

void gl4004_state::machine_start()
{
	std::string region_tag;
	uint8_t *bios = memregion("bios")->base();
	memory_region *cart_region = memregion(region_tag.assign(m_cart->tag()).append(GENERIC_ROM_REGION_TAG).c_str());
	uint8_t *cart = (cart_region != nullptr) ? cart_region->base() : memregion("bios")->base();

	m_bank0->configure_entries(0, 0x20, bios, 0x4000);
	m_bank1->configure_entries(0, 0x20, bios, 0x4000);
	m_bank2->configure_entries(0, 0x20, bios, 0x4000);
	m_bank2->configure_entries(0x80, 0x10, cart, 0x4000);
}

void pc2000_state::machine_reset()
{
	//set the initial bank
	m_bank0->set_entry(0);
	m_bank1->set_entry(0);
	m_bank2->set_entry(0);
}

void pc1000_state::machine_start()
{
	uint8_t *bios = memregion("bios")->base();
	m_bank1->configure_entries(0, 0x08, bios, 0x4000);
}

void pc1000_state::machine_reset()
{
	m_bank1->set_entry(0);
}

PALETTE_INIT_MEMBER(pc2000_state, pc2000)
{
	palette.set_pen_color(0, rgb_t(138, 146, 148));
	palette.set_pen_color(1, rgb_t(92, 83, 88));
}

static const gfx_layout hd44780_charlayout =
{
	5, 8,                   /* 5 x 8 characters */
	256,                    /* 256 characters */
	1,                      /* 1 bits per pixel */
	{ 0 },                  /* no bitplanes */
	{ 3, 4, 5, 6, 7},
	{ 0, 8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8},
	8*8                     /* 8 bytes */
};

static GFXDECODE_START( pc2000 )
	GFXDECODE_ENTRY( "hd44780:cgrom", 0x0000, hd44780_charlayout, 0, 1 )
GFXDECODE_END


DEVICE_IMAGE_LOAD_MEMBER( pc2000_state, pc2000_cart )
{
	uint32_t size = m_cart->common_get_size("rom");

	// we always allocate a 0x40000 region, even if most carts span only 0x20000,
	// because the bankswitch code accesses up to 16 x 16K banks...
	m_cart->rom_alloc(0x40000, GENERIC_ROM8_WIDTH, ENDIANNESS_LITTLE);
	m_cart->common_load_rom(m_cart->get_rom_base(), size, "rom");

	return image_init_result::PASS;
}

static MACHINE_CONFIG_START( pc2000 )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu",Z80, XTAL_4MHz) /* probably not accurate */
	MCFG_CPU_PROGRAM_MAP(pc2000_mem)
	MCFG_CPU_IO_MAP(pc2000_io)
	MCFG_CPU_PERIODIC_INT_DRIVER(pc2000_state, irq0_line_hold, 50)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", LCD)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) /* not accurate */
	MCFG_SCREEN_UPDATE_DEVICE("hd44780", hd44780_device, screen_update)
	MCFG_SCREEN_SIZE(120, 18) //2x20 chars
	MCFG_SCREEN_VISIBLE_AREA(0, 120-1, 0, 18-1)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 2)
	MCFG_PALETTE_INIT_OWNER(pc2000_state, pc2000)
	MCFG_GFXDECODE_ADD("gfxdecode", "palette", pc2000)
	MCFG_DEFAULT_LAYOUT(layout_lcd)

	MCFG_HD44780_ADD("hd44780")
	MCFG_HD44780_LCD_SIZE(2, 20)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO( "mono" )
	MCFG_SOUND_ADD( "beeper", BEEP, 3250 )
	MCFG_SOUND_ROUTE( ALL_OUTPUTS, "mono", 1.00 )

	MCFG_GENERIC_CARTSLOT_ADD("cartslot", generic_plain_slot, "genius_cart")
	MCFG_GENERIC_LOAD(pc2000_state, pc2000_cart)

	MCFG_SOFTWARE_LIST_COMPATIBLE_ADD("pc1000_cart", "pc1000")
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( gl2000, pc2000 )
	MCFG_SOFTWARE_LIST_ADD("cart_list", "gl2000")
	MCFG_SOFTWARE_LIST_COMPATIBLE_ADD("misterx_cart", "misterx")
MACHINE_CONFIG_END

HD44780_PIXEL_UPDATE(gl4004_state::gl4000_pixel_update)
{
	if (pos < 40)
	{
		static const uint8_t gl4000_display_layout[] =
		{
			0x00, 0x01, 0x02, 0x03, 0x28, 0x29, 0x2a, 0x2b, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x2c, 0x2d, 0x2e, 0x2f,
			0x30, 0x31, 0x32, 0x33, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
			0x14, 0x15, 0x16, 0x17, 0x3c, 0x3d, 0x3e, 0x3f, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x40, 0x41, 0x42, 0x43,
			0x44, 0x45, 0x46, 0x47, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f
		};

		uint8_t char_pos = gl4000_display_layout[line*40 + pos];
		bitmap.pix16((char_pos / 20) * 9 + y, (char_pos % 20) * 6 + x) = state;
	}
}

static MACHINE_CONFIG_DERIVED( gl3000s, pc2000 )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_IO_MAP(gl3000s_io)

	MCFG_DEVICE_REMOVE("hd44780")
	MCFG_SED1520_ADD("sed1520_l", gl3000s_screen_update_left)   // left panel is 59 pixels (0-58)
	MCFG_SED1520_ADD("sed1520_r", gl3000s_screen_update_right)  // right panel is 61 pixels (59-119)

	MCFG_SCREEN_MODIFY("screen")
	MCFG_SCREEN_SIZE(120, 24)
	MCFG_SCREEN_VISIBLE_AREA(0, 120-1, 0, 24-1)
	MCFG_SCREEN_UPDATE_DRIVER( gl3000s_state, screen_update )

	MCFG_DEFAULT_LAYOUT(layout_gl3000s)

	MCFG_DEVICE_REMOVE("gfxdecode")

	MCFG_SOFTWARE_LIST_COMPATIBLE_ADD("gl2000_cart", "gl2000")
	MCFG_SOFTWARE_LIST_COMPATIBLE_ADD("misterx_cart", "misterx")
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( gl4000, pc2000 )
	MCFG_SCREEN_MODIFY("screen")
	MCFG_SCREEN_SIZE(120, 36) // 4x20 chars
	MCFG_SCREEN_VISIBLE_AREA(0, 120-1, 0, 36-1)

	MCFG_DEVICE_MODIFY("hd44780")
	MCFG_HD44780_LCD_SIZE(4, 20)
	MCFG_HD44780_PIXEL_UPDATE_CB(gl4004_state,gl4000_pixel_update)

	MCFG_SOFTWARE_LIST_COMPATIBLE_ADD("gl2000_cart", "gl2000")
	MCFG_SOFTWARE_LIST_COMPATIBLE_ADD("misterx_cart", "misterx")
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( misterx, pc2000 )
	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(pc1000_mem)
	MCFG_CPU_IO_MAP(pc1000_io)
	MCFG_CPU_PERIODIC_INT_DRIVER(pc1000_state, irq0_line_hold,  10)

	/* video hardware */
	MCFG_SCREEN_MODIFY("screen")
	MCFG_SCREEN_SIZE(120, 9) //1x20 chars
	MCFG_SCREEN_VISIBLE_AREA(0, 120-1, 0, 9-1)

	MCFG_DEVICE_MODIFY("hd44780")
	MCFG_HD44780_LCD_SIZE(1, 20)
	MCFG_HD44780_PIXEL_UPDATE_CB(pc1000_state,pc1000_pixel_update)

	/* Software lists */
	MCFG_SOFTWARE_LIST_ADD("cart_list", "misterx")
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( pc1000, misterx )
	MCFG_SOFTWARE_LIST_REMOVE("cart_list")
	MCFG_SOFTWARE_LIST_REMOVE("pc1000_cart")
	MCFG_SOFTWARE_LIST_ADD("cart_list", "pc1000")
	MCFG_SOFTWARE_LIST_COMPATIBLE_ADD("misterx_cart", "misterx")
MACHINE_CONFIG_END

/* ROM definition */
ROM_START( pc2000 )
	ROM_REGION( 0x40000, "bios", 0 )
	ROM_LOAD( "lh532hee_9344_d.u4", 0x000000, 0x040000, CRC(0b03bf33) SHA1(cb344b94b14975c685041d3e669f386e8a21909f))
ROM_END

ROM_START( gl2000 )
	ROM_REGION( 0x40000, "bios", 0 )
	ROM_LOAD( "lh532hez_9416_d.bin", 0x000000, 0x040000, CRC(532f219e) SHA1(4044f0cf098087af4cc9d1b2a80c3c9ec06f154e))
ROM_END

ROM_START( gl2000c )
	ROM_REGION( 0x40000, "bios", 0 )
	ROM_LOAD( "27-5491-00", 0x000000, 0x020000, CRC(cbb9fe90) SHA1(a2a7a8afb027fe764a5998e3b35e87f291c24df1))
ROM_END

ROM_START( gl2000p )
	ROM_REGION( 0x40000, "bios", 0 )
	ROM_LOAD( "27-5615-00_9534_d.bin", 0x000000, 0x040000, CRC(481c1000) SHA1(da6f60e5bb25145ec5239310296bedaabeeaee28))
ROM_END

ROM_START( gl3000s )
	ROM_REGION(0x40000, "bios", 0)
	ROM_LOAD( "27-5713-00", 0x000000, 0x040000, CRC(18b113e0) SHA1(27a12893c38068efa35a99fa97a260dbfbd497e3) )
ROM_END

ROM_START( gl4000 )
	ROM_REGION(0x80000, "bios", 0)
	ROM_LOAD( "27-5480-00",   0x000000, 0x040000, CRC(8de047d3) SHA1(bb1d869954773bb7b8b51caa54531015d6b751ec) )
ROM_END

ROM_START( gl4004 )
	ROM_REGION(0x80000, "bios", 0)
	ROM_LOAD( "27-5762-00.u2", 0x000000, 0x080000, CRC(fb242f0f) SHA1(aae1beeb94873e29920726ad35475641d9f1e94e) )
ROM_END

ROM_START( gl5000 )
	ROM_REGION(0x80000, "bios", 0)
	ROM_LOAD( "27-5912-00.u1", 0x000000, 0x080000, CRC(9fe4c04a) SHA1(823d1d46e49e21f921260296874bc3ee5f718a5f) )
ROM_END

ROM_START( gl5005x )
	ROM_REGION(0x200000, "bios", 0)
	ROM_LOAD( "27-6426-00.u1", 0x000000, 0x200000, CRC(adde3581) SHA1(80f2bde7c5c339534614f24a9ca6ea362ee2f816) )
ROM_END

ROM_START( glpn )
	ROM_REGION( 0x200000, "bios", 0 )
	ROM_LOAD( "27-5755-01.u1", 0x00000, 0x80000, CRC(dc28346b) SHA1(148fe664bef5b2f68c6702c74462802b76900ca0) )
ROM_END

ROM_START( gmtt )
	ROM_REGION(0x100000, "bios", 0)
	ROM_LOAD( "27-6154-00.u4", 0x000000, 0x100000, CRC(e908262d) SHA1(a7964c9f9d304b6b2cce61822e8c6151b50388be) )
ROM_END

ROM_START( gbs5505x )
	ROM_REGION(0x200000, "bios", 0)
	ROM_LOAD( "27-7006-00.u5", 0x000000, 0x200000, CRC(28af3ca7) SHA1(5df7063c7327263c23d5ac2aac3aa66f7e0821c5) )
ROM_END

ROM_START( gln ) // not Z80 code
	ROM_REGION( 0x80000, "bios", 0 )
	ROM_LOAD( "27-5308-00_9524_d.bin", 0x000000, 0x080000, CRC(d1b994ee) SHA1(b5cf0810df0676712e4f30e279cc46c19b4277dd))
ROM_END

ROM_START( pc1000 )
	ROM_REGION( 0x20000, "bios", 0 )
	ROM_LOAD( "27-00780-002-002.u4", 0x000000, 0x020000, CRC(705170ae) SHA1(825ce0ff2c7d0a7b1e2577d1465a37f7e8da383b))
ROM_END

ROM_START( misterx )
	ROM_REGION( 0x20000, "bios", 0 )
	ROM_LOAD( "27-00882-001.bin", 0x000000, 0x020000, CRC(30e0dc94) SHA1(2f4675746a41399b3d9e3e8001a9b4a0dcc5b620))
ROM_END

ROM_START( ordisava )
	ROM_REGION( 0x20000, "bios", 0 )
	ROM_LOAD( "27-00874-001.u4", 0x000000, 0x020000, CRC(5e40764e) SHA1(636ea61d3d675e51c20f610aae6824369c01a804))
ROM_END

ROM_START( lexipcm )
	ROM_REGION( 0x200000, "bios", 0 )
	ROM_LOAD( "epoxy.u3", 0x00000, 0x100000, CRC(0a410790) SHA1(be04d5f74208a2f3b200daed75e04e966f64b545) )
ROM_END


/* Driver */

//    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT    CLASS          INIT  COMPANY                    FULLNAME                                  FLAGS
COMP( 1988, pc1000,   0,      0,      pc1000,   pc1000,  pc1000_state,  0,    "Video Technology",        "PreComputer 1000",                       MACHINE_NOT_WORKING )
COMP( 1988, misterx,  0,      0,      misterx,  pc1000,  pc1000_state,  0,    "Video Technology / Yeno", "MisterX",                                MACHINE_NOT_WORKING )
COMP( 1988, ordisava, 0,      0,      pc1000,   pc1000,  pc1000_state,  0,    "Video Technology",        "Ordisavant (France)",                    MACHINE_NOT_WORKING )
COMP( 1993, pc2000,   0,      0,      pc2000,   pc2000,  pc2000_state,  0,    "Video Technology",        "PreComputer 2000",                       MACHINE_NOT_WORKING )
COMP( 1993, gl2000,   0,      0,      gl2000,   pc2000,  pc2000_state,  0,    "Video Technology",        "Genius Leader 2000",                     MACHINE_NOT_WORKING )
COMP( 1994, gl2000c,  gl2000, 0,      gl2000,   pc2000,  pc2000_state,  0,    "Video Technology",        "Genius Leader 2000 Compact",             MACHINE_NOT_WORKING )
COMP( 1995, gl2000p,  gl2000, 0,      gl2000,   pc2000,  pc2000_state,  0,    "Video Technology",        "Genius Leader 2000 Plus",                MACHINE_NOT_WORKING )
COMP( 1996, gl3000s,  0,      0,      gl3000s,  gl3000s, gl3000s_state, 0,    "Video Technology",        "Genius Leader 3000S (Germany)",          MACHINE_NOT_WORKING )
COMP( 1994, gl4000,   0,      0,      gl4000,   pc2000,  gl4004_state,  0,    "Video Technology",        "Genius Leader 4000 Quadro (Germany)",    MACHINE_NOT_WORKING )
COMP( 1996, gl4004,   0,      0,      gl4000,   pc2000,  gl4004_state,  0,    "Video Technology",        "Genius Leader 4004 Quadro L (Germany)",  MACHINE_NOT_WORKING )
COMP( 1997, gl5000,   0,      0,      pc2000,   pc2000,  pc2000_state,  0,    "Video Technology",        "Genius Leader 5000 (Germany)",           MACHINE_IS_SKELETON )
COMP( 1997, gl5005x,  0,      0,      pc2000,   pc2000,  pc2000_state,  0,    "Video Technology",        "Genius Leader 5005X (Germany)",          MACHINE_IS_SKELETON )
COMP( 1997, glpn,     0,      0,      gl4000,   pc2000,  gl4004_state,  0,    "Video Technology",        "Genius Leader Power Notebook (Germany)", MACHINE_IS_SKELETON )
COMP( 1998, gmtt ,    0,      0,      gl4000,   pc2000,  gl4004_state,  0,    "Video Technology",        "Genius Master Table Top (Germany)",      MACHINE_IS_SKELETON )
COMP( 2001, gbs5505x, 0,      0,      pc2000,   pc2000,  pc2000_state,  0,    "Video Technology",        "Genius BrainStation 5505X (Germany)",    MACHINE_IS_SKELETON )
COMP( 1993, gln,      0,      0,      pc2000,   pc2000,  pc2000_state,  0,    "Video Technology",        "Genius Leader Notebook",                 MACHINE_IS_SKELETON )
COMP( 1999, lexipcm,  0,      0,      pc2000,   pc2000,  pc2000_state,  0,    "Lexibook",                "LexiPC Mega 2000 (Germany)",             MACHINE_IS_SKELETON )
