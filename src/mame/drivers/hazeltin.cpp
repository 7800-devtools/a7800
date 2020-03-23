// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/***************************************************************************

    Hazeltine 1500
    original machine (c) 1977 Hazeltine Corporation

    perliminary driver by Ryan Holtz

TODO (roughly in order of importance):
    - Figure out the correct keyboard decoding.
    - Proper RS232 hookup.
    - Iron out proper horizontal and vertical start/end values.
    - Hook up /FGBIT, REV FLD FRAME, and REV VIDEO SELECT lines on video
      board.
    - Reimplement logic probe (since removed) as a netlist device so other
      devs can use it.
    - Implement /BRESET line in netlist to possibly smooth out some sync
      issues.

References:
    [1]: Hazeltine_1500_Series_Maintenance_Manual_Dec77.pdf, on Bitsavers

****************************************************************************/

#include "emu.h"

#include "cpu/i8085/i8085.h"
#include "machine/ay31015.h"
#include "machine/clock.h"
#include "machine/com8116.h"
#include "machine/kb3600.h"
#include "machine/netlist.h"
#include "machine/nl_hazelvid.h"
#include "netlist/devices/net_lib.h"

#include "screen.h"


#define CPU_TAG         "maincpu"
#define NETLIST_TAG     "videobrd"
#define UART_TAG        "uart"
#define BAUDGEN_TAG     "baudgen"
#define KBDC_TAG        "ay53600"
#define CHARROM_TAG     "chargen"
#define BAUDPORT_TAG    "baud"
#define MISCPORT_TAG    "misc"
#define MISCKEYS_TAG    "misc_keys"
#define SCREEN_TAG      "screen"
#define BAUD_PROM_TAG   "u39"
//#define NL_PROM_TAG     "videobrd:u71"
//#define NL_EPROM_TAG    "videobrd:u78"
// VIDEO_PROM at u71
#define VIDEO_PROM_TAG  NETLIST_TAG ":u90_702128_82s129.bin"
// CHAR_EPROM at u78
#define CHAR_EPROM_TAG  NETLIST_TAG ":u83_chr.bin"
#define VIDEO_OUT_TAG   "videobrd:video_out"
#define VBLANK_OUT_TAG  "videobrd:vblank"
#define TVINTERQ_OUT_TAG "videobrd:tvinterq"

#define VIDEO_CLOCK     (XTAL_33_264MHz/2)
#define VIDEOBRD_CLOCK  (XTAL_33_264MHz*30)

#define SR2_FULL_DUPLEX (0x01)
#define SR2_UPPER_ONLY  (0x08)

#define SR3_PB_RESET    (0x04)

#define KBD_STATUS_KBDR     (0x01)
#define KBD_STATUS_TV_UB    (0x40)
#define KBD_STATUS_TV_INT   (0x80)

#define SCREEN_HTOTAL   (9*100)
#define SCREEN_HDISP    (9*80)
#define SCREEN_HSTART   (9*5)

#define SCREEN_VTOTAL   (28*11)
#define SCREEN_VDISP    (24*11)
#define SCREEN_VSTART   (3*11)

class hazl1500_state : public driver_device
{
public:
	hazl1500_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, CPU_TAG)
		, m_video_board(*this, NETLIST_TAG)
		, m_u9(*this, "videobrd:u9")
		, m_u10(*this, "videobrd:u10")
		, m_u11(*this, "videobrd:u11")
		, m_u12(*this, "videobrd:u12")
		, m_u13(*this, "videobrd:u13")
		, m_u14(*this, "videobrd:u14")
		, m_u15(*this, "videobrd:u15")
		, m_u16(*this, "videobrd:u16")
		, m_u22(*this, "videobrd:u22")
		, m_u23(*this, "videobrd:u23")
		, m_u24(*this, "videobrd:u24")
		, m_u25(*this, "videobrd:u25")
		, m_u26(*this, "videobrd:u26")
		, m_u27(*this, "videobrd:u27")
		, m_u28(*this, "videobrd:u28")
		, m_u29(*this, "videobrd:u29")
		, m_cpu_db0(*this, "videobrd:cpu_db0")
		, m_cpu_db1(*this, "videobrd:cpu_db1")
		, m_cpu_db2(*this, "videobrd:cpu_db2")
		, m_cpu_db3(*this, "videobrd:cpu_db3")
		, m_cpu_db4(*this, "videobrd:cpu_db4")
		, m_cpu_db5(*this, "videobrd:cpu_db5")
		, m_cpu_db6(*this, "videobrd:cpu_db6")
		, m_cpu_db7(*this, "videobrd:cpu_db7")
		, m_cpu_ba4(*this, "videobrd:cpu_ba4")
		, m_cpu_iowq(*this, "videobrd:cpu_iowq")
		, m_video_out(*this, VIDEO_OUT_TAG)
		, m_vblank_out(*this, VBLANK_OUT_TAG)
		, m_tvinterq_out(*this, TVINTERQ_OUT_TAG)
		, m_uart(*this, UART_TAG)
		, m_kbdc(*this, KBDC_TAG)
		, m_baud_dips(*this, BAUDPORT_TAG)
		, m_baud_prom(*this, BAUD_PROM_TAG)
		, m_misc_dips(*this, MISCPORT_TAG)
		, m_kbd_misc_keys(*this, MISCKEYS_TAG)
		, m_screen(*this, SCREEN_TAG)
		, m_iowq_timer(nullptr)
		, m_status_reg_3(0)
		, m_kbd_status_latch(0)
		, m_refresh_address(0)
		, m_screen_buf(nullptr)
		, m_last_beam(0.0)
		, m_last_hpos(0)
		, m_last_vpos(0)
		, m_last_fraction(0.0)
	{
	}

	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	static const device_timer_id TIMER_IOWQ = 0;

	uint32_t screen_update_hazl1500(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	DECLARE_WRITE_LINE_MEMBER(com5016_fr_w);

	DECLARE_READ8_MEMBER(ram_r);
	DECLARE_WRITE8_MEMBER(ram_w);

	DECLARE_READ8_MEMBER(system_test_r); // noted as "for use with auto test equip" in flowchart on pg. 30, ref[1], jumps to 0x8000 if bit 0 is unset
	DECLARE_READ8_MEMBER(status_reg_2_r);
	DECLARE_WRITE8_MEMBER(status_reg_3_w);

	DECLARE_READ8_MEMBER(uart_r);
	DECLARE_WRITE8_MEMBER(uart_w);

	DECLARE_READ8_MEMBER(kbd_status_latch_r);
	DECLARE_READ8_MEMBER(kbd_encoder_r);
	DECLARE_READ_LINE_MEMBER(ay3600_shift_r);
	DECLARE_READ_LINE_MEMBER(ay3600_control_r);
	DECLARE_WRITE_LINE_MEMBER(ay3600_data_ready_w);

	DECLARE_WRITE8_MEMBER(refresh_address_w);

	NETDEV_ANALOG_CALLBACK_MEMBER(video_out_cb);
	NETDEV_ANALOG_CALLBACK_MEMBER(vblank_cb);
	NETDEV_ANALOG_CALLBACK_MEMBER(tvinterq_cb);

private:
	required_device<cpu_device> m_maincpu;
	required_device<netlist_mame_device> m_video_board;
	required_device<netlist_mame_ram_pointer_device> m_u9;
	required_device<netlist_mame_ram_pointer_device> m_u10;
	required_device<netlist_mame_ram_pointer_device> m_u11;
	required_device<netlist_mame_ram_pointer_device> m_u12;
	required_device<netlist_mame_ram_pointer_device> m_u13;
	required_device<netlist_mame_ram_pointer_device> m_u14;
	required_device<netlist_mame_ram_pointer_device> m_u15;
	required_device<netlist_mame_ram_pointer_device> m_u16;
	required_device<netlist_mame_ram_pointer_device> m_u22;
	required_device<netlist_mame_ram_pointer_device> m_u23;
	required_device<netlist_mame_ram_pointer_device> m_u24;
	required_device<netlist_mame_ram_pointer_device> m_u25;
	required_device<netlist_mame_ram_pointer_device> m_u26;
	required_device<netlist_mame_ram_pointer_device> m_u27;
	required_device<netlist_mame_ram_pointer_device> m_u28;
	required_device<netlist_mame_ram_pointer_device> m_u29;
	required_device<netlist_mame_logic_input_device> m_cpu_db0;
	required_device<netlist_mame_logic_input_device> m_cpu_db1;
	required_device<netlist_mame_logic_input_device> m_cpu_db2;
	required_device<netlist_mame_logic_input_device> m_cpu_db3;
	required_device<netlist_mame_logic_input_device> m_cpu_db4;
	required_device<netlist_mame_logic_input_device> m_cpu_db5;
	required_device<netlist_mame_logic_input_device> m_cpu_db6;
	required_device<netlist_mame_logic_input_device> m_cpu_db7;
	required_device<netlist_mame_logic_input_device> m_cpu_ba4;
	required_device<netlist_mame_logic_input_device> m_cpu_iowq;
	required_device<netlist_mame_analog_output_device> m_video_out;
	required_device<netlist_mame_analog_output_device> m_vblank_out;
	required_device<netlist_mame_analog_output_device> m_tvinterq_out;
	required_device<ay31015_device> m_uart;
	required_device<ay3600_device> m_kbdc;
	required_ioport m_baud_dips;
	required_region_ptr<uint8_t> m_baud_prom;
	required_ioport m_misc_dips;
	required_ioport m_kbd_misc_keys;

	required_device<screen_device> m_screen;

	emu_timer* m_iowq_timer;

	uint8_t m_status_reg_3;
	uint8_t m_kbd_status_latch;

	uint8_t m_refresh_address;

	std::unique_ptr<float[]> m_screen_buf;

	double m_last_beam;
	int m_last_hpos;
	int m_last_vpos;
	double m_last_fraction;
};

void hazl1500_state::machine_start()
{
	m_screen_buf = std::make_unique<float[]>(SCREEN_HTOTAL * SCREEN_VTOTAL);

	m_iowq_timer = timer_alloc(TIMER_IOWQ);
	m_iowq_timer->adjust(attotime::never);

	save_item(NAME(m_status_reg_3));
	save_item(NAME(m_kbd_status_latch));
	save_item(NAME(m_refresh_address));
	save_item(NAME(m_last_beam));
	save_item(NAME(m_last_hpos));
	save_item(NAME(m_last_vpos));
	save_item(NAME(m_last_fraction));
}

void hazl1500_state::machine_reset()
{
	m_status_reg_3 = 0;
	m_kbd_status_latch = 0;
}

void hazl1500_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	m_cpu_iowq->write(1);
	m_cpu_ba4->write(1);
}

WRITE_LINE_MEMBER( hazl1500_state::com5016_fr_w )
{
	m_uart->rx_process();
	m_uart->tx_process();
}

uint32_t hazl1500_state::screen_update_hazl1500(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	int last_index = m_last_vpos * SCREEN_HTOTAL + m_last_hpos;
	while (last_index < SCREEN_HTOTAL * SCREEN_VTOTAL)
	{
		m_screen_buf[last_index++] = m_last_beam;
	}
	m_last_hpos = 0;
	m_last_vpos = 0;

	uint32_t pixindex = 0;
	for (int y = 0; y < SCREEN_VTOTAL; y++)
	{
		uint32_t *scanline = &bitmap.pix32(y);
		pixindex = y * SCREEN_HTOTAL;
		for (int x = 0; x < SCREEN_HTOTAL; x++)
			//*scanline++ = 0xff000000 | (uint8_t(m_screen_buf[pixindex++] * 0.5) * 0x010101);
			*scanline++ = 0xff000000 | (uint8_t(m_screen_buf[pixindex++] * 63.0) * 0x010101);
	}

	return 0;
}

READ8_MEMBER( hazl1500_state::ram_r )
{
	const uint8_t* chips[2][8] =
	{
		{ m_u29->ptr(), m_u28->ptr(), m_u27->ptr(), m_u26->ptr(), m_u25->ptr(), m_u24->ptr(), m_u23->ptr(), m_u22->ptr() },
		{ m_u16->ptr(), m_u15->ptr(), m_u14->ptr(), m_u13->ptr(), m_u12->ptr(), m_u11->ptr(), m_u10->ptr(), m_u9->ptr() }
	};

	int bank = ((offset & 0x400) != 0 ? 1 : 0);
	const int byte_pos = (offset >> 3) & 0x7f;
	const int bit_pos = offset & 7;

	uint8_t ret = 0;
	for (std::size_t bit = 0; bit < 8; bit++)
		ret |= ((chips[bank][bit][byte_pos] >> bit_pos) & 1) << bit;

	return ret;
}

WRITE8_MEMBER( hazl1500_state::ram_w )
{
	uint8_t* chips[2][8] =
	{
		{ m_u29->ptr(), m_u28->ptr(), m_u27->ptr(), m_u26->ptr(), m_u25->ptr(), m_u24->ptr(), m_u23->ptr(), m_u22->ptr() },
		{ m_u16->ptr(), m_u15->ptr(), m_u14->ptr(), m_u13->ptr(), m_u12->ptr(), m_u11->ptr(), m_u10->ptr(), m_u9->ptr() }
	};

	int bank = ((offset & 0x400) != 0 ? 1 : 0);
	const int byte_pos = (offset >> 3) & 0x7f;
	const int bit_pos = offset & 7;

	for (std::size_t bit = 0; bit < 8; bit++)
	{
		chips[bank][bit][byte_pos] &= ~(1 << bit_pos);
		chips[bank][bit][byte_pos] |= ((data >> bit) & 1) << bit_pos;
	}
}

READ8_MEMBER( hazl1500_state::system_test_r )
{
	return 0xff;
}

READ8_MEMBER( hazl1500_state::status_reg_2_r )
{
	uint8_t misc_dips = m_misc_dips->read();
	uint8_t status = 0;

	if (misc_dips & 0x10)
		status |= SR2_FULL_DUPLEX;
	if (misc_dips & 0x40)
		status |= SR2_UPPER_ONLY;

	return status ^ 0xff;
}

WRITE8_MEMBER( hazl1500_state::status_reg_3_w )
{
	m_status_reg_3 = data;
}

READ8_MEMBER( hazl1500_state::uart_r )
{
	return m_uart->get_received_data();
}

WRITE8_MEMBER( hazl1500_state::uart_w )
{
	m_uart->set_transmit_data(data);
}

READ8_MEMBER( hazl1500_state::kbd_status_latch_r )
{
	return m_kbd_status_latch;
}

READ8_MEMBER(hazl1500_state::kbd_encoder_r)
{
	return m_kbdc->b_r() & 0xff; // TODO: This should go through an 8048, but we have no dump of it currently.
}

READ_LINE_MEMBER(hazl1500_state::ay3600_shift_r)
{
	// either shift key
	if (m_kbd_misc_keys->read() & 0x06)
	{
		return 1;
	}

	return 0;
}

READ_LINE_MEMBER(hazl1500_state::ay3600_control_r)
{
	if (m_kbd_misc_keys->read() & 0x08)
	{
		return 1;
	}

	return 0;
}

WRITE_LINE_MEMBER(hazl1500_state::ay3600_data_ready_w)
{
	if (state)
		m_kbd_status_latch |= KBD_STATUS_KBDR;
	else
		m_kbd_status_latch &= ~KBD_STATUS_KBDR;
}

NETDEV_ANALOG_CALLBACK_MEMBER(hazl1500_state::vblank_cb)
{
	synchronize();
	if (int(data) > 1)
	{
		m_kbd_status_latch &= ~KBD_STATUS_TV_UB;
	}
	else
	{
		m_kbd_status_latch |= KBD_STATUS_TV_UB;
	}
}

NETDEV_ANALOG_CALLBACK_MEMBER(hazl1500_state::tvinterq_cb)
{
	synchronize();
	if (int(data) > 1)
	{
		m_kbd_status_latch &= ~KBD_STATUS_TV_INT;
		m_maincpu->set_input_line(INPUT_LINE_IRQ0, CLEAR_LINE);
	}
	else
	{
		m_kbd_status_latch |= KBD_STATUS_TV_INT;
		m_maincpu->set_input_line(INPUT_LINE_IRQ0, ASSERT_LINE);
	}
}

NETDEV_ANALOG_CALLBACK_MEMBER(hazl1500_state::video_out_cb)
{
	synchronize();
	attotime second_fraction(0, time.attoseconds());
	attotime frame_fraction(0, (second_fraction * 60).attoseconds());
	attotime pixel_time = frame_fraction * (SCREEN_HTOTAL * SCREEN_VTOTAL);
	int32_t pixel_index = (frame_fraction * (SCREEN_HTOTAL * SCREEN_VTOTAL)).seconds();
	double pixel_fraction = ATTOSECONDS_TO_DOUBLE(pixel_time.attoseconds());

	pixel_index -= 16; // take back 16 clock cycles to honor the circuitry god whose ark this is
	if (pixel_index < 0)
	{
		m_last_beam = float(data);
		m_last_hpos = 0;
		m_last_vpos = 0;
		m_last_fraction = 0.0;
		return;
	}

	const int hpos = pixel_index % SCREEN_HTOTAL;//m_screen->hpos();
	const int vpos = pixel_index / SCREEN_HTOTAL;//m_screen->vpos();
	const int curr_index = vpos * SCREEN_HTOTAL + hpos;

	int last_index = m_last_vpos * SCREEN_HTOTAL + m_last_hpos;
	if (last_index != curr_index)
	{
		m_screen_buf[last_index] *= m_last_fraction;
		m_screen_buf[last_index] += float(m_last_beam * (1.0 - m_last_fraction));
		last_index++;
		while (last_index <= curr_index)
			m_screen_buf[last_index++] = float(m_last_beam);
	}

	m_last_beam = float(data);
	m_last_hpos = hpos;
	m_last_vpos = vpos;
	m_last_fraction = pixel_fraction;
}

WRITE8_MEMBER(hazl1500_state::refresh_address_w)
{
	synchronize();
	//printf("refresh: %02x, %d, %d\n", data, m_screen->hpos(), m_screen->vpos());
	m_iowq_timer->adjust(attotime::from_hz(XTAL_18MHz/9));
	m_cpu_iowq->write(0);
	m_cpu_ba4->write(0);
	m_cpu_db0->write((data >> 0) & 1);
	m_cpu_db1->write((data >> 1) & 1);
	m_cpu_db2->write((data >> 2) & 1);
	m_cpu_db3->write((data >> 3) & 1);
	m_cpu_db4->write((data >> 4) & 1);
	m_cpu_db5->write((data >> 5) & 1);
	m_cpu_db6->write((data >> 6) & 1);
	m_cpu_db7->write((data >> 7) & 1);
}

static ADDRESS_MAP_START(hazl1500_mem, AS_PROGRAM, 8, hazl1500_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x07ff) AM_ROM
	AM_RANGE(0x3000, 0x377f) AM_READWRITE(ram_r, ram_w)
	AM_RANGE(0x3780, 0x37ff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START(hazl1500_io, AS_IO, 8, hazl1500_state)
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x7f, 0x7f) AM_READWRITE(status_reg_2_r, status_reg_3_w)
	AM_RANGE(0xbf, 0xbf) AM_READWRITE(uart_r, uart_w)
	AM_RANGE(0xdf, 0xdf) AM_READ(kbd_encoder_r)
	AM_RANGE(0xef, 0xef) AM_READWRITE(system_test_r, refresh_address_w)
	AM_RANGE(0xf7, 0xf7) AM_READ(kbd_status_latch_r)
ADDRESS_MAP_END

	/*
	  Hazeltine 1500 key matrix (from ref[1])

	      | Y0  | Y1  | Y2  | Y3  | Y4  | Y5  | Y6  | Y7  | Y8  | Y9  |
	      |     |     |     |     |     |     |     |     |     |     |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X1  | TAB |  1  |  3  |  5  |  7  |BREAK| ^~  |     |     |     |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X2  |BAKSP|  2  |  4  |  6  |  8  |  9  | -=  |     |     |     |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X3  |     | ESC |  W  |  R  |  Y  |  0  | `@  | CLR |NP , |NP 9 |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X4  |     |  Q  |  E  |  T  |  U  |  O  | {[  | |\  |NP 7 |NP 6 |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X5  |HOME |  A  |  D  |  G  |  I  |  P  | *:  | LF  |NP 8 |NP 5 |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X6  |     |  S  |  F  |  H  |  J  | +;  | }]  | DEL |NP 4 |NP 2 |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X7  |     |  Z  |  C  |  B  |  K  |  L  | <,  | CR  |NP 1 |NP 3 |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	  X8  |     |  X  |  V  |  N  |SPACE|  M  | >.  | ?/  |NP 0 |NP . |
	  ----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----|
	*/

static INPUT_PORTS_START( hazl1500 )
	PORT_START("X0")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_UNUSED)

	// X1  | TAB |  1  |  3  |  5  |  7  |BREAK| ^~  |     |     |     |
	PORT_START("X1")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_TAB) PORT_CHAR('\t') PORT_NAME("Tab")
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1)  PORT_CHAR('1') PORT_CHAR('!')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3)  PORT_CHAR('3') PORT_CHAR('#')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5)  PORT_CHAR('5') PORT_CHAR('%')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_7)  PORT_CHAR('7') PORT_CHAR('\'')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_PAUSE) PORT_NAME("Break")
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS) PORT_CHAR('^') PORT_CHAR('~')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_UNUSED)

	// X2  |BAKSP|  2  |  4  |  6  |  8  |  9  | -=  |     |     |     |
	PORT_START("X2")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSPACE)  PORT_NAME("Backspace")
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2)          PORT_CHAR('2') PORT_CHAR('\"')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4)          PORT_CHAR('4') PORT_CHAR('$')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6)          PORT_CHAR('6') PORT_CHAR('&')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_8)          PORT_CHAR('8') PORT_CHAR('(')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_9)          PORT_CHAR('9') PORT_CHAR(')')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS)      PORT_CHAR('-') PORT_CHAR('=')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_UNUSED)

	// X3  |     | ESC |  W  |  R  |  Y  |  0  | `@  | CLR |NP , |NP 9 |
	PORT_START("X3")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ESC)        PORT_NAME("Esc")
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_W)          PORT_CHAR('W') PORT_CHAR('w')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_R)          PORT_CHAR('R') PORT_CHAR('r')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y)          PORT_CHAR('Y') PORT_CHAR('y')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_0)          PORT_CHAR('0')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE)  PORT_CHAR('@') PORT_CHAR('`')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_DEL)        PORT_NAME("Clr")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_DEL_PAD)
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_9_PAD)      PORT_CHAR(UCHAR_MAMEKEY(9_PAD))

	// X4  |     |  Q  |  E  |  T  |  U  |  O  | {[  | |\  |NP 7 |NP 6 |
	PORT_START("X4")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q)          PORT_CHAR('Q') PORT_CHAR('q')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_E)          PORT_CHAR('E') PORT_CHAR('e')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_T)          PORT_CHAR('T') PORT_CHAR('t')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_U)          PORT_CHAR('U') PORT_CHAR('u')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_O)          PORT_CHAR('O') PORT_CHAR('o')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE)  PORT_CHAR('[') PORT_CHAR('{')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH)  PORT_CHAR('\\') PORT_CHAR('|')
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_7_PAD)      PORT_CHAR(UCHAR_MAMEKEY(7_PAD))
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6_PAD)      PORT_CHAR(UCHAR_MAMEKEY(6_PAD))

	// X5  |HOME |  A  |  D  |  G  |  I  |  P  | *:  | LF  |NP 8 |NP 5 |
	PORT_START("X5")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_HOME)   PORT_NAME("Home")
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_A)      PORT_CHAR('A') PORT_CHAR('a')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_D)      PORT_CHAR('D') PORT_CHAR('d')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_G)      PORT_CHAR('G') PORT_CHAR('g')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_I)      PORT_CHAR('I') PORT_CHAR('i')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_P)      PORT_CHAR('P') PORT_CHAR('p')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE)  PORT_CHAR(':') PORT_CHAR('*')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_PGDN)   PORT_NAME("Line Feed")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_8_PAD)  PORT_CHAR(UCHAR_MAMEKEY(8_PAD))
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5_PAD)  PORT_CHAR(UCHAR_MAMEKEY(5_PAD))

	// X6  |     |  S  |  F  |  H  |  J  | +;  | }]  | DEL |NP 4 |NP 2 |
	PORT_START("X6")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_S)      PORT_CHAR('S') PORT_CHAR('s')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F)      PORT_CHAR('F') PORT_CHAR('f')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_H)      PORT_CHAR('H') PORT_CHAR('h')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_J)      PORT_CHAR('J') PORT_CHAR('j')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON)  PORT_CHAR(';') PORT_CHAR('+')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR(']') PORT_CHAR('}')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_DEL)    PORT_NAME("Del")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4_PAD)  PORT_CHAR(UCHAR_MAMEKEY(4_PAD))
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2_PAD)  PORT_CHAR(UCHAR_MAMEKEY(2_PAD))

	// X7  |     |  Z  |  C  |  B  |  K  |  L  | <,  | CR  |NP 1 |NP 3 |
	PORT_START("X7")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z)      PORT_CHAR('Z') PORT_CHAR('z')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_C)      PORT_CHAR('C') PORT_CHAR('c')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_B)      PORT_CHAR('B') PORT_CHAR('b')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_K)      PORT_CHAR('K') PORT_CHAR('k')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_L)      PORT_CHAR('L') PORT_CHAR('l')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA)  PORT_CHAR(',') PORT_CHAR('<')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER)  PORT_CHAR(13)  PORT_NAME("Return")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1_PAD)  PORT_CHAR(UCHAR_MAMEKEY(1_PAD))
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3_PAD)  PORT_CHAR(UCHAR_MAMEKEY(3_PAD))

	// X8  |     |  X  |  V  |  N  |SPACE|  M  | >.  | ?/  |NP 0 |NP . |
	PORT_START("X8")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_X)      PORT_CHAR('X') PORT_CHAR('x')
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_V)      PORT_CHAR('V') PORT_CHAR('v')
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_N)      PORT_CHAR('N') PORT_CHAR('n')
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SPACE)  PORT_CHAR(' ')
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_M)      PORT_CHAR('M') PORT_CHAR('m')
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP)   PORT_CHAR('.') PORT_CHAR('>')
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH)  PORT_CHAR('/') PORT_CHAR('?')
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_0_PAD)  PORT_CHAR(UCHAR_MAMEKEY(0_PAD))
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER_PAD) PORT_CHAR(UCHAR_MAMEKEY(DEL_PAD))

	PORT_START(MISCKEYS_TAG)
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Left Shift")   PORT_CODE(KEYCODE_LSHIFT)   PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Right Shift")  PORT_CODE(KEYCODE_RSHIFT)   PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Control")      PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START(BAUDPORT_TAG)
	PORT_DIPNAME( 0xff, 0x08, "Baud Rate" )
	PORT_DIPSETTING( 0x01, "110" )
	PORT_DIPSETTING( 0x02, "300" )
	PORT_DIPSETTING( 0x04, "1200" )
	PORT_DIPSETTING( 0x08, "1800" )
	PORT_DIPSETTING( 0x10, "2400" )
	PORT_DIPSETTING( 0x20, "4800" )
	PORT_DIPSETTING( 0x40, "9600" )
	PORT_DIPSETTING( 0x80, "19.2K" )

	PORT_START(MISCPORT_TAG)
	PORT_DIPNAME( 0x0f, 0x01, "Parity" )
	PORT_DIPSETTING( 0x01, "Even" )
	PORT_DIPSETTING( 0x02, "Odd" )
	PORT_DIPSETTING( 0x04, "1" )
	PORT_DIPSETTING( 0x08, "0" )
	PORT_DIPNAME( 0x10, 0x10, "Duplex" )
	PORT_DIPSETTING( 0x00, "Half" )
	PORT_DIPSETTING( 0x10, "Full" )
	PORT_DIPNAME( 0x20, 0x20, "Auto" )
	PORT_DIPSETTING( 0x00, "LF" )
	PORT_DIPSETTING( 0x20, "CR" )
	PORT_DIPNAME( 0x40, 0x40, "Case" )
	PORT_DIPSETTING( 0x00, "Upper and Lower" )
	PORT_DIPSETTING( 0x40, "Upper" )
	PORT_DIPNAME( 0x80, 0x80, "Video" )
	PORT_DIPSETTING( 0x00, "Standard" )
	PORT_DIPSETTING( 0x80, "Reverse" )
INPUT_PORTS_END

/* F4 Character Displayer */
static const gfx_layout hazl1500_charlayout =
{
	8, 16,
	128,
	1,
	{ 0 },
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8, 8*8, 9*8, 10*8, 11*8, 12*8, 13*8, 14*8, 15*8 },
	8*16
};

static GFXDECODE_START( hazl1500 )
	GFXDECODE_ENTRY( CHAR_EPROM_TAG, 0x0000, hazl1500_charlayout, 0, 1 )
GFXDECODE_END

static MACHINE_CONFIG_START( hazl1500 )
	/* basic machine hardware */
	MCFG_CPU_ADD(CPU_TAG, I8080, XTAL_18MHz/9) // 18MHz crystal on schematics, using an i8224 clock gen/driver IC
	MCFG_CPU_PROGRAM_MAP(hazl1500_mem)
	MCFG_CPU_IO_MAP(hazl1500_io)
	MCFG_QUANTUM_PERFECT_CPU(CPU_TAG)

	/* video hardware */
	MCFG_SCREEN_ADD(SCREEN_TAG, RASTER)
	MCFG_SCREEN_UPDATE_DRIVER(hazl1500_state, screen_update_hazl1500)
	//MCFG_SCREEN_RAW_PARAMS(XTAL_33_264MHz / 2,
	//    SCREEN_HTOTAL, SCREEN_HSTART, SCREEN_HSTART + SCREEN_HDISP,
	//    SCREEN_VTOTAL, SCREEN_VSTART, SCREEN_VSTART + SCREEN_VDISP); // TODO: Figure out exact visibility
	MCFG_SCREEN_RAW_PARAMS(XTAL_33_264MHz / 2,
		SCREEN_HTOTAL, 0, SCREEN_HTOTAL,
		SCREEN_VTOTAL, 0, SCREEN_VTOTAL);

	MCFG_PALETTE_ADD_MONOCHROME("palette")
	MCFG_GFXDECODE_ADD("gfxdecode", "palette", hazl1500)

	MCFG_DEVICE_ADD(BAUDGEN_TAG, COM8116, XTAL_5_0688MHz)
	MCFG_COM8116_FR_HANDLER(WRITELINE(hazl1500_state, com5016_fr_w))

	MCFG_DEVICE_ADD(UART_TAG, AY51013, 0)

	MCFG_DEVICE_ADD(NETLIST_TAG, NETLIST_CPU, VIDEOBRD_CLOCK)
	MCFG_NETLIST_SETUP(hazelvid)

	// First 1K
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u22", "u22")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u23", "u23")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u24", "u24")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u25", "u25")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u26", "u26")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u27", "u27")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u28", "u28")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u29", "u29")

	// Second 1K
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u9",  "u9")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u10", "u10")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u11", "u11")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u12", "u12")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u13", "u13")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u14", "u14")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u15", "u15")
	MCFG_NETLIST_RAM_POINTER(NETLIST_TAG, "u16", "u16")

	MCFG_NETLIST_LOGIC_INPUT(NETLIST_TAG, "cpu_iowq", "cpu_iowq.IN", 0)
	MCFG_NETLIST_LOGIC_INPUT(NETLIST_TAG, "cpu_ba4", "cpu_ba4.IN", 0)
	MCFG_NETLIST_LOGIC_INPUT(NETLIST_TAG, "cpu_db0", "cpu_db0.IN", 0)
	MCFG_NETLIST_LOGIC_INPUT(NETLIST_TAG, "cpu_db1", "cpu_db1.IN", 0)
	MCFG_NETLIST_LOGIC_INPUT(NETLIST_TAG, "cpu_db2", "cpu_db2.IN", 0)
	MCFG_NETLIST_LOGIC_INPUT(NETLIST_TAG, "cpu_db3", "cpu_db3.IN", 0)
	MCFG_NETLIST_LOGIC_INPUT(NETLIST_TAG, "cpu_db4", "cpu_db4.IN", 0)
	MCFG_NETLIST_LOGIC_INPUT(NETLIST_TAG, "cpu_db5", "cpu_db5.IN", 0)
	MCFG_NETLIST_LOGIC_INPUT(NETLIST_TAG, "cpu_db6", "cpu_db6.IN", 0)
	MCFG_NETLIST_LOGIC_INPUT(NETLIST_TAG, "cpu_db7", "cpu_db7.IN", 0)

	MCFG_NETLIST_ANALOG_OUTPUT(NETLIST_TAG, "video_out", "video_out", hazl1500_state, video_out_cb, "")
	MCFG_NETLIST_ANALOG_OUTPUT(NETLIST_TAG, "vblank", "vblank", hazl1500_state, vblank_cb, "")
	MCFG_NETLIST_ANALOG_OUTPUT(NETLIST_TAG, "tvinterq", "tvinterq", hazl1500_state, tvinterq_cb, "")

	/* keyboard controller */
	MCFG_DEVICE_ADD(KBDC_TAG, AY3600, 0)
	MCFG_AY3600_MATRIX_X0(IOPORT("X0"))
	MCFG_AY3600_MATRIX_X1(IOPORT("X1"))
	MCFG_AY3600_MATRIX_X2(IOPORT("X2"))
	MCFG_AY3600_MATRIX_X3(IOPORT("X3"))
	MCFG_AY3600_MATRIX_X4(IOPORT("X4"))
	MCFG_AY3600_MATRIX_X5(IOPORT("X5"))
	MCFG_AY3600_MATRIX_X6(IOPORT("X6"))
	MCFG_AY3600_MATRIX_X7(IOPORT("X7"))
	MCFG_AY3600_MATRIX_X8(IOPORT("X8"))
	MCFG_AY3600_SHIFT_CB(READLINE(hazl1500_state, ay3600_shift_r))
	MCFG_AY3600_CONTROL_CB(READLINE(hazl1500_state, ay3600_control_r))
	MCFG_AY3600_DATA_READY_CB(WRITELINE(hazl1500_state, ay3600_data_ready_w))
MACHINE_CONFIG_END


ROM_START( hazl1500 )
	ROM_REGION( 0x10000, NETLIST_TAG, ROMREGION_ERASE00 )

	ROM_REGION( 0x10000, CPU_TAG, ROMREGION_ERASEFF )
	ROM_LOAD( "h15s-00I-10-3.bin", 0x0000, 0x0800, CRC(a2015f72) SHA1(357cde517c3dcf693de580881add058c7b26dfaa))

	ROM_REGION( 0x800, CHAR_EPROM_TAG, ROMREGION_ERASEFF )
	ROM_LOAD( "u83_chr.bin", 0x0000, 0x0800, CRC(e0c6b734) SHA1(7c42947235c66c41059fd4384e09f4f3a17c9857))

	ROM_REGION( 0x100, BAUD_PROM_TAG, ROMREGION_ERASEFF )
	ROM_LOAD( "u43_702129_82s129.bin", 0x0000, 0x0100, CRC(b35aea2b) SHA1(4702620cdef72b32a397580c22b75df36e24ac74))

	ROM_REGION( 0x100, VIDEO_PROM_TAG, ROMREGION_ERASEFF )
	ROM_LOAD( "u90_702128_82s129.bin", 0x0000, 0x0100, CRC(277bc424) SHA1(528a0de3b54d159bc14411961961706bf9ec41bf))
ROM_END

/* Driver */

//    YEAR  NAME      PARENT    COMPAT   MACHINE   INPUT     CLASS           INIT    COMPANY                     FULLNAME            FLAGS
COMP( 1977, hazl1500, 0,        0,       hazl1500, hazl1500, hazl1500_state, 0,      "Hazeltine Corporation",    "Hazeltine 1500",   MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW)
