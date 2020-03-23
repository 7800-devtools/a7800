// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Victor 9000 / ACT Sirius 1 emulation

**********************************************************************/

/*

    TODO:

    - contrast
    - MC6852
    - codec sound
    - expansion bus
        - Z80 card
        - Winchester DMA card (Xebec S1410 + Tandon TM502/TM603SE)
        - RAM cards
        - clock cards
    - floppy 8048

*/

#include "emu.h"
#include "bus/centronics/ctronics.h"
#include "bus/ieee488/ieee488.h"
#include "bus/rs232/rs232.h"
#include "cpu/i86/i86.h"
#include "formats/victor9k_dsk.h"
#include "imagedev/floppy.h"
#include "machine/6522via.h"
#include "machine/mc6852.h"
#include "machine/pit8253.h"
#include "machine/pic8259.h"
#include "machine/ram.h"
#include "machine/victor9k_kb.h"
#include "machine/victor9k_fdc.h"
#include "machine/z80dart.h"
#include "sound/hc55516.h"
#include "video/mc6845.h"
#include "screen.h"
#include "softlist.h"
#include "speaker.h"

#define I8088_TAG       "8l"
#define I8253_TAG       "13h"
#define I8259A_TAG      "7l"
#define UPD7201_TAG     "16e"
#define HD46505S_TAG    "11a"
#define MC6852_TAG      "11b"
#define HC55516_TAG     "15c"
#define M6522_1_TAG     "m6522_1"
#define M6522_2_TAG     "m6522_2"
#define M6522_3_TAG     "14l"
#define DAC0808_0_TAG   "5b"
#define DAC0808_1_TAG   "5c"
#define CENTRONICS_TAG  "centronics"
#define RS232_A_TAG     "rs232a"
#define RS232_B_TAG     "rs232b"
#define SCREEN_TAG      "screen"
#define KB_TAG          "kb"
#define FDC_TAG         "fdc"

class victor9k_state : public driver_device
{
public:
	victor9k_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, I8088_TAG),
		m_ieee488(*this, IEEE488_TAG),
		m_pic(*this, I8259A_TAG),
		m_upd7201(*this, UPD7201_TAG),
		m_ssda(*this, MC6852_TAG),
		m_via1(*this, M6522_1_TAG),
		m_via2(*this, M6522_2_TAG),
		m_via3(*this, M6522_3_TAG),
		m_cvsd(*this, HC55516_TAG),
		m_crtc(*this, HD46505S_TAG),
		m_ram(*this, RAM_TAG),
		m_kb(*this, KB_TAG),
		m_fdc(*this, FDC_TAG),
		m_centronics(*this, CENTRONICS_TAG),
		m_rs232a(*this, RS232_A_TAG),
		m_rs232b(*this, RS232_B_TAG),
		m_palette(*this, "palette"),
		m_rom(*this, I8088_TAG),
		m_video_ram(*this, "video_ram"),
		m_brt(0),
		m_cont(0),
		m_via1_irq(CLEAR_LINE),
		m_via2_irq(CLEAR_LINE),
		m_via3_irq(CLEAR_LINE),
		m_fdc_irq(CLEAR_LINE),
		m_ssda_irq(CLEAR_LINE),
		m_kbrdy(1),
		m_kbackctl(0)
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<ieee488_device> m_ieee488;
	required_device<pic8259_device> m_pic;
	required_device<upd7201_device> m_upd7201;
	required_device<mc6852_device> m_ssda;
	required_device<via6522_device> m_via1;
	required_device<via6522_device> m_via2;
	required_device<via6522_device> m_via3;
	required_device<hc55516_device> m_cvsd;
	required_device<mc6845_device> m_crtc;
	required_device<ram_device> m_ram;
	required_device<victor_9000_keyboard_device> m_kb;
	required_device<victor_9000_fdc_device> m_fdc;
	required_device<centronics_device> m_centronics;
	required_device<rs232_port_device> m_rs232a;
	required_device<rs232_port_device> m_rs232b;
	required_device<palette_device> m_palette;
	required_memory_region m_rom;
	required_shared_ptr<uint8_t> m_video_ram;

	virtual void machine_start() override;
	virtual void machine_reset() override;

	DECLARE_WRITE8_MEMBER( via1_pa_w );
	DECLARE_WRITE_LINE_MEMBER( write_nfrd );
	DECLARE_WRITE_LINE_MEMBER( write_ndac );
	DECLARE_WRITE8_MEMBER( via1_pb_w );
	DECLARE_WRITE_LINE_MEMBER( via1_irq_w );
	DECLARE_WRITE_LINE_MEMBER( codec_vol_w );

	DECLARE_WRITE8_MEMBER( via2_pa_w );
	DECLARE_WRITE8_MEMBER( via2_pb_w );
	DECLARE_WRITE_LINE_MEMBER( write_ria );
	DECLARE_WRITE_LINE_MEMBER( write_rib );
	DECLARE_WRITE_LINE_MEMBER( via2_irq_w );

	DECLARE_WRITE8_MEMBER( via3_pb_w );
	DECLARE_WRITE_LINE_MEMBER( via3_irq_w );

	DECLARE_WRITE_LINE_MEMBER( fdc_irq_w );

	DECLARE_WRITE_LINE_MEMBER( ssda_irq_w );
	DECLARE_WRITE_LINE_MEMBER( ssda_sm_dtr_w );

	DECLARE_WRITE_LINE_MEMBER( kbrdy_w );
	DECLARE_WRITE_LINE_MEMBER( kbdata_w );
	DECLARE_WRITE_LINE_MEMBER( vert_w );

	MC6845_UPDATE_ROW( crtc_update_row );

	DECLARE_WRITE_LINE_MEMBER( mux_serial_b_w );
	DECLARE_WRITE_LINE_MEMBER( mux_serial_a_w );

	DECLARE_PALETTE_INIT( victor9k );

	// video state
	int m_brt;
	int m_cont;
	int m_hires;

	// interrupts
	int m_via1_irq;
	int m_via2_irq;
	int m_via3_irq;
	int m_fdc_irq;
	int m_ssda_irq;

	// keyboard
	int m_kbrdy;
	int m_kbackctl;

	void update_kback();
};



//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define LOG 0



//**************************************************************************
//  ADDRESS MAPS
//**************************************************************************

//-------------------------------------------------
//  ADDRESS_MAP( victor9k_mem )
//-------------------------------------------------

static ADDRESS_MAP_START( victor9k_mem, AS_PROGRAM, 8, victor9k_state )
	AM_RANGE(0x00000, 0x1ffff) AM_RAM
	AM_RANGE(0x20000, 0xdffff) AM_NOP
	AM_RANGE(0xe0000, 0xe0001) AM_MIRROR(0x7f00) AM_DEVREADWRITE(I8259A_TAG, pic8259_device, read, write)
	AM_RANGE(0xe0020, 0xe0023) AM_MIRROR(0x7f00) AM_DEVREADWRITE(I8253_TAG, pit8253_device, read, write)
	AM_RANGE(0xe0040, 0xe0043) AM_MIRROR(0x7f00) AM_DEVREADWRITE(UPD7201_TAG, upd7201_device, cd_ba_r, cd_ba_w)
	AM_RANGE(0xe8000, 0xe8000) AM_MIRROR(0x7f00) AM_DEVREADWRITE(HD46505S_TAG, mc6845_device, status_r, address_w)
	AM_RANGE(0xe8001, 0xe8001) AM_MIRROR(0x7f00) AM_DEVREADWRITE(HD46505S_TAG, mc6845_device, register_r, register_w)
	AM_RANGE(0xe8020, 0xe802f) AM_MIRROR(0x7f00) AM_DEVREADWRITE(M6522_1_TAG, via6522_device, read, write)
	AM_RANGE(0xe8040, 0xe804f) AM_MIRROR(0x7f00) AM_DEVREADWRITE(M6522_2_TAG, via6522_device, read, write)
	AM_RANGE(0xe8060, 0xe8061) AM_MIRROR(0x7f00) AM_DEVREADWRITE(MC6852_TAG, mc6852_device, read, write)
	AM_RANGE(0xe8080, 0xe808f) AM_MIRROR(0x7f00) AM_DEVREADWRITE(M6522_3_TAG, via6522_device, read, write)
	AM_RANGE(0xe80a0, 0xe80af) AM_MIRROR(0x7f00) AM_DEVREADWRITE(FDC_TAG, victor_9000_fdc_device, cs5_r, cs5_w)
	AM_RANGE(0xe80c0, 0xe80cf) AM_MIRROR(0x7f00) AM_DEVREADWRITE(FDC_TAG, victor_9000_fdc_device, cs6_r, cs6_w)
	AM_RANGE(0xe80e0, 0xe80ef) AM_MIRROR(0x7f00) AM_DEVREADWRITE(FDC_TAG, victor_9000_fdc_device, cs7_r, cs7_w)
	AM_RANGE(0xf0000, 0xf0fff) AM_MIRROR(0x1000) AM_RAM AM_SHARE("video_ram")
	AM_RANGE(0xf8000, 0xf9fff) AM_MIRROR(0x6000) AM_ROM AM_REGION(I8088_TAG, 0)
ADDRESS_MAP_END



//**************************************************************************
//  INPUT PORTS
//**************************************************************************

//-------------------------------------------------
//  INPUT_PORTS( victor9k )
//-------------------------------------------------

static INPUT_PORTS_START( victor9k )
	// defined in machine/victor9kb.c
INPUT_PORTS_END



//**************************************************************************
//  DEVICE CONFIGURATION
//**************************************************************************

//-------------------------------------------------
//  MC6845
//-------------------------------------------------

#define DC_SECRET   0x1000
#define DC_UNDLN    0x2000
#define DC_LOWINT   0x4000
#define DC_RVS      0x8000

MC6845_UPDATE_ROW( victor9k_state::crtc_update_row )
{
	int hires = BIT(ma, 13);
	int dot_addr = BIT(ma, 12);
	int width = hires ? 16 : 10;

	if (m_hires != hires)
	{
		m_hires = hires;
		m_crtc->set_clock(XTAL_30MHz / width);
		m_crtc->set_hpixels_per_column(width);
	}

	address_space &program = m_maincpu->space(AS_PROGRAM);
	const rgb_t *palette = m_palette->palette()->entry_list_raw();

	int x = hbp;

	offs_t aa = (ma & 0x7ff) << 1;

	for (int sx = 0; sx < x_count; sx++)
	{
		uint16_t dc = (m_video_ram[aa + 1] << 8) | m_video_ram[aa];
		offs_t ab = (dot_addr << 15) | ((dc & 0x7ff) << 4) | (ra & 0x0f);
		uint16_t dd = program.read_word(ab << 1);

		int cursor = (sx == cursor_x) ? 1 : 0;
		int undln = !((dc & DC_UNDLN) && BIT(dd, 15)) ? 2 : 0;
		int rvs = (dc & DC_RVS) ? 4 : 0;
		int secret = (dc & DC_SECRET) ? 1 : 0;
		int lowint = (dc & DC_LOWINT) ? 1 : 0;

		for (int bit = 0; bit < width; bit++)
		{
			int pixel = 0;

			switch (rvs | undln | cursor)
			{
			case 0: case 5:
				pixel = 1;
				break;

			case 1: case 4:
				pixel = 0;
				break;

			case 2: case 7:
				pixel = !(!(BIT(dd, bit) && !secret));
				break;

			case 3: case 6:
				pixel = !(BIT(dd, bit) && !secret);
				break;
			}

			int color = 0;

			if (pixel && de)
			{
				int pen = 1 + m_brt;
				if (!lowint) pen = 9;
				color = palette[pen];
			}

			bitmap.pix32(vbp + y, x++) = color;
		}

		aa += 2;
		aa &= 0xfff;
	}
}

WRITE_LINE_MEMBER(victor9k_state::vert_w)
{
	m_via2->write_pa7(state);
	m_pic->ir7_w(state);
}



WRITE_LINE_MEMBER(victor9k_state::mux_serial_b_w)
{
}

WRITE_LINE_MEMBER(victor9k_state::mux_serial_a_w)
{
}

//-------------------------------------------------
//  PIC8259
//-------------------------------------------------

/*

    pin     signal      description

    IR0     SYN         sync detect
    IR1     COMM        serial communications (7201)
    IR2     TIMER       8253 timer
    IR3     PARALLEL    all 6522 IRQ (including disk)
    IR4     IR4         expansion IR4
    IR5     IR5         expansion IR5
    IR6     KBINT       keyboard data ready
    IR7     VINT        vertical sync or nonspecific interrupt

*/

//-------------------------------------------------
//  MC6852_INTERFACE( ssda_intf )
//-------------------------------------------------

WRITE_LINE_MEMBER( victor9k_state::ssda_irq_w )
{
	m_ssda_irq = state;

	m_pic->ir3_w(m_ssda_irq || m_via1_irq || m_via3_irq || m_fdc_irq);
}


WRITE_LINE_MEMBER( victor9k_state::ssda_sm_dtr_w )
{
	m_ssda->cts_w(state);
	m_ssda->dcd_w(!state);
	//m_cvsd->enc_dec_w(!state);
}


WRITE8_MEMBER( victor9k_state::via1_pa_w )
{
	/*

	    bit     description

	    PA0     DIO1
	    PA1     DIO2
	    PA2     DIO3
	    PA3     DIO4
	    PA4     DIO5
	    PA5     DIO6
	    PA6     DIO7
	    PA7     DIO8

	*/

	// centronics
	m_centronics->write_data0(BIT(data, 0));
	m_centronics->write_data1(BIT(data, 1));
	m_centronics->write_data2(BIT(data, 2));
	m_centronics->write_data3(BIT(data, 3));
	m_centronics->write_data4(BIT(data, 4));
	m_centronics->write_data5(BIT(data, 5));
	m_centronics->write_data6(BIT(data, 6));
	m_centronics->write_data7(BIT(data, 7));

	// IEEE-488
	m_ieee488->dio_w(data);
}

DECLARE_WRITE_LINE_MEMBER( victor9k_state::write_nfrd )
{
	m_via1->write_pb6(state);
	m_via1->write_ca1(state);
}

DECLARE_WRITE_LINE_MEMBER( victor9k_state::write_ndac )
{
	m_via1->write_pb7(state);
	m_via1->write_ca2(state);
}

WRITE8_MEMBER( victor9k_state::via1_pb_w )
{
	/*

	    bit     description

	    PB0     STROBE/DAV
	    PB1     PI/EOI
	    PB2     REN
	    PB3     ATN
	    PB4     IFC
	    PB5     SRQ/BUSY SRQ
	    PB6     NRFD/ACK RFD
	    PB7     SEL/DAC

	*/

	// centronics
	m_centronics->write_strobe(BIT(data, 0));

	// IEEE-488
	m_ieee488->dav_w(BIT(data, 0));
	m_ieee488->eoi_w(BIT(data, 1));
	m_ieee488->ren_w(BIT(data, 2));
	m_ieee488->atn_w(BIT(data, 3));
	m_ieee488->ifc_w(BIT(data, 4));
	m_ieee488->srq_w(BIT(data, 5));
	m_ieee488->nrfd_w(BIT(data, 6));
	m_ieee488->ndac_w(BIT(data, 7));
}

WRITE_LINE_MEMBER( victor9k_state::codec_vol_w )
{
}

WRITE_LINE_MEMBER( victor9k_state::via1_irq_w )
{
	m_via1_irq = state;

	m_pic->ir3_w(m_ssda_irq || m_via1_irq || m_via3_irq || m_fdc_irq);
}

WRITE8_MEMBER( victor9k_state::via2_pa_w )
{
	/*

	    bit     description

	    PA0     _INT/EXTA
	    PA1     _INT/EXTB
	    PA2
	    PA3
	    PA4
	    PA5
	    PA6
	    PA7

	*/
}

void victor9k_state::update_kback()
{
	int kback = !(!(m_kbrdy && !m_via2_irq) && !(m_kbackctl && m_via2_irq));

	m_kb->kback_w(kback);
}

WRITE8_MEMBER( victor9k_state::via2_pb_w )
{
	/*

	    bit     description

	    PB0     TALK/LISTEN
	    PB1     KBACKCTL
	    PB2     BRT0
	    PB3     BRT1
	    PB4     BRT2
	    PB5     CONT0
	    PB6     CONT1
	    PB7     CONT2

	*/

	// keyboard acknowledge
	m_kbackctl = BIT(data, 1);
	update_kback();

	// brightness
	m_brt = (data >> 2) & 0x07;

	// contrast
	m_cont = data >> 5;

	if (LOG) logerror("BRT %u CONT %u\n", m_brt, m_cont);
}

WRITE_LINE_MEMBER( victor9k_state::via2_irq_w )
{
	m_via2_irq = state;

	m_pic->ir6_w(m_via2_irq);
	update_kback();
}


WRITE_LINE_MEMBER( victor9k_state::write_ria )
{
	m_upd7201->ria_w(state);
	m_via2->write_pa2(state);
}


WRITE_LINE_MEMBER( victor9k_state::write_rib )
{
	m_upd7201->rib_w(state);
	m_via2->write_pa4(state);
}


/*
    bit    description

    PA0    J5-16
    PA1    J5-18
    PA2    J5-20
    PA3    J5-22
    PA4    J5-24
    PA5    J5-26
    PA6    J5-28
    PA7    J5-30
    PB0    J5-32
    PB1    J5-34
    PB2    J5-36
    PB3    J5-38
    PB4    J5-40
    PB5    J5-42
    PB6    J5-44
    PB7    J5-46
    CA1    J5-12
    CB1    J5-48
    CA2    J5-14
    CB2    J5-50
*/

WRITE8_MEMBER( victor9k_state::via3_pb_w )
{
	// codec clock output
	m_ssda->rx_clk_w(!BIT(data, 7));
	m_ssda->tx_clk_w(!BIT(data, 7));
	m_cvsd->clock_w(!BIT(data, 7));
}

WRITE_LINE_MEMBER( victor9k_state::via3_irq_w )
{
	m_via3_irq = state;

	m_pic->ir3_w(m_ssda_irq || m_via1_irq || m_via3_irq || m_fdc_irq);
}


//-------------------------------------------------
//  VICTOR9K_KEYBOARD_INTERFACE( kb_intf )
//-------------------------------------------------

WRITE_LINE_MEMBER( victor9k_state::kbrdy_w )
{
	if (LOG) logerror("KBRDY %u\n", state);

	m_via2->write_cb1(state);

	m_kbrdy = state;
	update_kback();
}

WRITE_LINE_MEMBER( victor9k_state::kbdata_w )
{
	if (LOG) logerror("KBDATA %u\n", state);

	m_via2->write_cb2(state);
	m_via2->write_pa6(state);
}


WRITE_LINE_MEMBER( victor9k_state::fdc_irq_w )
{
	m_fdc_irq = state;

	m_pic->ir3_w(m_ssda_irq || m_via1_irq || m_via3_irq || m_fdc_irq);
}


//**************************************************************************
//  MACHINE INITIALIZATION
//**************************************************************************

PALETTE_INIT_MEMBER(victor9k_state, victor9k)
{
	palette.set_pen_color(0, rgb_t(0x00, 0x00, 0x00));

	// BRT0 82K
	// BRT1 39K
	// BRT2 20K
	// 12V 220K pullup
	palette.set_pen_color(1, rgb_t(0x00, 0x10, 0x00));
	palette.set_pen_color(2, rgb_t(0x00, 0x20, 0x00));
	palette.set_pen_color(3, rgb_t(0x00, 0x40, 0x00));
	palette.set_pen_color(4, rgb_t(0x00, 0x60, 0x00));
	palette.set_pen_color(5, rgb_t(0x00, 0x80, 0x00));
	palette.set_pen_color(6, rgb_t(0x00, 0xa0, 0x00));
	palette.set_pen_color(7, rgb_t(0x00, 0xc0, 0x00));
	palette.set_pen_color(8, rgb_t(0x00, 0xff, 0x00));

	// CONT0 620R
	// CONT1 332R
	// CONT2 162R
	// 12V 110R pullup
	palette.set_pen_color(9, rgb_t(0xff, 0x00, 0x00));
}

void victor9k_state::machine_start()
{
	// state saving
	save_item(NAME(m_brt));
	save_item(NAME(m_cont));
	save_item(NAME(m_via1_irq));
	save_item(NAME(m_via2_irq));
	save_item(NAME(m_via3_irq));
	save_item(NAME(m_fdc_irq));
	save_item(NAME(m_ssda_irq));
	save_item(NAME(m_kbrdy));
	save_item(NAME(m_kbackctl));

#ifndef USE_SCP
	// patch out SCP self test
	m_rom->base()[0x11ab] = 0xc3;

	// patch out ROM checksum error
	m_rom->base()[0x1d51] = 0x90;
	m_rom->base()[0x1d52] = 0x90;
	m_rom->base()[0x1d53] = 0x90;
	m_rom->base()[0x1d54] = 0x90;
#endif
}

void victor9k_state::machine_reset()
{
	m_maincpu->reset();
	m_upd7201->reset();
	m_ssda->reset();
	m_via1->reset();
	m_via2->reset();
	m_via3->reset();
	m_crtc->reset();
	m_fdc->reset();
}



//**************************************************************************
//  MACHINE CONFIGURATION
//**************************************************************************

//-------------------------------------------------
//  MACHINE_CONFIG( victor9k )
//-------------------------------------------------

static MACHINE_CONFIG_START( victor9k )
	// basic machine hardware
	MCFG_CPU_ADD(I8088_TAG, I8088, XTAL_30MHz/6)
	MCFG_CPU_PROGRAM_MAP(victor9k_mem)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DEVICE(I8259A_TAG, pic8259_device, inta_cb)

	// video hardware
	MCFG_SCREEN_ADD_MONOCHROME(SCREEN_TAG, RASTER, rgb_t::green())
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) // not accurate
	MCFG_SCREEN_UPDATE_DEVICE(HD46505S_TAG, hd6845_device, screen_update)
	MCFG_SCREEN_SIZE(640, 480)
	MCFG_SCREEN_VISIBLE_AREA(0, 640-1, 0, 480-1)
	MCFG_PALETTE_ADD("palette", 16)
	MCFG_PALETTE_INIT_OWNER(victor9k_state, victor9k)

	MCFG_MC6845_ADD(HD46505S_TAG, HD6845, SCREEN_TAG, XTAL_30MHz/10) // HD6845 == HD46505S
	MCFG_MC6845_SHOW_BORDER_AREA(true)
	MCFG_MC6845_CHAR_WIDTH(10)
	MCFG_MC6845_UPDATE_ROW_CB(victor9k_state, crtc_update_row)
	MCFG_MC6845_OUT_VSYNC_CB(WRITELINE(victor9k_state, vert_w))

	// sound hardware
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD(HC55516_TAG, HC55516, 0)
	//MCFG_HC55516_DIG_OUT_CB(DEVWRITELINE(MC6852_TAG, mc6852_device, rx_w))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.25)

	// devices
	MCFG_IEEE488_BUS_ADD()
	MCFG_IEEE488_DAV_CALLBACK(DEVWRITELINE(M6522_1_TAG, via6522_device, write_pb0))
	MCFG_IEEE488_EOI_CALLBACK(DEVWRITELINE(M6522_1_TAG, via6522_device, write_pb1))
	MCFG_IEEE488_REN_CALLBACK(DEVWRITELINE(M6522_1_TAG, via6522_device, write_pb2))
	MCFG_IEEE488_ATN_CALLBACK(DEVWRITELINE(M6522_1_TAG, via6522_device, write_pb3))
	MCFG_IEEE488_IFC_CALLBACK(DEVWRITELINE(M6522_1_TAG, via6522_device, write_pb4))
	MCFG_IEEE488_SRQ_CALLBACK(DEVWRITELINE(M6522_1_TAG, via6522_device, write_pb5))
	MCFG_IEEE488_NRFD_CALLBACK(WRITELINE(victor9k_state, write_nfrd))
	MCFG_IEEE488_NDAC_CALLBACK(WRITELINE(victor9k_state, write_ndac))

	MCFG_PIC8259_ADD(I8259A_TAG, INPUTLINE(I8088_TAG, INPUT_LINE_IRQ0), VCC, NOOP)

	MCFG_DEVICE_ADD(I8253_TAG, PIT8253, 0)
	MCFG_PIT8253_CLK0(2500000)
	MCFG_PIT8253_OUT0_HANDLER(WRITELINE(victor9k_state, mux_serial_b_w))
	MCFG_PIT8253_CLK1(2500000)
	MCFG_PIT8253_OUT1_HANDLER(WRITELINE(victor9k_state, mux_serial_a_w))
	MCFG_PIT8253_CLK2(100000)
	MCFG_PIT8253_OUT2_HANDLER(DEVWRITELINE(I8259A_TAG, pic8259_device, ir2_w))

	MCFG_UPD7201_ADD(UPD7201_TAG, XTAL_30MHz/30, 0, 0, 0, 0)
	MCFG_Z80DART_OUT_TXDA_CB(DEVWRITELINE(RS232_A_TAG, rs232_port_device, write_txd))
	MCFG_Z80DART_OUT_DTRA_CB(DEVWRITELINE(RS232_A_TAG, rs232_port_device, write_dtr))
	MCFG_Z80DART_OUT_RTSA_CB(DEVWRITELINE(RS232_A_TAG, rs232_port_device, write_rts))
	MCFG_Z80DART_OUT_TXDB_CB(DEVWRITELINE(RS232_B_TAG, rs232_port_device, write_txd))
	MCFG_Z80DART_OUT_DTRB_CB(DEVWRITELINE(RS232_B_TAG, rs232_port_device, write_dtr))
	MCFG_Z80DART_OUT_RTSB_CB(DEVWRITELINE(RS232_B_TAG, rs232_port_device, write_rts))
	MCFG_Z80DART_OUT_INT_CB(DEVWRITELINE(I8259A_TAG, pic8259_device, ir1_w))

	MCFG_DEVICE_ADD(MC6852_TAG, MC6852, XTAL_30MHz/30)
	MCFG_MC6852_TX_DATA_CALLBACK(DEVWRITELINE(HC55516_TAG, hc55516_device, digit_w))
	MCFG_MC6852_SM_DTR_CALLBACK(WRITELINE(victor9k_state, ssda_sm_dtr_w))
	MCFG_MC6852_IRQ_CALLBACK(WRITELINE(victor9k_state, ssda_irq_w))

	MCFG_DEVICE_ADD(M6522_1_TAG, VIA6522, XTAL_30MHz/30)
	MCFG_VIA6522_READPA_HANDLER(DEVREAD8(IEEE488_TAG, ieee488_device, dio_r))
	MCFG_VIA6522_WRITEPA_HANDLER(WRITE8(victor9k_state, via1_pa_w))
	MCFG_VIA6522_WRITEPB_HANDLER(WRITE8(victor9k_state, via1_pb_w))
	MCFG_VIA6522_CB2_HANDLER(WRITELINE(victor9k_state, codec_vol_w))
	MCFG_VIA6522_IRQ_HANDLER(WRITELINE(victor9k_state, via1_irq_w))

	MCFG_DEVICE_ADD(M6522_2_TAG, VIA6522, XTAL_30MHz/30)
	MCFG_VIA6522_WRITEPA_HANDLER(WRITE8(victor9k_state, via2_pa_w))
	MCFG_VIA6522_WRITEPB_HANDLER(WRITE8(victor9k_state, via2_pb_w))
	MCFG_VIA6522_IRQ_HANDLER(WRITELINE(victor9k_state, via2_irq_w))

	MCFG_DEVICE_ADD(M6522_3_TAG, VIA6522, XTAL_30MHz/30)
	MCFG_VIA6522_WRITEPB_HANDLER(WRITE8(victor9k_state, via3_pb_w))
	MCFG_VIA6522_IRQ_HANDLER(WRITELINE(victor9k_state, via3_irq_w))

	MCFG_CENTRONICS_ADD(CENTRONICS_TAG, centronics_devices, "printer")
	MCFG_CENTRONICS_BUSY_HANDLER(DEVWRITELINE(M6522_1_TAG, via6522_device, write_pb5))
	MCFG_CENTRONICS_ACK_HANDLER(DEVWRITELINE(M6522_1_TAG, via6522_device, write_pb6))
	MCFG_CENTRONICS_SELECT_HANDLER(DEVWRITELINE(M6522_1_TAG, via6522_device, write_pb7))

	MCFG_RS232_PORT_ADD(RS232_A_TAG, default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE(UPD7201_TAG, z80dart_device, rxa_w))
	MCFG_RS232_DCD_HANDLER(DEVWRITELINE(UPD7201_TAG, z80dart_device, dcda_w))
	MCFG_RS232_RI_HANDLER(WRITELINE(victor9k_state, write_ria))
	MCFG_RS232_CTS_HANDLER(DEVWRITELINE(UPD7201_TAG, z80dart_device, ctsa_w))
	MCFG_RS232_DSR_HANDLER(DEVWRITELINE(M6522_2_TAG, via6522_device, write_pa3))

	MCFG_RS232_PORT_ADD(RS232_B_TAG, default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE(UPD7201_TAG, z80dart_device, rxb_w))
	MCFG_RS232_DCD_HANDLER(DEVWRITELINE(UPD7201_TAG, z80dart_device, dcdb_w))
	MCFG_RS232_RI_HANDLER(WRITELINE(victor9k_state, write_ria))
	MCFG_RS232_CTS_HANDLER(DEVWRITELINE(UPD7201_TAG, z80dart_device, ctsb_w))
	MCFG_RS232_DSR_HANDLER(DEVWRITELINE(M6522_2_TAG, via6522_device, write_pa5))

	MCFG_DEVICE_ADD(KB_TAG, VICTOR9K_KEYBOARD, 0)
	MCFG_VICTOR9K_KBRDY_HANDLER(WRITELINE(victor9k_state, kbrdy_w))
	MCFG_VICTOR9K_KBDATA_HANDLER(WRITELINE(victor9k_state, kbdata_w))

	MCFG_DEVICE_ADD(FDC_TAG, VICTOR_9000_FDC, 0)
	MCFG_VICTOR_9000_FDC_IRQ_CB(WRITELINE(victor9k_state, fdc_irq_w))
	MCFG_VICTOR_9000_FDC_SYN_CB(DEVWRITELINE(I8259A_TAG, pic8259_device, ir0_w)) MCFG_DEVCB_XOR(1)
	MCFG_VICTOR_9000_FDC_LBRDY_CB(INPUTLINE(I8088_TAG, INPUT_LINE_TEST)) MCFG_DEVCB_XOR(1)

	// internal ram
	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("128K")

	// software list
	MCFG_SOFTWARE_LIST_ADD("flop_list", "victor9k_flop")
MACHINE_CONFIG_END



//**************************************************************************
//  ROMS
//**************************************************************************

//-------------------------------------------------
//  ROM( victor9k )
//-------------------------------------------------

ROM_START( victor9k )
	ROM_REGION( 0x2000, I8088_TAG, 0 )
	ROM_DEFAULT_BIOS( "univ" )
	ROM_SYSTEM_BIOS( 0, "old", "Older" )
	ROMX_LOAD( "102320.7j", 0x0000, 0x1000, CRC(3d615fd7) SHA1(b22f7e5d66404185395d8effbf57efded0079a92), ROM_BIOS(1) )
	ROMX_LOAD( "102322.8j", 0x1000, 0x1000, CRC(9209df0e) SHA1(3ee8e0c15186bbd5768b550ecc1fa3b6b1dbb928), ROM_BIOS(1) )
	ROM_SYSTEM_BIOS( 1, "univ", "Universal" )
	ROMX_LOAD( "v9000 univ. fe f3f7 13db.7j", 0x0000, 0x1000, CRC(25c7a59f) SHA1(8784e9aa7eb9439f81e18b8e223c94714e033911), ROM_BIOS(2) )
	ROMX_LOAD( "v9000 univ. ff f3f7 39fe.8j", 0x1000, 0x1000, CRC(496c7467) SHA1(eccf428f62ef94ab85f4a43ba59ae6a066244a66), ROM_BIOS(2) )
ROM_END



//**************************************************************************
//  SYSTEM DRIVERS
//**************************************************************************

//    YEAR  NAME      PARENT  COMPAT  MACHINE   INPUT     STATE           INIT  COMPANY                     FULLNAME       FLAGS
COMP( 1982, victor9k, 0,      0,      victor9k, victor9k, victor9k_state, 0,    "Victor Business Products", "Victor 9000", MACHINE_IMPERFECT_COLORS | MACHINE_IMPERFECT_SOUND | MACHINE_SUPPORTS_SAVE )
