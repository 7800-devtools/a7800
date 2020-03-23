// license:BSD-3-Clause
// copyright-holders:Sergey Svishchev
/***************************************************************************

    Eltec Eurocom II V7 single-board computer.  Used in PPG Waveterm A.

    to do:
    - Eurocom board: timer, interrupts
    - Waveterm: PTM, ADC, DAC, 'end' button
    - autoboot Waveterm software without -debug being necessary
    - support more disk image formats (.dsk, flexemu .flx)

    manuals:
    - http://seib.synth.net/documents/EurocomIIHW.pdf
    - http://seib.synth.net/documents/wt_b_serv.pdf
    - http://seib.synth.net/documents/wt_a_usrd.pdf
    - http://seib.synth.net/documents/wavetermbroc.pdf

    useful links:
    - http://www.ppg.synth.net/waveterm/
    - http://www.hermannseib.com/english/synths/ppg/waveterm.htm
    - http://www.synthmuseum.com/ppg/ppgwterm01.html
    - http://www.theppgs.com/waveterma.html
    - http://machines.hyperreal.org/manufacturers/PPG/info/ppg.waveterm.revisions.txt
    - http://www.flexusergroup.com/flexusergroup/default.htm
    - http://web.archive.org/web/20091026234737/http://geocities.com/flexemu/

****************************************************************************/

#include "emu.h"

#include "bus/rs232/rs232.h"
#include "cpu/m6809/m6809.h"
#include "formats/ppg_dsk.h"
#include "machine/6821pia.h"
#include "machine/6840ptm.h"
#include "machine/6850acia.h"
#include "machine/keyboard.h"
#include "machine/wd_fdc.h"

#include "screen.h"


#define VC_TOTAL_HORZ 678
#define VC_DISP_HORZ  512

#define VC_TOTAL_VERT 312
#define VC_DISP_VERT  256


#define VERBOSE_DBG 2       /* general debug messages */

#define DBG_LOG(N,M,A) \
	do { \
	if(VERBOSE_DBG>=N) \
		{ \
			if( M ) \
				logerror("%11.6f at %s: %-10s",machine().time().as_double(),machine().describe_context(),(char*)M ); \
			logerror A; \
		} \
	} while (0)


class eurocom2_state : public driver_device
{
public:
	eurocom2_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_pia1(*this, "pia1")
		, m_pia2(*this, "pia2")
		, m_acia(*this, "acia")
		, m_fdc(*this, "fdc")
		, m_p_videoram(*this, "videoram")
		, m_screen(*this, "screen")
	{
	}

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	DECLARE_READ8_MEMBER(fdc_aux_r);
	DECLARE_WRITE8_MEMBER(fdc_aux_w);
	DECLARE_FLOPPY_FORMATS(floppy_formats);

	DECLARE_WRITE8_MEMBER(vico_w);

	DECLARE_READ8_MEMBER(kbd_get);
	void kbd_put(u8 data);

	DECLARE_READ_LINE_MEMBER(pia1_ca1_r);
	DECLARE_READ_LINE_MEMBER(pia1_ca2_r);
	DECLARE_READ_LINE_MEMBER(pia1_cb1_r);
	DECLARE_WRITE_LINE_MEMBER(pia1_cb2_w);

protected:
	// driver_device overrides
	virtual void machine_reset() override;
	virtual void machine_start() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	emu_timer *m_sst;

	floppy_image_device *m_floppy;
	bool m_sst_state, m_kbd_ready;
	bitmap_ind16 m_tmpbmp;

	uint8_t m_vico[2];
	uint8_t m_kbd_data;

	required_device<cpu_device> m_maincpu;
	required_device<pia6821_device> m_pia1;
	required_device<pia6821_device> m_pia2;
	required_device<acia6850_device> m_acia;
	required_device<fd1793_device> m_fdc;
	required_shared_ptr<uint8_t> m_p_videoram;
	required_device<screen_device> m_screen;
};

class waveterm_state : public eurocom2_state
{
public:
	waveterm_state(const machine_config &mconfig, device_type type, const char *tag)
		: eurocom2_state(mconfig, type, tag)
		, m_pia3(*this, "pia3")
		, m_ptm(*this, "ptm")
	{ }

	DECLARE_READ8_MEMBER(waveterm_kb_r);
	DECLARE_WRITE8_MEMBER(waveterm_kb_w);
	DECLARE_WRITE_LINE_MEMBER(waveterm_kbh_w);

	DECLARE_READ8_MEMBER(pia3_pa_r);
	DECLARE_WRITE8_MEMBER(pia3_pa_w);
	DECLARE_WRITE8_MEMBER(pia3_pb_w);
	DECLARE_READ_LINE_MEMBER(pia3_ca1_r);
	DECLARE_READ_LINE_MEMBER(pia3_ca2_r);
	DECLARE_WRITE_LINE_MEMBER(pia3_cb2_w);

	DECLARE_READ8_MEMBER(waveterm_adc);
	DECLARE_WRITE8_MEMBER(waveterm_dac);

protected:
	bool m_driveh;
	uint8_t m_drive;

	required_device<pia6821_device> m_pia3;
	required_device<ptm6840_device> m_ptm;
};


/*
 * b0 -- timer output
 * b1 -- 1 == two-sided diskette in drive
 * b2..b5 nc
 * b6 -- irq
 * b7 -- drq
 */
READ8_MEMBER(eurocom2_state::fdc_aux_r)
{
	uint8_t data = 0;

	data |= (m_floppy ? m_floppy->twosid_r() : 1) << 1;
	data |= (m_fdc->intrq_r() << 6);
	data |= (m_fdc->drq_r() << 7);

	DBG_LOG(3, "Floppy", ("%d == %02x\n", offset, data));

	return data;
}

/*
 * b0 -- 1 = select 0
 * ..
 * b3 -- 1 = select 3
 * b4 -- 1 = top head
 * b5 -- 1 = single density
 * b6 -- nc
 * b7 -- 1 = enable timer interrupt
 */
WRITE8_MEMBER(eurocom2_state::fdc_aux_w)
{
	floppy_image_device *floppy0 = m_fdc->subdevice<floppy_connector>("0")->get_device();
	floppy_image_device *floppy1 = m_fdc->subdevice<floppy_connector>("1")->get_device();

	if (BIT(data, 0))
		m_floppy = floppy0;
	else if (BIT(data, 1))
		m_floppy = floppy1;
	else
		m_floppy = nullptr;

	if (m_floppy)
	{
		m_fdc->set_floppy(m_floppy);
		m_floppy->ss_w(BIT(data, 4));
		m_floppy->mon_w(0);
	}
	else
	{
		floppy0->mon_w(1);
		floppy1->mon_w(1);
	}

	m_fdc->dden_w(BIT(data, 5));

	DBG_LOG(3, "Floppy", ("%d <- %02x\n", offset, data));
}

WRITE8_MEMBER(eurocom2_state::vico_w)
{
	DBG_LOG(2, "VICO", ("%d <- %02x\n", offset, data));

	m_vico[offset & 1] = data;
}


READ_LINE_MEMBER(eurocom2_state::pia1_ca2_r)
{
	DBG_LOG(3, "PIA1", ("CA2 == %d (SST Q14)\n", m_sst_state));

	return m_sst_state;
}

READ_LINE_MEMBER(eurocom2_state::pia1_cb1_r)
{
	DBG_LOG(3, "PIA1", ("CB1 == %d (SST Q6)\n", m_sst_state));

	return m_sst_state;
}

WRITE_LINE_MEMBER(eurocom2_state::pia1_cb2_w)
{
	DBG_LOG(2, "PIA1", ("CB2 <- %d (SST reset)\n", state));
	// reset single-step timer
}

void eurocom2_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	m_sst_state = !m_sst_state;
	m_pia1->ca2_w(m_sst_state);
}


READ_LINE_MEMBER(eurocom2_state::pia1_ca1_r)
{
	return m_kbd_ready;
}

/* bit 7 may be connected to something else -- see section 6.2 of Eurocom manual */
READ8_MEMBER(eurocom2_state::kbd_get)
{
	return m_kbd_data;
}

void eurocom2_state::kbd_put(u8 data)
{
	m_kbd_ready = true;
	m_kbd_data = data;
	m_pia1->ca1_w(false);
	m_pia1->ca1_w(true);
}


READ8_MEMBER(waveterm_state::waveterm_kb_r)
{
	uint8_t data = 0xff;

	if (BIT(m_drive, 0)) data &= ioport("ROW.0")->read();
	if (BIT(m_drive, 1)) data &= ioport("ROW.1")->read();
	if (BIT(m_drive, 2)) data &= ioport("ROW.2")->read();
	if (BIT(m_drive, 3)) data &= ioport("ROW.3")->read();
	if (m_driveh)        data &= ioport("ROW.4")->read();

	return data;
}

WRITE8_MEMBER(waveterm_state::waveterm_kb_w)
{
	m_drive = (~data) >> 4;
}

WRITE_LINE_MEMBER(waveterm_state::waveterm_kbh_w)
{
	m_driveh = !state;
}

WRITE8_MEMBER(waveterm_state::pia3_pb_w)
{
}

READ8_MEMBER(waveterm_state::waveterm_adc)
{
	return m_screen->frame_number() % 255; // XXX
}


uint32_t eurocom2_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	int x, y, offset, page;
	uint16_t gfx, *p;

	page = (m_vico[0] & 3) << 14;

	for (y = 0; y < VC_DISP_VERT; y++)
	{
		offset = (VC_DISP_HORZ / 8) * ((m_vico[1] + y) % VC_DISP_VERT);
		p = &m_tmpbmp.pix16(y);

		for (x = offset; x < offset + VC_DISP_HORZ / 8; x++)
		{
			gfx = m_p_videoram[page + x];

			*p++ = BIT(gfx, 7);
			*p++ = BIT(gfx, 6);
			*p++ = BIT(gfx, 5);
			*p++ = BIT(gfx, 4);
			*p++ = BIT(gfx, 3);
			*p++ = BIT(gfx, 2);
			*p++ = BIT(gfx, 1);
			*p++ = BIT(gfx, 0);
		}
	}

	copybitmap(bitmap, m_tmpbmp, 0, 0, 0, 0, cliprect);

	return 0;
}

static ADDRESS_MAP_START(eurocom2_map, AS_PROGRAM, 8, eurocom2_state)
	AM_RANGE(0x0000, 0xefff) AM_RAM AM_SHARE("videoram")
	AM_RANGE(0xf000, 0xfcef) AM_ROM AM_REGION("maincpu", 0)
	AM_RANGE(0xfcf0, 0xfcf3) AM_DEVREADWRITE("pia1", pia6821_device, read, write)
	AM_RANGE(0xfcf4, 0xfcf4) AM_DEVREADWRITE("acia", acia6850_device, status_r, control_w)
	AM_RANGE(0xfcf5, 0xfcf5) AM_DEVREADWRITE("acia", acia6850_device, data_r, data_w)
	AM_RANGE(0xfcf6, 0xfcf7) AM_WRITE(vico_w)
	AM_RANGE(0xfcf8, 0xfcfb) AM_DEVREADWRITE("pia2", pia6821_device, read, write)
	AM_RANGE(0xfd30, 0xfd37) AM_DEVREADWRITE("fdc", fd1793_device, read, write)
	AM_RANGE(0xfd38, 0xfd38) AM_READWRITE(fdc_aux_r, fdc_aux_w)
	AM_RANGE(0xfd40, 0xffff) AM_ROM AM_REGION("maincpu", 0xd40) AM_WRITENOP
ADDRESS_MAP_END

static ADDRESS_MAP_START(waveterm_map, AS_PROGRAM, 8, waveterm_state)
	AM_IMPORT_FROM(eurocom2_map)
	AM_RANGE(0xfd00, 0xfd03) AM_DEVREADWRITE("pia3", pia6821_device, read, write)
	AM_RANGE(0xfd08, 0xfd0f) AM_DEVREADWRITE("ptm", ptm6840_device, read, write)
	AM_RANGE(0xfd10, 0xfd17) AM_UNMAP
	AM_RANGE(0xfd18, 0xfd18) AM_READ(waveterm_adc)  //  AD558 ADC
//  AM_RANGE(0xfd20, 0xfd20) AM_READ(waveterm_dac)  //  ZN432 DAC ??
ADDRESS_MAP_END

static INPUT_PORTS_START(eurocom2)
	PORT_START("S1")
	PORT_DIPNAME(0x0f, 0x01, "Serial baud rate")
	PORT_DIPSETTING(0x01, "9600")

	PORT_DIPNAME(0x40, 0x40, "7 or 8-bit keyboard")
	PORT_DIPSETTING(0x40, "8-bit")
	PORT_DIPSETTING(0x00, "7-bit")

	PORT_DIPNAME(0x80, 0x80, "Periodic timer")
	PORT_DIPSETTING(0x80, DEF_STR(Yes))
	PORT_DIPSETTING(0x00, DEF_STR(No))
INPUT_PORTS_END

static INPUT_PORTS_START(waveterm)
	PORT_INCLUDE(eurocom2)

	PORT_START("FP")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("End") PORT_CODE(KEYCODE_PRTSCR) PORT_CHAR(UCHAR_MAMEKEY(PRTSCR))

	PORT_START("ROW.0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("F1") PORT_CODE(KEYCODE_F1) PORT_CHAR(UCHAR_MAMEKEY(F1))
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("F2") PORT_CODE(KEYCODE_F2) PORT_CHAR(UCHAR_MAMEKEY(F2))
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("F3") PORT_CODE(KEYCODE_F3) PORT_CHAR(UCHAR_MAMEKEY(F3))
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("F4") PORT_CODE(KEYCODE_F4) PORT_CHAR(UCHAR_MAMEKEY(F4))

	PORT_START("ROW.1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("F5") PORT_CODE(KEYCODE_F5) PORT_CHAR(UCHAR_MAMEKEY(F5))
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("F6") PORT_CODE(KEYCODE_F6) PORT_CHAR(UCHAR_MAMEKEY(F6))
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("F7") PORT_CODE(KEYCODE_F7) PORT_CHAR(UCHAR_MAMEKEY(F7))
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("F8") PORT_CODE(KEYCODE_F8) PORT_CHAR(UCHAR_MAMEKEY(F8))

	PORT_START("ROW.2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("F9") PORT_CODE(KEYCODE_F9) PORT_CHAR(UCHAR_MAMEKEY(F9))
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("F0") PORT_CODE(KEYCODE_F10) PORT_CHAR(UCHAR_MAMEKEY(F10))
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("Num 1") PORT_CODE(KEYCODE_1_PAD) PORT_CHAR(UCHAR_MAMEKEY(1_PAD))
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("Num 2") PORT_CODE(KEYCODE_2_PAD) PORT_CHAR(UCHAR_MAMEKEY(2_PAD))

	PORT_START("ROW.3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("Num 3") PORT_CODE(KEYCODE_3_PAD) PORT_CHAR(UCHAR_MAMEKEY(3_PAD))
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("Num 4") PORT_CODE(KEYCODE_4_PAD) PORT_CHAR(UCHAR_MAMEKEY(4_PAD))
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("Num 5") PORT_CODE(KEYCODE_5_PAD) PORT_CHAR(UCHAR_MAMEKEY(5_PAD))
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("Num 6") PORT_CODE(KEYCODE_6_PAD) PORT_CHAR(UCHAR_MAMEKEY(6_PAD))

	PORT_START("ROW.4")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("Num 7") PORT_CODE(KEYCODE_7_PAD) PORT_CHAR(UCHAR_MAMEKEY(7_PAD))
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("Num 8") PORT_CODE(KEYCODE_8_PAD) PORT_CHAR(UCHAR_MAMEKEY(8_PAD))
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("Num 9") PORT_CODE(KEYCODE_9_PAD) PORT_CHAR(UCHAR_MAMEKEY(9_PAD))
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("Num 0") PORT_CODE(KEYCODE_0_PAD) PORT_CHAR(UCHAR_MAMEKEY(0_PAD))
INPUT_PORTS_END


void eurocom2_state::machine_reset()
{
	m_kbd_ready = false;
	m_floppy = nullptr;

	if (ioport("S1")->read() & 0x80)
		m_sst->adjust(attotime::from_usec(12200), 0, attotime::from_usec(12200));
	else
		m_sst->adjust(attotime::never, 0, attotime::never);
}

void eurocom2_state::machine_start()
{
	m_sst = timer_alloc(0);
	m_tmpbmp.allocate(VC_DISP_HORZ, VC_DISP_VERT);
}


FLOPPY_FORMATS_MEMBER( eurocom2_state::floppy_formats )
	FLOPPY_PPG_FORMAT
FLOPPY_FORMATS_END

static SLOT_INTERFACE_START( eurocom_floppies )
	SLOT_INTERFACE( "525qd", FLOPPY_525_QD )
	SLOT_INTERFACE( "8dsdd", FLOPPY_8_DSDD )
SLOT_INTERFACE_END

static MACHINE_CONFIG_START( eurocom2 )
	MCFG_CPU_ADD("maincpu", M6809, XTAL_5_3586MHz/4)
	MCFG_CPU_PROGRAM_MAP(eurocom2_map)

	MCFG_SCREEN_ADD_MONOCHROME("screen", RASTER, rgb_t::green())
	MCFG_SCREEN_RAW_PARAMS(XTAL_5_3586MHz*2, VC_TOTAL_HORZ, 0, VC_DISP_HORZ, VC_TOTAL_VERT, 0, VC_DISP_VERT)
	MCFG_SCREEN_UPDATE_DRIVER(eurocom2_state, screen_update)

	MCFG_SCREEN_PALETTE("palette")
	MCFG_PALETTE_ADD_MONOCHROME("palette")

	MCFG_DEVICE_ADD("keyboard", GENERIC_KEYBOARD, 0)
	MCFG_GENERIC_KEYBOARD_CB(PUT(eurocom2_state, kbd_put))

	MCFG_DEVICE_ADD("pia1", PIA6821, 0)
	MCFG_PIA_READCA1_HANDLER(READLINE(eurocom2_state, pia1_ca1_r))  // keyboard strobe
	MCFG_PIA_READCA2_HANDLER(READLINE(eurocom2_state, pia1_ca2_r))  // SST output Q14
	MCFG_PIA_READCB1_HANDLER(READLINE(eurocom2_state, pia1_cb1_r))  // SST output Q6
	MCFG_PIA_CB2_HANDLER(WRITELINE(eurocom2_state, pia1_cb2_w)) // SST reset input
	MCFG_PIA_READPA_HANDLER(READ8(eurocom2_state, kbd_get))
//  MCFG_PIA_READPB_HANDLER(READ8(eurocom2_state, kbd_get))
//  MCFG_PIA_IRQA_HANDLER(INPUTLINE("maincpu", M6809_IRQ_LINE))
//  MCFG_PIA_IRQB_HANDLER(INPUTLINE("maincpu", M6809_IRQ_LINE))

	MCFG_DEVICE_ADD("pia2", PIA6821, 0)
//  MCFG_PIA_IRQA_HANDLER(INPUTLINE("maincpu", M6809_FIRQ_LINE))
//  MCFG_PIA_IRQB_HANDLER(INPUTLINE("maincpu", M6809_FIRQ_LINE))

	MCFG_DEVICE_ADD("acia", ACIA6850, 0)
	MCFG_ACIA6850_TXD_HANDLER(DEVWRITELINE ("rs232", rs232_port_device, write_txd))
	MCFG_ACIA6850_RTS_HANDLER(DEVWRITELINE ("rs232", rs232_port_device, write_rts))
	MCFG_RS232_PORT_ADD("rs232", default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE ("acia", acia6850_device, write_rxd))
	MCFG_RS232_CTS_HANDLER(DEVWRITELINE ("acia", acia6850_device, write_cts))

	MCFG_FD1793_ADD("fdc", XTAL_2MHz/2)
//  MCFG_WD_FDC_INTRQ_CALLBACK(INPUTLINE("maincpu", M6809_IRQ_LINE))
	MCFG_FLOPPY_DRIVE_ADD("fdc:0", eurocom_floppies, "525qd", eurocom2_state::floppy_formats)
//  MCFG_FLOPPY_DRIVE_SOUND(true)
	MCFG_FLOPPY_DRIVE_ADD("fdc:1", eurocom_floppies, "525qd", eurocom2_state::floppy_formats)
//  MCFG_FLOPPY_DRIVE_SOUND(true)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED(waveterm, eurocom2)
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(waveterm_map)

	MCFG_DEVICE_MODIFY("pia2")
	MCFG_PIA_CB2_HANDLER(WRITELINE(waveterm_state, waveterm_kbh_w))
	MCFG_PIA_WRITEPB_HANDLER(WRITE8(waveterm_state, waveterm_kb_w))
	MCFG_PIA_READPB_HANDLER(READ8(waveterm_state, waveterm_kb_r))

	// ports A(in/out), B(out), CA1(in), CA2(in), and CB2(out) = interface to PPG bus via DIL socket on WTI board
	// CB1 -- front panel "End" button
	MCFG_DEVICE_ADD("pia3", PIA6821, 0)
//  MCFG_PIA_READPA_HANDLER(READ8(waveterm_state, pia3_pa_r))
//  MCFG_PIA_WRITEPA_HANDLER(WRITE8(waveterm_state, pia3_pa_w))
	MCFG_PIA_WRITEPB_HANDLER(WRITE8(waveterm_state, pia3_pb_w))
//  MCFG_PIA_READCA1_HANDLER(READLINE(waveterm_state, pia3_ca1_r))
//  MCFG_PIA_READCA2_HANDLER(READLINE(waveterm_state, pia3_ca2_r))
	MCFG_PIA_READCB1_HANDLER(IOPORT("FP"))
//  MCFG_PIA_CB2_HANDLER(WRITELINE(waveterm_state, pia3_cb2_w))

	MCFG_DEVICE_ADD("ptm", PTM6840, 0)

	MCFG_SOFTWARE_LIST_ADD("disk_list", "waveterm")
MACHINE_CONFIG_END


ROM_START(eurocom2)
	ROM_REGION(0x10000, "maincpu", 0)

	ROM_DEFAULT_BIOS("mon54")
	ROM_SYSTEM_BIOS(0, "mon24", "Eurocom Control V2.4")
	ROMX_LOAD("mon24.bin", 0x0000, 0x1000, CRC(abf5e115) SHA1(d056705779e109bb56c82f906e2e5a52efe77ec1), ROM_BIOS(1))
	ROM_SYSTEM_BIOS(1, "mon53", "Eurocom Control V5.3")
	ROMX_LOAD("mon53.bin", 0x0000, 0x1000, CRC(fb39c2ad) SHA1(8ce07c349c56f92503f11bb63e32e32c139c003a), ROM_BIOS(2))
	ROM_SYSTEM_BIOS(2, "mon54", "Eurocom Control V5.4")
	ROMX_LOAD("mon54.bin", 0x0000, 0x1000, CRC(2c5a4ad2) SHA1(67b9deec5a6a71d768e35ac97c16cb8992ae159f), ROM_BIOS(3))
	ROM_SYSTEM_BIOS(3, "monu546", "Eurocom Control U5.4")
	ROMX_LOAD("monu54-6.bin", 0x0000, 0x1000, CRC(80c82fa8) SHA1(7255bc2dd536d3dd08cca3ea46992e5ca59323b1), ROM_BIOS(4))
	ROM_SYSTEM_BIOS(4, "neumon54", "New Monitor 5.4")
	ROMX_LOAD("neumon54.bin", 0x0000, 0x1000, CRC(2b60ca41) SHA1(c7252d2e9b267b046f4f3ea6cd77e40d4744a33e), ROM_BIOS(5))
ROM_END

ROM_START(waveterm)
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD("rom.bin", 0x0000, 0x1000, CRC(add3c20f) SHA1(4d47d99231bff2209634e6aac5710e782ee2f6da))
ROM_END


//    YEAR  NAME      PARENT    COMPAT  MACHINE   INPUT     CLASS           INIT  COMPANY  FULLNAME         FLAGS
COMP( 1981, eurocom2, 0,        0,      eurocom2, eurocom2, eurocom2_state, 0,    "Eltec", "Eurocom II V7", MACHINE_IS_SKELETON )
COMP( 1982, waveterm, eurocom2, 0,      waveterm, waveterm, waveterm_state, 0,    "PPG",   "Waveterm A",    MACHINE_IS_SKELETON )
