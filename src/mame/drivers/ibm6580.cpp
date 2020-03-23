// license:BSD-3-Clause
// copyright-holders:Robbbert, Sergey Svishchev
/***************************************************************************

IBM 6580 Displaywriter.

A green-screen dedicated word-processing workstation. It uses 8" floppy
disks. It could have up to 224k of ram.  Consists of:
    Electronics Module 6580
    Display 3300
    Keyboard 5330 [a "beamspring"-type]
    Dual Diskette Unit 6360
Optional:
    Printers: 5215, 5218, 5228
    Printer Sharing feature
    Mag Card Unit
    Asynchronous and Bisynchronous communications features
    66-line display and adapter (800x1056 px, 8x16 character cell)


All chips have IBM part numbers on them.  F.e. on system board:
    8493077 - 8086
    4178619 - 8251A
    4178617 - 8257-5
    4178623 - 8259A
    4178628 - 8255A-5
    4178625 - 8253-5


IRQ levels per PSM p. 6-5
    0   incoming data for printer sharing/3277 DE
    1   transfer data to commo data link
    2   printer and mag card data xfer
    3   keyboard incoming data
    4   diskette
    5   (not in use)
    6   software timer [50 ms period]
    7   error on commo data link
    nmi "or when a dump switch operation is initiated" ["memory record" button]


To do:
- verify all frequency sources, document ROM revisions
- memory size options
- bus errors, interrupts

- 92-key keyboard variant, keyboard click/beep, keyboard layouts

- 25-line video board (instant scroll, sub/superscripts, graphics mode)
- 66-line video board

- either emulate floppy board, or complete its HLE (drive select, etc.)
- add support for 8" drives with no track 0 sensor
  (recalibrate command is expected to return 0x70 in ST0)
- double density floppies
- "memory record" (system dump) generation to floppies

- pass BAT with no errors (Basic Assurance Test)
- pass RNA with no errors (Resident Non-Automatic Test)
- pass PDD with no errors (Problem Determination Disk)
- pass CED with no errors (Customer Engineering Diagnostics)

- boot Textpack successfully (currently crashes with *90x* message)


Useful documents:

bitsavers://pdf/ibm/6580_Displaywriter/S241-6248-3_Displaywriter_Product_Support_Manual_Feb83.pdf
bitsavers://pdf/ibm/6580_Displaywriter/S241-6248-2_Displaywriter_6360_6580_Product_Support_Manual_May82.pdf
bitsavers://pdf/ibm/6580_Displaywriter/S241-6250-5_Displaywriter_6250_6580_Maintenance_Analysis_Procedures_May82.pdf
http://www.nostalgia8.nl/cpm/ibm/cpm6dwrm.pdf
http://www.kbdbabel.org/schematic/kbdbabel_doc_ibm_displaywriter.pdf
https://docs.google.com/spreadsheets/d/1SYY_HrBqKjSOX9W4fe5xUsjbfiCt0Umjpo4ZIwgG3Nk/edit?usp=sharing


Wanted:

Displaywriter System Manual S544-2023-0 (?) -- mentioned in US patents 4648071 and 5675827
"IBM Displaywriter System Printer Guide," Order No. S544-0861-2, Copyright 1980.
"Displaywriter System Product Support Manual," Order No. S241-6248-1, Copyright 1980

****************************************************************************/

#include "emu.h"

#include "bus/rs232/rs232.h"
#include "cpu/i86/i86.h"
#include "imagedev/floppy.h"
#include "machine/i8251.h"
#include "machine/i8255.h"
#include "machine/i8257.h"
#include "machine/pic8259.h"
#include "machine/pit8253.h"
#include "machine/ibm6580_kbd.h"
//nclude "machine/ibm6580_fdc.h"
#include "machine/ram.h"
#include "machine/upd765.h"

#include "screen.h"

#include "ibm6580.lh"


#define I8086_TAG       "i8086"
#define I8259A_TAG      "i8259"
#define I8255A_TAG      "i8255a"
#define I8253_TAG       "i8253"
#define UPD765_TAG      "upd765"


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


uint8_t gfx_expand[16] = {
	0x00,   0x03,   0x0c,   0x0f,
	0x30,   0x33,   0x3c,   0x3f,
	0xc0,   0xc3,   0xcc,   0xcf,
	0xf0,   0xf3,   0xfc,   0xff
};


class ibm6580_state : public driver_device
{
public:
	ibm6580_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_p_videoram(*this, "videoram")
		, m_ram(*this, RAM_TAG)
		, m_maincpu(*this, "maincpu")
		, m_pic8259(*this, "pic8259")
		, m_pit8253(*this, "pit8253")
		, m_ppi8255(*this, "ppi8255")
		, m_dma8257(*this, "dma8257")
		, m_screen(*this, "screen")
		, m_kbd(*this, "kbd")
		, m_fdc(*this, UPD765_TAG)
		, m_flop0(*this, UPD765_TAG ":0")
		, m_flop1(*this, UPD765_TAG ":1")
		, m_p_chargen(*this, "chargen")
	{ }

	DECLARE_PALETTE_INIT(ibm6580);

	DECLARE_WRITE16_MEMBER(pic_latch_w);
	DECLARE_WRITE16_MEMBER(unk_latch_w);

	DECLARE_WRITE8_MEMBER(p40_w);
	DECLARE_READ8_MEMBER(p40_r);

	DECLARE_WRITE8_MEMBER(video_w);
	DECLARE_READ8_MEMBER(video_r);
	DECLARE_WRITE_LINE_MEMBER(vblank_w);

	DECLARE_READ8_MEMBER(ppi_a_r);
	DECLARE_WRITE8_MEMBER(led_w);
	DECLARE_WRITE8_MEMBER(ppi_c_w);
	DECLARE_READ8_MEMBER(ppi_c_r);

	DECLARE_WRITE_LINE_MEMBER(kb_data_w);
	DECLARE_WRITE_LINE_MEMBER(kb_clock_w);
	DECLARE_WRITE_LINE_MEMBER(kb_strobe_w);

	DECLARE_WRITE8_MEMBER(floppy_w);
	DECLARE_READ8_MEMBER(floppy_r);
	DECLARE_FLOPPY_FORMATS(floppy_formats);
	DECLARE_WRITE_LINE_MEMBER(floppy_intrq);
	DECLARE_WRITE_LINE_MEMBER(floppy_hdl);
	DECLARE_WRITE8_MEMBER(dmapg_w);
	DECLARE_WRITE_LINE_MEMBER(hrq_w);
	DECLARE_READ8_MEMBER(memory_read_byte);
	DECLARE_WRITE8_MEMBER(memory_write_byte);

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

private:
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;

	uint8_t m_p40, m_p50, m_e000, m_kb_data, m_ppi_c;
	bool m_kb_data_bit, m_kb_strobe;
	util::fifo<uint8_t, 4> m_kb_fifo;

	util::fifo<uint8_t, 4> m_floppy_mcu_sr, m_floppy_mcu_cr;
	int m_floppy_mcu_cr_fifo;
	uint8_t m_floppy_sr;
	floppy_image_device *m_floppies[2];
	floppy_image_device *m_floppy;
	bool m_floppy_intrq, m_floppy_hdl, m_floppy_idle;
	uint8_t m_dma0pg;
	uint8_t floppy_mcu_command();
	bool floppy_mcu_cr_full();

	required_shared_ptr<uint16_t> m_p_videoram;
	required_device<ram_device> m_ram;
	required_device<cpu_device> m_maincpu;
	required_device<pic8259_device> m_pic8259;
	required_device<pit8253_device> m_pit8253;
	required_device<i8255_device> m_ppi8255;
	required_device<i8257_device> m_dma8257;
	required_device<screen_device> m_screen;
	required_device<dw_keyboard_device> m_kbd;
	required_device<upd765a_device> m_fdc;
	required_device<floppy_connector> m_flop0;
	required_device<floppy_connector> m_flop1;
	required_region_ptr<u8> m_p_chargen;
};


WRITE8_MEMBER(ibm6580_state::p40_w)
{
	DBG_LOG(2,"___", ("%02x <- %02x\n", 0x40 + (offset << 1), data));

	switch (offset)
	{
	case 0:
		m_p40 = data | 0x80;
		break;

	case 2:
		if (data)
			m_p40 |= 4;
		break;

	case 5:
		// write_gate0 doesn't work -- counter is read back as 0
		if (BIT(data, 2))
			// hack. video test checks timer counter value and this lets it pass.
			m_pit8253->set_clockin(0, (double)26880000);
		else
			m_pit8253->set_clockin(0, 0.0);
		m_p50 = 0;
		m_ppi_c = data;
		break;

	case 6:
		m_dma0pg = data;
		break;

	case 8:
		m_p50 = data;
		break;

	case 12:
		if (data)
			m_p40 &= ~0x14;
		break;
	}
}

READ8_MEMBER(ibm6580_state::p40_r)
{
	uint8_t data = 0;

	switch (offset)
	{
	case 0:
		data = m_p40;
		m_p40 &= ~4;
		break;

	case 8:
		data = m_p50;
		m_p50 = 1;
		break;
	}

	DBG_LOG(3,"___", ("%02x == %02x\n", 0x40 + (offset << 1), data));

	return data;
}

WRITE8_MEMBER(ibm6580_state::video_w)
{
	DBG_LOG(2,"Video", ("%02x <- %02x\n", 0xe000 + (offset << 1), data));

	switch (offset)
	{
	case 2:
		// some kind of gate
		m_e000 = data;
		break;
	}
}

READ8_MEMBER(ibm6580_state::video_r)
{
	uint8_t data = 0;

	switch (offset)
	{
	case 8:
		data = 1;   // 25-line video board ID.  66-line is 0x40.
		data |= (m_screen->hblank() ? 8 : 0);
		data |= (m_screen->vblank() ? 4 : 0);
		// pure guesswork.  0x2, 0x10 and 0x20 are unknown video signals.
		// 0x20 cannot be zero when 0x10 is zero.
		data |= ((m_screen->vpos() < 2) ? 2 : 0);
		if (m_e000) {
			data |= (m_screen->vblank() ? 0x20 : 0);
			data |= (m_screen->vblank() ? 0 : 0x10);
		}
		break;
	}

	if (offset != 8)
	DBG_LOG(2,"Video", ("%02x == %02x\n", 0xe000 + (offset << 1), data));

	return data;
}

WRITE_LINE_MEMBER(ibm6580_state::vblank_w)
{
//  if (state)
//      m_pic8259->ir6_w(state);

	if (ioport("DUMP")->read())
		m_p40 |= 4;
}

WRITE16_MEMBER(ibm6580_state::pic_latch_w)
{
	DBG_LOG(2,"PIC", ("latch <- %02x\n", data));

	if (data)
		m_p40 |= 8;

	m_pic8259->ir0_w(data == 2 ? ASSERT_LINE : CLEAR_LINE);
	m_pic8259->ir1_w(data == 2 ? ASSERT_LINE : CLEAR_LINE);
	m_pic8259->ir2_w(data == 2 ? ASSERT_LINE : CLEAR_LINE);
	m_pic8259->ir3_w(data == 2 ? ASSERT_LINE : CLEAR_LINE);
	m_pic8259->ir4_w(data == 2 ? ASSERT_LINE : CLEAR_LINE);
	m_pic8259->ir5_w(data == 2 ? ASSERT_LINE : CLEAR_LINE);
	m_pic8259->ir6_w(data == 2 ? ASSERT_LINE : CLEAR_LINE);
	m_pic8259->ir7_w(data == 2 ? ASSERT_LINE : CLEAR_LINE);
}

WRITE16_MEMBER(ibm6580_state::unk_latch_w)
{
	DBG_LOG(2,"UNK", ("latch <- %02x\n", data));

	m_p40 |= 0x10;
}

WRITE8_MEMBER(ibm6580_state::led_w)
{
	output().set_value("led5", BIT(data, 7));
	output().set_value("led6", BIT(data, 6));
	output().set_value("led7", BIT(data, 5));
	output().set_value("led8", BIT(data, 4));

	if (data & 0xf)
		return;

	switch (data >> 4)
	{
	case 0x1:
		printf ("LED 0 0001: Parity Generator/Checker\n");
		break;

	case 0xe:
		printf ("LED 0 1110: Base RAM\n");
		break;

	case 0x3:
		printf ("LED 0 0011: Processor Extension Test\n");
		break;

	case 0x4:
		printf ("LED 0 0100: Display RAM\n");
		break;

	case 0x5:
		printf ("LED 0 0101: Display Adapter Timing Test, Video Test\n");
		break;

	case 0x6:
		printf ("LED 0 0110: Keyboard Cable Test, Physical Keyboard Test\n");
		break;

	case 0x7:
		printf ("LED 0 0111: DMA Controller Test\n");
		break;

	case 0x8:
		printf ("LED 0 1000: Diskette Module Wrap Test, Adapter Test\n");
		break;

	case 0x9:
		printf ("LED 0 1001: Extra RAM Test\n");
		break;

	case 0xa:
		printf ("LED 0 1010: Bus Time-Out Test\n");
		break;

	case 0xc:
		printf ("LED 0 1100: RAM Addressability Test\n");
		break;

	default:
//      printf ("LED 0x%08x: unknown\n", data);
		break;
	}
}

WRITE8_MEMBER(ibm6580_state::ppi_c_w)
{
	DBG_LOG(2,"PPI", ("Port C <- %02x\n", data));

	// bit 5 -- acknowledge
	// bit 6 -- reset
	// bit 7 -- ?? gate

	if (!BIT(data, 6)) {
		m_kb_fifo.clear();
	}

	m_kbd->reset_w(!BIT(data, 6));
	m_kbd->ack_w(BIT(data, 5));
}

READ8_MEMBER(ibm6580_state::ppi_c_r)
{
	uint8_t data = 0;

	data |= (m_kb_strobe << 3);

	DBG_LOG(3,"PPI", ("Port C == %02x\n", data));

	return data;
}

READ8_MEMBER(ibm6580_state::ppi_a_r)
{
	uint8_t data = m_kb_fifo.dequeue();

	DBG_LOG(2,"PPI", ("Port A == %02x (fifo full: %d)\n", data, m_kb_fifo.full()));

	return data;
}

WRITE_LINE_MEMBER(ibm6580_state::kb_data_w)
{
	m_kb_data_bit = !state;
}

WRITE_LINE_MEMBER(ibm6580_state::kb_clock_w)
{
	if (!state)
		m_kb_data = (m_kb_data >> 1) | (m_kb_data_bit << 7);
}

WRITE_LINE_MEMBER(ibm6580_state::kb_strobe_w)
{
	m_kb_strobe = !state;
	if (!state && BIT(m_ppi_c, 0)) {
		m_kb_fifo.enqueue(m_kb_data);
		DBG_LOG(1,"Kbd", ("enqueue %02x (fifo full: %d, m_ppi_c %02x)\n", m_kb_data, m_kb_fifo.full(), m_ppi_c));
	}
}

WRITE_LINE_MEMBER(ibm6580_state::floppy_intrq)
{
	m_floppy_intrq = state;
	if (state)
		m_floppy_idle = true;
}

WRITE_LINE_MEMBER(ibm6580_state::hrq_w)
{
	m_maincpu->set_input_line(INPUT_LINE_HALT, state);
	m_dma8257->hlda_w(state);
}

READ8_MEMBER(ibm6580_state::memory_read_byte)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM);
	return prog_space.read_byte(offset | (m_dma0pg << 16));
}

WRITE8_MEMBER(ibm6580_state::memory_write_byte)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM);
	prog_space.write_byte(offset | (m_dma0pg << 16), data);
}

bool ibm6580_state::floppy_mcu_cr_full()
{
	uint8_t command = m_floppy_mcu_cr.peek();

	if ((command & 1) && m_floppy_mcu_cr_fifo == 1)
		return true;
	if (command == 0x0c && m_floppy_mcu_cr_fifo == 4)
		return true;
	if (m_floppy_mcu_cr_fifo == 3)
		return true;
	else
		return false;
}

uint8_t ibm6580_state::floppy_mcu_command()
{
	uint8_t data = 0, command = m_floppy_mcu_cr.dequeue(), i;

	DBG_LOG(2,"Floppy", ("mcu_command %02x\n", command));

	m_floppy_mcu_sr.clear();
	m_floppy_idle = true;

	switch (command)
	{
	case 0:
		m_fdc->soft_reset();
		break;

	// 3 bytes
	case 4:
		break;

	// 1 byte -- get status?
	case 5:
		m_floppy_mcu_sr.enqueue(0x00);
		if (m_flop0->get_device()->exists())
			m_floppy_mcu_sr.enqueue( m_flop0->get_device()->idx_r() ? 0x08 : 0);
		else
			m_floppy_mcu_sr.enqueue(0x08);
		break;

	// 3 bytes, no return -- engage head
	case 6:
		m_floppy_mcu_cr.dequeue();
		i = m_floppy_mcu_cr.dequeue();
		m_floppy_hdl = i;
		break;

	// 1 byte -- read head engage signal
	case 7:
		m_floppy_mcu_sr.enqueue(0x00);
		m_floppy_mcu_sr.enqueue(m_floppy_hdl);
		break;

	// 3 bytes
	case 8:
		break;

	// 4 bytes -- used by cp/m.  drive select?
	case 0xc:
		break;

	// 1 byte -- get drive ready status?
	case 0xd:
		m_floppy_mcu_sr.enqueue(0x00);
		i = 0;
		if (m_flop0->get_device()->exists())
			i |= m_flop0->get_device()->ready_r() ? 0 : 0x40;
		if (m_flop1->get_device()->exists())
			i |= m_flop1->get_device()->ready_r() ? 0 : 0x80;
		m_floppy_mcu_sr.enqueue(i);
		break;

	// 3 bytes, no return -- recalibrate?
	case 0xe:
		m_floppy_mcu_cr.dequeue();
		i = m_floppy_mcu_cr.dequeue();
#if 1
		if (i & 1)
			m_fdc->set_floppy(m_flop0->get_device());
		else if (i & 2)
			m_fdc->set_floppy(m_flop1->get_device());
#endif
		break;

	// 1 byte
	case 0x13:
		break;

	// 1 byte
	case 0x15:
		break;
	}

	m_floppy_mcu_cr.clear();
	m_floppy_mcu_cr_fifo = 0;

	return data;
}

WRITE8_MEMBER(ibm6580_state::floppy_w)
{
	DBG_LOG(2,"Floppy", ("%02x <- %02x\n", 0x8150 + (offset << 1), data));

	switch (offset)
	{
	case 0: // 8150 -- mcu reset?
		m_floppy_mcu_sr.enqueue(0x00);
		m_floppy_mcu_sr.enqueue(0x00);
		break;

	case 1: // 8152
		m_fdc->soft_reset();
		break;

	case 5: // 815A
		m_fdc->fifo_w(space, offset, data);
		if (m_floppy_idle)
			m_floppy_idle = false;
		break;

	case 6: // 815C
		m_floppy_mcu_cr.enqueue(data);
		m_floppy_mcu_cr_fifo++;
		if (floppy_mcu_cr_full())
			floppy_mcu_command();
		break;
	}
}

READ8_MEMBER(ibm6580_state::floppy_r)
{
	uint8_t data = 0;

	switch (offset)
	{
	case 0: // 8150
		// bit 4 -- ?? ready
		// bit 5 -- mcu busy
		// bit 6 -- ?? idle
		if (m_floppy_mcu_sr.empty() && (m_floppy_idle || m_floppy_intrq))
			data |= 0x40;
		break;

	case 4: // 8158
		data = m_fdc->msr_r(space, offset);
		break;

	case 5: // 815a
		data = m_fdc->fifo_r(space, offset);
		break;

	case 6: // 815c
		if (!m_floppy_mcu_sr.empty())
			data = m_floppy_mcu_sr.dequeue();
		break;
	}

	if (offset)
		DBG_LOG(2,"Floppy", ("%02x == %02x\n", 0x8150 + (offset << 1), data));
	else {
		floppy_image_device *f = m_flop0->get_device();

		if (f)
		DBG_LOG(2,"Floppy", ("%02x == %02x (empty %d hdl %d + idle %d irq %d drq %d + dskchg %d idx %d cyl %d)\n",
			0x8150 + (offset << 1), data,
			m_floppy_mcu_sr.empty(), m_floppy_hdl,
			m_floppy_idle, m_fdc->get_irq(), m_fdc->get_drq(),
			f->dskchg_r(), f->idx_r(), f->get_cyl()
			));
		else
		DBG_LOG(2,"Floppy", ("%02x == %02x (idle %d irq %d drq %d)\n",
			0x8150 + (offset << 1), data,
			m_floppy_idle, m_fdc->get_irq(), m_fdc->get_drq()
			));
	}

	return data;
}


static ADDRESS_MAP_START(ibm6580_mem, AS_PROGRAM, 16, ibm6580_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x90000, 0x90001) AM_WRITE(unk_latch_w)
	AM_RANGE(0xef000, 0xeffff) AM_RAM AM_SHARE("videoram")  // 66-line vram starts at 0xec000
	AM_RANGE(0xfc000, 0xfffff) AM_ROM AM_REGION("user1", 0)
ADDRESS_MAP_END

static ADDRESS_MAP_START(ibm6580_io, AS_IO, 16, ibm6580_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0000, 0x0007) AM_DEVREADWRITE8("pic8259", pic8259_device, read, write, 0x00ff)
	AM_RANGE(0x0008, 0x000f) AM_WRITE (pic_latch_w)
	AM_RANGE(0x0010, 0x0017) AM_DEVREADWRITE8("ppi8255", i8255_device, read, write, 0x00ff)
	AM_RANGE(0x0020, 0x003f) AM_DEVREADWRITE8("dma8257", i8257_device, read, write, 0x00ff)
	AM_RANGE(0x0040, 0x005f) AM_READWRITE8(p40_r, p40_w, 0x00ff)
	AM_RANGE(0x0070, 0x007f) AM_UNMAP
	AM_RANGE(0x0120, 0x0127) AM_DEVREADWRITE8("pit8253", pit8253_device, read, write, 0x00ff)
	AM_RANGE(0x0140, 0x0141) AM_DEVREADWRITE8("upd8251a", i8251_device, data_r, data_w, 0x00ff)
	AM_RANGE(0x0142, 0x0143) AM_DEVREADWRITE8("upd8251a", i8251_device, status_r, control_w, 0x00ff)
	AM_RANGE(0x0160, 0x0161) AM_DEVREADWRITE8("upd8251b", i8251_device, data_r, data_w, 0x00ff)
	AM_RANGE(0x0162, 0x0163) AM_DEVREADWRITE8("upd8251b", i8251_device, status_r, control_w, 0x00ff)
	AM_RANGE(0x4000, 0x400f) AM_UNMAP
	AM_RANGE(0x5000, 0x500f) AM_UNMAP
	AM_RANGE(0x6000, 0x601f) AM_UNMAP
	AM_RANGE(0x8060, 0x807f) AM_UNMAP
	AM_RANGE(0x8150, 0x815f) AM_READWRITE8(floppy_r, floppy_w, 0x00ff)  // HLE of floppy board
	AM_RANGE(0x81a0, 0x81af) AM_UNMAP
	AM_RANGE(0xc000, 0xc00f) AM_UNMAP
	AM_RANGE(0xe000, 0xe02f) AM_READWRITE8(video_r, video_w, 0x00ff)
ADDRESS_MAP_END


/* Input ports */
static INPUT_PORTS_START( ibm6580 )
	PORT_START("DUMP")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD ) PORT_NAME("Memory Record") PORT_CODE(KEYCODE_PRTSCR) PORT_CHAR(UCHAR_MAMEKEY(PRTSCR))
INPUT_PORTS_END


uint32_t ibm6580_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	uint8_t y,ra,gfx,fg,bg,chr,attr;
	uint16_t sy=0,ma=25,x,ca;

	fg = 1; bg = 0;

	for (y = 0; y < 25; y++)
	{
		for (ra = 0; ra < 16; ra++)
		{
			uint16_t *p = &bitmap.pix16(sy++);

			// graphics mode
			if (m_p_videoram[ma] & 0x100) {
				for (x = ma; x < ma + 80; x++)
				{
					chr = m_p_videoram[x];
					attr = m_p_videoram[x] >> 8;

					switch (ra >> 1)
					{
					case 0:
						gfx = gfx_expand[chr & 15];
						break;

					case 2:
						gfx = gfx_expand[chr >> 4];
						break;

					case 4:
						gfx = gfx_expand[attr & 15];
						break;

					case 6:
						gfx = gfx_expand[attr >> 4];
						break;

					default:
						gfx = 0;
						break;
					}

					/* Display a scanline of a character */
					*p++ = BIT(gfx, 7) ? fg : bg;
					*p++ = BIT(gfx, 6) ? fg : bg;
					*p++ = BIT(gfx, 5) ? fg : bg;
					*p++ = BIT(gfx, 4) ? fg : bg;
					*p++ = BIT(gfx, 3) ? fg : bg;
					*p++ = BIT(gfx, 2) ? fg : bg;
					*p++ = BIT(gfx, 1) ? fg : bg;
					*p++ = BIT(gfx, 0) ? fg : bg;
				}
			} else {
			// text mode
				for (x = ma; x < ma + 80; x++)
				{
					chr = m_p_videoram[x];
					attr = m_p_videoram[x] >> 8;
					ca = (chr<<4);

					// font 2
					if (attr & 0x02)
						ca += 0x1000;
#if 0
					// superscript
					if (attr & 0x20)
						ca |= (ra < 13) ? ra + 3 : 0;
					// subscript
					if (attr & 0x20)
						ca |= (ra > 2) ? ra - 3 : 0;
#endif

					// underline
					if (attr & 0x08 && ra == 13)
						gfx = 0xff;
					else
						gfx = m_p_chargen[ca | ra];

					// cursor
					if (attr & 0x04 && ra == 14)
						gfx = 0xff;
					else
						gfx = m_p_chargen[ca | ra];

					// reverse video
					if (attr & 0x10)
						gfx ^= 255;

					// intense
					if (attr & 0x04)
						fg = 2;
					else
						fg = 1;

					/* Display a scanline of a character */
					*p++ = BIT(gfx, 7) ? fg : bg;
					*p++ = BIT(gfx, 6) ? fg : bg;
					*p++ = BIT(gfx, 5) ? fg : bg;
					*p++ = BIT(gfx, 4) ? fg : bg;
					*p++ = BIT(gfx, 3) ? fg : bg;
					*p++ = BIT(gfx, 2) ? fg : bg;
					*p++ = BIT(gfx, 1) ? fg : bg;
					*p++ = BIT(gfx, 0) ? fg : bg;
				}
			}
		}
		ma+=80;
	}
	return 0;
}


PALETTE_INIT_MEMBER( ibm6580_state, ibm6580 )
{
	palette.set_pen_color(0, 0, 0, 0 ); /* Black */
	palette.set_pen_color(1, 0, 192, 0 );   /* Normal */
	palette.set_pen_color(2, 0, 255, 0 );   /* Bright */
}

void ibm6580_state::machine_start()
{
	address_space &program = m_maincpu->space(AS_PROGRAM);

	program.install_readwrite_bank(0, m_ram->size() - 1, "bank10");
	membank("bank10")->set_base(m_ram->pointer());

	m_fdc->set_rate(500000); // XXX workaround
}

void ibm6580_state::machine_reset()
{
	m_p40 = m_p50 = m_e000 = m_ppi_c = m_floppy_sr = 0;
	m_kb_data_bit = false;
	m_floppy_idle = true;
	m_kb_fifo.clear();

	m_pit8253->set_clockin(0, 0.0);

	if (ioport("DUMP")->read())
		m_p40 |= 4;

	m_flop0->get_device()->mon_w(!m_flop0->get_device()->exists());
	m_flop1->get_device()->mon_w(!m_flop1->get_device()->exists());
	m_fdc->set_floppy(m_flop0->get_device());
	m_floppy_mcu_sr.clear();
	m_floppy_mcu_cr.clear();
	m_floppy_mcu_cr_fifo = 0;
}

void ibm6580_state::video_start()
{
	memset(m_p_videoram, 0x0, 0x1000);
}

static SLOT_INTERFACE_START( dw_floppies )
	SLOT_INTERFACE( "8sssd", IBM_6360 )
SLOT_INTERFACE_END

static MACHINE_CONFIG_START( ibm6580 )
	MCFG_CPU_ADD("maincpu", I8086, XTAL_14_7456MHz/3)
	MCFG_CPU_PROGRAM_MAP(ibm6580_mem)
	MCFG_CPU_IO_MAP(ibm6580_io)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DEVICE("pic8259", pic8259_device, inta_cb)

	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("128K")
	MCFG_RAM_EXTRA_OPTIONS("160K,192K,224K,256K,320K,384K")

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_RAW_PARAMS(XTAL_25MHz/2, 833, 0, 640, 428, 0, 400)
	MCFG_SCREEN_UPDATE_DRIVER(ibm6580_state, screen_update)
	MCFG_SCREEN_PALETTE("palette")
	MCFG_SCREEN_VBLANK_CALLBACK(WRITELINE(ibm6580_state, vblank_w))
	MCFG_DEFAULT_LAYOUT(layout_ibm6580)

	MCFG_PALETTE_ADD("palette", 3)
	MCFG_PALETTE_INIT_OWNER(ibm6580_state, ibm6580)

	MCFG_PIC8259_ADD("pic8259", INPUTLINE("maincpu", 0), VCC, NOOP)

	MCFG_DEVICE_ADD("ppi8255", I8255, 0)
	MCFG_I8255_IN_PORTA_CB(READ8(ibm6580_state, ppi_a_r))
	MCFG_I8255_OUT_PORTB_CB(WRITE8(ibm6580_state, led_w))
	MCFG_I8255_OUT_PORTC_CB(WRITE8(ibm6580_state, ppi_c_w))
	MCFG_I8255_IN_PORTC_CB(READ8(ibm6580_state, ppi_c_r))

	MCFG_DEVICE_ADD("pit8253", PIT8253, 0)

	MCFG_DEVICE_ADD("kbd", DW_KEYBOARD, 0)
	MCFG_DW_KEYBOARD_OUT_DATA_HANDLER(WRITELINE(ibm6580_state, kb_data_w))
	MCFG_DW_KEYBOARD_OUT_CLOCK_HANDLER(WRITELINE(ibm6580_state, kb_clock_w))
	MCFG_DW_KEYBOARD_OUT_STROBE_HANDLER(WRITELINE(ibm6580_state, kb_strobe_w))
	MCFG_DEVCB_CHAIN_OUTPUT(DEVWRITELINE("ppi8255", i8255_device, pc4_w))

	MCFG_DEVICE_ADD("dma8257", I8257, XTAL_14_7456MHz/3)
	MCFG_I8257_OUT_HRQ_CB(WRITELINE(ibm6580_state, hrq_w))
	MCFG_I8257_OUT_TC_CB(DEVWRITELINE(UPD765_TAG, upd765a_device, tc_line_w))
	MCFG_I8257_IN_MEMR_CB(READ8(ibm6580_state, memory_read_byte))
	MCFG_I8257_OUT_MEMW_CB(WRITE8(ibm6580_state, memory_write_byte))
	MCFG_I8257_IN_IOR_0_CB(DEVREAD8(UPD765_TAG, upd765a_device, mdma_r))
	MCFG_I8257_OUT_IOW_0_CB(DEVWRITE8(UPD765_TAG, upd765a_device, mdma_w))

	MCFG_UPD765A_ADD(UPD765_TAG, false, false)
	MCFG_UPD765_INTRQ_CALLBACK(WRITELINE(ibm6580_state, floppy_intrq))
//  MCFG_DEVCB_CHAIN_OUTPUT(DEVWRITELINE("pic8259", pic8259_device, ir4_w))
	MCFG_UPD765_DRQ_CALLBACK(DEVWRITELINE("dma8257", i8257_device, dreq0_w))
	MCFG_FLOPPY_DRIVE_ADD(UPD765_TAG ":0", dw_floppies, "8sssd", floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD(UPD765_TAG ":1", dw_floppies, "8sssd", floppy_image_device::default_floppy_formats)

	MCFG_DEVICE_ADD( "upd8251a", I8251, 0)
	MCFG_I8251_TXD_HANDLER(DEVWRITELINE("rs232a", rs232_port_device, write_txd))
	MCFG_I8251_DTR_HANDLER(DEVWRITELINE("rs232a", rs232_port_device, write_dtr))
	MCFG_I8251_RTS_HANDLER(DEVWRITELINE("rs232a", rs232_port_device, write_rts))
	MCFG_I8251_RXRDY_HANDLER(DEVWRITELINE("pic8259", pic8259_device, ir2_w))
	MCFG_I8251_TXRDY_HANDLER(DEVWRITELINE("pic8259", pic8259_device, ir2_w))

	MCFG_RS232_PORT_ADD("rs232a", default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE("upd8251a", i8251_device, write_rxd))
	MCFG_RS232_DSR_HANDLER(DEVWRITELINE("upd8251a", i8251_device, write_dsr))
	MCFG_RS232_CTS_HANDLER(DEVWRITELINE("upd8251a", i8251_device, write_cts))

	MCFG_DEVICE_ADD( "upd8251b", I8251, 0)
	MCFG_I8251_TXD_HANDLER(DEVWRITELINE("rs232b", rs232_port_device, write_txd))
	MCFG_I8251_DTR_HANDLER(DEVWRITELINE("rs232b", rs232_port_device, write_dtr))
	MCFG_I8251_RTS_HANDLER(DEVWRITELINE("rs232b", rs232_port_device, write_rts))
	MCFG_I8251_RXRDY_HANDLER(DEVWRITELINE("pic8259", pic8259_device, ir2_w))
	MCFG_I8251_TXRDY_HANDLER(DEVWRITELINE("pic8259", pic8259_device, ir2_w))

	MCFG_RS232_PORT_ADD("rs232b", default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE("upd8251b", i8251_device, write_rxd))
	MCFG_RS232_DSR_HANDLER(DEVWRITELINE("upd8251b", i8251_device, write_dsr))
	MCFG_RS232_CTS_HANDLER(DEVWRITELINE("upd8251b", i8251_device, write_cts))

	MCFG_SOFTWARE_LIST_ADD("flop_list", "ibm6580")
MACHINE_CONFIG_END

/* ROM definition */
ROM_START( ibm6580 )
	ROM_REGION16_LE( 0x4000, "user1", 0 )
	ROM_DEFAULT_BIOS("old")

	ROM_SYSTEM_BIOS(0, "old", "old bios - 1981")
	ROMX_LOAD("8493823_8K.BIN", 0x0001, 0x2000, CRC(aa5524c0) SHA1(9938f2a82828b17966cb0be7fdbf73803c1f10d3),ROM_SKIP(1)|ROM_BIOS(1))
	ROMX_LOAD("8493822_8K.BIN", 0x0000, 0x2000, CRC(90e7e73a) SHA1(d3ee7a4d2cb8f4920b5d95e8c7f4fef06599d24e),ROM_SKIP(1)|ROM_BIOS(1))

	ROM_SYSTEM_BIOS(1, "new", "new bios - 1983?")
	// was downloaded via DDT86
	ROMX_LOAD( "DWROM16KB.bin", 0x0000, 0x4000, BAD_DUMP CRC(ced87929) SHA1(907a46f288809bc93a1f59f3fbef18bd44be42d9),ROM_BIOS(2))

	ROM_REGION( 0x2000, "chargen", 0 )
	ROM_LOAD( "8493383_CHR.BIN", 0x0000, 0x2000, CRC(779044df) SHA1(95ec46f9edf4d44c5dd3c955c73e00754d58e180))
ROM_END

/* Driver */

/*    YEAR  NAME     PARENT  COMPAT   MACHINE   INPUT    CLASS           INIT  COMPANY  FULLNAME       FLAGS */
COMP( 1980, ibm6580, 0,      0,       ibm6580,  ibm6580, ibm6580_state,  0,    "IBM",   "IBM 6580 Displaywriter", MACHINE_IS_SKELETON)
