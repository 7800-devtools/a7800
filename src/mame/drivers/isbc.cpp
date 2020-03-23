// license:BSD-3-Clause
// copyright-holders:Robbbert
/***************************************************************************

        Intel iSBC series

        09/12/2009 Skeleton driver.

Notes:

isbc86 commands: BYTE WORD REAL EREAL ROMTEST. ROMTEST works, the others hang.

Press capital-U to drop into the monitor on the isbc 86/05 and 86/30
The 86/05 can boot floppies with the b command but appears to mostly be
able to deal with 256byte sectors so fails to load the irmx 512byte sector images.

****************************************************************************/

#include "emu.h"
#include "bus/rs232/rs232.h"
#include "cpu/i86/i86.h"
#include "cpu/i86/i286.h"
#include "machine/terminal.h"
#include "machine/pic8259.h"
#include "machine/pit8253.h"
#include "machine/i8255.h"
#include "machine/i8251.h"
//#include "machine/z80dart.h"
#include "machine/z80sio.h"
#include "bus/centronics/ctronics.h"
#include "bus/isbx/isbx.h"
#include "machine/isbc_215g.h"
#include "machine/isbc_208.h"

class isbc_state : public driver_device
{
public:
	isbc_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_uart8251(*this, "uart8251"),
		m_uart8274(*this, "uart8274"),
		m_pic_0(*this, "pic_0"),
		m_pic_1(*this, "pic_1"),
		m_centronics(*this, "centronics"),
		m_cent_status_in(*this, "cent_status_in"),
		m_bios(*this, "user1"),
		m_biosram(*this, "biosram")
	{
	}

	required_device<cpu_device> m_maincpu;
	optional_device<i8251_device> m_uart8251;
//  optional_device<i8274_device> m_uart8274;
	optional_device<i8274_new_device> m_uart8274;
	required_device<pic8259_device> m_pic_0;
	optional_device<pic8259_device> m_pic_1;
	optional_device<centronics_device> m_centronics;
	optional_device<input_buffer_device> m_cent_status_in;
	optional_memory_region m_bios;
	optional_shared_ptr<u16> m_biosram;

	DECLARE_WRITE_LINE_MEMBER(write_centronics_ack);

	DECLARE_WRITE_LINE_MEMBER(isbc86_tmr2_w);
	DECLARE_WRITE_LINE_MEMBER(isbc286_tmr2_w);
//  DECLARE_WRITE_LINE_MEMBER(isbc_uart8274_irq);
	DECLARE_READ8_MEMBER(get_slave_ack);
	DECLARE_WRITE8_MEMBER(ppi_c_w);
	DECLARE_WRITE8_MEMBER(upperen_w);
	DECLARE_READ16_MEMBER(bioslo_r);
	DECLARE_WRITE16_MEMBER(bioslo_w);
protected:
	void machine_reset() override;
private:
	bool m_upperen;
};

void isbc_state::machine_reset()
{
	if(m_centronics)
	{
		m_centronics->write_busy(0);  // centronics_device sets busy to 1 at reset causing spurious irqs
		m_pic_1->ir7_w(0);
	}
	if(m_uart8251)
		m_uart8251->write_cts(0);
	m_upperen = false;
}

static ADDRESS_MAP_START(rpc86_mem, AS_PROGRAM, 16, isbc_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x00000, 0xcffff) AM_RAM
	AM_RANGE(0xf8000, 0xfffff) AM_ROM AM_REGION("user1",0)
ADDRESS_MAP_END

static ADDRESS_MAP_START(rpc86_io, AS_IO, 16, isbc_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0080, 0x008f) AM_DEVREADWRITE8("sbx1", isbx_slot_device, mcs0_r, mcs0_w, 0x00ff)
	AM_RANGE(0x0090, 0x009f) AM_DEVREADWRITE8("sbx1", isbx_slot_device, mcs1_r, mcs1_w, 0x00ff)
	AM_RANGE(0x00a0, 0x00af) AM_DEVREADWRITE8("sbx2", isbx_slot_device, mcs0_r, mcs0_w, 0x00ff)
	AM_RANGE(0x00b0, 0x00bf) AM_DEVREADWRITE8("sbx2", isbx_slot_device, mcs1_r, mcs1_w, 0x00ff)
	AM_RANGE(0x00c0, 0x00c3) AM_DEVREADWRITE8("pic_0", pic8259_device, read, write, 0x00ff)
	AM_RANGE(0x00c4, 0x00c7) AM_DEVREADWRITE8("pic_0", pic8259_device, read, write, 0x00ff)
	AM_RANGE(0x00c8, 0x00cf) AM_DEVREADWRITE8("ppi", i8255_device, read, write, 0x00ff)
	AM_RANGE(0x00d0, 0x00d7) AM_DEVREADWRITE8("pit", pit8253_device, read, write, 0x00ff)
	AM_RANGE(0x00d8, 0x00d9) AM_DEVREADWRITE8("uart8251", i8251_device, data_r, data_w, 0x00ff)
	AM_RANGE(0x00da, 0x00db) AM_DEVREADWRITE8("uart8251", i8251_device, status_r, control_w, 0x00ff)
	AM_RANGE(0x00dc, 0x00dd) AM_DEVREADWRITE8("uart8251", i8251_device, data_r, data_w, 0x00ff)
	AM_RANGE(0x00de, 0x00df) AM_DEVREADWRITE8("uart8251", i8251_device, status_r, control_w, 0x00ff)
ADDRESS_MAP_END

static ADDRESS_MAP_START(isbc8605_io, AS_IO, 16, isbc_state)
	AM_RANGE(0x0000, 0x002f) AM_DEVICE8("isbc_208", isbc_208_device, map, 0xffff)
	AM_IMPORT_FROM(rpc86_io)
ADDRESS_MAP_END

static ADDRESS_MAP_START(isbc8630_io, AS_IO, 16, isbc_state)
	AM_RANGE(0x0100, 0x0101) AM_DEVWRITE8("isbc_215g", isbc_215g_device, write, 0x00ff)
	AM_IMPORT_FROM(rpc86_io)
ADDRESS_MAP_END

static ADDRESS_MAP_START(isbc86_mem, AS_PROGRAM, 16, isbc_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x00000, 0xfbfff) AM_RAM
	AM_RANGE(0xfc000, 0xfffff) AM_ROM AM_REGION("user1",0)
ADDRESS_MAP_END

static ADDRESS_MAP_START(isbc_io, AS_IO, 16, isbc_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x00c0, 0x00c3) AM_DEVREADWRITE8("pic_0", pic8259_device, read, write, 0x00ff)
	AM_RANGE(0x00c4, 0x00c7) AM_DEVREADWRITE8("pic_0", pic8259_device, read, write, 0x00ff)
	AM_RANGE(0x00c8, 0x00cf) AM_DEVREADWRITE8("ppi", i8255_device, read, write, 0x00ff)
	AM_RANGE(0x00d0, 0x00d7) AM_DEVREADWRITE8("pit", pit8253_device, read, write, 0x00ff)
	AM_RANGE(0x00d8, 0x00d9) AM_DEVREADWRITE8("uart8251", i8251_device, data_r, data_w, 0x00ff)
	AM_RANGE(0x00da, 0x00db) AM_DEVREADWRITE8("uart8251", i8251_device, status_r, control_w, 0x00ff)
	AM_RANGE(0x00dc, 0x00dd) AM_DEVREADWRITE8("uart8251", i8251_device, data_r, data_w, 0x00ff)
	AM_RANGE(0x00de, 0x00df) AM_DEVREADWRITE8("uart8251", i8251_device, status_r, control_w, 0x00ff)
ADDRESS_MAP_END

static ADDRESS_MAP_START(isbc286_io, AS_IO, 16, isbc_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x0080, 0x008f) AM_DEVREADWRITE8("sbx1", isbx_slot_device, mcs0_r, mcs0_w, 0x00ff)
	AM_RANGE(0x0080, 0x008f) AM_DEVREADWRITE8("sbx1", isbx_slot_device, mcs0_r, mcs0_w, 0xff00)
	AM_RANGE(0x0090, 0x009f) AM_DEVREADWRITE8("sbx1", isbx_slot_device, mcs1_r, mcs1_w, 0x00ff)
	AM_RANGE(0x0090, 0x009f) AM_DEVREADWRITE8("sbx1", isbx_slot_device, mcs1_r, mcs1_w, 0xff00)
	AM_RANGE(0x00a0, 0x00af) AM_DEVREADWRITE8("sbx2", isbx_slot_device, mcs0_r, mcs0_w, 0x00ff)
	AM_RANGE(0x00a0, 0x00af) AM_DEVREADWRITE8("sbx2", isbx_slot_device, mcs0_r, mcs0_w, 0xff00)
	AM_RANGE(0x00b0, 0x00bf) AM_DEVREADWRITE8("sbx2", isbx_slot_device, mcs1_r, mcs1_w, 0x00ff)
	AM_RANGE(0x00b0, 0x00bf) AM_DEVREADWRITE8("sbx2", isbx_slot_device, mcs1_r, mcs1_w, 0xff00)
	AM_RANGE(0x00c0, 0x00c3) AM_DEVREADWRITE8("pic_0", pic8259_device, read, write, 0x00ff)
	AM_RANGE(0x00c4, 0x00c7) AM_DEVREADWRITE8("pic_1", pic8259_device, read, write, 0x00ff)
	AM_RANGE(0x00c8, 0x00cf) AM_DEVREADWRITE8("ppi", i8255_device, read, write, 0x00ff)
	AM_RANGE(0x00c8, 0x00cf) AM_WRITE8(upperen_w, 0xff00)
	AM_RANGE(0x00d0, 0x00d7) AM_DEVREADWRITE8("pit", pit8254_device, read, write, 0x00ff)
	AM_RANGE(0x00d8, 0x00df) AM_DEVREADWRITE8("uart8274", i8274_new_device, cd_ba_r, cd_ba_w, 0x00ff)
	AM_RANGE(0x0100, 0x0101) AM_DEVWRITE8("isbc_215g", isbc_215g_device, write, 0x00ff)
ADDRESS_MAP_END

static ADDRESS_MAP_START(isbc286_mem, AS_PROGRAM, 16, isbc_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x00000, 0xdffff) AM_RAM
	AM_RANGE(0xe0000, 0xfffff) AM_ROM AM_REGION("user1",0)
	AM_RANGE(0xfe0000, 0xffffff) AM_ROM AM_REGION("user1",0)
ADDRESS_MAP_END

static ADDRESS_MAP_START(isbc2861_mem, AS_PROGRAM, 16, isbc_state)
	ADDRESS_MAP_UNMAP_HIGH
	AM_RANGE(0x00000, 0xdffff) AM_RAM
	AM_RANGE(0xe0000, 0xfffff) AM_READWRITE(bioslo_r, bioslo_w) AM_SHARE("biosram")
//  AM_RANGE(0x100000, 0x1fffff) AM_RAM // FIXME: XENIX doesn't like this, IRMX is okay with it
	AM_RANGE(0xff0000, 0xffffff) AM_ROM AM_REGION("user1",0)
ADDRESS_MAP_END

/* Input ports */
static INPUT_PORTS_START( isbc )
INPUT_PORTS_END

static DEVICE_INPUT_DEFAULTS_START( isbc86_terminal )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_300 )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_300 )
	DEVICE_INPUT_DEFAULTS( "RS232_STARTBITS", 0xff, RS232_STARTBITS_1 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_NONE )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_2 )
DEVICE_INPUT_DEFAULTS_END

static DEVICE_INPUT_DEFAULTS_START( isbc286_terminal )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_STARTBITS", 0xff, RS232_STARTBITS_1 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_7 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_EVEN )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
DEVICE_INPUT_DEFAULTS_END

WRITE_LINE_MEMBER( isbc_state::isbc86_tmr2_w )
{
	m_uart8251->write_rxc(state);
	m_uart8251->write_txc(state);
}

READ8_MEMBER( isbc_state::get_slave_ack )
{
	if (offset == 7)
		return m_pic_1->acknowledge();

	return 0x00;
}

WRITE_LINE_MEMBER( isbc_state::isbc286_tmr2_w )
{
	m_uart8274->rxca_w(state);
	m_uart8274->txca_w(state);
}

WRITE_LINE_MEMBER( isbc_state::write_centronics_ack )
{
	m_cent_status_in->write_bit4(state);

	if(state)
		m_pic_1->ir7_w(1);
}

WRITE8_MEMBER( isbc_state::ppi_c_w )
{
	m_centronics->write_strobe(data & 1);

	if(data & 0x80)
		m_pic_1->ir7_w(0);
}

WRITE8_MEMBER(isbc_state::upperen_w)
{
	m_upperen = true;
}

READ16_MEMBER(isbc_state::bioslo_r)
{
	if(m_upperen)
		return m_biosram[offset];
	else if(offset >= 0x8000)
		return m_bios->as_u16(offset - 0x8000);
	return 0xffff;
}

WRITE16_MEMBER(isbc_state::bioslo_w)
{
	if(m_upperen)
		COMBINE_DATA(&m_biosram[offset]);
}

#if 0
WRITE_LINE_MEMBER(isbc_state::isbc_uart8274_irq)
{
	m_uart8274->m1_r(); // always set
	m_pic_0->ir6_w(state);
}
#endif

static MACHINE_CONFIG_START( isbc86 )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", I8086, XTAL_5MHz)
	MCFG_CPU_PROGRAM_MAP(isbc86_mem)
	MCFG_CPU_IO_MAP(isbc_io)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DEVICE("pic_0", pic8259_device, inta_cb)

	MCFG_PIC8259_ADD("pic_0", INPUTLINE(":maincpu", 0), VCC, NOOP)

	MCFG_DEVICE_ADD("pit", PIT8253, 0)
	MCFG_PIT8253_CLK0(XTAL_22_1184MHz/18)
	MCFG_PIT8253_OUT0_HANDLER(DEVWRITELINE("pic_0", pic8259_device, ir0_w))
	MCFG_PIT8253_CLK1(XTAL_22_1184MHz/18)
	MCFG_PIT8253_CLK2(XTAL_22_1184MHz/18)
	MCFG_PIT8253_OUT2_HANDLER(WRITELINE(isbc_state, isbc86_tmr2_w))

	MCFG_DEVICE_ADD("ppi", I8255A, 0)

	MCFG_DEVICE_ADD("uart8251", I8251, 0)
	MCFG_I8251_TXD_HANDLER(DEVWRITELINE("rs232", rs232_port_device, write_txd))
	MCFG_I8251_DTR_HANDLER(DEVWRITELINE("rs232", rs232_port_device, write_dtr))
	MCFG_I8251_RTS_HANDLER(DEVWRITELINE("rs232", rs232_port_device, write_rts))
	MCFG_I8251_RXRDY_HANDLER(DEVWRITELINE("pic_0", pic8259_device, ir6_w))

	/* video hardware */
	MCFG_RS232_PORT_ADD("rs232", default_rs232_devices, "terminal")
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE("uart8251", i8251_device, write_rxd))
	MCFG_RS232_CTS_HANDLER(DEVWRITELINE("uart8251", i8251_device, write_cts))
	MCFG_RS232_DSR_HANDLER(DEVWRITELINE("uart8251", i8251_device, write_dsr))
	MCFG_DEVICE_CARD_DEVICE_INPUT_DEFAULTS("terminal", isbc86_terminal)
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( rpc86 )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", I8086, XTAL_5MHz)
	MCFG_CPU_PROGRAM_MAP(rpc86_mem)
	MCFG_CPU_IO_MAP(rpc86_io)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DEVICE("pic_0", pic8259_device, inta_cb)

	MCFG_PIC8259_ADD("pic_0", INPUTLINE(":maincpu", 0), VCC, NOOP)

	MCFG_DEVICE_ADD("pit", PIT8253, 0)
	MCFG_PIT8253_CLK0(XTAL_22_1184MHz/18)
	MCFG_PIT8253_OUT0_HANDLER(DEVWRITELINE("pic_0", pic8259_device, ir2_w))
	MCFG_PIT8253_CLK1(XTAL_22_1184MHz/144)
	MCFG_PIT8253_CLK2(XTAL_22_1184MHz/18)
	MCFG_PIT8253_OUT2_HANDLER(WRITELINE(isbc_state, isbc86_tmr2_w))

	MCFG_DEVICE_ADD("ppi", I8255A, 0)

	MCFG_DEVICE_ADD("uart8251", I8251, 0)
	MCFG_I8251_TXD_HANDLER(DEVWRITELINE("rs232", rs232_port_device, write_txd))
	MCFG_I8251_DTR_HANDLER(DEVWRITELINE("rs232", rs232_port_device, write_dtr))
	MCFG_I8251_RTS_HANDLER(DEVWRITELINE("rs232", rs232_port_device, write_rts))
	MCFG_I8251_RXRDY_HANDLER(DEVWRITELINE("pic_0", pic8259_device, ir6_w))
	MCFG_I8251_TXRDY_HANDLER(DEVWRITELINE("pic_0", pic8259_device, ir7_w))

	/* video hardware */
	MCFG_RS232_PORT_ADD("rs232", default_rs232_devices, "terminal")
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE("uart8251", i8251_device, write_rxd))
	//MCFG_RS232_CTS_HANDLER(DEVWRITELINE("uart8251", i8251_device, write_cts))
	MCFG_RS232_DSR_HANDLER(DEVWRITELINE("uart8251", i8251_device, write_dsr))
	MCFG_DEVICE_CARD_DEVICE_INPUT_DEFAULTS("terminal", isbc286_terminal)

	MCFG_ISBX_SLOT_ADD("sbx1", 0, isbx_cards, nullptr)
	//MCFG_ISBX_SLOT_MINTR0_CALLBACK(DEVWRITELINE("pic_0", pic8259_device, ir3_w))
	//MCFG_ISBX_SLOT_MINTR1_CALLBACK(DEVWRITELINE("pic_0", pic8259_device, ir4_w))
	MCFG_ISBX_SLOT_ADD("sbx2", 0, isbx_cards, nullptr)
	//MCFG_ISBX_SLOT_MINTR0_CALLBACK(DEVWRITELINE("pic_0", pic8259_device, ir5_w))
	//MCFG_ISBX_SLOT_MINTR1_CALLBACK(DEVWRITELINE("pic_0", pic8259_device, ir6_w))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( isbc8605, rpc86 )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_IO_MAP(isbc8605_io)

	MCFG_DEVICE_ADD("isbc_208", ISBC_208, 0)
	MCFG_ISBC_208_MAINCPU("maincpu")
	MCFG_ISBC_208_IRQ(DEVWRITELINE("pic_0", pic8259_device, ir5_w))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( isbc8630, rpc86 )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_IO_MAP(isbc8630_io)

	MCFG_ISBC_215_ADD("isbc_215g", 0x100, "maincpu")
	MCFG_ISBC_215_IRQ(DEVWRITELINE("pic_0", pic8259_device, ir5_w))
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( isbc286 )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", I80286, XTAL_16MHz/2)
	MCFG_CPU_PROGRAM_MAP(isbc286_mem)
	MCFG_CPU_IO_MAP(isbc286_io)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DEVICE("pic_0", pic8259_device, inta_cb)

	MCFG_PIC8259_ADD("pic_0", INPUTLINE(":maincpu", 0), VCC, READ8(isbc_state, get_slave_ack))
	MCFG_PIC8259_ADD("pic_1", DEVWRITELINE("pic_0", pic8259_device, ir7_w), GND, NOOP)

	MCFG_DEVICE_ADD("pit", PIT8254, 0)
	MCFG_PIT8253_CLK0(XTAL_22_1184MHz/18)
	MCFG_PIT8253_OUT0_HANDLER(DEVWRITELINE("pic_0", pic8259_device, ir0_w))
	MCFG_PIT8253_CLK1(XTAL_22_1184MHz/18)
//  MCFG_PIT8253_OUT1_HANDLER(DEVWRITELINE("uart8274", z80dart_device, rxtxcb_w))
	MCFG_PIT8253_OUT1_HANDLER(DEVWRITELINE("uart8274", i8274_new_device, rxtxcb_w))
	MCFG_PIT8253_CLK2(XTAL_22_1184MHz/18)
	MCFG_PIT8253_OUT2_HANDLER(WRITELINE(isbc_state, isbc286_tmr2_w))

	MCFG_DEVICE_ADD("ppi", I8255A, 0)
	MCFG_I8255_OUT_PORTA_CB(DEVWRITE8("cent_data_out", output_latch_device, write))
	MCFG_I8255_IN_PORTB_CB(DEVREAD8("cent_status_in", input_buffer_device, read))
	MCFG_I8255_OUT_PORTC_CB(WRITE8(isbc_state, ppi_c_w))

	MCFG_CENTRONICS_ADD("centronics", centronics_devices, "printer")
	MCFG_CENTRONICS_ACK_HANDLER(WRITELINE(isbc_state, write_centronics_ack))
	MCFG_CENTRONICS_BUSY_HANDLER(DEVWRITELINE("cent_status_in", input_buffer_device, write_bit7))
	MCFG_CENTRONICS_FAULT_HANDLER(DEVWRITELINE("cent_status_in", input_buffer_device, write_bit6))

	MCFG_DEVICE_ADD("cent_status_in", INPUT_BUFFER, 0)

	MCFG_CENTRONICS_OUTPUT_LATCH_ADD("cent_data_out", "centronics")

	MCFG_I8274_ADD("uart8274", XTAL_16MHz/4, 0, 0, 0, 0)
#if 0
	MCFG_Z80DART_OUT_TXDA_CB(DEVWRITELINE("rs232a", rs232_port_device, write_txd))
	MCFG_Z80DART_OUT_DTRA_CB(DEVWRITELINE("rs232a", rs232_port_device, write_dtr))
	MCFG_Z80DART_OUT_RTSA_CB(DEVWRITELINE("rs232a", rs232_port_device, write_rts))
	MCFG_Z80DART_OUT_TXDB_CB(DEVWRITELINE("rs232b", rs232_port_device, write_txd))
	MCFG_Z80DART_OUT_DTRB_CB(DEVWRITELINE("rs232b", rs232_port_device, write_dtr))
	MCFG_Z80DART_OUT_RTSB_CB(DEVWRITELINE("rs232b", rs232_port_device, write_rts))
	MCFG_Z80DART_OUT_INT_CB(WRITELINE(isbc_state, isbc_uart8274_irq))
#else
	MCFG_Z80SIO_OUT_TXDA_CB(DEVWRITELINE("rs232a", rs232_port_device, write_txd))
	MCFG_Z80SIO_OUT_DTRA_CB(DEVWRITELINE("rs232a", rs232_port_device, write_dtr))
	MCFG_Z80SIO_OUT_RTSA_CB(DEVWRITELINE("rs232a", rs232_port_device, write_rts))
	MCFG_Z80SIO_OUT_TXDB_CB(DEVWRITELINE("rs232b", rs232_port_device, write_txd))
	MCFG_Z80SIO_OUT_DTRB_CB(DEVWRITELINE("rs232b", rs232_port_device, write_dtr))
	MCFG_Z80SIO_OUT_RTSB_CB(DEVWRITELINE("rs232b", rs232_port_device, write_rts))
//  MCFG_Z80SIO_OUT_INT_CB(WRITELINE(isbc_state, isbc_uart8274_irq))
	MCFG_Z80SIO_OUT_INT_CB(DEVWRITELINE("pic_0", pic8259_device, ir6_w))
#endif

	MCFG_RS232_PORT_ADD("rs232a", default_rs232_devices, nullptr)
#if 0
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE("uart8274", z80dart_device, rxa_w))
	MCFG_RS232_DCD_HANDLER(DEVWRITELINE("uart8274", z80dart_device, dcda_w))
	MCFG_RS232_CTS_HANDLER(DEVWRITELINE("uart8274", z80dart_device, ctsa_w))
#else
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE("uart8274", i8274_new_device, rxa_w))
	MCFG_RS232_DCD_HANDLER(DEVWRITELINE("uart8274", i8274_new_device, dcda_w))
	MCFG_RS232_CTS_HANDLER(DEVWRITELINE("uart8274", i8274_new_device, ctsa_w))
#endif

	MCFG_RS232_PORT_ADD("rs232b", default_rs232_devices, "terminal")
#if 0
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE("uart8274", z80dart_device, rxb_w))
	MCFG_RS232_DCD_HANDLER(DEVWRITELINE("uart8274", z80dart_device, dcdb_w))
	MCFG_RS232_CTS_HANDLER(DEVWRITELINE("uart8274", z80dart_device, ctsb_w))
#else
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE("uart8274", i8274_new_device, rxb_w))
	MCFG_RS232_DCD_HANDLER(DEVWRITELINE("uart8274", i8274_new_device, dcdb_w))
	MCFG_RS232_CTS_HANDLER(DEVWRITELINE("uart8274", i8274_new_device, ctsb_w))
#endif
	MCFG_DEVICE_CARD_DEVICE_INPUT_DEFAULTS("terminal", isbc286_terminal)

	MCFG_ISBX_SLOT_ADD("sbx1", 0, isbx_cards, nullptr)
	MCFG_ISBX_SLOT_MINTR0_CALLBACK(DEVWRITELINE("pic_1", pic8259_device, ir3_w))
	MCFG_ISBX_SLOT_MINTR1_CALLBACK(DEVWRITELINE("pic_1", pic8259_device, ir4_w))
	MCFG_ISBX_SLOT_ADD("sbx2", 0, isbx_cards, nullptr)
	MCFG_ISBX_SLOT_MINTR0_CALLBACK(DEVWRITELINE("pic_1", pic8259_device, ir5_w))
	MCFG_ISBX_SLOT_MINTR1_CALLBACK(DEVWRITELINE("pic_1", pic8259_device, ir6_w))

	MCFG_ISBC_215_ADD("isbc_215g", 0x100, "maincpu")
	MCFG_ISBC_215_IRQ(DEVWRITELINE("pic_0", pic8259_device, ir5_w))
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( isbc2861, isbc286 )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(isbc2861_mem)
MACHINE_CONFIG_END

/* ROM definition */
ROM_START( isbc86 )
	ROM_REGION( 0x4000, "user1", ROMREGION_ERASEFF )
	ROM_LOAD16_BYTE( "8612_2u.bin", 0x0001, 0x1000, CRC(84fa14cf) SHA1(783e1459ab121201fd49368d4bf769c1bab6447a))
	ROM_LOAD16_BYTE( "8612_2l.bin", 0x0000, 0x1000, CRC(922bda5f) SHA1(15743e69f3aba56425fa004d19b82ec20532fd72))
	ROM_LOAD16_BYTE( "8612_3u.bin", 0x2001, 0x1000, CRC(68d47c3e) SHA1(16c17f26b33daffa84d065ff7aefb581544176bd))
	ROM_LOAD16_BYTE( "8612_3l.bin", 0x2000, 0x1000, CRC(17f27ad2) SHA1(c3f379ac7d67dc4a0a7a611a0bc6323b8a3d4840))
ROM_END

ROM_START( isbc8605 )
	ROM_REGION( 0x8000, "user1", ROMREGION_ERASEFF )
	ROM_LOAD( "i8605mon.bin", 0x4000, 0x4000, CRC(e16acb6e) SHA1(eb9a3fd21f7609d44f8052b6a0603ecbb52dc3f3))
ROM_END

ROM_START( isbc8630 )
	ROM_REGION( 0x8000, "user1", ROMREGION_ERASEFF )
	ROM_SYSTEM_BIOS( 0, "14378", "14378" )
	ROMX_LOAD( "143780-001_isdm_for_isbc_86-30_socket_u57_i2732a.bin", 0x4000, 0x1000, CRC(db0ef880) SHA1(8ef296066d16881217618e54b410d12157f318ea), ROM_SKIP(1) | ROM_BIOS(1))
	ROMX_LOAD( "143782-001_isdm_for_isbc_86-30_socket_u39_i2732a.bin", 0x4001, 0x1000, CRC(ea1ebe78) SHA1(f03b63659e8f5e96f481dbc6c2ddef1d22850ebb), ROM_SKIP(1) | ROM_BIOS(1))
	ROMX_LOAD( "143781-001_isdm_for_isbc_86-30_socket_u58_i2732a.bin", 0x6000, 0x1000, CRC(93732612) SHA1(06e751d0f5ab1fe2c52fd79f6f4725ccf3379791), ROM_SKIP(1) | ROM_BIOS(1))
	ROMX_LOAD( "143783-001_isdm_for_isbc_86-30_socket_u40_i2732a.bin", 0x6001, 0x1000, CRC(337102d5) SHA1(535f63d24c3948187b208ea594f979bc33579a15), ROM_SKIP(1) | ROM_BIOS(1))
	ROM_SYSTEM_BIOS( 1, "14503", "14503" )
	ROMX_LOAD( "145032-001_u57.bin", 0x0000, 0x2000, CRC(09a24dea) SHA1(e21277f1d4d72e0858846f7293ac48417b392e3b), ROM_SKIP(1) | ROM_BIOS(2))
	ROMX_LOAD( "145030-001_u39.bin", 0x0001, 0x2000, CRC(c58f3a98) SHA1(76f6d5be8ea6854a98f6555320cfcdb814e5c633), ROM_SKIP(1) | ROM_BIOS(2))
	ROMX_LOAD( "145033-001_u58.bin", 0x4000, 0x2000, CRC(496aca5f) SHA1(c09f4d2254ece1eb139ef5fd4ad0ce6a55376da5), ROM_SKIP(1) | ROM_BIOS(2))
	ROMX_LOAD( "145031-001_u40.bin", 0x4001, 0x2000, CRC(150fcd90) SHA1(4bca0f46b9b05ef0124bac5dea09ddd952e73af2), ROM_SKIP(1) | ROM_BIOS(2))
ROM_END

ROM_START( isbc286 )
	ROM_REGION( 0x20000, "user1", ROMREGION_ERASEFF )
	ROM_LOAD16_BYTE( "u79.bin", 0x00001, 0x10000, CRC(144182ea) SHA1(4620ca205a6ac98fe2636183eaead7c4bfaf7a72))
	ROM_LOAD16_BYTE( "u36.bin", 0x00000, 0x10000, CRC(22db075f) SHA1(fd29ea77f5fc0697c8f8b66aca549aad5b9db3ea))
ROM_END

/*
 * :uart8274 A Reg 00 <- 18 - Channel reset command
 * :uart8274 A Reg 04 <- 44 - x16 clock, 1 stop bit, no parity
 * :uart8274 A Reg 05 <- ea - Tx Enabled, Tx 8 bits, Send Break 0, RTS=0, <DTR=0
 * :uart8274 A Reg 03 <- c1 - Rx Enabled, Rx 8 bits, No Auto Enable
 * :uart8274 B Reg 00 <- 18 - Channel reset command
 * :uart8274 B Reg 04 <- 44 - x16 clock, 1 stop bit, no parity
 * :uart8274 B Reg 05 <- ea - Tx Enabled, Transmitter Bits/Character 8, Send Break 0, RTS=0, DTR=0
 * :uart8274 B Reg 03 <- c1 - Rx 8 bits, No Auto Enables, Rx Enabled,

 * :uart8274 B Reg 00 <- 18 - Channel reset command
 * :uart8274 B Reg 04 <- 4e - x16 clock, 2 stop bit, even parity but parity disabled
 * :uart8274 B Reg 05 <- ea - Tx Enabled, Tx 8 bits, Send Break 0, RTS=0, DTR=0
 * :uart8274 B Reg 03 <- c1 - Rx Enabled, Rx 8 bits, No Auto Enables
 * :uart8274 B Reg 07 <- 00 - Hi SYNC bits
 * :uart8274 B Reg 06 <- 00 - Lo SYNC bits
 * :uart8274 A Reg 02 <- 04 - RTSB selected, non vectored mode, 85-1 mode selected, A over B interleaved int prios
 * :uart8274 B Reg 02 <- 26 - interrupt vector 26
 * :uart8274 B Reg 01 <- 00 - Rx INT/DMA int disabled, no vector modification

 * :uart8274 B Reg 00 <- 18 - Channel reset command
 * :uart8274 B Reg 00 <- 18 - Channel reset command
 * :uart8274 B Reg 04 <- 44 - x16 clock, 1 stop bit, no parity
 * :uart8274 B Reg 05 <- ea - Tx Enabled, Tx 8 bits, Send Break 0, RTS=0, DTR=0
 * :uart8274 B Reg 03 <- c1 - Rx Enabled, Rx 8 bits, No Auto Enables
 * :uart8274 B Reg 00 <- 28 - Reset Transmitter Interrupt Pending
 * :uart8274 B Reg 00 <- 28 - Reset Transmitter Interrupt Pending
 * :uart8274 B Reg 00 <- 28 - Reset Transmitter Interrupt Pending
 * :uart8274 B Reg 00 <- 28 - Reset Transmitter Interrupt Pending

 * :uart8274 A Reg 00 <- 18 - Channel reset command
 * :uart8274 A Reg 04 <- 4e - x16 clock, 2 stop bit, even parity but parity disabled
 * :uart8274 A Reg 05 <- ea - Tx Enabled, Tx 8 bits, Send Break 0, RTS=0, DTR=0
 * :uart8274 A Reg 03 <- c1 - Rx Enabled, Rx 8 bits, No Auto Enables
 * :uart8274 A Reg 07 <- 00 - Hi SYNC bits
 * :uart8274 A Reg 06 <- 00 - Lo SYNC bits
 * :uart8274 A Reg 02 <- 04 - RTSB selected, non vectored mode, 85-1 mode selected, A over B interleaved int prios
 * :uart8274 B Reg 02 <- 26 - interrupt vector 26
 * :uart8274 A Reg 01 <- 00 - Rx INT/DMA int disabled, no vector modification

 * :uart8274 A Reg 01 -> ?? - Read out Status Register 1 (Errors and All Sent flag)
 * :uart8274 A Reg 05 <- e2 - Tx Disabled, Tx 8 bits, Send Break 0, RTS=0, DTR=0
 * :uart8274 A Reg 03 <- c0 - Rx Disabled, Rx 8 bits, No Auto Enables
 * :uart8274 A Reg 05 <- ea - Tx Enabled, Tx 8 bits, Send Break 0, RTS=0, DTR=0
 * :uart8274 A Reg 04 <- 4e - x16 clock, 2 stop bit, even parity but parity disabled
 * :uart8274 A Reg 05 <- ea - Tx Enabled, Tx 8 bits, Send Break 0, RTS=0, DTR=0
 * :uart8274 A Reg 03 <- c1 - Rx Enabled, Rx 8 bits, No Auto Enables
 * :uart8274 A Reg 07 <- 00 - Hi SYNC bits
 * :uart8274 A Reg 06 <- 00 - Lo SYNC bits
 * :uart8274 A Reg 02 <- 04 - RTSB selected, non vectored mode, 85-1 mode selected, A over B interleaved int prios
 * :uart8274 B Reg 02 <- 26 - interrupt vector 26
 * :uart8274 A Reg 01 <- 00 - Rx INT/DMA int disabled, no vector modification
 * :uart8274 A Reg 02 <- 04 - RTSB selected, non vectored mode, 85-1 mode selected, A over B interleaved int prios

 * :uart8274 B Reg 02 <- a5 - interrupt vector a5
 * :uart8274 B Reg 02 <- 00 - interrupt vector 0

 * :uart8274 B Reg 04 <- 44 - x16 clock, 1 stop bit, no parity
 * :uart8274 B Reg 01 <- 1e - Wait disabled, Int mode 3, vector modified, Tx int/DMA enabled
 * :uart8274 A Reg 04 <- 44 - x16 clock, 1 stop bit, no parity
 * :uart8274 A Reg 01 <- 1e - Wait disabled, Int mode 3, vector modified, Tx int/DMA enabled

 * :uart8274 B Reg 04 <- 44 - x16 clock, 1 stop bit, no parity
 * :uart8274 B Reg 01 <- 1e - Wait disabled, Int mode 3, vector modified, Tx int/DMA enabled
 * :uart8274 B Reg 03 <- c1 - Rx Enabled, Rx 8 bits, No Auto Enables
 * :uart8274 B Reg 05 <- ea - Tx Enabled, Tx 8 bits, Send Break 0, RTS=0, DTR=0

 * :uart8274 B Reg 04 <- 44 - x16 clock, 1 stop bit, no parity
 * :uart8274 B Reg 01 <- 1e - Wait disabled, Int mode 3, vector modified, Tx int/DMA enabled
 * :uart8274 B Reg 03 <- c1 - Rx Enabled, Rx 8 bits, No Auto Enables
 * :uart8274 B Reg 05 <- ea - Tx Enabled, Tx 8 bits, Send Break 0, RTS=0, DTR=0
*/
ROM_START( isbc2861 )
	ROM_REGION( 0x10000, "user1", ROMREGION_ERASEFF )
	ROM_SYSTEM_BIOS( 0, "v11", "iSDM Monitor V1.1" )
	ROMX_LOAD( "174894-001.bin", 0x0000, 0x4000, CRC(79e4f7af) SHA1(911a4595d35e6e82b1149e75bb027927cd1c1658), ROM_SKIP(1) | ROM_BIOS(1))
	ROMX_LOAD( "174894-002.bin", 0x0001, 0x4000, CRC(66747d21) SHA1(4094b1f10a8bc7db8d6dd48d7128e14e875776c7), ROM_SKIP(1) | ROM_BIOS(1))
	ROMX_LOAD( "174894-003.bin", 0x8000, 0x4000, CRC(c98c7f17) SHA1(6e9a14aedd630824dccc5eb6052867e73b1d7db6), ROM_SKIP(1) | ROM_BIOS(1))
	ROMX_LOAD( "174894-004.bin", 0x8001, 0x4000, CRC(61bc1dc9) SHA1(feed5a5f0bb4630c8f6fa0d5cca30654a80b4ee5), ROM_SKIP(1) | ROM_BIOS(1))
	ROM_SYSTEM_BIOS( 1, "v10", "iSDM Monitor V1.0" )
	ROMX_LOAD( "rmx286-_in_socket_u41_on_isbc_286-10.bin.u41", 0x0000, 0x4000, CRC(00996834) SHA1(a17a0f8909be642d89199660b24574b71a9d0c13), ROM_SKIP(1) | ROM_BIOS(2))
	ROMX_LOAD( "rmx286-_in_socket_u76_on_isbc_286-10.bin.u76", 0x0001, 0x4000, CRC(90c9c7e8) SHA1(b5f961ab236976713266fe7a378e8750825fd5dc), ROM_SKIP(1) | ROM_BIOS(2))
	ROMX_LOAD( "rmx286-_in_socket_u40_on_isbc_286-10.bin.u40", 0x8000, 0x4000, CRC(35716c9b) SHA1(5b717b4c2f6c59ec140635df7448294a22123a16), ROM_SKIP(1) | ROM_BIOS(2))
	ROMX_LOAD( "rmx286-_in_socket_u75_on_isbc_286-10.bin.u75", 0x8001, 0x4000, CRC(68c3eb50) SHA1(3eeef2676e4fb187adb8ab50645f4bd172426c15), ROM_SKIP(1) | ROM_BIOS(2))
ROM_END

ROM_START( isbc28612 )
	ROM_REGION( 0x10000, "user1", ROMREGION_ERASEFF )
	ROM_LOAD16_BYTE( "176346-001.bin", 0x0000, 0x8000, CRC(f86c8be5) SHA1(e2bb16b0aeb718219e65d61edabd7838ef34c560))
	ROM_LOAD16_BYTE( "176346-002.bin", 0x0001, 0x8000, CRC(b964c6c3) SHA1(c3de8541182e32b3568fde77da8c435eab397498))
ROM_END

ROM_START( rpc86 )
	ROM_REGION( 0x8000, "user1", ROMREGION_ERASEFF )
	ROM_LOAD16_BYTE( "145068-001.bin", 0x4001, 0x1000, CRC(0fa9db83) SHA1(4a44f8683c263c9ef6850cbe05aaa73f4d4d4e06))
	ROM_LOAD16_BYTE( "145069-001.bin", 0x6001, 0x1000, CRC(1692a076) SHA1(0ce3a4a867cb92340871bb8f9c3e91ce2984c77c))
	ROM_LOAD16_BYTE( "145070-001.bin", 0x4000, 0x1000, CRC(8c8303ef) SHA1(60f94daa76ab9dea6e309ac580152eb212b847a0))
	ROM_LOAD16_BYTE( "145071-001.bin", 0x6000, 0x1000, CRC(a49681d8) SHA1(e81f8b092cfa2d1737854b1fa270a4ce07d61a9f))
ROM_END
/* Driver */

/*    YEAR  NAME       PARENT  COMPAT  MACHINE    INPUT  STATE        INIT  COMPANY   FULLNAME        FLAGS */
COMP( 19??, rpc86,     0,      0,      rpc86,     isbc,  isbc_state,  0,    "Intel",  "RPC 86",       MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW)
COMP( 1978, isbc86,    0,      0,      isbc86,    isbc,  isbc_state,  0,    "Intel",  "iSBC 86/12A",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW)
COMP( 1981, isbc8605,  0,      0,      isbc8605,  isbc,  isbc_state,  0,    "Intel",  "iSBC 86/05",   MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW)
COMP( 1981, isbc8630,  0,      0,      isbc8630,  isbc,  isbc_state,  0,    "Intel",  "iSBC 86/30",   MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW)
COMP( 19??, isbc286,   0,      0,      isbc286,   isbc,  isbc_state,  0,    "Intel",  "iSBC 286",     MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW)
COMP( 1983, isbc2861,  0,      0,      isbc2861,  isbc,  isbc_state,  0,    "Intel",  "iSBC 286/10",  MACHINE_NO_SOUND_HW)
COMP( 1983, isbc28612, 0,      0,      isbc2861,  isbc,  isbc_state,  0,    "Intel",  "iSBC 286/12",  MACHINE_NO_SOUND_HW)
