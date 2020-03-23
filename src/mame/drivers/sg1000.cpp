// license:BSD-3-Clause
// copyright-holders:Curt Coder
/*

Sega SG-1000

PCB Layout
----------

171-5078 (C) SEGA 1983
171-5046 REV. A (C) SEGA 1983

|---------------------------|                              |----------------------------|
|   SW1     CN2             |   |------|---------------|   |    SW2     CN4             |
|                           |---|         CN3          |---|                            |
|  CN1                                                                              CN5 |
|                                                                                       |
|   10.738635MHz            |------------------------------|                7805        |
|   |---|                   |------------------------------|                            |
|   |   |                                 CN6                                           |
|   | 9 |                                                                               |
|   | 9 |                                                                       LS32    |
|   | 1 |       |---------|                                                             |
|   | 8 |       | TMM2009 |                                                     LS139   |
|   | A |       |---------|             |------------------|                            |
|   |   |                               |       Z80        |                            |
|   |---|                               |------------------|                            |
|                                                                                       |
|                                                                                       |
|       MB8118  MB8118  MB8118  MB8118              SN76489A            SW3             |
|           MB8118  MB8118  MB8118  MB8118                          LS257   LS257       |
|---------------------------------------------------------------------------------------|

Notes:
    All IC's shown.

    Z80     - NEC D780C-1 / Zilog Z8400A (REV.A) Z80A CPU @ 3.579545
    TMS9918A- Texas Instruments TMS9918ANL Video Display Processor @ 10.738635MHz
    MB8118  - Fujitsu MB8118-12 16K x 1 Dynamic RAM
    TMM2009 - Toshiba TMM2009P-A / TMM2009P-B (REV.A)
    SN76489A- Texas Instruments SN76489AN Digital Complex Sound Generator @ 3.579545
    CN1     - player 1 joystick connector
    CN2     - RF video connector
    CN3     - keyboard connector
    CN4     - power connector (+9VDC)
    CN5     - player 2 joystick connector
    CN6     - cartridge connector
    SW1     - TV channel select switch
    SW2     - power switch
    SW3     - hold switch

*/

/*

    TODO:

    - SC-3000 return instruction referenced by R when reading ports 60-7f,e0-ff
    - connect PSG /READY signal to Z80 WAIT
    - accurate video timing
    - SH-400 racing controller
    - SF-7000 serial comms

*/


#include "emu.h"
#include "includes/sg1000.h"
#include "bus/rs232/rs232.h"
#include "softlist.h"
#include "speaker.h"


/***************************************************************************
    READ/WRITE HANDLERS
***************************************************************************/

// TODO: not sure if the OMV bios actually detects the presence of a cart,
// or if the cart data simply overwrites the internal bios...
// for the moment let assume the latter!
READ8_MEMBER( sg1000_state::omv_r )
{
	if (m_cart && m_cart->exists())
		return m_cart->read_cart(space, offset);
	else
		return m_rom->base()[offset];
}

WRITE8_MEMBER( sg1000_state::omv_w )
{
	if (m_cart && m_cart->exists())
		m_cart->write_cart(space, offset, data);
}

READ8_MEMBER( sg1000_state::peripheral_r )
{
	bool joy_ports_disabled = m_sgexpslot->is_readable(offset);

	if (joy_ports_disabled)
	{
		return m_sgexpslot->read(space, offset);
	}
	else
	{
		if (offset & 0x01)
			return m_pb7->read();
		else
			return m_pa7->read();
	}
}

WRITE8_MEMBER( sg1000_state::peripheral_w )
{
	bool joy_ports_disabled = m_sgexpslot->is_writeable(offset);

	if (joy_ports_disabled)
	{
		m_sgexpslot->write(space, offset, data);
	}
}

/*-------------------------------------------------
    ADDRESS_MAP( sg1000_map )
-------------------------------------------------*/

static ADDRESS_MAP_START( sg1000_map, AS_PROGRAM, 8, sg1000_state )
	AM_RANGE(0x0000, 0xbfff) AM_DEVREADWRITE(CARTSLOT_TAG, sega8_cart_slot_device, read_cart, write_cart)
	AM_RANGE(0xc000, 0xc3ff) AM_MIRROR(0x3c00) AM_RAM
ADDRESS_MAP_END

/*-------------------------------------------------
    ADDRESS_MAP( sg1000_io_map )
-------------------------------------------------*/

static ADDRESS_MAP_START( sg1000_io_map, AS_IO, 8, sg1000_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x40, 0x40) AM_MIRROR(0x3f) AM_DEVWRITE(SN76489AN_TAG, sn76489a_device, write)
	AM_RANGE(0x80, 0x80) AM_MIRROR(0x3e) AM_DEVREADWRITE(TMS9918A_TAG, tms9918a_device, vram_read, vram_write)
	AM_RANGE(0x81, 0x81) AM_MIRROR(0x3e) AM_DEVREADWRITE(TMS9918A_TAG, tms9918a_device, register_read, register_write)
	AM_RANGE(0xdc, 0xdf) AM_READWRITE(peripheral_r, peripheral_w)
ADDRESS_MAP_END

/*-------------------------------------------------
    ADDRESS_MAP( omv_map )
-------------------------------------------------*/

static ADDRESS_MAP_START( omv_map, AS_PROGRAM, 8, sg1000_state )
	AM_RANGE(0x0000, 0xbfff) AM_READWRITE(omv_r, omv_w)
	AM_RANGE(0xc000, 0xc7ff) AM_MIRROR(0x3800) AM_RAM
ADDRESS_MAP_END

/*-------------------------------------------------
    ADDRESS_MAP( omv_io_map )
-------------------------------------------------*/

static ADDRESS_MAP_START( omv_io_map, AS_IO, 8, sg1000_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x40, 0x40) AM_MIRROR(0x3f) AM_DEVWRITE(SN76489AN_TAG, sn76489a_device, write)
	AM_RANGE(0x80, 0x80) AM_MIRROR(0x3e) AM_DEVREADWRITE(TMS9918A_TAG, tms9918a_device, vram_read, vram_write)
	AM_RANGE(0x81, 0x81) AM_MIRROR(0x3e) AM_DEVREADWRITE(TMS9918A_TAG, tms9918a_device, register_read, register_write)
	AM_RANGE(0xc0, 0xc0) AM_MIRROR(0x38) AM_READ_PORT("C0")
	AM_RANGE(0xc1, 0xc1) AM_MIRROR(0x38) AM_READ_PORT("C1")
	AM_RANGE(0xc2, 0xc2) AM_MIRROR(0x38) AM_READ_PORT("C2")
	AM_RANGE(0xc3, 0xc3) AM_MIRROR(0x38) AM_READ_PORT("C3")
	AM_RANGE(0xc4, 0xc4) AM_MIRROR(0x3a) AM_READ_PORT("C4")
	AM_RANGE(0xc5, 0xc5) AM_MIRROR(0x3a) AM_READ_PORT("C5")
ADDRESS_MAP_END

/*-------------------------------------------------
    ADDRESS_MAP( sc3000_map )
-------------------------------------------------*/

static ADDRESS_MAP_START( sc3000_map, AS_PROGRAM, 8, sg1000_state )
	AM_RANGE(0x0000, 0xbfff) AM_DEVREADWRITE(CARTSLOT_TAG, sega8_cart_slot_device, read_cart, write_cart)
	AM_RANGE(0xc000, 0xc7ff) AM_MIRROR(0x3800) AM_RAM
ADDRESS_MAP_END

/*-------------------------------------------------
    ADDRESS_MAP( sc3000_io_map )
-------------------------------------------------*/

static ADDRESS_MAP_START( sc3000_io_map, AS_IO, 8, sg1000_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x7f, 0x7f) AM_DEVWRITE(SN76489AN_TAG, sn76489a_device, write)
	AM_RANGE(0xbe, 0xbe) AM_DEVREADWRITE(TMS9918A_TAG, tms9918a_device, vram_read, vram_write)
	AM_RANGE(0xbf, 0xbf) AM_DEVREADWRITE(TMS9918A_TAG, tms9918a_device, register_read, register_write)
	AM_RANGE(0xdc, 0xdf) AM_READWRITE(peripheral_r, peripheral_w)
ADDRESS_MAP_END

/* This is how the I/O ports are really mapped, but MAME does not support overlapping ranges
static ADDRESS_MAP_START( sc3000_io_map, AS_IO, 8, sg1000_state )
    ADDRESS_MAP_GLOBAL_MASK(0xff)
    AM_RANGE(0x00, 0x00) AM_MIRROR(0xdf) AM_DEVREADWRITE(UPD9255_TAG, i8255_device, read, write)
    AM_RANGE(0x00, 0x00) AM_MIRROR(0x7f) AM_DEVWRITE(SN76489AN_TAG, sn76489a_device, write)
    AM_RANGE(0x00, 0x00) AM_MIRROR(0xae) AM_DEVREADWRITE(TMS9918A_TAG, tms9918a_device, vram_read, vram_write)
    AM_RANGE(0x01, 0x01) AM_MIRROR(0xae) AM_DEVREADWRITE(TMS9918A_TAG, tms9918a_device, register_read, register_write)
    AM_RANGE(0x60, 0x60) AM_MIRROR(0x9f) AM_READ(sc3000_r_r)
ADDRESS_MAP_END
*/

/*-------------------------------------------------
    ADDRESS_MAP( sf7000_map )
-------------------------------------------------*/

static ADDRESS_MAP_START( sf7000_map, AS_PROGRAM, 8, sf7000_state )
	AM_RANGE(0x0000, 0x3fff) AM_READ_BANK("bank1") AM_WRITE_BANK("bank2")
	AM_RANGE(0x4000, 0xffff) AM_RAM
ADDRESS_MAP_END

/*-------------------------------------------------
    ADDRESS_MAP( sf7000_io_map )
-------------------------------------------------*/

static ADDRESS_MAP_START( sf7000_io_map, AS_IO, 8, sf7000_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x7f, 0x7f) AM_DEVWRITE(SN76489AN_TAG, sn76489a_device, write)
	AM_RANGE(0xbe, 0xbe) AM_DEVREADWRITE(TMS9918A_TAG, tms9918a_device, vram_read, vram_write)
	AM_RANGE(0xbf, 0xbf) AM_DEVREADWRITE(TMS9918A_TAG, tms9918a_device, register_read, register_write)
	AM_RANGE(0xdc, 0xdf) AM_READWRITE(peripheral_r, peripheral_w)
	AM_RANGE(0xe0, 0xe1) AM_DEVICE(UPD765_TAG, upd765a_device, map)
	AM_RANGE(0xe4, 0xe7) AM_DEVREADWRITE(UPD9255_1_TAG, i8255_device, read, write)
	AM_RANGE(0xe8, 0xe8) AM_DEVREADWRITE(UPD8251_TAG, i8251_device, data_r, data_w)
	AM_RANGE(0xe9, 0xe9) AM_DEVREADWRITE(UPD8251_TAG, i8251_device, status_r, control_w)
ADDRESS_MAP_END

/***************************************************************************
    INPUT PORTS
***************************************************************************/

/*-------------------------------------------------
    INPUT_CHANGED_MEMBER( trigger_nmi )
-------------------------------------------------*/

INPUT_CHANGED_MEMBER( sg1000_state::trigger_nmi )
{
	m_maincpu->set_input_line(INPUT_LINE_NMI, newval ? CLEAR_LINE : ASSERT_LINE);
}

/*-------------------------------------------------
    INPUT_PORTS( sg1000 )
-------------------------------------------------*/

static INPUT_PORTS_START( sg1000_joy )
	PORT_START("PA7")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_PLAYER(2)
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_PLAYER(2)

	PORT_START("PB7")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_PLAYER(2)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(2)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0xf0, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END

static INPUT_PORTS_START( sg1000 )
	PORT_INCLUDE( sg1000_joy )

	PORT_START("NMI")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_START ) PORT_NAME("PAUSE") PORT_CODE(KEYCODE_P) PORT_CHANGED_MEMBER(DEVICE_SELF, sg1000_state, trigger_nmi, 0)
INPUT_PORTS_END

/*-------------------------------------------------
    INPUT_PORTS( omv )
-------------------------------------------------*/

static INPUT_PORTS_START( omv1000 )
	PORT_START("C0")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_1) PORT_CHAR('1')
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_2) PORT_CHAR('2')
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_3) PORT_CHAR('3')
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_4) PORT_CHAR('4')
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_5) PORT_CHAR('5')
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_6) PORT_CHAR('6')
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("C1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_7) PORT_CHAR('7')
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_8) PORT_CHAR('8')
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("9 #") PORT_CODE(KEYCODE_9) PORT_CHAR('9')
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_NAME("0 *") PORT_CODE(KEYCODE_0) PORT_CHAR('0')
	PORT_BIT( 0xf0, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("C2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_A) PORT_CHAR('A')
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_B) PORT_CHAR('B')
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_C) PORT_CHAR('C')
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_D) PORT_CHAR('D')
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_E) PORT_CHAR('E')
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_F) PORT_CHAR('F')
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("C3")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_G) PORT_CHAR('G')
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYPAD ) PORT_CODE(KEYCODE_H) PORT_CHAR('H')
	PORT_BIT( 0xfc, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("C4")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("S-1")
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_NAME("S-2")
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_PLAYER(2)
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_PLAYER(2)

	// The OMV FG-1000 has no 2nd joystick
	// http://www.famitsu.com/image/29819/pEllnbNQfCJ58skZ25uB511N6eSFfAu6.jpg
	PORT_START("C5")
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END

static INPUT_PORTS_START( omv2000 )
	PORT_INCLUDE( omv1000 )

	PORT_MODIFY("C5")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_PLAYER(2)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(2)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0xf0, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END

/*-------------------------------------------------
    INPUT_PORTS( sc3000 )
-------------------------------------------------*/

static INPUT_PORTS_START( sc3000 )
	PORT_INCLUDE( sg1000_joy )

	// keyboard keys are added by the embedded sk1100 device

	PORT_START("NMI")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD ) PORT_NAME("RESET") PORT_CODE(KEYCODE_F10) PORT_CHANGED_MEMBER(DEVICE_SELF, sg1000_state, trigger_nmi, 0)
INPUT_PORTS_END

/*-------------------------------------------------
    INPUT_PORTS( sf7000 )
-------------------------------------------------*/

static INPUT_PORTS_START( sf7000 )
	PORT_INCLUDE( sc3000 )

	PORT_START("BAUD")
	PORT_CONFNAME( 0x07, 0x05, "Baud rate")
	PORT_CONFSETTING( 0x00, "9600 baud" )
	PORT_CONFSETTING( 0x01, "4800 baud" )
	PORT_CONFSETTING( 0x02, "2400 baud" )
	PORT_CONFSETTING( 0x03, "1200 baud" )
	PORT_CONFSETTING( 0x04, "600 baud" )
	PORT_CONFSETTING( 0x05, "300 baud" )
INPUT_PORTS_END

/***************************************************************************
    DEVICE CONFIGURATION
***************************************************************************/

/*-------------------------------------------------
    I8255 INTERFACE
-------------------------------------------------*/

WRITE_LINE_MEMBER( sf7000_state::write_centronics_busy )
{
	m_centronics_busy = state;
}

READ8_MEMBER( sf7000_state::ppi_pa_r )
{
	/*
	    Signal  Description

	    PA0     INT from FDC
	    PA1     BUSY from Centronics printer
	    PA2     INDEX from FDD
	    PA3
	    PA4
	    PA5
	    PA6
	    PA7
	*/

	uint8_t data = 0;

	data |= m_fdc->get_irq() ? 0x01 : 0x00;
	data |= m_centronics_busy << 1;
	data |= m_floppy0->idx_r() << 2;

	return data;
}

WRITE8_MEMBER( sf7000_state::ppi_pc_w )
{
	/*
	    Signal  Description

	    PC0     /INUSE signal to FDD
	    PC1     /MOTOR ON signal to FDD
	    PC2     TC signal to FDC
	    PC3     RESET signal to FDC
	    PC4     not connected
	    PC5     not connected
	    PC6     /ROM SEL (switch between IPL ROM and RAM)
	    PC7     /STROBE to Centronics printer
	*/

	if (!BIT(data, 0))
		m_fdc->set_floppy(m_floppy0);
	else
		m_fdc->set_floppy(nullptr);

	/* floppy motor */
	m_floppy0->mon_w(BIT(data, 1));

	/* FDC terminal count */
	m_fdc->tc_w(BIT(data, 2));

	/* FDC reset */
	if (BIT(data, 3))
	{
		m_fdc->reset();
	}

	/* ROM selection */
	membank("bank1")->set_entry(BIT(data, 6));

	/* printer strobe */
	m_centronics->write_strobe(BIT(data, 7));
}

/*-------------------------------------------------
    upd765_interface sf7000_upd765_interface
-------------------------------------------------*/

FLOPPY_FORMATS_MEMBER( sf7000_state::floppy_formats )
	FLOPPY_SF7000_FORMAT
FLOPPY_FORMATS_END

/*-------------------------------------------------
    floppy_interface sf7000_floppy_interface
-------------------------------------------------*/

static SLOT_INTERFACE_START( sf7000_floppies )
	SLOT_INTERFACE( "3ssdd", FLOPPY_3_SSDD )
SLOT_INTERFACE_END

/*-------------------------------------------------
    MACHINE_START( sg1000 )
-------------------------------------------------*/

void sg1000_state::machine_start()
{
	if (m_cart->get_type() == SEGA8_DAHJEE_TYPEA || m_cart->get_type() == SEGA8_DAHJEE_TYPEB)
	{
		m_maincpu->space(AS_PROGRAM).install_read_handler(0xc000, 0xffff, read8_delegate(FUNC(sega8_cart_slot_device::read_ram),(sega8_cart_slot_device*)m_cart));
		m_maincpu->space(AS_PROGRAM).install_write_handler(0xc000, 0xffff, write8_delegate(FUNC(sega8_cart_slot_device::write_ram),(sega8_cart_slot_device*)m_cart));
	}

	if (m_cart)
		m_cart->save_ram();
}

/*-------------------------------------------------
    MACHINE_START( sc3000 )
-------------------------------------------------*/

void sc3000_state::machine_start()
{
	/* toggle light gun crosshair */
	timer_set(attotime::zero, TIMER_LIGHTGUN_TICK);

	if (m_cart && m_cart->exists() && (m_cart->get_type() == SEGA8_BASIC_L3 || m_cart->get_type() == SEGA8_MUSIC_EDITOR
								|| m_cart->get_type() == SEGA8_DAHJEE_TYPEA || m_cart->get_type() == SEGA8_DAHJEE_TYPEB))
	{
		m_maincpu->space(AS_PROGRAM).install_read_handler(0xc000, 0xffff, read8_delegate(FUNC(sega8_cart_slot_device::read_ram),(sega8_cart_slot_device*)m_cart));
		m_maincpu->space(AS_PROGRAM).install_write_handler(0xc000, 0xffff, write8_delegate(FUNC(sega8_cart_slot_device::write_ram),(sega8_cart_slot_device*)m_cart));
	}

	if (m_cart)
		m_cart->save_ram();
}


/*-------------------------------------------------
    MACHINE_START( sf7000 )
-------------------------------------------------*/

void sf7000_state::machine_start()
{
	sc3000_state::machine_start();

	save_item(NAME(m_centronics_busy));

	/* configure memory banking */
	membank("bank1")->configure_entry(0, m_rom->base());
	membank("bank1")->configure_entry(1, m_ram->pointer());
	membank("bank2")->configure_entry(0, m_ram->pointer());
}

void sf7000_state::machine_reset()
{
	membank("bank1")->set_entry(0);
	membank("bank2")->set_entry(0);
}

/***************************************************************************
    MACHINE DRIVERS
***************************************************************************/

/*-------------------------------------------------
    MACHINE_CONFIG_START( sg1000 )
-------------------------------------------------*/

static MACHINE_CONFIG_START( sg1000 )
	/* basic machine hardware */
	MCFG_CPU_ADD(Z80_TAG, Z80, XTAL_10_738635MHz/3)
	MCFG_CPU_PROGRAM_MAP(sg1000_map)
	MCFG_CPU_IO_MAP(sg1000_io_map)

	/* video hardware */
	MCFG_DEVICE_ADD( TMS9918A_TAG, TMS9918A, XTAL_10_738635MHz / 2 )
	MCFG_TMS9928A_VRAM_SIZE(0x4000)
	MCFG_TMS9928A_OUT_INT_LINE_CB(INPUTLINE(Z80_TAG, INPUT_LINE_IRQ0))
	MCFG_TMS9928A_SCREEN_ADD_NTSC( SCREEN_TAG )
	MCFG_SCREEN_UPDATE_DEVICE( TMS9918A_TAG, tms9918a_device, screen_update )

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD(SN76489AN_TAG, SN76489A, XTAL_10_738635MHz/3)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.00)

	/* expansion slot */
	MCFG_SG1000_EXPANSION_ADD(EXPSLOT_TAG, sg1000_expansion_devices, nullptr, false)

	/* cartridge */
	MCFG_SG1000_CARTRIDGE_ADD(CARTSLOT_TAG, sg1000_cart, nullptr)

	/* software lists */
	MCFG_SOFTWARE_LIST_ADD("cart_list","sg1000")

	/* internal ram */
	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("1K")
MACHINE_CONFIG_END

/*-------------------------------------------------
    MACHINE_CONFIG_DERIVED( omv, sg1000 )
-------------------------------------------------*/

static MACHINE_CONFIG_DERIVED( omv, sg1000 )
	MCFG_CPU_MODIFY(Z80_TAG)
	MCFG_CPU_PROGRAM_MAP(omv_map)
	MCFG_CPU_IO_MAP(omv_io_map)

	MCFG_DEVICE_REMOVE(CARTSLOT_TAG)
	MCFG_OMV_CARTRIDGE_ADD(CARTSLOT_TAG, sg1000_cart, nullptr)

	MCFG_RAM_MODIFY(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("2K")
MACHINE_CONFIG_END

/*-------------------------------------------------
    MACHINE_CONFIG_START( sc3000 )
-------------------------------------------------*/

static MACHINE_CONFIG_START( sc3000 )
	/* basic machine hardware */
	MCFG_CPU_ADD(Z80_TAG, Z80, XTAL_10_738635MHz/3) // LH0080A
	MCFG_CPU_PROGRAM_MAP(sc3000_map)
	MCFG_CPU_IO_MAP(sc3000_io_map)

	/* video hardware */
	MCFG_DEVICE_ADD( TMS9918A_TAG, TMS9918A, XTAL_10_738635MHz / 2 )
	MCFG_TMS9928A_VRAM_SIZE(0x4000)
	MCFG_TMS9928A_OUT_INT_LINE_CB(INPUTLINE(Z80_TAG, INPUT_LINE_IRQ0))
	MCFG_TMS9928A_SCREEN_ADD_NTSC( SCREEN_TAG )
	MCFG_SCREEN_UPDATE_DEVICE( TMS9918A_TAG, tms9918a_device, screen_update )

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD(SN76489AN_TAG, SN76489A, XTAL_10_738635MHz/3)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.00)

	/* sc3000 has all sk1100 features built-in, so add it as a fixed slot */
	MCFG_SG1000_EXPANSION_ADD(EXPSLOT_TAG, sg1000_expansion_devices, "sk1100", true)

	/* cartridge */
	MCFG_SC3000_CARTRIDGE_ADD(CARTSLOT_TAG, sg1000_cart, nullptr)

	/* software lists */
	MCFG_SOFTWARE_LIST_ADD("cart_list","sg1000")
	/* the sk1100 device will add sc3000 cart and cass lists */

	/* internal ram */
	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("2K")
MACHINE_CONFIG_END

/*-------------------------------------------------
    MACHINE_CONFIG_START( sf7000 )
-------------------------------------------------*/

static MACHINE_CONFIG_START( sf7000 )
	/* basic machine hardware */
	MCFG_CPU_ADD(Z80_TAG, Z80, XTAL_10_738635MHz/3)
	MCFG_CPU_PROGRAM_MAP(sf7000_map)
	MCFG_CPU_IO_MAP(sf7000_io_map)

	/* video hardware */
	MCFG_DEVICE_ADD( TMS9918A_TAG, TMS9918A, XTAL_10_738635MHz / 2 )
	MCFG_TMS9928A_VRAM_SIZE(0x4000)
	MCFG_TMS9928A_OUT_INT_LINE_CB(INPUTLINE(Z80_TAG, INPUT_LINE_IRQ0))
	MCFG_TMS9928A_SCREEN_ADD_NTSC( SCREEN_TAG )
	MCFG_SCREEN_UPDATE_DEVICE( TMS9918A_TAG, tms9918a_device, screen_update )

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD(SN76489AN_TAG, SN76489A, XTAL_10_738635MHz/3)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.00)

	/* devices */
	MCFG_DEVICE_ADD(UPD9255_1_TAG, I8255, 0)
	MCFG_I8255_IN_PORTA_CB(READ8(sf7000_state, ppi_pa_r))
	MCFG_I8255_OUT_PORTB_CB(DEVWRITE8("cent_data_out", output_latch_device, write))
	MCFG_I8255_OUT_PORTC_CB(WRITE8(sf7000_state, ppi_pc_w))

	MCFG_DEVICE_ADD(UPD8251_TAG, I8251, 0)
	MCFG_I8251_TXD_HANDLER(DEVWRITELINE(RS232_TAG, rs232_port_device, write_txd))
	MCFG_I8251_DTR_HANDLER(DEVWRITELINE(RS232_TAG, rs232_port_device, write_dtr))
	MCFG_I8251_RTS_HANDLER(DEVWRITELINE(RS232_TAG, rs232_port_device, write_rts))

	MCFG_RS232_PORT_ADD(RS232_TAG, default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(DEVWRITELINE(UPD8251_TAG, i8251_device, write_rxd))
	MCFG_RS232_DSR_HANDLER(DEVWRITELINE(UPD8251_TAG, i8251_device, write_dsr))

	MCFG_UPD765A_ADD(UPD765_TAG, false, false)
	MCFG_FLOPPY_DRIVE_ADD(UPD765_TAG ":0", sf7000_floppies, "3ssdd", sf7000_state::floppy_formats)

	MCFG_CENTRONICS_ADD(CENTRONICS_TAG, centronics_devices, "printer")

	MCFG_CENTRONICS_OUTPUT_LATCH_ADD("cent_data_out", "centronics")

	/* sf7000 (sc3000) has all sk1100 features built-in, so add it as a fixed slot */
	MCFG_SG1000_EXPANSION_ADD(EXPSLOT_TAG, sg1000_expansion_devices, "sk1100", true)

	/* software lists */
	MCFG_SOFTWARE_LIST_ADD("flop_list","sf7000")

	/* internal ram */
	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("64K")
MACHINE_CONFIG_END

/***************************************************************************
    ROMS
***************************************************************************/

ROM_START( sg1000 )
	ROM_REGION( 0x10000, Z80_TAG, ROMREGION_ERASE00 )
ROM_END

#define rom_sg1000m2 rom_sg1000

#define rom_sc3000 rom_sg1000

#define rom_sc3000h rom_sg1000

ROM_START( sf7000 )
	ROM_REGION( 0x10000, Z80_TAG, ROMREGION_ERASE00 )
	ROM_LOAD( "ipl.rom", 0x0000, 0x2000, CRC(d76810b8) SHA1(77339a6db2593aadc638bed77b8e9bed5d9d87e3) )
ROM_END

ROM_START( omv1000 )
	ROM_REGION( 0x10000, Z80_TAG, ROMREGION_ERASE00 )
	ROM_LOAD( "omvbios.bin", 0x0000, 0x4000, BAD_DUMP CRC(c5a67b95) SHA1(6d7c64dd60dee4a33061d3d3a7c2ed190d895cdb) )    // The BIOS comes from a Multivision FG-2000. It is still unknown if the FG-1000 BIOS differs
ROM_END

ROM_START( omv2000 )
	ROM_REGION( 0x10000, Z80_TAG, ROMREGION_ERASE00 )
	ROM_LOAD( "omvbios.bin", 0x0000, 0x4000, CRC(c5a67b95) SHA1(6d7c64dd60dee4a33061d3d3a7c2ed190d895cdb) )
ROM_END

/***************************************************************************
    SYSTEM DRIVERS
***************************************************************************/

/*    YEAR  NAME        PARENT      COMPAT      MACHINE     INPUT    STATE            INIT    COMPANY             FULLNAME                                    FLAGS */
CONS( 1983, sg1000,     0,          0,          sg1000,     sg1000,  sg1000_state,    0,      "Sega",             "SG-1000",                                  MACHINE_SUPPORTS_SAVE )
CONS( 1984, sg1000m2,   sg1000,     0,          sc3000,     sc3000,  sc3000_state,    0,      "Sega",             "SG-1000 II",                               MACHINE_SUPPORTS_SAVE )
COMP( 1983, sc3000,     0,          sg1000,     sc3000,     sc3000,  sc3000_state,    0,      "Sega",             "SC-3000",                                  MACHINE_SUPPORTS_SAVE )
COMP( 1983, sc3000h,    sc3000,     0,          sc3000,     sc3000,  sc3000_state,    0,      "Sega",             "SC-3000H",                                 MACHINE_SUPPORTS_SAVE )
COMP( 1983, sf7000,     sc3000,     0,          sf7000,     sf7000,  sf7000_state,    0,      "Sega",             "SC-3000/Super Control Station SF-7000",    MACHINE_SUPPORTS_SAVE )
CONS( 1984, omv1000,    sg1000,     0,          omv,        omv1000, sg1000_state,    0,      "Tsukuda Original", "Othello Multivision FG-1000",              MACHINE_SUPPORTS_SAVE )
CONS( 1984, omv2000,    sg1000,     0,          omv,        omv2000, sg1000_state,    0,      "Tsukuda Original", "Othello Multivision FG-2000",              MACHINE_SUPPORTS_SAVE )
