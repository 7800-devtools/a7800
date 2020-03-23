// license:LGPL-2.1+
// copyright-holders:Michael Zapf
/****************************************************************************

    SNUG Second Generation CPU (SGCPU, aka TI-99/4P)

    This system is known both as the TI-99/4P ("Peripheral box", since the
    system is a card to be inserted in the peripheral box, instead of a
    self-contained console), and as the SGCPU ("Second Generation CPU",
    which was originally the name used in TI documentation to refer to either
    (or both) TI-99/5 and TI-99/8 projects).

    The SGCPU was designed and built by the SNUG (System 99 Users Group),
    namely by Michael Becker for the hardware part and Harald Glaab for the
    software part.  It has no relationship with TI.

    The card is a complete redesign of the original TI-99/4A mainboard to fit
    on a peripheral card, thus replacing the console. It shows no original
    circuits on its board; the concept is to cannibalize a TI-99/4A console,
    moving its main circuits (TMS9900, TMS9901) into the sockets on this board.

    The sound chip is not plugged on the SGCPU but on the EVPC card which
    provides the video processor for SGCPU card (see below).

    The card offers a PC-style keyboard interface which adapts the keyboard
    to the matrix organisation expected by the operating system of the TI.

    All decoding and further features are implemented by a MACH chip, which
    appears on many SNUG cards.

    On the card, most circuits are directly accessed by a 16-bit data bus,
    which ensures a significant speed-up compared to the original console. Only
    when accessing external devices via the PEB, a databus multiplexer comes into
    play which is implemented in the same way as the one in the original console,
    also contained in the MACH.

    The SGCPU offers a special connector at the back, containing the remaining
    8 data bus lines; by this feature, expansion cards can be connected at
    full 16 bit width. Only the HRD16 card (not yet emulated), which is a
    RAMDisk card, actually uses it.

    EPROM layout 64K
    ----------------
    The memory region is shifted by 4000 in the EPROM address space
    According to the designers, this is caused by the next-to most significant
    address line (2^14) being locked to 1. This is done to allow for smaller
    24pin EPROM to be used.

       Area        EPROM offset      Mapped at
       ---------------------------------------
       ROM0        4000   (0100)     0000
       DSR         C000   (1100)     4000
       ROM6A       6000   (0110)     6000
       ROM6B       E000   (1110)     6000

    System ROM
    ----------
    The GPL interpreter is located in the EPROM as ROM0 (see above). The
    SGCPU does not contain any GROM, which contain the actual TI operating
    system and the BASIC interpreter. The GROMs are replaced by the HSGPL card.

    ==== CAUTION ====: This means that the HSGPL must be properly set up before
    starting up the SGCPU. Otherwise, the emulation locks up immediately with a
    BLACK SCREEN.

    In the real environment, the HSGPL has usually been set up on delivery.
    In MESS we have to create a suitable HSGPL memory content. Best practice
    is to start the TI-99/4A console with EVPC support (driver ti99_4ev) with
    a plugged-in HSGPL and to go through the setup process there.
    Finally, the nvram files of the HSGPL must be copied into this driver's nvram
    subdirectory. The contents will be directly usable for the SGCPU.

    RAM: AEMS emulation
    --------------------
    The Asgard Expanded Memory System is a peripheral card whose successor
    (Super AMS) is available in MESS. The AEMS card is emulated inside the MACH
    chip of the SGCPU. For more information see samsmem.cpp.

    The first four address lines are used to select one of 16 mapper values with
    8 bits each. Instead of these first 4 lines, the 8 bits are prepended to
    the remaining address, yielding a 20 bit address space.

    The mapper values are mapped into the address space at 4000 by setting
    CRU bit 1E00. Only the even addresses are used, so the first mapper byte is
    at 4000, the second at 4002, the last one at 401E.

    The mapping mode can be turned on and off by the CRU bit at address 1E02.
    When turned off, the address is passed through to the RAM circuits.

    Since the only RAM areas on the TI systems are at 2000-3FFF and A000-FFFF,
    the typical usage is to use the AEMS as a 32K expansion in unmapped mode
    (the remaining 32K of the address space is decoded earlier, and does not
    affect the card), and to use it as paged memory in the 2000-3FFF and A000-FFFF
    areas by setting the mapper appropriately. Mapper registers referring to
    other memory areas have no effect.

    Video and sound
    ---------------
    The SGCPU relies on the EVPC or EVPC2 card to provide video capabilities.
    This card (rel.1) is emulated in MESS and is based on the v9938 video
    display processor.
    In order to route the VDP interrupt to the SGCPU card, the previously
    unused LCP* line in the Peripheral Expansion Box is used.

    The sound chip requires the video clock, and therefore it is moved from the
    console to the EVPC card.

    Joystick and cassette
    ---------------------
    The card features a 25-pin connector at the back which contains the lines
    for the joysticks and one cassette input/output. An adapter must be built
    to be able to use the common cables.

    Michael Zapf

*****************************************************************************/

#include "emu.h"
#include "bus/ti99/ti99defs.h"
#include "bus/ti99/joyport/joyport.h"
#include "bus/ti99/peb/peribox.h"
#include "cpu/tms9900/tms9900.h"
#include "imagedev/cassette.h"
#include "machine/ram.h"
#include "machine/tms9901.h"
#include "sound/wave.h"
#include "speaker.h"

#define TI99_SGCPU_TAG "sgcpu"
#define TI99_AMSRAM_TAG "amsram1meg"

#define TRACE_ILLWRITE 0
#define TRACE_READY 0
#define TRACE_INT 0
#define TRACE_ADDRESS 0
#define TRACE_MEM 0
#define TRACE_MUX 0

class ti99_4p_state : public driver_device
{
public:
	ti99_4p_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_cpu(*this, "maincpu"),
		m_tms9901(*this, TI_TMS9901_TAG),
		m_cassette(*this, "cassette"),
		m_peribox(*this, TI_PERIBOX_TAG),
		m_joyport(*this, TI_JOYPORT_TAG),
		m_scratchpad(*this, TI99_PADRAM_TAG),
		m_amsram(*this, TI99_AMSRAM_TAG)
	{ }

	DECLARE_WRITE_LINE_MEMBER( ready_line );
	DECLARE_WRITE_LINE_MEMBER( extint );
	DECLARE_WRITE_LINE_MEMBER( notconnected );
	DECLARE_READ8_MEMBER( interrupt_level );

	DECLARE_SETOFFSET_MEMBER( setoffset );
	DECLARE_READ16_MEMBER( memread );
	DECLARE_WRITE16_MEMBER( memwrite );
	DECLARE_WRITE_LINE_MEMBER( dbin_in );

	DECLARE_READ16_MEMBER( samsmem_read );
	DECLARE_WRITE16_MEMBER( samsmem_write );

	DECLARE_WRITE8_MEMBER(external_operation);
	DECLARE_WRITE_LINE_MEMBER( clock_out );
	DECLARE_WRITE_LINE_MEMBER( dbin_line );

	// CRU (Communication Register Unit) handling
	DECLARE_READ8_MEMBER( cruread );
	DECLARE_WRITE8_MEMBER( cruwrite );
	DECLARE_READ8_MEMBER( read_by_9901 );
	DECLARE_WRITE_LINE_MEMBER(keyC0);
	DECLARE_WRITE_LINE_MEMBER(keyC1);
	DECLARE_WRITE_LINE_MEMBER(keyC2);
	DECLARE_WRITE_LINE_MEMBER(cs_motor);
	DECLARE_WRITE_LINE_MEMBER(audio_gate);
	DECLARE_WRITE_LINE_MEMBER(cassette_output);
	DECLARE_WRITE8_MEMBER(tms9901_interrupt);
	DECLARE_WRITE_LINE_MEMBER(alphaW);
	virtual void machine_start() override;
	DECLARE_MACHINE_RESET(ti99_4p);

	DECLARE_WRITE_LINE_MEMBER(video_interrupt_in);

private:
	void    datamux_clock_in(int clock);

	// Devices
	required_device<tms9900_device>        m_cpu;
	required_device<tms9901_device>        m_tms9901;
	required_device<cassette_image_device> m_cassette;
	required_device<bus::ti99::peb::peribox_device>        m_peribox;
	required_device<bus::ti99::joyport::joyport_device>   m_joyport;
	required_device<ram_device> m_scratchpad;
	required_device<ram_device> m_amsram;

	int decode_address(int address);
	DECLARE_READ16_MEMBER( debugger_read );
	DECLARE_WRITE16_MEMBER( debugger_write );
	void ready_join();
	void    set_keyboard_column(int number, int data);

	// Pointer to EPROM
	uint16_t *m_rom;

	// First joystick. 6 for TI-99/4A
	int     m_firstjoy;

	int     m_keyboard_column;
	int     m_check_alphalock;

	// true if SGCPU DSR is enabled
	bool m_internal_dsr;

	// true if SGCPU rom6 is enabled
	bool m_internal_rom6;

	// Offset to the ROM6 bank.
	int m_rom6_bank;

	// Wait states
	int m_waitcount;

	// true when mapper is active
	bool m_map_mode;

	// true when mapper registers are accessible
	bool m_access_mapper;

	// Value on address bus (after being set by setaddress)
	int m_addr_buf;

	// Address decoding result
	int m_decode;

	// Ready state of the databus multiplexer
	bool m_muxready;

	// Incoming Ready level
	int m_sysready;

	// Saves a pointer to the address space
	address_space* m_spacep;

	// Internal DSR mapped in
	bool m_internal_dsr_active;

	// Mapper visible in 4000 area
	bool m_mapper_active;

	// ROM6 visible in 6000
	bool m_rom6_active;

	// Upper bank of ROM6 selected
	bool m_rom6_upper;

	// State of the DBIN line
	int m_dbin;

	uint8_t   m_lowbyte;
	uint8_t   m_highbyte;
	uint8_t   m_latch;

	// Mapper registers
	uint8_t m_mapper[16];

	// Latch for 9901 INT2, INT1 lines
	int     m_9901_int;
	void    set_9901_int(int line, line_state state);

	int     m_ready_prev;       // for debugging purposes only
};

enum
{
	ROM0BASE = 0x4000,
	DSRBASE = 0xc000,
	ROM6LBASE = 0x6000,
	ROM6UBASE = 0xe000
};

static ADDRESS_MAP_START(memmap, AS_PROGRAM, 16, ti99_4p_state)
	AM_RANGE(0x0000, 0xffff) AM_READWRITE( memread, memwrite ) AM_SETOFFSET( setoffset )
ADDRESS_MAP_END

static ADDRESS_MAP_START(cru_map, AS_IO, 8, ti99_4p_state)
	AM_RANGE(0x0000, 0x003f) AM_DEVREAD(TI_TMS9901_TAG, tms9901_device, read)
	AM_RANGE(0x0000, 0x01ff) AM_READ( cruread )

	AM_RANGE(0x0000, 0x01ff) AM_DEVWRITE(TI_TMS9901_TAG, tms9901_device, write)
	AM_RANGE(0x0000, 0x0fff) AM_WRITE( cruwrite )
ADDRESS_MAP_END

/*
    Input ports, used by machine code for TI keyboard and joystick emulation.

    Since the keyboard microcontroller is not emulated, we use the TI99/4a 48-key keyboard,
    plus two optional joysticks.
*/

static INPUT_PORTS_START(ti99_4p)
	/* 4 ports for keyboard and joystick */
	PORT_START("COL0")  // col 0
		PORT_BIT(0x88, IP_ACTIVE_LOW, IPT_UNUSED)
		/* The original control key is located on the left, but we accept the right control key as well */
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CTRL")      PORT_CODE(KEYCODE_LCONTROL) PORT_CODE(KEYCODE_RCONTROL)
		/* TI99/4a has a second shift key which maps the same */
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT) PORT_CHAR(UCHAR_SHIFT_1)
		/* The original function key is located on the right, but we accept the left alt key as well */
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("FCTN")      PORT_CODE(KEYCODE_RALT) PORT_CODE(KEYCODE_LALT) PORT_CHAR(UCHAR_SHIFT_2)
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER) PORT_CHAR(13)
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_SPACE) PORT_CHAR(' ')
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("= + QUIT")  PORT_CODE(KEYCODE_EQUALS) PORT_CHAR('=') PORT_CHAR('+') PORT_CHAR(UCHAR_MAMEKEY(F12))

	PORT_START("COL1")  // col 1
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_X)     PORT_CHAR('x') PORT_CHAR('X') PORT_CHAR(UCHAR_MAMEKEY(DOWN))
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_W)     PORT_CHAR('w') PORT_CHAR('W') PORT_CHAR('~')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_S)     PORT_CHAR('s') PORT_CHAR('S') PORT_CHAR(UCHAR_MAMEKEY(LEFT))
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_2)     PORT_CHAR('2') PORT_CHAR('@') PORT_CHAR(UCHAR_MAMEKEY(F2))
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("9 ( BACK")  PORT_CODE(KEYCODE_9) PORT_CHAR('9') PORT_CHAR('(') PORT_CHAR(UCHAR_MAMEKEY(F9))
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_O)     PORT_CHAR('o') PORT_CHAR('O') PORT_CHAR('\'')
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_L)     PORT_CHAR('l') PORT_CHAR('L')
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP)  PORT_CHAR('.') PORT_CHAR('>')

	PORT_START("COL2")  // col 2
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_C)     PORT_CHAR('c') PORT_CHAR('C') PORT_CHAR('`')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_E)     PORT_CHAR('e') PORT_CHAR('E') PORT_CHAR(UCHAR_MAMEKEY(UP))
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_D)     PORT_CHAR('d') PORT_CHAR('D') PORT_CHAR(UCHAR_MAMEKEY(RIGHT))
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("3 # ERASE") PORT_CODE(KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#') PORT_CHAR(UCHAR_MAMEKEY(F3))
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("8 * REDO")  PORT_CODE(KEYCODE_8) PORT_CHAR('8') PORT_CHAR('*') PORT_CHAR(UCHAR_MAMEKEY(F8))
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_I)     PORT_CHAR('i') PORT_CHAR('I') PORT_CHAR('?')
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_K)     PORT_CHAR('k') PORT_CHAR('K')
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',') PORT_CHAR('<')

	PORT_START("COL3")  // col 3
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_V)     PORT_CHAR('v') PORT_CHAR('V')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_R)     PORT_CHAR('r') PORT_CHAR('R') PORT_CHAR('[')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_F)     PORT_CHAR('f') PORT_CHAR('F') PORT_CHAR('{')
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("4 $ CLEAR") PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$') PORT_CHAR(UCHAR_MAMEKEY(F4))
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("7 & AID")   PORT_CODE(KEYCODE_7) PORT_CHAR('7') PORT_CHAR('&') PORT_CHAR(UCHAR_MAMEKEY(F7))
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_U)     PORT_CHAR('u') PORT_CHAR('U') PORT_CHAR('_')
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_J)     PORT_CHAR('j') PORT_CHAR('J')
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_M)     PORT_CHAR('m') PORT_CHAR('M')

	PORT_START("COL4")  // col 4
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_B)     PORT_CHAR('b') PORT_CHAR('B')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_T)     PORT_CHAR('t') PORT_CHAR('T') PORT_CHAR(']')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_G)     PORT_CHAR('g') PORT_CHAR('G') PORT_CHAR('}')
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("5 % BEGIN")  PORT_CODE(KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%') PORT_CHAR(UCHAR_MAMEKEY(F5))
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("6 ^ PROC'D") PORT_CODE(KEYCODE_6) PORT_CHAR('6') PORT_CHAR('^') PORT_CHAR(UCHAR_MAMEKEY(F6))
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y)     PORT_CHAR('y') PORT_CHAR('Y')
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_H)     PORT_CHAR('h') PORT_CHAR('H')
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_N)     PORT_CHAR('n') PORT_CHAR('N')

	PORT_START("COL5")  // col 5
		PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z)     PORT_CHAR('z') PORT_CHAR('Z') PORT_CHAR('\\')
		PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q)     PORT_CHAR('q') PORT_CHAR('Q')
		PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_A)     PORT_CHAR('a') PORT_CHAR('A') PORT_CHAR('|')
		PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_1)     PORT_CHAR('1') PORT_CHAR('!') PORT_CHAR(UCHAR_MAMEKEY(DEL))
		PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_0)     PORT_CHAR('0') PORT_CHAR(')') PORT_CHAR(UCHAR_MAMEKEY(F10))
		PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_P)     PORT_CHAR('p') PORT_CHAR('P') PORT_CHAR('\"')
		PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON) PORT_CHAR(';') PORT_CHAR(':')
		PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH) PORT_CHAR('/') PORT_CHAR('-')

	PORT_START("ALPHA") /* one more port for Alpha line */
		PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Alpha Lock") PORT_CODE(KEYCODE_CAPSLOCK) PORT_TOGGLE


INPUT_PORTS_END

enum
{
	SGCPU_NONE = 0,
	SGCPU_SYSROM,
	SGCPU_RAM,
	SGCPU_INTDSR,
	SGCPU_MAPPER,
	SGCPU_ROM6,
	SGCPU_PADRAM,
	SGCPU_PEB
};

int ti99_4p_state::decode_address(int address)
{
	int dec = SGCPU_NONE;
	switch (address & 0xe000)
	{
	case 0x0000:
		dec = SGCPU_SYSROM;
		break;
	case 0x2000:
	case 0xa000:
	case 0xc000:
	case 0xe000:
		dec = SGCPU_RAM;
		break;
	case 0x4000:
		if (m_internal_dsr_active) dec = SGCPU_INTDSR;
		else if (m_mapper_active) dec = SGCPU_MAPPER;
		break;
	case 0x6000:
		if (m_rom6_active) dec = SGCPU_ROM6;
		break;
	case 0x8000:
		if ((m_addr_buf & 0x1c00)==0x0000) dec = SGCPU_PADRAM;
		break;

	default:
		break;
	}
	return dec;
}

/*
    Called when the memory access starts by setting the address bus. From that
    point on, we suspend the CPU until all operations are done.
*/
SETOFFSET_MEMBER( ti99_4p_state::setoffset )
{
	m_addr_buf = offset << 1;
	m_waitcount = 0;

	if (TRACE_ADDRESS) logerror("set address %04x\n", m_addr_buf);

	m_decode = SGCPU_NONE;
	m_muxready = true;
	m_spacep = &space;

	m_decode = decode_address(m_addr_buf);

	if (m_decode == SGCPU_NONE)
	{
		// not found - pass on to PEB, 8 bit access with wait states as in TI-99/4A console
		// PEB gets remaining accesses
		// HSGPL, EVPC, other devices
		m_decode = SGCPU_PEB;
		m_waitcount = 5;
		m_muxready = false;
		m_peribox->memen_in(ASSERT_LINE);
		m_peribox->setaddress_dbin(space, m_addr_buf+1, m_dbin);
	}

	ready_join();
}

READ16_MEMBER( ti99_4p_state::memread )
{
	int address = 0;
	uint8_t hbyte = 0;

	uint16_t value = 0;

	int addr_off8k = m_addr_buf & 0x1fff;

	// If we use the debugger, decode the address now (normally done in setaddress)
	if (machine().side_effect_disabled())
	{
		m_addr_buf = offset << 1;
		m_decode = decode_address(m_addr_buf);
	}

	switch (m_decode)
	{
	case SGCPU_SYSROM:
		value = m_rom[(ROM0BASE | addr_off8k) >> 1];
		break;

	case SGCPU_RAM:
		// Memory read. The AEMS emulation has two address areas: The memory is at locations
		// 0x2000-0x3fff and 0xa000-0xffff, and the mapper area is at 0x4000-0x401e
		// (only even addresses).
		if (m_map_mode)
			address = (m_mapper[(m_addr_buf & 0xf000)>>12] << 12) | (m_addr_buf & 0x0fff);
		else // transparent mode
			address = m_addr_buf;

		value = ((m_amsram->pointer()[address] & 0xff) << 8) | (m_amsram->pointer()[address+1] & 0xff);
		break;

	case SGCPU_INTDSR:
		value = m_rom[(DSRBASE | addr_off8k)>>1];
		break;

	case SGCPU_MAPPER:
		value = (m_mapper[m_addr_buf & 0x000f]<<8) & 0xff00;
		break;

	case SGCPU_ROM6:
		value = m_rom[((m_rom6_upper? ROM6UBASE : ROM6LBASE) | addr_off8k)>>1];
		break;
	case SGCPU_PADRAM:
		// Scratch pad RAM (16 bit)
		// 8000 ... 83ff (1K, 4 times the size of the internal RAM of the TI-99/4A)
		value = ((m_scratchpad->pointer()[m_addr_buf & 0x03ff] & 0xff)<<8)
				| (m_scratchpad->pointer()[(m_addr_buf & 0x03ff)+1] & 0xff);
		break;

	case SGCPU_PEB:
		if (machine().side_effect_disabled()) return debugger_read(space, offset);
		// The byte from the odd address has already been read into the latch
		// Reading the even address now
		m_peribox->readz(space, m_addr_buf, &hbyte);
		m_peribox->memen_in(CLEAR_LINE);
		if (TRACE_MEM) logerror("Read even byte from address %04x -> %02x\n",  m_addr_buf, hbyte);
		value = (hbyte<<8) | m_latch;
	}

	return value;
}


WRITE16_MEMBER( ti99_4p_state::memwrite )
{
	int address = 0;

	// If we use the debugger, decode the address now (normally done in setaddress)
	if (machine().side_effect_disabled())
	{
		m_addr_buf = offset << 1;
		m_decode = decode_address(m_addr_buf);
	}

	switch (m_decode)
	{
	case SGCPU_SYSROM:
		if (TRACE_ILLWRITE) logerror("Ignoring ROM write access at %04x\n", m_addr_buf);
		break;

	case SGCPU_RAM:
		// see above
		if (m_map_mode)
			address = (m_mapper[(m_addr_buf & 0xf000)>>12] << 12) | (m_addr_buf & 0x0fff);
		else // transparent mode
			address = m_addr_buf;

		m_amsram->pointer()[address] = (data >> 8) & 0xff;
		m_amsram->pointer()[address+1] = data & 0xff;
		break;

	case SGCPU_INTDSR:
		if (TRACE_ILLWRITE) logerror("Ignoring DSR write access at %04x\n", m_addr_buf);
		break;

	case SGCPU_MAPPER:
		m_mapper[(m_addr_buf>>1) & 0x000f] = data;  // writing both bytes, but only the first is accepted
		break;

	case SGCPU_ROM6:
		// Writing to 6002 sets upper bank
		m_rom6_upper = (m_addr_buf & 0x0002)!=0;
		break;

	case SGCPU_PADRAM:
		// Scratch pad RAM (16 bit)
		// 8000 ... 83ff (1K, 4 times the size of the internal RAM of the TI-99/4A)
		m_scratchpad->pointer()[m_addr_buf & 0x03ff] = (data >> 8) & 0xff;
		m_scratchpad->pointer()[(m_addr_buf & 0x03ff)+1] = data & 0xff;
		break;

	case SGCPU_PEB:
		if (machine().side_effect_disabled()) { debugger_write(space, offset, data); return; }

		// Writing the even address now (addr)
		// The databus multiplexer puts the even value into the latch and outputs the odd value now.
		m_latch = (data >> 8) & 0xff;

		// write odd byte
		if (TRACE_MEM) logerror("datamux: write odd byte to address %04x <- %02x\n",  m_addr_buf+1, data & 0xff);
		m_peribox->write(space, m_addr_buf+1, data & 0xff);
		m_peribox->memen_in(CLEAR_LINE);
	}
}

/*
    Used when the debugger is reading values from PEB cards.
*/
READ16_MEMBER( ti99_4p_state::debugger_read )
{
	uint8_t lval = 0;
	uint8_t hval = 0;
	uint16_t addrb = offset << 1;
	m_peribox->memen_in(ASSERT_LINE);
	m_peribox->readz(space, addrb+1, &lval);
	m_peribox->readz(space, addrb, &hval);
	m_peribox->memen_in(CLEAR_LINE);
	return ((hval << 8)&0xff00) | (lval & 0xff);
}

/*
    Used when the debugger is writing values to PEB cards.
*/
WRITE16_MEMBER( ti99_4p_state::debugger_write )
{
	int addrb = offset << 1;
	m_peribox->memen_in(ASSERT_LINE);
	m_peribox->write(space, addrb+1, data & 0xff);
	m_peribox->write(space, addrb,  (data>>8) & 0xff);
	m_peribox->memen_in(CLEAR_LINE);
}

/*
   Data bus in (DBIN) line from the CPU.
*/
WRITE_LINE_MEMBER( ti99_4p_state::dbin_line )
{
	m_dbin = (line_state)state;
}

/*
    The datamux is connected to the clock line in order to operate
    the wait state counter and to read/write the bytes.
*/
WRITE_LINE_MEMBER( ti99_4p_state::datamux_clock_in )
{
	// return immediately if the datamux is currently inactive
	if (m_waitcount>0)
	{
		if (TRACE_MUX) logerror("datamux: wait count %d\n", m_waitcount);
		if (m_sysready==CLEAR_LINE)
		{
			if (TRACE_MUX) logerror("datamux: stalled due to external READY=0\n");
			return;
		}

		if (m_dbin==ASSERT_LINE)
		{
			// Reading
			if (state==ASSERT_LINE)
			{   // raising edge
				if (--m_waitcount==0)
				{
					m_muxready = true;
					ready_join();
				}
				if (m_waitcount==2)
				{
					// read odd byte
					m_peribox->readz(*m_spacep, m_addr_buf+1, &m_latch);
					m_peribox->memen_in(CLEAR_LINE);

					if (TRACE_MEM) logerror("datamux: read odd byte from address %04x -> %02x\n",  m_addr_buf+1, m_latch);

					// do the setaddress for the even address
					m_peribox->memen_in(ASSERT_LINE);
					m_peribox->setaddress_dbin(*m_spacep, m_addr_buf, m_dbin);
				}
			}
		}
		else    // write access
		{
			if (state==ASSERT_LINE)
			{   // raising edge
				if (--m_waitcount==0)
				{
					m_muxready = true;
					ready_join();
				}
			}
			else
			{   // falling edge
				if (m_waitcount==2)
				{
					// do the setaddress for the even address
					m_peribox->memen_in(ASSERT_LINE);
					m_peribox->setaddress_dbin(*m_spacep, m_addr_buf, m_dbin);

					// write even byte
					if (TRACE_MEM) logerror("datamux: write even byte to address %04x <- %02x\n",  m_addr_buf, m_latch);
					m_peribox->write(*m_spacep,  m_addr_buf, m_latch);
					m_peribox->memen_in(CLEAR_LINE);
				}
			}
		}
	}
}

/***************************************************************************
   CRU interface
***************************************************************************/

#define MAP_CRU_BASE 0x0f00
#define SAMS_CRU_BASE 0x1e00

/*
    CRU write
*/
WRITE8_MEMBER( ti99_4p_state::cruwrite )
{
	int addroff = offset<<1;

	if ((addroff & 0xff00)==MAP_CRU_BASE)
	{
		if ((addroff & 0x000e)==0)  m_internal_dsr = data;
		if ((addroff & 0x000e)==2)  m_internal_rom6 = data;
		if ((addroff & 0x000e)==4)  m_peribox->senila((data!=0)? ASSERT_LINE : CLEAR_LINE);
		if ((addroff & 0x000e)==6)  m_peribox->senilb((data!=0)? ASSERT_LINE : CLEAR_LINE);
		// TODO: more CRU bits? 8=Fast timing / a=KBENA
		return;
	}
	if ((addroff & 0xff00)==SAMS_CRU_BASE)
	{
		if ((addroff & 0x000e)==0) m_access_mapper = data;
		if ((addroff & 0x000e)==2) m_map_mode = data;
		return;
	}

	// No match - pass to peribox
	m_peribox->cruwrite(space, addroff, data);
}

READ8_MEMBER( ti99_4p_state::cruread )
{
	uint8_t value = 0;
	m_peribox->crureadz(space, offset<<4, &value);
	return value;
}

/***************************************************************************
    Keyboard/tape control
****************************************************************************/
static const char *const column[] = { "COL0", "COL1", "COL2", "COL3", "COL4", "COL5" };

READ8_MEMBER( ti99_4p_state::read_by_9901 )
{
	int answer=0;

	switch (offset & 0x03)
	{
	case tms9901_device::CB_INT7:
		// Read pins INT3*-INT7* of TI99's 9901.
		// bit 1: INT1 status
		// bit 2: INT2 status
		// bit 3-7: keyboard status bits 0 to 4
		//
		// |K|K|K|K|K|I2|I1|C|
		//
		if (m_keyboard_column >= m_firstjoy) // joy 1 and 2
		{
			answer = m_joyport->read_port();
		}
		else
		{
			answer = ioport(column[m_keyboard_column])->read();
		}
		if (m_check_alphalock)
		{
			answer &= ~(ioport("ALPHA")->read());
		}
		answer = (answer << 3) | m_9901_int;
		break;

	case tms9901_device::INT8_INT15:
		// Read pins int8_t*-INT15* of TI99's 9901.
		// bit 0-2: keyboard status bits 5 to 7
		// bit 3: tape input mirror
		// bit 5-7: weird, not emulated

		// |1|1|1|1|0|K|K|K|
		if (m_keyboard_column >= m_firstjoy) answer = 0x07;
		else answer = ((ioport(column[m_keyboard_column])->read())>>5) & 0x07;
		answer |= 0xf0;
		break;

	case tms9901_device::P0_P7:
		break;

	case tms9901_device::P8_P15:
		// Read pins P8-P15 of TI99's 9901.
		// bit 26: high
		// bit 27: tape input
		answer = 4;
		if (m_cassette->input() > 0) answer |= 8;
		break;
	}
	return answer;
}

/*
    WRITE key column select (P2-P4)
*/
void ti99_4p_state::set_keyboard_column(int number, int data)
{
	if (data!=0)    m_keyboard_column |= 1 << number;
	else            m_keyboard_column &= ~(1 << number);

	if (m_keyboard_column >= m_firstjoy)
	{
		m_joyport->write_port(m_keyboard_column - m_firstjoy + 1);
	}
}

WRITE_LINE_MEMBER( ti99_4p_state::keyC0 )
{
	set_keyboard_column(0, state);
}

WRITE_LINE_MEMBER( ti99_4p_state::keyC1 )
{
	set_keyboard_column(1, state);
}

WRITE_LINE_MEMBER( ti99_4p_state::keyC2 )
{
	set_keyboard_column(2, state);
}

/*
    WRITE alpha lock line (P5)
*/
WRITE_LINE_MEMBER( ti99_4p_state::alphaW )
{
	m_check_alphalock = (state==0);
}

/*
    command CS1 (only) tape unit motor (P6)
*/
WRITE_LINE_MEMBER( ti99_4p_state::cs_motor )
{
	m_cassette->change_state((state!=0)? CASSETTE_MOTOR_ENABLED : CASSETTE_MOTOR_DISABLED,CASSETTE_MASK_MOTOR);
}

/*
    audio gate (P8)

    Set to 1 before using tape: this enables the mixing of tape input sound
    with computer sound.

    We do not really need to emulate this as the tape recorder generates sound
    on its own.
*/
WRITE_LINE_MEMBER( ti99_4p_state::audio_gate )
{
}

/*
    tape output (P9)
*/
WRITE_LINE_MEMBER( ti99_4p_state::cassette_output )
{
	m_cassette->output((state!=0)? +1 : -1);
}


/***************************************************************************
    Control lines
****************************************************************************/

/*
    Combine the external (sysready) and the own (muxready) READY states.
*/
void ti99_4p_state::ready_join()
{
	int combined = (m_sysready == ASSERT_LINE && m_muxready)? ASSERT_LINE : CLEAR_LINE;

	if (TRACE_READY)
	{
		if (m_ready_prev != combined) logerror("READY level = %d\n", combined);
	}
	m_ready_prev = combined;
	m_cpu->set_ready(combined);
}

/*
    Incoming READY line from other cards in the Peripheral Expansion Box.
*/
WRITE_LINE_MEMBER( ti99_4p_state::ready_line )
{
	if (TRACE_READY)
	{
		if (state != m_sysready) logerror("READY line from PBox = %d\n", state);
	}
	m_sysready = (line_state)state;
	// Also propagate to CPU via driver
	ready_join();
}

void ti99_4p_state::set_9901_int( int line, line_state state)
{
	m_tms9901->set_single_int(line, state);
	// We latch the value for the read operation. Mind the negative logic.
	if (state==CLEAR_LINE) m_9901_int |= (1<<line);
	else m_9901_int &= ~(1<<line);
}

WRITE_LINE_MEMBER( ti99_4p_state::extint )
{
	if (TRACE_INT) logerror("EXTINT level = %02x\n", state);
	set_9901_int(1, (line_state)state);
}

WRITE_LINE_MEMBER( ti99_4p_state::notconnected )
{
	if (TRACE_INT) logerror("Setting a not connected line ... ignored\n");
}

/*
    Clock line from the CPU. Used to control wait state generation.
*/
WRITE_LINE_MEMBER( ti99_4p_state::clock_out )
{
	datamux_clock_in(state);
	m_peribox->clock_in(state);
}

WRITE8_MEMBER( ti99_4p_state::tms9901_interrupt )
{
	// offset contains the interrupt level (0-15)
	// However, the TI board just ignores that level and hardwires it to 1
	// See below (interrupt_level)
	m_cpu->set_input_line(INT_9900_INTREQ, data);
}

READ8_MEMBER( ti99_4p_state::interrupt_level )
{
	// On the TI-99 systems these IC lines are not used; the input lines
	// at the CPU are hardwired to level 1.
	return 1;
}

WRITE8_MEMBER( ti99_4p_state::external_operation )
{
	static const char* extop[8] = { "inv1", "inv2", "IDLE", "RSET", "inv3", "CKON", "CKOF", "LREX" };
	if (offset != IDLE_OP) logerror("External operation %s not implemented on the SGCPU board\n", extop[offset]);
}

/*****************************************************************************/

void ti99_4p_state::machine_start()
{
	m_peribox->senila(CLEAR_LINE);
	m_peribox->senilb(CLEAR_LINE);

	m_firstjoy = 6;

	m_sysready = ASSERT_LINE;
	m_muxready = true;

	m_rom = (uint16_t*)(memregion("maincpu")->base());

	save_item(NAME(m_firstjoy));
	save_item(NAME(m_keyboard_column));
	save_item(NAME(m_check_alphalock));
	save_item(NAME(m_internal_dsr));
	save_item(NAME(m_internal_rom6));
	save_item(NAME(m_rom6_bank));
	save_item(NAME(m_waitcount));
	save_item(NAME(m_map_mode));
	save_item(NAME(m_access_mapper));
	save_item(NAME(m_addr_buf));
	save_item(NAME(m_decode));
	save_item(NAME(m_muxready));
	save_item(NAME(m_sysready));
	save_item(NAME(m_internal_dsr_active));
	save_item(NAME(m_mapper_active));
	save_item(NAME(m_rom6_active));
	save_item(NAME(m_rom6_upper));
	save_item(NAME(m_dbin));
	save_item(NAME(m_lowbyte));
	save_item(NAME(m_highbyte));
	save_item(NAME(m_latch));
	save_pointer(NAME(m_mapper),16);
	save_item(NAME(m_9901_int));
}

/*
    set the state of int2 (called by the v9938)
*/
WRITE_LINE_MEMBER(ti99_4p_state::video_interrupt_in)
{
	set_9901_int(2, (line_state)state);
}

/*
    Reset the machine.
*/
MACHINE_RESET_MEMBER(ti99_4p_state,ti99_4p)
{
	set_9901_int(12, CLEAR_LINE);

	m_cpu->set_ready(ASSERT_LINE);
	m_cpu->set_hold(CLEAR_LINE);
	m_9901_int = 0x03; // INT2* and INT1* set to 1, i.e. inactive
}


/*
    Machine description.
*/
static MACHINE_CONFIG_START( ti99_4p_60hz )
	/* basic machine hardware */
	/* TMS9900 CPU @ 3.0 MHz */
	MCFG_TMS99xx_ADD("maincpu", TMS9900, 3000000, memmap, cru_map)
	MCFG_TMS99xx_EXTOP_HANDLER( WRITE8(ti99_4p_state, external_operation) )
	MCFG_TMS99xx_INTLEVEL_HANDLER( READ8(ti99_4p_state, interrupt_level) )
	MCFG_TMS99xx_CLKOUT_HANDLER( WRITELINE(ti99_4p_state, clock_out) )
	MCFG_TMS99xx_DBIN_HANDLER( WRITELINE(ti99_4p_state, dbin_line) )

	// tms9901
	MCFG_DEVICE_ADD(TI_TMS9901_TAG, TMS9901, 3000000)
	MCFG_TMS9901_READBLOCK_HANDLER( READ8(ti99_4p_state, read_by_9901) )
	MCFG_TMS9901_P2_HANDLER( WRITELINE( ti99_4p_state, keyC0) )
	MCFG_TMS9901_P3_HANDLER( WRITELINE( ti99_4p_state, keyC1) )
	MCFG_TMS9901_P4_HANDLER( WRITELINE( ti99_4p_state, keyC2) )
	MCFG_TMS9901_P5_HANDLER( WRITELINE( ti99_4p_state, alphaW) )
	MCFG_TMS9901_P6_HANDLER( WRITELINE( ti99_4p_state, cs_motor) )
	MCFG_TMS9901_P8_HANDLER( WRITELINE( ti99_4p_state, audio_gate) )
	MCFG_TMS9901_P9_HANDLER( WRITELINE( ti99_4p_state, cassette_output) )
	MCFG_TMS9901_INTLEVEL_HANDLER( WRITE8( ti99_4p_state, tms9901_interrupt) )

	// Peripheral expansion box (SGCPU composition)
	MCFG_DEVICE_ADD( TI_PERIBOX_TAG, TI99_PERIBOX_SG, 0)
	MCFG_PERIBOX_INTA_HANDLER( WRITELINE(ti99_4p_state, extint) )
	MCFG_PERIBOX_INTB_HANDLER( WRITELINE(ti99_4p_state, notconnected) )
	MCFG_PERIBOX_READY_HANDLER( WRITELINE(ti99_4p_state, ready_line) )

	// The SGCPU actually makes use of this pin which was unused before
	MCFG_PERIBOX_LCP_HANDLER( WRITELINE(ti99_4p_state, video_interrupt_in) )

	// Scratch pad RAM 1024 bytes (4 times the size of the TI-99/4A)
	MCFG_RAM_ADD(TI99_PADRAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("1k")
	MCFG_RAM_DEFAULT_VALUE(0)

	// AMS RAM 1 MiB
	MCFG_RAM_ADD(TI99_AMSRAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("1M")
	MCFG_RAM_DEFAULT_VALUE(0)

	// Cassette drives
	MCFG_SPEAKER_STANDARD_MONO("cass_out")
	MCFG_CASSETTE_ADD( "cassette" )

	MCFG_SOUND_WAVE_ADD(WAVE_TAG, "cassette")
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "cass_out", 0.25)

	// Joystick port
	MCFG_TI_JOYPORT4A_ADD( TI_JOYPORT_TAG )

MACHINE_CONFIG_END


ROM_START(ti99_4p)
	/*CPU memory space*/
	ROM_REGION16_BE(0x10000, "maincpu", 0)
	ROM_LOAD16_BYTE("sgcpu_hb.bin", 0x0000, 0x8000, CRC(aa100730) SHA1(35e585b2dcd3f2a0005bebb15ede6c5b8c787366) ) /* system ROMs */
	ROM_LOAD16_BYTE("sgcpu_lb.bin", 0x0001, 0x8000, CRC(2a5dc818) SHA1(dec141fe2eea0b930859cbe1ebd715ac29fa8ecb) ) /* system ROMs */
ROM_END

//    YEAR  NAME      PARENT   COMPAT   MACHINE       INPUT    STATE          INIT  COMPANY                 FULLNAME                 FLAGS
COMP( 1996, ti99_4p,  0,       0,       ti99_4p_60hz, ti99_4p, ti99_4p_state, 0,    "System-99 User Group", "SGCPU (aka TI-99/4P)" , MACHINE_SUPPORTS_SAVE )
