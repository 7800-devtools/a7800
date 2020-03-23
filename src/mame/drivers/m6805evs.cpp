// license:BSD-3-Clause
// copyright-holders:Robbbert
/******************************************************************************************************

Motorola M68HC05EVS evaluation system

Chips:
Main board: XC68HC26P, R65C52P2, MS62256l-70PC, MS6264L-70PC, eprom. Xtal = 3.6864MHz
Emulator board: MC68HC705P9CP, undumped 28-pin prom. Xtal = 4MHz

R65C52 = Dual ACIA with inbuilt baud rate divider, uses 8 addresses, uses the 3.6864MHz crystal
XC68HC26P = PPI (3 ports), uses 8 addresses.

2014-01-12 Skeleton driver

Memory map:
0000, 0000  PORTA       Port A data register
0001, 0001  PORTB       Port B data register
0002, 0002  PORTC       Port C data register
0003, 0003  PORTD       Port D data register
0004, 0004  DDRA        Data direction register A
0005, 0005  DDRB        Data direction register B
0006, 0006  DDRC        Data direction register C
0007, 0007  DDRD        Data direction register D
0008, 0009              unimplemented
000A, 000A  SCR         SIOP control register
000B, 000B  SSR         SIOP status register
0009, 0009  SDR         SIOP data register
000D, 0011              unimplemented
0012, 0012  TCR         Timer control register
0013, 0013  TDR         Timer data register
0014, 0014  ICRH        Input capture register high
0015, 0015  ICRL        Input capture register low
0016, 0016  OCRH        Output compare register high
0017, 0017  OCRL        Output compare register low
0018, 0018  TRH         Timer register high
0019, 0019  TRL         Timer register low
001A, 001A  ATRH        Alternate timer register high
001B, 001B  ATRL        Alternate timer register low
001C, 001C  EPROG       EPROM programming register
001D, 001D  ADDR        ADC data register
001E, 001E  ADSCR       ADC status/control register
001F, 001F              reserved
0020, 004F              Page zero user EPROM
0050, 007F              unimplemented
0080, 00FF              RAM
0100, 08FF              User EPROM
0900, 0900  MOR         Mask option register
0901, 1EFF              unimplemented
1F00, 1FEF              Bootloader ROM
1FF1, 1FF7              reserved
1FF8, 1FF9              Timer interrupt vector
1FFA, 1FFB              External interrupt vector
1FFC, 1FFD              Software interrupt vector
1FFE, 1FFF              Reset vector


ToDo:
- Add CMOS family support to M6085 CPU core (different timings, different peripherals)
- Everything

******************************************************************************************************/

#include "emu.h"
#include "cpu/m6805/m6805.h"


class m6805evs_state : public driver_device
{
public:
	m6805evs_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
	{ }

private:
	required_device<cpu_device> m_maincpu;
	virtual void machine_reset() override;
};


static ADDRESS_MAP_START( m6805evs_mem, AS_PROGRAM, 8, m6805evs_state )
	ADDRESS_MAP_GLOBAL_MASK(0x1fff)
	ADDRESS_MAP_UNMAP_HIGH

	// AM_RANGE(0x0000, 0x001f) I/O registers live here
	AM_RANGE(0x0020, 0x004f) AM_ROM AM_REGION("eprom", 0x0020)
	AM_RANGE(0x0080, 0x00ff) AM_RAM
	AM_RANGE(0x0100, 0x0900) AM_ROM AM_REGION("eprom", 0x0100)
	// AM_RANGE(0x1f00, 0x1fef) bootloader ROM lives here
	AM_RANGE(0x1ff8, 0x1fff) AM_ROM AM_REGION("eprom", 0x1ff0)
ADDRESS_MAP_END

static INPUT_PORTS_START( m6805evs )
INPUT_PORTS_END

void m6805evs_state::machine_reset()
{
}

static MACHINE_CONFIG_START( m6805evs )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M6805, XTAL_4MHz)
	MCFG_CPU_PROGRAM_MAP(m6805evs_mem)
MACHINE_CONFIG_END

ROM_START(m6805evs)
	ROM_REGION(0x2000, "eprom", 0)
	ROM_LOAD( "evsbug12.bin", 0x0000, 0x2000, CRC(8b581aef) SHA1(eacf425cc8a042085ccc4097cc61570b633b1e38) )
ROM_END


/***************************************************************************

  Game driver(s)

***************************************************************************/

//    YEAR  NAME      PARENT    COMPAT  MACHINE   INPUT     CLASS           INIT  COMPANY      FULLNAME      FLAGS
COMP( 1990, m6805evs, 0,        0,      m6805evs, m6805evs, m6805evs_state, 0,    "Motorola",  "M68HC05EVS", MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW )
