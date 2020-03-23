// license:BSD-3-Clause
// copyright-holders:David Haywood
// thanks to: Ricky2001, ArcadeHacker, IFW

/*
 todo:
  - fix sound emulation
  - fix sprite communication / banking
    * bit "output bit 0x02 %d (IC21)" at 0x42 might be important
    * mag_exzi currently requires a gross hack to stop the sprite CPU crashing on startup
    * mag_xain sometimes leaves old sprites on the screen, probably due to a lost clear
      command
  - fix flipscreen
  - verify behavior of unknown / unused ports / interrupt sources etc.
  - verify the disk images, convert to a better format that can natively store protection
    * RAW data also available if required
    * as mentioned, the disks are copy protected, see notes below
  - Use proper floppy drive emulation code that originally came from MESS (tied with above)
  - verify all clocks and screen params (50hz seems to match original videos)
  - work out why we need a protection hack and replace it with proper emulation
    * there are no per-game protection devices, so it's something to do with the base hardware
    * there seem to be 2 checks, one based on a weird sector on the discs, the other based on
      a port read
  - add additional hardware notes from ArcadeHacker

*/


/*

 Magnet System by

 EFO SA (Electrónica Funcional Operativa SA).
 based on Cedar hardware


 http://retrolaser.es/cedar-computer-el-ordenador-profesional-de-efo-sa/
 http://www.recreativas.org/magnet-system-2427-efosa

 A number of original games as well as conversions were advertised for this system, it is however
 believed that EFO went bankrupt before anything hit the market.  The only 3 dumped games are
 conversions and appear to be in incomplete states (it is rather easy to break Time Scanner for
 example, the ball simply gets stuck in some places)  These are not simply bootlegs, they're
 completely original pieces of code more akin to home computer ports.

 The following were advertised
  Original Games
  - A Day in Space **
  - Crazy Driver
  - Jungle Trophy
  - Quadrum
  - War Mission **
  - The Burning Cave
  - Scorpio
  - Paris Dakar **
  - Sailing Race
  - Formula

  Ports / Conversions
  - Exzisus *
  - Double Dragon
  - Flying Shark
  - Time Scanner *
  - Xain d'Sleena *
  - Boody Kids (Booby Kids?)

  ** screenshots present on flyer
  * dumps exist



Disk Protection

Sectors are 1024 (0x400) bytes long but marked on the disc as 512 bytes as a copy protection
Sector numbering for each track starts at 200 (0xC8) again, presumably as a protection.
Each track has 6 sectors (200 - 205)
The drive runs at 240 RPM allowing for ~25% extra data. (attempting to dump at other speeds won't work)

 data order / sector marking
 track 0, side 0, sector 200...205 (instead of sector 0...5)
 track 0, side 1, sector 200...205
 track 1, side 0, sector 200...205
 track 1, side 1, sector 200...205

Note, the games store settings / scores to the disk and don't provide any kind of 'factory reset'
option, so if used the data will not be pristine.

PCB Setup

The hardware consists of 5 main PCBs in a cage.
1x Audio PCB (on top)
1x Master PCB
2x Plane PCBs (both identical aside from jumper settings)
1x Sprite PCB

There are small memory sub-boards on the Master PCB and Sprite PCB; due to the awkwardness of
the banking at times (and the fact that even with 4 banks of 256 colours, only one can be active)
I suspect the additional memory was an afterthought.

(put more details hardware notes here)


*/

#include "emu.h"
#include "cpu/z80/z80.h"
#include "cpu/z80/z80daisy.h"
#include "machine/z80pio.h"
#include "machine/bankdev.h"
#include "machine/z80ctc.h"
#include "sound/ay8910.h"

#include "audio/efo_zsu.h"
#include "machine/cedar_magnet_plane.h"
#include "machine/cedar_magnet_sprite.h"
#include "machine/cedar_magnet_flop.h"

#include "screen.h"


#define LOG_IC49_PIO_PB 0
#define LOG_IC48_PIO_PB 0
#define LOG_IC48_PIO_PA 0

class cedar_magnet_state : public driver_device
{
public:
	cedar_magnet_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_bank0(*this, "bank0"),
		m_sub_ram_bankdev(*this, "mb_sub_ram"),
		m_sub_pal_bankdev(*this, "mb_sub_pal"),
		m_ram0(*this, "ram0"),
		m_pal_r(*this, "pal_r"),
		m_pal_g(*this, "pal_g"),
		m_pal_b(*this, "pal_b"),

		m_ic48_pio(*this, "z80pio_ic48"),
		m_ic49_pio(*this, "z80pio_ic49"),
		m_palette(*this, "palette"),
		m_maincpu(*this, "maincpu"),
		m_cedsound(*this, "cedtop"),
		m_cedplane0(*this, "cedplane0"),
		m_cedplane1(*this, "cedplane1"),
		m_cedsprite(*this, "cedsprite")
	{
		m_ic48_pio_pa_val = 0xff;
		m_ic48_pio_pb_val = 0xff;
		m_ic49_pio_pb_val = 0xff;
		m_prothack = nullptr;
	}

	required_device<address_map_bank_device> m_bank0;
	required_device<address_map_bank_device> m_sub_ram_bankdev;
	required_device<address_map_bank_device> m_sub_pal_bankdev;

	required_shared_ptr<uint8_t> m_ram0;
	required_shared_ptr<uint8_t> m_pal_r;
	required_shared_ptr<uint8_t> m_pal_g;
	required_shared_ptr<uint8_t> m_pal_b;

	required_device<z80pio_device> m_ic48_pio;
	required_device<z80pio_device> m_ic49_pio;

	DECLARE_READ8_MEMBER(ic48_pio_pa_r);
	DECLARE_WRITE8_MEMBER(ic48_pio_pa_w);

	DECLARE_READ8_MEMBER(ic48_pio_pb_r);
	DECLARE_WRITE8_MEMBER(ic48_pio_pb_w);

	DECLARE_READ8_MEMBER(ic49_pio_pb_r);
	DECLARE_WRITE8_MEMBER(ic49_pio_pb_w);

	// 1x range ports
	DECLARE_WRITE8_MEMBER(port18_w);
	DECLARE_WRITE8_MEMBER(port19_w);
	DECLARE_WRITE8_MEMBER(port1b_w);

	DECLARE_READ8_MEMBER(port18_r);
	DECLARE_READ8_MEMBER(port19_r);
	DECLARE_READ8_MEMBER(port1a_r);

	// 7x range ports
	DECLARE_WRITE8_MEMBER(rambank_palbank_w);
	DECLARE_WRITE8_MEMBER(palupload_w);
	DECLARE_WRITE8_MEMBER(paladdr_w);
	DECLARE_READ8_MEMBER(watchdog_r);
	DECLARE_READ8_MEMBER(port7c_r);

	// other ports

	DECLARE_READ8_MEMBER(other_cpu_r);
	DECLARE_WRITE8_MEMBER(other_cpu_w);

	uint8_t m_paladdr;
	int m_palbank;

	uint8_t m_ic48_pio_pa_val;
	uint8_t m_ic48_pio_pb_val;
	uint8_t m_ic49_pio_pb_val;

	void set_palette(int offset);
	DECLARE_WRITE8_MEMBER(palette_r_w);
	DECLARE_WRITE8_MEMBER(palette_g_w);
	DECLARE_WRITE8_MEMBER(palette_b_w);

	void handle_sub_board_cpu_lines(cedar_magnet_board_interface &dev, int old_data, int data);
	INTERRUPT_GEN_MEMBER(irq);
	typedef void (cedar_magnet_state::*prot_func)();
	prot_func m_prothack;
	void mag_time_protection_hack();
	void mag_xain_protection_hack();
	void mag_exzi_protection_hack();

	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_cedar_magnet(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	required_device<palette_device> m_palette;
	required_device<cpu_device> m_maincpu;

	required_device<cedar_magnet_sound_device> m_cedsound;
	required_device<cedar_magnet_plane_device> m_cedplane0;
	required_device<cedar_magnet_plane_device> m_cedplane1;
	required_device<cedar_magnet_sprite_device> m_cedsprite;

	DECLARE_DRIVER_INIT(mag_time);
	DECLARE_DRIVER_INIT(mag_xain);
	DECLARE_DRIVER_INIT(mag_exzi);
};

/***********************

  Memory maps

***********************/

static ADDRESS_MAP_START( cedar_magnet_mainboard_sub_pal_map, AS_PROGRAM, 8, cedar_magnet_state )
// these are 3x MOTOROLA MM2114N SRAM 4096 bit RAM (twice the size because we map bytes, but only 4 bits are used)
// these are on the master board memory sub-board
	AM_RANGE(0x2400, 0x27ff) AM_RAM_WRITE(palette_r_w) AM_SHARE("pal_r")
	AM_RANGE(0x2800, 0x2bff) AM_RAM_WRITE(palette_g_w) AM_SHARE("pal_g")
	AM_RANGE(0x3000, 0x33ff) AM_RAM_WRITE(palette_b_w) AM_SHARE("pal_b")
ADDRESS_MAP_END

static ADDRESS_MAP_START( cedar_magnet_mainboard_sub_ram_map, AS_PROGRAM, 8, cedar_magnet_state )
// these are 8x SIEMENS HYB 41256-15 AA - 262,144 bit DRAM (32kbytes)
// these are on the master board memory sub-board
	AM_RANGE(0x00000, 0x3ffff) AM_RAM AM_SHARE("ram0")
ADDRESS_MAP_END

static ADDRESS_MAP_START( cedar_magnet_map, AS_PROGRAM, 8, cedar_magnet_state )
	AM_RANGE(0x0000, 0xffff) AM_DEVICE("bank0", address_map_bank_device, amap8)
ADDRESS_MAP_END

static ADDRESS_MAP_START( cedar_magnet_io, AS_IO, 8, cedar_magnet_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)

	AM_RANGE(0x18, 0x18) AM_READWRITE(port18_r, port18_w)
	AM_RANGE(0x19, 0x19) AM_READWRITE(port19_r, port19_w)
	AM_RANGE(0x1a, 0x1a) AM_READ(port1a_r)
	AM_RANGE(0x1b, 0x1b) AM_WRITE(port1b_w)

	AM_RANGE(0x20, 0x23) AM_DEVREADWRITE("z80pio_ic48", z80pio_device, read_alt, write_alt)
	AM_RANGE(0x40, 0x43) AM_DEVREADWRITE("z80pio_ic49", z80pio_device, read_alt, write_alt)

	AM_RANGE(0x60, 0x63) AM_DEVREADWRITE("flop", cedar_magnet_flop_device, read, write)

	AM_RANGE(0x64, 0x64) AM_READ_PORT("P1_IN")
	AM_RANGE(0x68, 0x68) AM_READ_PORT("P2_IN")
	AM_RANGE(0x6c, 0x6c) AM_READ_PORT("TEST")

	// banking / access controls to the sub-board memory
	AM_RANGE(0x70, 0x70) AM_WRITE(rambank_palbank_w)
	AM_RANGE(0x74, 0x74) AM_WRITE(palupload_w)
	AM_RANGE(0x78, 0x78) AM_READWRITE(watchdog_r, paladdr_w)
	AM_RANGE(0x7c, 0x7c) AM_READ(port7c_r) // protection??

	AM_RANGE(0xff, 0xff) AM_DEVWRITE("cedtop", cedar_magnet_sound_device, sound_command_w)
ADDRESS_MAP_END

static ADDRESS_MAP_START( cedar_bank0, AS_PROGRAM, 8, cedar_magnet_state )
	/* memory configuration 0 */
	AM_RANGE(0x00000, 0x0ffff) AM_DEVICE("mb_sub_ram", address_map_bank_device, amap8)

	/* memory configuration  1 */
	AM_RANGE(0x10000, 0x1dfff) AM_DEVICE("mb_sub_ram", address_map_bank_device, amap8)
	AM_RANGE(0x1e000, 0x1ffff) AM_ROM AM_REGION("maincpu", 0x0000)

	/* memory configuration  2*/
	AM_RANGE(0x20000, 0x2bfff) AM_DEVICE("mb_sub_ram", address_map_bank_device, amap8)
	AM_RANGE(0x2c000, 0x2ffff) AM_READWRITE(other_cpu_r, other_cpu_w)

	/* memory configuration 3*/
	AM_RANGE(0x30000, 0x31fff) AM_ROM AM_REGION("maincpu", 0x0000) AM_MIRROR(0x0e000)
ADDRESS_MAP_END


/***********************

  7x - ports
  Main board RAM sub-board

***********************/

WRITE8_MEMBER(cedar_magnet_state::rambank_palbank_w)
{
	// ---- --xx
	// xx = program bank
	m_sub_ram_bankdev->set_bank(data & 0x03);

	// yyy? yy-- palette bank
	m_palbank = data;
	int palbank = ((data & 0xc0) >> 6) | (data & 0x3c);
	m_sub_pal_bankdev->set_bank(palbank);
}

WRITE8_MEMBER(cedar_magnet_state::palupload_w)
{
	m_sub_pal_bankdev->write8(space, m_paladdr, data);
}

WRITE8_MEMBER(cedar_magnet_state::paladdr_w)
{
	m_paladdr = data;
}

READ8_MEMBER(cedar_magnet_state::watchdog_r)
{
	// watchdog
	return 0x00;
}


/***********************

  7c - protection??

***********************/

READ8_MEMBER(cedar_magnet_state::port7c_r)
{
	//logerror("%s: port7c_r\n", machine().describe_context());
	return 0x01;
}


/***********************

  1x ports
  Unknown, debug? protection?

***********************/

READ8_MEMBER(cedar_magnet_state::port18_r)
{
//  logerror("%s: port18_r\n", machine().describe_context());
	return 0x00;
}

WRITE8_MEMBER(cedar_magnet_state::port18_w)
{
//  logerror("%s: port18_w %02x\n", machine().describe_context(), data);
}

READ8_MEMBER(cedar_magnet_state::port19_r)
{
	uint8_t ret = 0x00;
//  logerror("%s: port19_r\n", machine().describe_context());

// 9496 in a,($19)
// 9498 bit 2,a

	ret |= 0x04;

	return ret;
}

READ8_MEMBER(cedar_magnet_state::port1a_r)
{
//  logerror("%s: port1a_r\n", machine().describe_context());
	return 0x00;
}


WRITE8_MEMBER(cedar_magnet_state::port19_w)
{
//  logerror("%s: port19_w %02x\n", machine().describe_context(), data);
}

WRITE8_MEMBER(cedar_magnet_state::port1b_w)
{
//  logerror("%s: port1b_w %02x\n", machine().describe_context(), data);
}

/***********************

  Palette / Video

***********************/

void cedar_magnet_state::set_palette(int offset)
{
	m_palette->set_pen_color(offset^0xff, pal4bit(m_pal_r[offset]), pal4bit(m_pal_g[offset]), pal4bit(m_pal_b[offset]));
}

WRITE8_MEMBER(cedar_magnet_state::palette_r_w)
{
	m_pal_r[offset] = data;
	set_palette(offset);
}

WRITE8_MEMBER(cedar_magnet_state::palette_g_w)
{
	m_pal_g[offset] = data;
	set_palette(offset);
}

WRITE8_MEMBER(cedar_magnet_state::palette_b_w)
{
	m_pal_b[offset] = data;
	set_palette(offset);
}

uint32_t cedar_magnet_state::screen_update_cedar_magnet(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{

	bitmap.fill(m_palette->black_pen(), cliprect);

	int pal = (m_palbank >> 6);

	m_cedplane1->draw(screen, bitmap, cliprect,pal);
	m_cedplane0->draw(screen, bitmap, cliprect,pal);
	m_cedsprite->draw(screen, bitmap, cliprect,pal);

	return 0;
}

void cedar_magnet_state::video_start()
{
}

/***********************

  Access to other CPUs

***********************/

READ8_MEMBER(cedar_magnet_state::other_cpu_r)
{
	int bankbit0 = (m_ic48_pio_pa_val & 0x60) >> 5;
	int plane0select = (m_ic48_pio_pa_val & 0x07) >> 0;
	int plane1select = (m_ic48_pio_pb_val & 0x07) >> 0;
	int spriteselect = (m_ic48_pio_pb_val & 0x70) >> 4;
	int soundselect = (m_ic49_pio_pb_val & 0x70) >> 4;
	int windowbank = (m_ic49_pio_pb_val & 0x0c) >> 2;
	int unk2 = (m_ic49_pio_pb_val & 0x03) >> 0;

	int cpus_accessed = 0;
	uint8_t ret = 0x00;

	int offset2 = offset + windowbank * 0x4000;

	if (spriteselect == 0x1)
	{
		cpus_accessed++;
		ret |= m_cedsprite->read_cpu_bus(offset2);
	}

	if (plane0select == 0x1)
	{
		cpus_accessed++;
		ret |= m_cedplane0->read_cpu_bus(offset2);
	}

	if (plane1select == 0x1)
	{
		cpus_accessed++;
		ret |= m_cedplane1->read_cpu_bus(offset2);
	}

	if (soundselect == 0x1)
	{
		cpus_accessed++;
		ret |= m_cedsound->read_cpu_bus(offset2);
		logerror("%s: reading soundselect! %04x - bank bits %d %d %d %d %d %d %d\n", machine().describe_context(), offset,bankbit0, plane0select, plane1select, spriteselect, soundselect, windowbank, unk2);
	}

	if (cpus_accessed != 1)
		logerror("%s: reading multiple CPUS!!! %04x - bank bits %d %d %d %d %d %d %d\n", machine().describe_context(), offset,bankbit0, plane0select, plane1select, spriteselect, soundselect, windowbank, unk2);

//  if ((offset==0) || (offset2 == 0xe) || (offset2 == 0xf) || (offset2 == 0x68))
//      logerror("%s: reading banked bus area %04x - bank bits %d %d %d %d %d %d %d\n", machine().describe_context(), offset,bankbit0, plane0select, plane1select, spriteselect, soundselect, windowbank, unk2);

	return ret;
}

WRITE8_MEMBER(cedar_magnet_state::other_cpu_w)
{
	int bankbit0 = (m_ic48_pio_pa_val & 0x60) >> 5;
	int plane0select = (m_ic48_pio_pa_val & 0x07) >> 0;
	int plane1select = (m_ic48_pio_pb_val & 0x07) >> 0;
	int spriteselect = (m_ic48_pio_pb_val & 0x70) >> 4;
	int soundselect = (m_ic49_pio_pb_val & 0x70) >> 4;
	int windowbank = (m_ic49_pio_pb_val & 0x0c) >> 2;
	int unk2 = (m_ic49_pio_pb_val & 0x03) >> 0;

	int cpus_accessed = 0;

	int offset2 = offset + windowbank * 0x4000;

	if (spriteselect == 0x1)
	{
		cpus_accessed++;
		m_cedsprite->write_cpu_bus(offset2, data);
	}

	if (plane0select == 0x1)
	{
		cpus_accessed++;
		m_cedplane0->write_cpu_bus(offset2, data);
	}

	if (plane1select == 0x1)
	{
		cpus_accessed++;
		m_cedplane1->write_cpu_bus(offset2, data);
	}

	if (soundselect == 0x1)
	{
		cpus_accessed++;
		m_cedsound->write_cpu_bus(offset2, data);
	//  logerror("%s: sound cpu write %04x %02x - bank bits %d %d %d %d %d %d %d\n", machine().describe_context(), offset,data, bankbit0, plane0select, plane1select, spriteselect, soundselect, windowbank, unk2);
	}

	if (cpus_accessed != 1)
		logerror("%s: writing multiple CPUS!!! %04x %02x - bank bits %d %d %d %d %d %d %d\n", machine().describe_context(), offset,data, bankbit0, plane0select, plane1select, spriteselect, soundselect, windowbank, unk2);

//  if ((offset==0) || (offset2 == 0xe) || (offset2 == 0xf) || (offset2 == 0x68))
//      logerror("%s: other cpu write %04x %02x - bank bits %d %d %d %d %d %d %d\n", machine().describe_context(), offset,data, bankbit0, plane0select, plane1select, spriteselect, soundselect, windowbank, unk2);
}


void cedar_magnet_state::handle_sub_board_cpu_lines(cedar_magnet_board_interface &dev, int old_data, int data)
{
	if (old_data != data)
	{
		if (data & 0x04)
			dev.reset_assert();
		else
			dev.reset_clear();

		if (data & 0x02)
			dev.halt_clear();
		else
			dev.halt_assert();
	}
}

/***********************

  IC 48 PIO handlers
   (mapped at 0x20 / 0x22)

***********************/

READ8_MEMBER( cedar_magnet_state::ic48_pio_pa_r ) // 0x20
{
	uint8_t ret = m_ic48_pio_pa_val & ~0x08;

	ret |= ioport("COIN1")->read()<<3;
	if (!m_cedplane0->is_running()) ret &= ~0x01;

	// interrupt source stuff??
	ret &= ~0x10;

	if (LOG_IC48_PIO_PA) logerror("%s: ic48_pio_pa_r (returning %02x)\n", machine().describe_context(), ret);
	return ret;
}

WRITE8_MEMBER( cedar_magnet_state::ic48_pio_pa_w ) // 0x20
{
	int oldplane0select = (m_ic48_pio_pa_val & 0x07) >> 0;

	// bits 19 are set to input?
	m_ic48_pio_pa_val = data;

	// address 0x20 - pio ic48 port a
	if (LOG_IC48_PIO_PA) logerror("%s: ic48_pio_pa_w %02x (memory banking etc.)\n", machine().describe_context(), data);

	if (LOG_IC48_PIO_PA) logerror("output bit 0x80 %d (unused)\n", (data >> 7)&1); // A7 -> 12 J4 unpopulated
	if (LOG_IC48_PIO_PA) logerror("output bit 0x40 %d (bank)\n", (data >> 6)&1); // A6 -> 2 74HC10 3NAND IC19
	if (LOG_IC48_PIO_PA) logerror("output bit 0x20 %d (bank)\n", (data >> 5)&1); // A5 -> 4 74HC10 3NAND IC19
	if (LOG_IC48_PIO_PA) logerror("input  bit 0x10 %d (interrupt source related?)\n", (data >> 4)&1); // 10 in // A4 <- 9 74HC74 IC20 <- input from 18 74LS244 IC61
	if (LOG_IC48_PIO_PA) logerror("input  bit 0x08 %d (COIN1)\n", (data >> 3)&1); // 08 in // A3 <- 4 74HC14P (inverter) IC4 <- EDGE 21 COIN1
	if (LOG_IC48_PIO_PA) logerror("output bit 0x04 %d (plane0 CPU/bus related?)\n", (data >> 2)&1); // A2 -> 45 J6
	if (LOG_IC48_PIO_PA) logerror("output bit 0x02 %d (plane0 CPU/bus related?)\n", (data >> 1)&1); // A1 -> 47 J6
	if (LOG_IC48_PIO_PA) logerror("input  bit 0x01 %d (plane0 CPU/bus related?)\n", (data >> 0)&1); // A0 -> 49 J6

	int bankbit0 = (m_ic48_pio_pa_val & 0x60) >> 5;
	m_bank0->set_bank(bankbit0);

	int plane0select = (m_ic48_pio_pa_val & 0x07) >> 0;

	handle_sub_board_cpu_lines(*m_cedplane0, oldplane0select, plane0select);
}


READ8_MEMBER( cedar_magnet_state::ic48_pio_pb_r ) // 0x22
{
	uint8_t ret = m_ic48_pio_pb_val & ~0x80;

	ret |= ioport("COIN2")->read()<<7;

	if (!m_cedsprite->is_running()) ret &= ~0x10;
	if (!m_cedplane1->is_running()) ret &= ~0x01;

	if (LOG_IC48_PIO_PB) logerror("%s: ic48_pio_pb_r (returning %02x)\n", machine().describe_context(), ret);
	return ret;
}

WRITE8_MEMBER(cedar_magnet_state::ic48_pio_pb_w) // 0x22
{
	int oldplane1select = (m_ic48_pio_pb_val & 0x07) >> 0;
	int oldspriteselect = (m_ic48_pio_pb_val & 0x70) >> 4;

	m_ic48_pio_pb_val = data;

	if (LOG_IC48_PIO_PB)  logerror("%s: ic48_pio_pb_w %02x\n", machine().describe_context(), data);

	// address 0x22 - pio ic48 port b
	if (LOG_IC48_PIO_PB) logerror("input  bit 0x80 %d (COIN2)\n", (data >> 7)&1); // B7 <- 2 74HC14P (inverter) IC4 <- EDGE 22 COIN2
	if (LOG_IC48_PIO_PB) logerror("output bit 0x40 (J6) (sprite CPU/bus related?) %d\n", (data >> 6)&1); // B6 -> 41 J6
	if (LOG_IC48_PIO_PB) logerror("output bit 0x20 (J6) (sprite CPU/bus related?) %d\n", (data >> 5)&1); // B5 -> 43 J6
	if (LOG_IC48_PIO_PB) logerror("input  bit 0x10 (J6) (sprite CPU/bus related?) %d\n", (data >> 4)&1); // B4 -> 44 J6
	if (LOG_IC48_PIO_PB) logerror("output bit 0x08 (Q8) %d\n", (data >> 3)&1); // B3 -> Q8 transistor
	if (LOG_IC48_PIO_PB) logerror("output bit 0x04 (J6) (plane1 CPU/bus related?) %d\n", (data >> 2)&1); // B2 -> 46 J6
	if (LOG_IC48_PIO_PB) logerror("output bit 0x02 (J6) (plane1 CPU/bus related?) %d\n", (data >> 1)&1); // B1 -> 48 J6
	if (LOG_IC48_PIO_PB) logerror("input  bit 0x01 (J6) (plane1 CPU/bus related?) %d\n", (data >> 0)&1); // B0 -> 50 J6

	int plane1select = (m_ic48_pio_pb_val & 0x07) >> 0;
	int spriteselect = (m_ic48_pio_pb_val & 0x70) >> 4;

	handle_sub_board_cpu_lines(*m_cedplane1, oldplane1select, plane1select);
	handle_sub_board_cpu_lines(*m_cedsprite, oldspriteselect, spriteselect);
}

/***********************

  IC 49 PIO handlers
     (mapped at 0x42)

***********************/

READ8_MEMBER( cedar_magnet_state::ic49_pio_pb_r ) // 0x42
{
	uint8_t ret = m_ic49_pio_pb_val;

	if (!m_cedsound->is_running()) ret &= ~0x10;

	if (LOG_IC49_PIO_PB) logerror("%s: ic49_pio_pb_r (returning %02x)\n", machine().describe_context(), ret);
	return ret;
}

WRITE8_MEMBER( cedar_magnet_state::ic49_pio_pb_w ) // 0x42
{
	int oldsoundselect = (m_ic49_pio_pb_val & 0x70) >> 4;

	m_ic49_pio_pb_val = data;

	//logerror("%s: ic49_pio_pb_w %02x\n", machine().describe_context(), data);

	// address 0x42 - pio ic49 port b
	if (LOG_IC49_PIO_PB) logerror("output bit 0x80 %d (Q9)\n", (data >> 7)&1); // B7 -> Q9 transistor
	if (LOG_IC49_PIO_PB) logerror("output bit 0x40 %d (sound CPU bus related) (J3)\n", (data >> 6)&1); // B6 -> 9 J3
	if (LOG_IC49_PIO_PB) logerror("output bit 0x20 %d (sound CPU bus related) (J3)\n", (data >> 5)&1); // B5 -> 8 J3
	if (LOG_IC49_PIO_PB) logerror("input  bit 0x10 %d (sound CPU bus related) (J3)\n", (data >> 4)&1); // B4 -> 7 J3       // input?
	if (LOG_IC49_PIO_PB) logerror("output bit 0x08 %d (J7)\n", (data >> 3)&1); // B3 -> 35 J7  bank bits
	if (LOG_IC49_PIO_PB) logerror("output bit 0x04 %d (J7)\n", (data >> 2)&1); // B2 -> 36 J7  bank bits
	// there is code to mask out both bottom bits here before load operations?
	if (LOG_IC49_PIO_PB) logerror("output bit 0x02 %d (IC21)\n", (data >> 1)&1); // B1 -> 3 74HC04 IC21 (set before some SPRITE cpu operations, possibly halts the blitter?)
	if (LOG_IC49_PIO_PB) logerror("output bit 0x01 (LED) %d\n", (data >> 0)&1); // B0 -> LED LD1



	int soundselect = (m_ic49_pio_pb_val & 0x70) >> 4;

	handle_sub_board_cpu_lines(*m_cedsound, oldsoundselect, soundselect);
}

/***********************

  Init / Inputs / Machine

***********************/

void cedar_magnet_state::machine_start()
{
	save_item(NAME(m_paladdr));
}

void cedar_magnet_state::machine_reset()
{
	m_ic48_pio_pa_val = 0xff;

	int bankbit0 = (m_ic48_pio_pa_val & 0x60) >> 5;
	m_bank0->set_bank(bankbit0);
	m_sub_ram_bankdev->set_bank(3);
	m_sub_pal_bankdev->set_bank(0);
}


static INPUT_PORTS_START( cedar_magnet )
	PORT_START("COIN1")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_COIN1 )

	PORT_START("COIN2")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_COIN2 )

	PORT_START("P1_IN")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON3 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_START1 )

	PORT_START("P2_IN")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_8WAY PORT_PLAYER(2)
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_PLAYER(2)
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_START2 )

	PORT_START("TEST")
	PORT_BIT( 0x7f, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_SERVICE_NO_TOGGLE( 0x80, IP_ACTIVE_LOW )
INPUT_PORTS_END

INTERRUPT_GEN_MEMBER(cedar_magnet_state::irq)
{
	if (m_prothack)
		(this->*m_prothack)();

	m_maincpu->set_input_line(0, HOLD_LINE);
	m_cedplane0->irq_hold();
	m_cedplane1->irq_hold();
	m_cedsprite->irq_hold();
}

static MACHINE_CONFIG_START( cedar_magnet )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80,4000000)         /* ? MHz */
	MCFG_CPU_PROGRAM_MAP(cedar_magnet_map)
	MCFG_CPU_IO_MAP(cedar_magnet_io)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", cedar_magnet_state,  irq)

	MCFG_DEVICE_ADD("bank0", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(cedar_bank0)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_LITTLE)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_ADDRBUS_WIDTH(18)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x10000)

	MCFG_DEVICE_ADD("mb_sub_ram", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(cedar_magnet_mainboard_sub_ram_map)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_LITTLE)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_ADDRBUS_WIDTH(18)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x10000)

	MCFG_DEVICE_ADD("mb_sub_pal", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(cedar_magnet_mainboard_sub_pal_map)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_LITTLE)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_ADDRBUS_WIDTH(8+6)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x100)

	MCFG_DEVICE_ADD("z80pio_ic48", Z80PIO, 4000000/2)
//  MCFG_Z80PIO_OUT_INT_CB(INPUTLINE("maincpu", INPUT_LINE_IRQ0))
	MCFG_Z80PIO_IN_PA_CB(READ8(cedar_magnet_state, ic48_pio_pa_r))
	MCFG_Z80PIO_OUT_PA_CB(WRITE8(cedar_magnet_state, ic48_pio_pa_w))
	MCFG_Z80PIO_IN_PB_CB(READ8(cedar_magnet_state, ic48_pio_pb_r))
	MCFG_Z80PIO_OUT_PB_CB(WRITE8(cedar_magnet_state, ic48_pio_pb_w))

	MCFG_DEVICE_ADD("z80pio_ic49", Z80PIO, 4000000/2)
//  MCFG_Z80PIO_OUT_INT_CB(INPUTLINE("maincpu", INPUT_LINE_IRQ0))
//  MCFG_Z80PIO_IN_PA_CB(READ8(cedar_magnet_state, ic49_pio_pa_r)) // NOT USED
//  MCFG_Z80PIO_OUT_PA_CB(WRITE8(cedar_magnet_state, ic49_pio_pa_w)) // NOT USED
	MCFG_Z80PIO_IN_PB_CB(READ8(cedar_magnet_state, ic49_pio_pb_r))
	MCFG_Z80PIO_OUT_PB_CB(WRITE8(cedar_magnet_state, ic49_pio_pb_w))

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(256, 256)
	MCFG_SCREEN_VISIBLE_AREA(0, 256-8-1, 0, 192-1)
	MCFG_SCREEN_UPDATE_DRIVER(cedar_magnet_state, screen_update_cedar_magnet)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 0x400)

	MCFG_CEDAR_MAGNET_SOUND_ADD("cedtop")
	MCFG_CEDAR_MAGNET_PLANE_ADD("cedplane0")
	MCFG_CEDAR_MAGNET_PLANE_ADD("cedplane1")
	MCFG_CEDAR_MAGNET_SPRITE_ADD("cedsprite")

	MCFG_CEDAR_MAGNET_FLOP_ADD("flop")

	MCFG_QUANTUM_PERFECT_CPU("maincpu")
MACHINE_CONFIG_END


#define BIOS_ROM \
	ROM_REGION( 0x10000, "maincpu", 0 ) \
	ROM_LOAD( "Magnet-Master-VID-E03.BIN", 0x00000, 0x02000, CRC(86c4a4f0) SHA1(6db1a006b2e0b2a7cc9748ade881debb098b6757) )



ROM_START( cedmag )
	BIOS_ROM

	ROM_REGION( 0x100000, "flop:disk", ROMREGION_ERASE00 )
	// no disk inserted
ROM_END

ROM_START( mag_time )
	BIOS_ROM

	ROM_REGION( 0x100000, "flop:disk", 0 )
	ROM_LOAD( "timescanner.img", 0x00000, 0xf0000, CRC(214c558c) SHA1(9c71fce35acaf17ac685f77aebb1b0a930060f0b) )
ROM_END

ROM_START( mag_exzi )
	BIOS_ROM

	ROM_REGION( 0x100000, "flop:disk", ROMREGION_ERASE00 )
	ROM_LOAD( "exzisus.img", 0x00000, 0xf0000, CRC(3705e9dc) SHA1(78c8010d224f5deb202a29bd273ea7dc85ddcdb4) )
ROM_END

ROM_START( mag_xain )
	BIOS_ROM

	ROM_REGION( 0x100000, "flop:disk", ROMREGION_ERASE00 )
	ROM_LOAD( "xain.img", 0x00000, 0xf0000, CRC(5647849f) SHA1(edd2f3f6359424583bf526bf4601476dc849e617) )
ROM_END

/*
    protection? (Time Scanner note)

    one part of the code is a weird loop checking values from port 0x7c while doing other nonsensical stuff, a flag gets set to 0xff if it fails

    the other part is after reading the weird extra block on the disk (score / protection data at 0xea400 in the disk image*) and again a flag
    gets set to 0xff in certain conditions there's then a check after inserting a coin, these values can't be 0xff at that point, and there
    doesn't appear to be any code to reset them.

    *0xea400 is/was track 4e, side 00, sector 01 for future reference if the floppy format changes

    all games have the same code in them but at different addresses
*/


void protection_hack(uint8_t* ram, int address1, int address2)
{
	if ((ram[address1] == 0x3e) && (ram[address1+1] == 0xff)) ram[address1] = 0xc9;
	if ((ram[address2] == 0x3e) && (ram[address2+1] == 0xff)) ram[address2] = 0xc9;
}

void cedar_magnet_state::mag_time_protection_hack()
{
	protection_hack(m_ram0, 0x8bc, 0x905);
}

void cedar_magnet_state::mag_xain_protection_hack()
{
	protection_hack(m_ram0, 0x796, 0x7df);
}

void cedar_magnet_state::mag_exzi_protection_hack()
{
	protection_hack(m_ram0, 0x8b6, 0x8ff);
}


DRIVER_INIT_MEMBER(cedar_magnet_state, mag_time)
{
	m_prothack = &cedar_magnet_state::mag_time_protection_hack;
}

DRIVER_INIT_MEMBER(cedar_magnet_state, mag_xain)
{
	m_prothack = &cedar_magnet_state::mag_xain_protection_hack;
}

DRIVER_INIT_MEMBER(cedar_magnet_state, mag_exzi)
{
	m_prothack = &cedar_magnet_state::mag_exzi_protection_hack;
}

GAME( 1987, cedmag,    0,         cedar_magnet, cedar_magnet, cedar_magnet_state,  0,        ROT0,  "EFO SA / Cedar", "Magnet System",                         MACHINE_IS_BIOS_ROOT )

GAME( 1987, mag_time,  cedmag,    cedar_magnet, cedar_magnet, cedar_magnet_state,  mag_time, ROT90, "EFO SA / Cedar", "Time Scanner (TS 2.0, Magnet System)",  MACHINE_IMPERFECT_GRAPHICS | MACHINE_IMPERFECT_SOUND ) // original game was by Sega
GAME( 1987, mag_exzi,  cedmag,    cedar_magnet, cedar_magnet, cedar_magnet_state,  mag_exzi, ROT0,  "EFO SA / Cedar", "Exzisus (EX 1.0, Magnet System)",       MACHINE_IMPERFECT_GRAPHICS | MACHINE_IMPERFECT_SOUND ) // original game was by Taito
GAME( 1987, mag_xain,  cedmag,    cedar_magnet, cedar_magnet, cedar_magnet_state,  mag_xain, ROT0,  "EFO SA / Cedar", "Xain'd Sleena (SC 3.0, Magnet System)", MACHINE_IMPERFECT_GRAPHICS | MACHINE_IMPERFECT_SOUND ) // original game was by Technos
