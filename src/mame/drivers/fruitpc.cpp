// license:BSD-3-Clause
// copyright-holders:R. Belmont, Carl
/************************************************************************************

  "Fruit Land" (c) ????
  hack of an open source game by MesSoft with coin handling added

  preliminary driver by R. Belmont and Carl

  Hardware:
  - ST STPCD0166BTC3 486/66 + PC + VGA all on one chip
  - 4x AS4LC1M16E5-60TC 1M x 16 EDO DRAM

=====================================================================================*/

#include "emu.h"
#include "cpu/i386/i386.h"
#include "machine/lpci.h"
#include "machine/pckeybrd.h"
#include "machine/idectrl.h"
#include "video/pc_vga.h"
#include "machine/pcshare.h"
#include "bus/isa/isa.h"
#include "bus/isa/sblaster.h"

class fruitpc_state : public pcat_base_state
{
public:
	fruitpc_state(const machine_config &mconfig, device_type type, const char *tag)
		: pcat_base_state(mconfig, type, tag)
		, m_isabus(*this, "isa")
		, m_inp1(*this, "INP1")
		, m_inp2(*this, "INP2")
		, m_inp3(*this, "INP3")
		, m_inp4(*this, "INP4")
	{ }

	required_device<isa8_device> m_isabus;
	required_ioport m_inp1;
	required_ioport m_inp2;
	required_ioport m_inp3;
	required_ioport m_inp4;

	DECLARE_READ8_MEMBER(fruit_inp_r);
	DECLARE_WRITE8_MEMBER(dma8237_1_dack_w);
};

READ8_MEMBER(fruitpc_state::fruit_inp_r)
{
	switch(offset)
	{
		case 0:
			return m_inp1->read();
		case 1:
			return m_inp2->read();
		case 2:
			return m_inp3->read();
		case 3:
			return m_inp4->read();
	}
	return 0;
}

static ADDRESS_MAP_START( fruitpc_map, AS_PROGRAM, 32, fruitpc_state )
	AM_RANGE(0x00000000, 0x0009ffff) AM_RAM
	AM_RANGE(0x000a0000, 0x000bffff) AM_DEVREADWRITE8("vga", vga_device, mem_r, mem_w, 0xffffffff) // VGA VRAM
	AM_RANGE(0x000c0000, 0x000dffff) AM_ROM AM_REGION("bios", 0)
	AM_RANGE(0x000e0000, 0x000fffff) AM_RAM AM_REGION("bios", 0)
	AM_RANGE(0x00100000, 0x008fffff) AM_RAM  // 8MB RAM
	AM_RANGE(0x02000000, 0x28ffffff) AM_NOP
	AM_RANGE(0xfffe0000, 0xffffffff) AM_ROM AM_REGION("bios", 0)    /* System BIOS */
ADDRESS_MAP_END

static ADDRESS_MAP_START( fruitpc_io, AS_IO, 32, fruitpc_state )
	AM_IMPORT_FROM(pcat32_io_common)
	AM_RANGE(0x01f0, 0x01f7) AM_DEVREADWRITE16("ide", ide_controller_device, read_cs0, write_cs0, 0xffffffff)
	AM_RANGE(0x0310, 0x0313) AM_READ8(fruit_inp_r, 0xffffffff)
	AM_RANGE(0x03b0, 0x03bf) AM_DEVREADWRITE8("vga", vga_device, port_03b0_r, port_03b0_w, 0xffffffff)
	AM_RANGE(0x03c0, 0x03cf) AM_DEVREADWRITE8("vga", vga_device, port_03c0_r, port_03c0_w, 0xffffffff)
	AM_RANGE(0x03d0, 0x03df) AM_DEVREADWRITE8("vga", vga_device, port_03d0_r, port_03d0_w, 0xffffffff)
	AM_RANGE(0x03f0, 0x03f7) AM_DEVREADWRITE16("ide", ide_controller_device, read_cs1, write_cs1, 0xffffffff)
ADDRESS_MAP_END

static INPUT_PORTS_START( fruitpc )
	PORT_START("INP1")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x00fe, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_START("INP2")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT )
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT )
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_JOYSTICK_UP )
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN )
	PORT_BIT( 0x00c0, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_START("INP3")
	PORT_BIT( 0x0003, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_DIPNAME( 0x0004, 0x0004, "CONFIGURATION" )
	PORT_DIPSETTING( 0x0004, DEF_STR( Off ) )
	PORT_DIPSETTING( 0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0008, 0x0008, "STATISTICHE" )
	PORT_DIPSETTING( 0x0008, DEF_STR( Off ) )
	PORT_DIPSETTING( 0x0000, DEF_STR( On ) )
	PORT_BIT( 0x00f0, IP_ACTIVE_HIGH, IPT_UNUSED )
	PORT_START("INP4")
	PORT_BIT( 0x00ff, IP_ACTIVE_LOW, IPT_UNKNOWN )
INPUT_PORTS_END

//TODO: use atmb device
WRITE8_MEMBER( fruitpc_state::dma8237_1_dack_w ){ m_isabus->dack_w(1, data); }

static SLOT_INTERFACE_START( fruitpc_isa8_cards )
	SLOT_INTERFACE("sb15",  ISA8_SOUND_BLASTER_1_5)
SLOT_INTERFACE_END

static DEVICE_INPUT_DEFAULTS_START( fruitpc_sb_def )
	DEVICE_INPUT_DEFAULTS("CONFIG", 0x03, 0x01)
DEVICE_INPUT_DEFAULTS_END

static MACHINE_CONFIG_START( fruitpc_sb_conf )
	MCFG_DEVICE_MODIFY("pc_joy")
	MCFG_DEVICE_SLOT_INTERFACE(pc_joysticks, nullptr, true) // remove joystick
MACHINE_CONFIG_END

static MACHINE_CONFIG_START( fruitpc )
	MCFG_CPU_ADD("maincpu", I486, 66000000) // ST STPCD0166BTC3 66 MHz 486 CPU
	MCFG_CPU_PROGRAM_MAP(fruitpc_map)
	MCFG_CPU_IO_MAP(fruitpc_io)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DEVICE("pic8259_1", pic8259_device, inta_cb)

	MCFG_FRAGMENT_ADD( pcat_common )

	MCFG_IDE_CONTROLLER_ADD("ide", ata_devices, "hdd", nullptr, true)
	MCFG_ATA_INTERFACE_IRQ_HANDLER(DEVWRITELINE("pic8259_2", pic8259_device, ir6_w))

	/* video hardware */
	MCFG_FRAGMENT_ADD( pcvideo_vga )

	MCFG_DEVICE_MODIFY("dma8237_1")
	MCFG_I8237_OUT_IOW_1_CB(WRITE8(fruitpc_state, dma8237_1_dack_w))

	MCFG_DEVICE_ADD("isa", ISA8, 0)
	MCFG_ISA8_CPU(":maincpu")
	MCFG_ISA_OUT_IRQ2_CB(DEVWRITELINE("pic8259_2", pic8259_device, ir2_w))
	MCFG_ISA_OUT_IRQ3_CB(DEVWRITELINE("pic8259_1", pic8259_device, ir3_w))
	MCFG_ISA_OUT_IRQ4_CB(DEVWRITELINE("pic8259_1", pic8259_device, ir4_w))
	MCFG_ISA_OUT_IRQ5_CB(DEVWRITELINE("pic8259_1", pic8259_device, ir5_w))
	MCFG_ISA_OUT_IRQ6_CB(DEVWRITELINE("pic8259_1", pic8259_device, ir6_w))
	MCFG_ISA_OUT_IRQ7_CB(DEVWRITELINE("pic8259_1", pic8259_device, ir7_w))
	MCFG_ISA_OUT_DRQ1_CB(DEVWRITELINE("dma8237_1", am9517a_device, dreq1_w))
	MCFG_ISA_OUT_DRQ2_CB(DEVWRITELINE("dma8237_1", am9517a_device, dreq2_w))
	MCFG_ISA_OUT_DRQ3_CB(DEVWRITELINE("dma8237_1", am9517a_device, dreq3_w))

	MCFG_ISA8_SLOT_ADD("isa", "isa1", fruitpc_isa8_cards, "sb15", true)
	MCFG_DEVICE_CARD_DEVICE_INPUT_DEFAULTS("sb15", fruitpc_sb_def)
	MCFG_DEVICE_CARD_MACHINE_CONFIG("sb15", fruitpc_sb_conf)
MACHINE_CONFIG_END

ROM_START( fruitpc )
	ROM_REGION( 0x20000, "bios", 0 )
	ROM_LOAD( "at-gs001.bin", 0x000000, 0x020000, CRC(7dec34d0) SHA1(81d194d67fef9f6531bd3cd1ee0baacb5c2558bf) )

	DISK_REGION( "ide:0:hdd:image" )    // 8 MB Compact Flash card
	DISK_IMAGE( "fruit", 0,SHA1(df250ff06a97fa141a4144034f7035ac2947c53c) )
ROM_END

GAME( 2006, fruitpc,  0, fruitpc, fruitpc, fruitpc_state,  0, ROT0, "<unknown>", "Fruit Land", MACHINE_IMPERFECT_GRAPHICS )

// this doesn't really belong here, but is some kind of x86 pc-like hardware, exact CPU type etc. unknown
// hardware ia by Paokai, motherboard has logos, large chip with logo too, http://www.paokai.com.tw/
ROM_START( gogostrk )
	ROM_REGION( 0x40000, "bios", 0 )
	ROM_LOAD( "39sf020a.rom1", 0x000000, 0x040000, CRC(236d4d95) SHA1(50579acddc93c05d5f8e17ad3669a29d2dc49965) )

	DISK_REGION( "ide:0:hdd:image" )    // 128 MB CF Card
	DISK_IMAGE( "ggs-5-2-07", 0,SHA1(f214fd39ec8ac02f008823f4b179ea6c6835e1b8) )
ROM_END

GAME( 2007, gogostrk, 0, fruitpc, fruitpc, fruitpc_state, 0, ROT0, "American Alpha / Paokai", "Go Go Strike", MACHINE_NOT_WORKING ) // motherboard is dated 2006, if the CF card string is a date it's 2007
