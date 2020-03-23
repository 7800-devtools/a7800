// license:BSD-3-Clause
// copyright-holders:?
/***************************************************************************

    XBOX (c) 2001 Microsoft

    Skeleton driver

***************************************************************************/


#include "emu.h"
#include "includes/xbox.h"
#include "includes/xbox_pci.h"

#include "cpu/i386/i386.h"
#include "machine/atapicdr.h"
#include "machine/idehd.h"
#include "machine/pit8253.h"

#include "debug/debugcmd.h"
#include "debug/debugcon.h"
#include "debugger.h"
#include "speaker.h"

#include "bitmap.h"


#define CPU_DIV 64

class xbox_state : public xbox_base_state
{
public:
	xbox_state(const machine_config &mconfig, device_type type, const char *tag) :
		xbox_base_state(mconfig, type, tag)
	{ }

protected:
	// driver_device overrides
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;

	virtual void hack_eeprom() override;

	struct chihiro_devices {
		bus_master_ide_controller_device    *ide;
	} xbox_devs;
};

void xbox_state::video_start()
{
}

static ADDRESS_MAP_START(xbox_map, AS_PROGRAM, 32, xbox_state)
	AM_IMPORT_FROM(xbox_base_map)
	AM_RANGE(0xff000000, 0xff0fffff) AM_ROM AM_REGION("bios", 0) AM_MIRROR(0x00f00000)
ADDRESS_MAP_END

static ADDRESS_MAP_START(xbox_map_io, AS_IO, 32, xbox_state)
	AM_IMPORT_FROM(xbox_base_map_io)
ADDRESS_MAP_END

static INPUT_PORTS_START( xbox )
	/* dummy active high structure */
	PORT_START("SYSA")
	PORT_DIPNAME( 0x01, 0x00, "SYSA" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x01, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x02, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x20, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x40, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x80, DEF_STR( On ) )

	/* dummy active low structure */
	PORT_START("DSWA")
	PORT_DIPNAME( 0x01, 0x01, "DSWA" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
INPUT_PORTS_END


void xbox_state::hack_eeprom()
{
	// 8004e5da,4e5da=0xc3
	m_maincpu->space(0).write_byte(0x4e5da, 0xc3); // remove audio wait
	// 8006e654,6e654=0
	m_maincpu->space(0).write_byte(0x6e654, 0); // disable boot animation
	// 800375f0,375f0=0
	m_maincpu->space(0).write_byte(0x375f0, m_maincpu->space(0).read_byte(0x375f0) & 0xfe); // internal hub not used
}

void xbox_state::machine_start()
{
	xbox_base_state::machine_start();
	xbox_devs.ide = machine().device<bus_master_ide_controller_device>("ide");
	// savestates
	//save_item(NAME(item));
}

void xbox_state::machine_reset()
{
	ata_mass_storage_device *devh;
	atapi_cdrom_device *devc;
	uint16_t *id;

	// set some needed parameters
	devh = machine().device<ata_mass_storage_device>(":pci:09.0:ide:0:hdd");
	id = devh->identify_device_buffer();
	id[88] |= (1 << 2); // ultra dma mode 2 supported
	id[128] |= 2; // bits 2-1=01 drive already unlocked
	devc = machine().device<atapi_cdrom_device>(":pci:09.0:ide:1:cdrom");
	id = devc->identify_device_buffer();
	id[64] |= (1 << 1);
	id[88] |= (1 << 2); // ultra dma mode 2 supported
}

SLOT_INTERFACE_START(usb_xbox)
	SLOT_INTERFACE("xbox_controller", OHCI_GAME_CONTROLLER)
SLOT_INTERFACE_END

SLOT_INTERFACE_START(xbox_ata_devices)
	SLOT_INTERFACE("hdd", IDE_HARDDISK)
	SLOT_INTERFACE("cdrom", ATAPI_CDROM)
SLOT_INTERFACE_END

static MACHINE_CONFIG_DERIVED(xbox, xbox_base)
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(xbox_map)
	MCFG_CPU_IO_MAP(xbox_map_io)

	MCFG_DEVICE_MODIFY(":pci:09.0:ide:0")
	MCFG_DEVICE_SLOT_INTERFACE(xbox_ata_devices, "hdd", true)
	MCFG_DEVICE_MODIFY(":pci:09.0:ide:1")
	MCFG_DEVICE_SLOT_INTERFACE(xbox_ata_devices, "cdrom", true)

	MCFG_USB_PORT_ADD(":pci:02.0:port1", usb_xbox, nullptr, false)
	MCFG_USB_PORT_ADD(":pci:02.0:port2", usb_xbox, nullptr, false)
	MCFG_USB_PORT_ADD(":pci:02.0:port3", usb_xbox, "xbox_controller", false)
	MCFG_USB_PORT_ADD(":pci:02.0:port4", usb_xbox, nullptr, false)

/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
//  MCFG_SOUND_ADD("aysnd", AY8910, MAIN_CLOCK/4)
//  MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.30)

	MCFG_DEVICE_ADD("ohci_gamepad", OHCI_GAME_CONTROLLER, 0)
MACHINE_CONFIG_END


/***************************************************************************

  Machine driver(s)

***************************************************************************/
#define ROM_LOAD_BIOS(bios,name,offset,length,hash) \
		ROMX_LOAD(name, offset, length, hash, ROM_BIOS(bios+1)) /* Note '+1' */

ROM_START( xbox )
	ROM_REGION( 0x400, "mcpx", 0 )
	ROM_LOAD( "mcpx_1_0.bin", 0, 0x200, CRC(0b07d1f1) SHA1(5d270675b54eb8071b480e42d22a3015ac211cef) )
	ROM_LOAD( "mcpx_1_1.bin", 0x200, 0x200, CRC(94ce376b) SHA1(6c875f17f773aaec51eb434068bb6c657c4343c0) )

	ROM_REGION( 0x100000, "bios", 0)
	ROM_SYSTEM_BIOS(0, "bios0", "Chihiro Bios 4134 1024k") \
	ROM_LOAD_BIOS(0, "4134_1024k.bin", 0x000000, 0x100000, CRC(49d8055a) SHA1(d46cef771a63dc8024fe36d7ab5b959087ac999f)) \
	ROM_SYSTEM_BIOS(1, "bios1", "Chihiro Bios 3944 1024k") \
	ROM_LOAD_BIOS(1, "3944_1024k.bin", 0x000000, 0x100000, CRC(32a9ecb6) SHA1(67054fc88bda94e33e86f1b19be60efec0724fb6)) \
	ROM_SYSTEM_BIOS(2, "bios2", "Chihiro Bios 4034 1024k") \
	ROM_LOAD_BIOS(2, "4034_1024k.bin", 0x000000, 0x100000, CRC(0d6fc88f) SHA1(ab676b712204fb1728bf89f9cd541a8f5a64ab97)) \
	ROM_SYSTEM_BIOS(3, "bios3", "Chihiro Bios 4817 1024k") \
	ROM_LOAD_BIOS(3, "4817_1024k.bin", 0x000000, 0x100000, CRC(3f30863a) SHA1(dc955bd4d3ca71e01214a49e5d0aba615270c03c))
	ROM_COPY( "mcpx", 0, 0x3fe00, 0x200)
	ROM_COPY( "mcpx", 0, 0x7fe00, 0x200)
	ROM_COPY( "mcpx", 0, 0xbfe00, 0x200)
	ROM_COPY( "mcpx", 0, 0xffe00, 0x200)


	ROM_REGION( 0x1000000, "tbp", 0 ) // To Be Processed, of course
	ROM_LOAD( "5101_256k.bin", 0x000000, 0x040000, CRC(e8a9224e) SHA1(5108e1025f48071c07a6823661d708c66dee97a9) )
	ROM_LOAD( "xbox-5530.bin", 0x040000, 0x040000, CRC(9569c4d3) SHA1(40fa73277013be3168135e1768b09623a987ff63) )
	ROM_LOAD( "xbox-5713.bin", 0x080000, 0x040000, CRC(58fd8173) SHA1(8b7ccc4648ccd78cdb7b65cfca09621eaf2d4238) )
	ROM_LOAD( "5838_256k.bin", 0x0C0000, 0x040000, CRC(5be2413d) SHA1(b9489e883c650b5e5fe2f83a32237dbf74f0e9f1) )
ROM_END
// See src/emu/gamedrv.h for details
// For a game:
// GAME(YEAR,NAME,PARENT,MACHINE,INPUT,CLASS,INIT,MONITOR,COMPANY,FULLNAME,FLAGS)

// For a console:
// CONS(YEAR,NAME,PARENT,COMPAT,MACHINE,INPUT,CLASS,INIT,COMPANY,FULLNAME,FLAGS)

// For a computer:
// COMP(YEAR,NAME,PARENT,COMPAT,MACHINE,INPUT,CLASS,INIT,COMPANY,FULLNAME,FLAGS)

// For a generic system:
// SYST(YEAR,NAME,PARENT,COMPAT,MACHINE,INPUT,CLASS,INIT,COMPANY,FULLNAME,FLAGS)

CONS( 2001, xbox,  0,  0,   xbox,  xbox, xbox_state,  0,       "Microsoft",      "XBOX", MACHINE_IS_SKELETON )
