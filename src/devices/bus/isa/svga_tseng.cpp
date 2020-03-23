// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/***************************************************************************

  ISA SVGA Tseng wrapper

***************************************************************************/

#include "emu.h"
#include "svga_tseng.h"
#include "video/pc_vga.h"

#include "screen.h"


ROM_START( et4000 )
	ROM_REGION(0x8000,"et4000", 0)
	ROM_LOAD("et4000.bin", 0x00000, 0x8000, CRC(f1e817a8) SHA1(945d405b0fb4b8f26830d495881f8587d90e5ef9) )
ROM_END

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(ISA8_SVGA_ET4K, isa8_svga_et4k_device, "et4000", "SVGA Tseng ET4000 Graphics Card")


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( isa8_svga_et4k_device::device_add_mconfig )
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_RAW_PARAMS(XTAL_25_1748MHz,900,0,640,526,0,480)
	MCFG_SCREEN_UPDATE_DEVICE("vga", tseng_vga_device, screen_update)

	MCFG_PALETTE_ADD("palette", 0x100)

	MCFG_DEVICE_ADD("vga", TSENG_VGA, 0)
MACHINE_CONFIG_END

//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *isa8_svga_et4k_device::device_rom_region() const
{
	return ROM_NAME( et4000 );
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  isa8_vga_device - constructor
//-------------------------------------------------

isa8_svga_et4k_device::isa8_svga_et4k_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, ISA8_SVGA_ET4K, tag, owner, clock),
	device_isa8_card_interface(mconfig, *this),
	m_vga(nullptr)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------
READ8_MEMBER(isa8_svga_et4k_device::input_port_0_r ) { return 0xff; } //return space.machine().root_device().ioport("IN0")->read(); }

void isa8_svga_et4k_device::device_start()
{
	set_isa_device();

	m_vga = subdevice<tseng_vga_device>("vga");

	m_isa->install_rom(this, 0xc0000, 0xc7fff, "et4000", "et4000");

	m_isa->install_device(0x3b0, 0x3bf, read8_delegate(FUNC(tseng_vga_device::port_03b0_r),m_vga), write8_delegate(FUNC(tseng_vga_device::port_03b0_w),m_vga));
	m_isa->install_device(0x3c0, 0x3cf, read8_delegate(FUNC(tseng_vga_device::port_03c0_r),m_vga), write8_delegate(FUNC(tseng_vga_device::port_03c0_w),m_vga));
	m_isa->install_device(0x3d0, 0x3df, read8_delegate(FUNC(tseng_vga_device::port_03d0_r),m_vga), write8_delegate(FUNC(tseng_vga_device::port_03d0_w),m_vga));

	m_isa->install_memory(0xa0000, 0xbffff, read8_delegate(FUNC(tseng_vga_device::mem_r),m_vga), write8_delegate(FUNC(tseng_vga_device::mem_w),m_vga));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void isa8_svga_et4k_device::device_reset()
{
}
