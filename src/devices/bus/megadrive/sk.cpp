// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***********************************************************************************************************


 Sonic & Knuckles pass-thorugh cart emulation


 TODO: currently we only support loading of base carts with no bankswitch or protection...
       shall we support other as well?


 ***********************************************************************************************************/




#include "emu.h"
#include "sk.h"
#include "rom.h"


//-------------------------------------------------
//  md_rom_device - constructor
//-------------------------------------------------

DEFINE_DEVICE_TYPE(MD_ROM_SK, md_rom_sk_device, "md_rom_sk", "MD Sonic & Knuckles")


md_rom_sk_device::md_rom_sk_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_md_cart_interface(mconfig, *this)
	, m_exp(*this, "subslot")
{
}

md_rom_sk_device::md_rom_sk_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: md_rom_sk_device(mconfig, MD_ROM_SK, tag, owner, clock)
{
}


void md_rom_sk_device::device_start()
{
}

/*-------------------------------------------------
 mapper specific handlers
 -------------------------------------------------*/

READ16_MEMBER(md_rom_sk_device::read)
{
	if (m_exp->m_cart != nullptr && m_exp->m_cart->get_rom_base() != nullptr && offset >= 0x200000/2 && offset < (0x200000 + m_exp->m_cart->get_rom_size())/2)
		return m_exp->m_cart->m_rom[offset - 0x200000/2];
	if (offset < 0x400000/2)
		return m_rom[MD_ADDR(offset)];
	else
		return 0xffff;
}

WRITE16_MEMBER(md_rom_sk_device::write)
{
// should there be anything here?
}


static SLOT_INTERFACE_START(sk_sub_cart)
	SLOT_INTERFACE_INTERNAL("rom",  MD_STD_ROM)
	SLOT_INTERFACE_INTERNAL("rom_svp",  MD_STD_ROM)
	SLOT_INTERFACE_INTERNAL("rom_sram",  MD_ROM_SRAM)
	SLOT_INTERFACE_INTERNAL("rom_sramsafe",  MD_ROM_SRAM)
	SLOT_INTERFACE_INTERNAL("rom_fram",  MD_ROM_FRAM)
// add all types??
SLOT_INTERFACE_END


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_MEMBER( md_rom_sk_device::device_add_mconfig )
	MCFG_MD_CARTRIDGE_ADD("subslot", sk_sub_cart, nullptr)
	MCFG_MD_CARTRIDGE_NOT_MANDATORY
MACHINE_CONFIG_END
