// license:BSD-3-Clause
// copyright-holders:Mike Saarna
/***********************************************************************************************************

 A7800 Bankset homebrew board by Fred Quimby (batari)

 Here we emulate the base configuration of this board:

 Bankset = 2 * 128K. Sally sees one 128K supergame type image (a bankset), and Maria sees 
 another. 

 There are presently 2 versions of the bankset supergame scheme - with banked ram, one without.
 For the bnke ram variant, Sally sees one 16K bank of RAM at $4000, and Maria sees another at $4000.
 Sally can write to Maria's RAM via writes to $C000-$FFFF.


***********************************************************************************************************/

// TODO: this global used to pass the halt line state between classes needs to be excised
extern int m_dmaactive;

#include "emu.h"
#include "bankset.h"
#include "speaker.h"


//-------------- bankset supergame (no ram) --------------

//-------------------------------------------------
//  constructor
//-------------------------------------------------

DEFINE_DEVICE_TYPE(A78_ROM_BANKSET_SG, a78_bankset_sg_device, "a78_bankset_sg", "Atari 7800 Bankset SG Cart")


a78_bankset_sg_device::a78_bankset_sg_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a78_rom_sg_device(mconfig, type, tag, owner, clock)
{
}

a78_bankset_sg_device::a78_bankset_sg_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a78_bankset_sg_device(mconfig, A78_ROM_BANKSET_SG, tag, owner, clock)
{
}

void a78_bankset_sg_device::device_start()
{
	save_item(NAME(m_bank));
}

void a78_bankset_sg_device::device_reset()
{
	m_bank = 0;
}

READ8_MEMBER(a78_bankset_sg_device::read_40xx)
{
	if(m_dmaactive==0)
	{
		if (offset < 0x4000)
			return 0xff; // can we do better?
		else if (offset < 0x8000)
			return m_rom[(offset & 0x3fff) + (m_bank * 0x4000)];
		else
			return m_rom[(offset & 0x3fff) + ((m_bank_mask/2) * 0x4000)];   // last bank
	}
	else // m_dmaactive!=0
	{
		if (offset < 0x4000)
			return 0xff; // can we do better?
		else if (offset < 0x8000)
			return m_rom[(offset & 0x3fff) + (m_bank * 0x4000) + (((m_bank_mask/2)+1) * 0x4000)];
		else
			return m_rom[(offset & 0x3fff) + ((m_bank_mask/2) * 0x4000) + (((m_bank_mask/2)+1) * 0x4000)];   // last bank
	}
	
}

WRITE8_MEMBER(a78_bankset_sg_device::write_40xx)
{
	if (offset < 0x4000)
		return;
	else if (offset < 0x8000)
	{
		// allow up to 2*256K banksets
		m_bank = (data & 0x0f) & (m_bank_mask/2);
	}
}


//-------------- bankset supergame banked ram --------------

DEFINE_DEVICE_TYPE(A78_ROM_BANKSET_SG_BANKRAM, a78_bankset_sg_bankram_device, "a78_bankset_sg_bankram", "Atari 7800 Bankset SG BankRAM Cart")


a78_bankset_sg_bankram_device::a78_bankset_sg_bankram_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a78_rom_sg_device(mconfig, type, tag, owner, clock)
{
}

a78_bankset_sg_bankram_device::a78_bankset_sg_bankram_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a78_bankset_sg_bankram_device(mconfig, A78_ROM_BANKSET_SG_BANKRAM, tag, owner, clock)
{
}

void a78_bankset_sg_bankram_device::device_start()
{
	save_item(NAME(m_bank));
}

void a78_bankset_sg_bankram_device::device_reset()
{
	m_bank = 0;
}


READ8_MEMBER(a78_bankset_sg_bankram_device::read_40xx)
{
	if(m_dmaactive==0)
	{
		if (offset < 0x4000)
			return m_ram[offset ];
		else if (offset < 0x8000)
			return m_rom[(offset & 0x3fff) + (m_bank * 0x4000)];
		else
			return m_rom[(offset & 0x3fff) + ((m_bank_mask/2) * 0x4000)];   // last bank
	}
	else // m_dmaactive!=0
	{
		if (offset < 0x4000)
			return m_ram[offset + 0x4000];
		else if (offset < 0x8000)
			return m_rom[(offset & 0x3fff) + (m_bank * 0x4000) + (((m_bank_mask/2)+1) * 0x4000)];
		else
			return m_rom[(offset & 0x3fff) + ((m_bank_mask/2) * 0x4000) + (((m_bank_mask/2)+1) * 0x4000)];   // last bank
	}
	
}

WRITE8_MEMBER(a78_bankset_sg_bankram_device::write_40xx)
{
	if (offset < 0x4000)
		m_ram[offset] = data; // Maria can only read, so this has to be Sally's bankset
	else if (offset < 0x8000)
	{
		// allow up to 2*256K banksets
		m_bank = (data & 0x0f) & (m_bank_mask/2);
	}
	else // offset >= 0x8000
		m_ram[offset -0x8000 + 0x4000] = data;

}

//-------------- bankset rom --------------

DEFINE_DEVICE_TYPE(A78_ROM_BANKSET_ROM, a78_bankset_rom_device, "a78_bankset_rom", "Atari 7800 Bankset Rom Cart")


a78_bankset_rom_device::a78_bankset_rom_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a78_rom_device(mconfig, type, tag, owner, clock)
{
}

a78_bankset_rom_device::a78_bankset_rom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a78_bankset_rom_device(mconfig, A78_ROM_BANKSET_ROM, tag, owner, clock)
{
}

READ8_MEMBER(a78_bankset_rom_device::read_40xx)
{
	uint32_t addrstart = 0xC000 - (m_rom_size / 2);

	if(m_dmaactive==0)
	{
		if(offset<addrstart)
			return(0xff);
		else
			return m_rom [offset - addrstart];
	}
	else // m_dmaactive!=0
	{
		if(offset<addrstart)
			return(0xff);
		else
			return m_rom[offset - addrstart + (m_rom_size/2)];
	}
	
}

//-------------- bankset rom --------------

DEFINE_DEVICE_TYPE(A78_ROM_BANKSET_BANKRAM, a78_bankset_bankram_device, "a78_bankset_bankram", "Atari 7800 Bankset BankRAM Cart")


a78_bankset_bankram_device::a78_bankset_bankram_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a78_rom_device(mconfig, type, tag, owner, clock)
{
}

a78_bankset_bankram_device::a78_bankset_bankram_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a78_bankset_bankram_device(mconfig, A78_ROM_BANKSET_ROM, tag, owner, clock)
{
}

READ8_MEMBER(a78_bankset_bankram_device::read_40xx)
{
	uint32_t addrstart = 0xC000 - (m_rom_size / 2);

	if(m_dmaactive==0)
	{
		if(offset<addrstart)
			return(0xff);
		else
			return m_rom [offset - addrstart];
	}
	else // m_dmaactive!=0
	{
		if(offset<addrstart)
			return(0xff);
		else
			return m_rom[offset - addrstart + (m_rom_size/2)];
	}
	
}

WRITE8_MEMBER(a78_bankset_bankram_device::write_40xx)
{
	if (offset < 0x4000)
		m_ram[offset] = data; // Maria can only read, so this has to be Sally's bankset
	else if (offset >= 0x8000)
		m_ram[offset -0x8000 + 0x4000] = data;

}




