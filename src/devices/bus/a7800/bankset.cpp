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

#define CLK_NTSC  1789772
#define CLK_PAL   1773447

//-------------- bankset supergame (no ram) --------------

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
			return m_rom[offset + ((m_bank_mask/2) * 0x4000) - 0x4000];   // second last bank
		else if (offset < 0x8000)
			return m_rom[(offset & 0x3fff) + (m_bank * 0x4000)];
		else
			return m_rom[(offset & 0x3fff) + ((m_bank_mask/2) * 0x4000)];   // last bank
	}
	else // m_dmaactive!=0
	{
		if (offset < 0x4000)
			return m_rom[offset + ((m_bank_mask/2) * 0x4000) + (((m_bank_mask/2)+1) * 0x4000) - 0x4000];   // second last bank
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

//-------------- bankset supergame pokey800 (no ram) --------------

DEFINE_DEVICE_TYPE(A78_ROM_BANKSET_SG_POK800, a78_bankset_sg_p800_device, "a78_bankset_sg_p800", "Atari 7800 Bankset SG Pokey800 Cart")


a78_bankset_sg_p800_device::a78_bankset_sg_p800_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a78_rom_sg_device(mconfig, type, tag, owner, clock)
	, m_pokey800(*this, "pokey800")
{
}

a78_bankset_sg_p800_device::a78_bankset_sg_p800_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a78_bankset_sg_p800_device(mconfig, A78_ROM_BANKSET_SG_POK800, tag, owner, clock)
{
}

MACHINE_CONFIG_MEMBER( a78_bankset_sg_p800_device::device_add_mconfig )
        MCFG_SPEAKER_STANDARD_MONO("pokey_800")

        MCFG_SOUND_ADD("pokey800", POKEY, CLK_NTSC)
        MCFG_SOUND_ROUTE(ALL_OUTPUTS, "pokey_800", 1.00)
MACHINE_CONFIG_END


void a78_bankset_sg_p800_device::device_start()
{
	save_item(NAME(m_bank));
}

void a78_bankset_sg_p800_device::device_reset()
{
	m_bank = 0;
}

READ8_MEMBER(a78_bankset_sg_p800_device::read_40xx)
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

WRITE8_MEMBER(a78_bankset_sg_p800_device::write_40xx)
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


//-------------- bankset supergame banked ram pokey800--------------

DEFINE_DEVICE_TYPE(A78_ROM_BANKSET_SG_BANKRAM_POK800, a78_bankset_sg_bankram_p800_device, "a78_bankset_sg_bankram_p800", "Atari 7800 Bankset SG BankRAM Pokey800 Cart")


a78_bankset_sg_bankram_p800_device::a78_bankset_sg_bankram_p800_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a78_rom_sg_device(mconfig, type, tag, owner, clock)
	, m_pokey800(*this, "pokey800")
{
}

a78_bankset_sg_bankram_p800_device::a78_bankset_sg_bankram_p800_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a78_bankset_sg_bankram_p800_device(mconfig, A78_ROM_BANKSET_SG_BANKRAM_POK800, tag, owner, clock)
{
}

MACHINE_CONFIG_MEMBER( a78_bankset_sg_bankram_p800_device::device_add_mconfig )
        MCFG_SPEAKER_STANDARD_MONO("pokey_800")

        MCFG_SOUND_ADD("pokey800", POKEY, CLK_NTSC)
        MCFG_SOUND_ROUTE(ALL_OUTPUTS, "pokey_800", 1.00)
MACHINE_CONFIG_END


void a78_bankset_sg_bankram_p800_device::device_start()
{
	save_item(NAME(m_bank));
}

void a78_bankset_sg_bankram_p800_device::device_reset()
{
	m_bank = 0;
}


READ8_MEMBER(a78_bankset_sg_bankram_p800_device::read_40xx)
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

WRITE8_MEMBER(a78_bankset_sg_bankram_p800_device::write_40xx)
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

//-------------- bankset rom pokey 800 --------------

DEFINE_DEVICE_TYPE(A78_ROM_BANKSET_ROM_POK800, a78_bankset_rom_p800_device, "a78_bankset_rom_p800", "Atari 7800 Bankset Rom Pokey800 Cart")

a78_bankset_rom_p800_device::a78_bankset_rom_p800_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a78_rom_device(mconfig, type, tag, owner, clock)
	, m_pokey800(*this, "pokey800")
{
}

a78_bankset_rom_p800_device::a78_bankset_rom_p800_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a78_bankset_rom_p800_device(mconfig, A78_ROM_BANKSET_ROM_POK800, tag, owner, clock)
{
}

MACHINE_CONFIG_MEMBER( a78_bankset_rom_p800_device::device_add_mconfig )
        MCFG_SPEAKER_STANDARD_MONO("pokey_800")

        MCFG_SOUND_ADD("pokey800", POKEY, CLK_NTSC)
        MCFG_SOUND_ROUTE(ALL_OUTPUTS, "pokey_800", 1.00)
MACHINE_CONFIG_END



READ8_MEMBER(a78_bankset_rom_p800_device::read_40xx)
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


//-------------- bankset rom pokey 4000 --------------

DEFINE_DEVICE_TYPE(A78_ROM_BANKSET_ROM_POK4000, a78_bankset_rom_p4000_device, "a78_bankset_rom_p4000", "Atari 7800 Bankset Rom Pokey4000 Cart")

a78_bankset_rom_p4000_device::a78_bankset_rom_p4000_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a78_rom_device(mconfig, type, tag, owner, clock)
	, m_pokey4000(*this, "pokey4000")
{
}

a78_bankset_rom_p4000_device::a78_bankset_rom_p4000_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a78_bankset_rom_p4000_device(mconfig, A78_ROM_BANKSET_ROM_POK4000, tag, owner, clock)
{
}

MACHINE_CONFIG_MEMBER( a78_bankset_rom_p4000_device::device_add_mconfig )
        MCFG_SPEAKER_STANDARD_MONO("pokey_4000")

        MCFG_SOUND_ADD("pokey4000", POKEY, CLK_NTSC)
        MCFG_SOUND_ROUTE(ALL_OUTPUTS, "pokey_4000", 1.00)
MACHINE_CONFIG_END



READ8_MEMBER(a78_bankset_rom_p4000_device::read_40xx)
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

//-------------- bankset bankram --------------

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
		if(offset<0x4000)
			return(m_ram[offset]);
		else if(offset<addrstart)
			return(0xff);
		else
			return m_rom [offset - addrstart];
	}
	else // m_dmaactive!=0
	{
		if(offset<0x4000)
			return(m_ram[offset+0x4000]);
		else if(offset<addrstart)
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

//-------------- bankset bankram --------------

DEFINE_DEVICE_TYPE(A78_ROM_BANKSET_BANKRAM_POK800, a78_bankset_bankram_p800_device, "a78_bankset_bankram_p800", "Atari 7800 Bankset BankRAM Pokey800 Cart")


a78_bankset_bankram_p800_device::a78_bankset_bankram_p800_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a78_rom_device(mconfig, type, tag, owner, clock)
	, m_pokey800(*this, "pokey800")
{
}

a78_bankset_bankram_p800_device::a78_bankset_bankram_p800_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a78_bankset_bankram_p800_device(mconfig, A78_ROM_BANKSET_ROM_POK800, tag, owner, clock)
{
}

MACHINE_CONFIG_MEMBER( a78_bankset_bankram_p800_device::device_add_mconfig )
        MCFG_SPEAKER_STANDARD_MONO("pokey_800")

        MCFG_SOUND_ADD("pokey800", POKEY, CLK_NTSC)
        MCFG_SOUND_ROUTE(ALL_OUTPUTS, "pokey_800", 1.00)
MACHINE_CONFIG_END


READ8_MEMBER(a78_bankset_bankram_p800_device::read_40xx)
{
	uint32_t addrstart = 0xC000 - (m_rom_size / 2);

	if(m_dmaactive==0)
	{
		if(offset<0x4000)
			return(m_ram[offset]);
		else if(offset<addrstart)
			return(0xff);
		else
			return m_rom [offset - addrstart];
	}
	else // m_dmaactive!=0
	{
		if(offset<0x4000)
			return(m_ram[offset+0x4000]);
		else if(offset<addrstart)
			return(0xff);
		else
			return m_rom[offset - addrstart + (m_rom_size/2)];
	}
	
}

WRITE8_MEMBER(a78_bankset_bankram_p800_device::write_40xx)
{
	if (offset < 0x4000)
		m_ram[offset] = data; // Maria can only read, so this has to be Sally's bankset
	else if (offset >= 0x8000)
		m_ram[offset -0x8000 + 0x4000] = data;

}


//-------------- bankset rom 52k --------------

DEFINE_DEVICE_TYPE(A78_ROM_BANKSET_ROM_52K, a78_bankset_rom_52k_device, "a78_bankset_rom_52k", "Atari 7800 Bankset Rom 52K Cart")


a78_bankset_rom_52k_device::a78_bankset_rom_52k_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a78_rom_device(mconfig, type, tag, owner, clock)
{
}

a78_bankset_rom_52k_device::a78_bankset_rom_52k_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a78_bankset_rom_52k_device(mconfig, A78_ROM_BANKSET_ROM, tag, owner, clock)
{
}

READ8_MEMBER(a78_bankset_rom_52k_device::read_40xx)
{
	if(m_dmaactive==0)
		return m_rom [offset + 0x1000];
	else // m_dmaactive!=0
		return m_rom[offset + 0x1000 + 0xD000];
}

READ8_MEMBER(a78_bankset_rom_52k_device::read_30xx)
{
	if(m_dmaactive==0)
		return m_rom [offset];
	else // m_dmaactive!=0
		return m_rom[offset + 0xD000];
}

//-------------- bankset rom 52k p4000 --------------

DEFINE_DEVICE_TYPE(A78_ROM_BANKSET_ROM_52K_POK4000, a78_bankset_rom_52k_p4000_device, "a78_bankset_rom_52k", "Atari 7800 Bankset Rom 52K Pokey4000 Cart")


a78_bankset_rom_52k_p4000_device::a78_bankset_rom_52k_p4000_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a78_rom_device(mconfig, type, tag, owner, clock)
	, m_pokey4000(*this, "pokey4000")
{
}

a78_bankset_rom_52k_p4000_device::a78_bankset_rom_52k_p4000_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a78_bankset_rom_52k_p4000_device(mconfig, A78_ROM_BANKSET_ROM, tag, owner, clock)
{
}

READ8_MEMBER(a78_bankset_rom_52k_p4000_device::read_40xx)
{
	if(m_dmaactive==0)
		return m_rom [offset + 0x1000];
	else // m_dmaactive!=0
		return m_rom[offset + 0x1000 + 0xD000];
}

READ8_MEMBER(a78_bankset_rom_52k_p4000_device::read_30xx)
{
	if(m_dmaactive==0)
		return m_rom [offset];
	else // m_dmaactive!=0
		return m_rom[offset + 0xD000];
}


MACHINE_CONFIG_MEMBER( a78_bankset_rom_52k_p4000_device::device_add_mconfig )
        MCFG_SPEAKER_STANDARD_MONO("pokey_4000")

        MCFG_SOUND_ADD("pokey4000", POKEY, CLK_NTSC)
        MCFG_SOUND_ROUTE(ALL_OUTPUTS, "pokey_4000", 1.00)
MACHINE_CONFIG_END


