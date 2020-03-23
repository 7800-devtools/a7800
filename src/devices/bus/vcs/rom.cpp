// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***********************************************************************************************************

 A2600 VCS ROM cart emulation
 Simple cart hardware with no additional hw

 Mapper implementation based on the wonderful docs by Kevtris
 http://blog.kevtris.org/blogfiles/Atari%202600%20Mappers.txt

 (also inspired by previous work by Wilbert Pol et al.)

 ***********************************************************************************************************/


#include "emu.h"
#include "rom.h"


//-------------------------------------------------
//  a26_rom_*k_device - constructor
//-------------------------------------------------

DEFINE_DEVICE_TYPE(A26_ROM_2K,    a26_rom_2k_device,    "vcs_2k",    "Atari VCS 2600 2K ROM Carts")
DEFINE_DEVICE_TYPE(A26_ROM_4K,    a26_rom_4k_device,    "vcs_4k",    "Atari VCS 2600 4K ROM Carts")
DEFINE_DEVICE_TYPE(A26_ROM_F4,    a26_rom_f4_device,    "vcs_f4",    "Atari VCS 2600 ROM Carts w/F4 bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_F6,    a26_rom_f6_device,    "vcs_f6",    "Atari VCS 2600 ROM Carts w/F6 bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_F8,    a26_rom_f8_device,    "vcs_f8",    "Atari VCS 2600 ROM Carts w/F8 bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_F8_SW, a26_rom_f8_sw_device, "vcs_f8_sw", "Atari VCS 2600 ROM Cart Snow White")
DEFINE_DEVICE_TYPE(A26_ROM_FA,    a26_rom_fa_device,    "vcs_fa",    "Atari VCS 2600 ROM Carts w/FA bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_FE,    a26_rom_fe_device,    "vcs_fe",    "Atari VCS 2600 ROM Carts w/FE bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_3E,    a26_rom_3e_device,    "vcs_3e",    "Atari VCS 2600 ROM Carts w/3E bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_3F,    a26_rom_3f_device,    "vcs_3f",    "Atari VCS 2600 ROM Carts w/3F bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_E0,    a26_rom_e0_device,    "vcs_e0",    "Atari VCS 2600 ROM Carts w/E0 bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_E7,    a26_rom_e7_device,    "vcs_e7",    "Atari VCS 2600 ROM Carts w/E7 bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_UA,    a26_rom_ua_device,    "vcs_ua",    "Atari VCS 2600 ROM Carts w/UA bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_CV,    a26_rom_cv_device,    "vcs_cv",    "Atari VCS 2600 ROM Carts w/Commavid bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_DC,    a26_rom_dc_device,    "vcs_dc",    "Atari VCS 2600 ROM Carts w/Dynacom bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_FV,    a26_rom_fv_device,    "vcs_fv",    "Atari VCS 2600 ROM Carts w/FV bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_JVP,   a26_rom_jvp_device,   "vcs_jvp",   "Atari VCS 2600 ROM Carts w/JVP bankswitch")
DEFINE_DEVICE_TYPE(A26_ROM_4IN1,  a26_rom_4in1_device,  "vcs_4in1",  "Atari VCS 2600 ROM Cart 4 in 1")
DEFINE_DEVICE_TYPE(A26_ROM_8IN1,  a26_rom_8in1_device,  "vcs_8in1",  "Atari VCS 2600 ROM Cart 8 in 1")
DEFINE_DEVICE_TYPE(A26_ROM_32IN1, a26_rom_32in1_device, "vcs_32in1", "Atari VCS 2600 ROM Cart 32 in 1")
DEFINE_DEVICE_TYPE(A26_ROM_X07,   a26_rom_x07_device,   "vcs_x07",   "Atari VCS 2600 ROM Carts w/X07 bankswitch")


a26_rom_2k_device::a26_rom_2k_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock), device_vcs_cart_interface(mconfig, *this)
{
}

a26_rom_2k_device::a26_rom_2k_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_2k_device(mconfig, A26_ROM_2K, tag, owner, clock)
{
}


a26_rom_4k_device::a26_rom_4k_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_2k_device(mconfig, A26_ROM_4K, tag, owner, clock)
{
}


a26_rom_f6_device::a26_rom_f6_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_2k_device(mconfig, type, tag, owner, clock)
	, m_base_bank(-1) // set to -1 to help the Xin1 multicart...
{
}

a26_rom_f6_device::a26_rom_f6_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_F6, tag, owner, clock)
{
}


a26_rom_f4_device::a26_rom_f4_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_F4, tag, owner, clock)
{
}


a26_rom_f8_device::a26_rom_f8_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, type, tag, owner, clock)
{
}

a26_rom_f8_device::a26_rom_f8_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f8_device(mconfig, A26_ROM_F8, tag, owner, clock)
{
}

a26_rom_f8_sw_device::a26_rom_f8_sw_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f8_device(mconfig, A26_ROM_F8_SW, tag, owner, clock)
{
}

a26_rom_fa_device::a26_rom_fa_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_FA, tag, owner, clock)
{
}


a26_rom_fe_device::a26_rom_fe_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_2k_device(mconfig, A26_ROM_FE, tag, owner, clock), m_base_bank(0), m_trigger_on_next_access(0)
{
}


a26_rom_3e_device::a26_rom_3e_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_3E, tag, owner, clock), m_num_bank(0), m_ram_bank(0), m_ram_enable(0)
{
}


a26_rom_3f_device::a26_rom_3f_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_3F, tag, owner, clock), m_num_bank(0)
{
}


a26_rom_e0_device::a26_rom_e0_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_E0, tag, owner, clock)
{
}


a26_rom_e7_device::a26_rom_e7_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_E7, tag, owner, clock), m_ram_bank(0)
{
}


a26_rom_ua_device::a26_rom_ua_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_UA, tag, owner, clock)
{
}


a26_rom_cv_device::a26_rom_cv_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_2k_device(mconfig, A26_ROM_CV, tag, owner, clock)
{
}


a26_rom_dc_device::a26_rom_dc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_DC, tag, owner, clock)
{
}


a26_rom_fv_device::a26_rom_fv_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_FV, tag, owner, clock), m_locked(0)
{
}


a26_rom_jvp_device::a26_rom_jvp_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_JVP, tag, owner, clock)
{
}


a26_rom_4in1_device::a26_rom_4in1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_4IN1, tag, owner, clock)
{
}


a26_rom_8in1_device::a26_rom_8in1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f8_device(mconfig, A26_ROM_8IN1, tag, owner, clock), m_reset_bank(0)
{
}


a26_rom_32in1_device::a26_rom_32in1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_32IN1, tag, owner, clock)
{
}

a26_rom_x07_device::a26_rom_x07_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: a26_rom_f6_device(mconfig, A26_ROM_X07, tag, owner, clock)
{
}


void a26_rom_2k_device::device_start()
{
}

void a26_rom_2k_device::device_reset()
{
}

void a26_rom_f6_device::device_start()
{
	save_item(NAME(m_base_bank));
}

void a26_rom_f6_device::device_reset()
{
	m_base_bank = 0;
}

void a26_rom_f4_device::device_reset()
{
	m_base_bank = 7;
}

void a26_rom_f8_sw_device::device_reset()
{
	// Snow White proto starts from bank 1!!!
	m_base_bank = 1;
}

void a26_rom_fe_device::device_start()
{
	save_item(NAME(m_base_bank));
	save_item(NAME(m_trigger_on_next_access));
}

void a26_rom_fe_device::device_reset()
{
	m_base_bank = 0;
	m_trigger_on_next_access = 0;
}

void a26_rom_3e_device::device_start()
{
	save_item(NAME(m_base_bank));
	save_item(NAME(m_ram_bank));
	save_item(NAME(m_ram_enable));
}

void a26_rom_3e_device::device_reset()
{
	m_num_bank = m_rom_size / 0x800;
	m_base_bank = m_num_bank - 1;
	m_ram_bank = 0;
	m_ram_enable = 0;
}

void a26_rom_3f_device::device_reset()
{
	m_num_bank = m_rom_size / 0x800;
	m_base_bank = m_num_bank - 1;
}

void a26_rom_e0_device::device_start()
{
	save_item(NAME(m_base_banks));
}

void a26_rom_e0_device::device_reset()
{
	m_base_banks[0] = 4;
	m_base_banks[1] = 5;
	m_base_banks[2] = 6;
	m_base_banks[3] = 7;
}

void a26_rom_e7_device::device_start()
{
	save_item(NAME(m_base_bank));
	save_item(NAME(m_ram_bank));
}

void a26_rom_e7_device::device_reset()
{
	m_base_bank = 0;
	m_ram_bank = 0;
}

void a26_rom_ua_device::device_reset()
{
	m_base_bank = 0;
}

void a26_rom_fv_device::device_start()
{
	save_item(NAME(m_base_bank));
	save_item(NAME(m_locked));
}

void a26_rom_fv_device::device_reset()
{
	m_base_bank = 0;
	m_locked = 0;
}


void a26_rom_4in1_device::device_reset()
{
	m_base_bank++;
	m_base_bank &= 3;
}


void a26_rom_8in1_device::device_start()
{
	save_item(NAME(m_base_bank));
	save_item(NAME(m_reset_bank));
}

void a26_rom_8in1_device::device_reset()
{
	// we use here two different bank counter: the main one for the 8x8K chunks,
	// and the usual one (m_base_bank) for the 4K bank of the current game
	m_reset_bank++;
	m_reset_bank &= 7;
	m_base_bank = 0;
}


void a26_rom_32in1_device::device_reset()
{
	m_base_bank++;
	m_base_bank &= 0x1f;
}


/*-------------------------------------------------
 mapper specific handlers
 -------------------------------------------------*/

/*-------------------------------------------------
 BASE 2K & 4K Carts:
 no bankswitch

 GAMES: a large majority
 -------------------------------------------------*/

READ8_MEMBER(a26_rom_2k_device::read_rom)
{
	// Super Chip RAM reads are mapped in 0x1080-0x10ff
	if (!m_ram.empty() && offset >= 0x80 && offset < 0x100)
	{
		return m_ram[offset & (m_ram.size() - 1)];
	}

	return m_rom[offset & (m_rom_size - 1)];
}

/*-------------------------------------------------
 "F4 Bankswitch" Carts:
 read/write access to 0x1ff4-0x1ffb determines the
 4K ROM bank to be read

 GAMES: Fatal Run
 -------------------------------------------------*/

READ8_MEMBER(a26_rom_f4_device::read_rom)
{
	// Super Chip RAM reads are mapped in 0x1080-0x10ff
	if (!m_ram.empty() && offset >= 0x80 && offset < 0x100)
	{
		return m_ram[offset & (m_ram.size() - 1)];
	}

	// update banks
	if (!machine().side_effect_disabled())
	{
		switch (offset)
		{
			case 0x0ff4:
			case 0x0ff5:
			case 0x0ff6:
			case 0x0ff7:
			case 0x0ff8:
			case 0x0ff9:
			case 0x0ffa:
			case 0x0ffb:
				m_base_bank = offset - 0x0ff4;
				break;
		}
	}

	return m_rom[offset + (m_base_bank * 0x1000)];
}

WRITE8_MEMBER(a26_rom_f4_device::write_bank)
{
	// Super Chip RAM writes are mapped in 0x1000-0x107f
	if (!m_ram.empty() && offset < 0x80)
	{
		m_ram[offset & (m_ram.size() - 1)] = data;
		return;
	}

	switch (offset)
	{
		case 0x0ff4:
		case 0x0ff5:
		case 0x0ff6:
		case 0x0ff7:
		case 0x0ff8:
		case 0x0ff9:
		case 0x0ffa:
		case 0x0ffb:
			m_base_bank = offset - 0x0ff4;
			break;
		default:
			logerror("Write Bank outside expected range (0x%X).\n", offset + 0x1000);
			break;
	}
}

/*-------------------------------------------------
 "F6 Bankswitch" Carts:
 read/write access to 0x1ff6-0x1ff9 determines the
 4K ROM bank to be read

 GAMES: Atari 16K games, like Crossbow, Crystal
 Castles and the 2-in-1 carts

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_f6_device::read_rom)
{
	// Super Chip RAM reads are mapped in 0x1080-0x10ff
	if (!m_ram.empty() && offset >= 0x80 && offset < 0x100)
	{
		return m_ram[offset & (m_ram.size() - 1)];
	}

	// update banks
	if (!machine().side_effect_disabled())
	{
		switch (offset)
		{
			case 0x0ff6:
			case 0x0ff7:
			case 0x0ff8:
			case 0x0ff9:
				m_base_bank = offset - 0x0ff6;
				break;
		}
	}

	return m_rom[offset + (m_base_bank * 0x1000)];
}

WRITE8_MEMBER(a26_rom_f6_device::write_bank)
{
	// Super Chip RAM writes are mapped in 0x1000-0x107f
	if (!m_ram.empty() && offset < 0x80)
	{
		m_ram[offset & (m_ram.size() - 1)] = data;
		return;
	}

	switch (offset)
	{
		case 0x0ff6:
		case 0x0ff7:
		case 0x0ff8:
		case 0x0ff9:
			m_base_bank = offset - 0x0ff6;
			break;
		default:
			logerror("Write Bank outside expected range (0x%X).\n", offset + 0x1000);
			break;
	}
}

/*-------------------------------------------------
 "F8 Bankswitch" Carts:
 read/write access to 0x1ff8-0x1ff9 determines the
 4K ROM bank to be read

 GAMES: Atari 8K games, like Asteroids, Battlezone
 and Taz

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_f8_device::read_rom)
{
	// Super Chip RAM reads are mapped in 0x1080-0x10ff
	if (!m_ram.empty() && offset >= 0x80 && offset < 0x100)
	{
		return m_ram[offset & (m_ram.size() - 1)];
	}

	// update banks
	if (!machine().side_effect_disabled())
	{
		switch (offset)
		{
			case 0x0ff8:
			case 0x0ff9:
				m_base_bank = offset - 0x0ff8;
				break;
		}
	}

	return m_rom[offset + (m_base_bank * 0x1000)];
}

WRITE8_MEMBER(a26_rom_f8_device::write_bank)
{
	// Super Chip RAM writes are mapped in 0x1000-0x107f
	if (!m_ram.empty() && offset < 0x80)
	{
		m_ram[offset & (m_ram.size() - 1)] = data;
		return;
	}

	switch (offset)
	{
		case 0x0ff8:
		case 0x0ff9:
			m_base_bank = offset - 0x0ff8;
			break;
		default:
			logerror("Write Bank outside expected range (0x%X).\n", offset + 0x1000);
			break;
	}
}

/*-------------------------------------------------
 "FA Bankswitch" Carts:
 read/write access to 0x1ff8-0x1ffa determines the
 4K ROM bank to be read
 These games contained the CBS RAM+ chip (256bytes
 of RAM)

 GAMES: CBS RAM Plus games like Omega Race and Tunnel
 Runner

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_fa_device::read_rom)
{
	// CBS RAM+ reads are mapped in 0x1100-0x11ff
	if (!m_ram.empty() && offset >= 0x100 && offset < 0x200)
	{
		return m_ram[offset & (m_ram.size() - 1)];
	}

	// update banks
	if (!machine().side_effect_disabled())
	{
		switch (offset)
		{
			case 0x0ff8:
			case 0x0ff9:
			case 0x0ffa:
				m_base_bank = offset - 0x0ff8;
				break;
		}
	}

	return m_rom[offset + (m_base_bank * 0x1000)];
}

WRITE8_MEMBER(a26_rom_fa_device::write_bank)
{
	// CBS RAM+ writes are mapped in 0x1000-0x10ff
	if (!m_ram.empty() && offset < 0x100)
	{
		m_ram[offset & (m_ram.size() - 1)] = data;
	}

	switch (offset)
	{
		case 0x0ff8:
		case 0x0ff9:
		case 0x0ffa:
			m_base_bank = offset - 0x0ff8;
			break;
		default:
			logerror("Write Bank outside expected range (0x%X).\n", offset + 0x1000);
			break;
	}
}

/*-------------------------------------------------
 "FE Bankswitch" Carts:
 read/write access to 0x01fe-0x1ff determines the
 4K ROM bank to be read

 GAMES: Activision 8K games like Decathlon

 -------------------------------------------------*/
/*

 There seems to be a kind of lag between the writing to address 0x1FE and the
 Activision switcher springing into action. It waits for the next byte to arrive
 on the data bus, which is the new PCH in the case of a JSR, and the PCH of the
 stored PC on the stack in the case of an RTS.

 depending on last byte & 0x20 -> 0x00 -> switch to bank #1
 -> 0x20 -> switch to bank #0

 */

READ8_MEMBER(a26_rom_fe_device::read_rom)
{
	uint8_t data;

	// Super Chip RAM reads are mapped in 0x1080-0x10ff
	if (!m_ram.empty() && offset >= 0x80 && offset < 0x100)
	{
		return m_ram[offset & (m_ram.size() - 1)];
	}

	data = m_rom[offset + (m_base_bank * 0x1000)];

	if (!machine().side_effect_disabled())
	{
		if (m_trigger_on_next_access)
		{
			m_base_bank = BIT(data, 5) ? 0 : 1;
			m_trigger_on_next_access = 0;
		}
	}

	return data;
}

WRITE8_MEMBER(a26_rom_fe_device::write_ram)
{
	// Super Chip RAM writes are mapped in 0x1000-0x107f
	if (!m_ram.empty() && offset < 0x80)
	{
		m_ram[offset & (m_ram.size() - 1)] = data;
	}
}

READ8_MEMBER(a26_rom_fe_device::read_bank)
{
	uint8_t data = space.read_byte(0xfe + offset);

	if (!machine().side_effect_disabled())
	{
		switch (offset & 1)
		{
			case 0:
				// The next byte on the data bus determines which bank to switch to
				m_trigger_on_next_access = 1;
				break;

			case 1:
				if (m_trigger_on_next_access)
				{
					m_base_bank = BIT(data, 5) ? 0 : 1;
					m_trigger_on_next_access = 0;
				}
				break;
		}
	}
	return data;
}

WRITE8_MEMBER(a26_rom_fe_device::write_bank)
{
	space.write_byte(0xfe, data);
	if (!machine().side_effect_disabled())
	{
		// The next byte on the data bus determines which bank to switch to
		m_trigger_on_next_access = 1;
	}
}

/*-------------------------------------------------
 "3E Bankswitch" Carts:
 write access to 0x3e determines the 2K ROM bank to
 be read, write access to 0x3f determines the RAM bank
 to be read

 GAMES: Boulder Dash (Homebrew)

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_3e_device::read_rom)
{
	if (!m_ram.empty() && m_ram_enable && offset < 0x400)
		return m_ram[offset + (m_ram_bank * 0x400)];

	if (offset >= 0x800)
		return m_rom[(offset & 0x7ff) + (m_num_bank - 1) * 0x800];
	else
		return m_rom[offset + m_base_bank * 0x800];
}

WRITE8_MEMBER(a26_rom_3e_device::write_bank)
{
	if (offset == 0x3f)
	{
		m_base_bank = data & (m_num_bank - 1);
		m_ram_enable = 0;
	}
	else if (offset == 0x3e)
	{
		m_ram_bank = data & 0x1f;
		m_ram_enable = 1;
	}
}

WRITE8_MEMBER(a26_rom_3e_device::write_ram)
{
	if (!m_ram.empty() && m_ram_enable && offset >= 0x400 && offset < 0x800)
		m_ram[(offset & 0x3ff) + (m_ram_bank * 0x400)] = data;
}


/*-------------------------------------------------
 "3F Bankswitch" Carts:
 write access to 0x00-0x3f determines the 2K ROM bank
 to be read

 GAMES: Tigervision 8K games like Espial and Miner
 2049er. Extended version with bankswitch up to 512K
 shall be supported as well (but we lack a test case)

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_3f_device::read_rom)
{
	if (offset >= 0x800)
		return m_rom[(offset & 0x7ff)  + (m_num_bank - 1) * 0x800];
	else
		return m_rom[offset + m_base_bank * 0x800];
}

WRITE8_MEMBER(a26_rom_3f_device::write_bank)
{
	m_base_bank = data & (m_num_bank - 1);
}

/*-------------------------------------------------
 "E0 Bankswitch" Carts:
 read/write access to 0x1fe0-0x1ff8 determines the
 1K ROM bank to be read in each 1K chunk (0x1c00-0x1fff
 always points to the last 1K of the ROM)

 GAMES: Parker Bros. 8K games like Gyruss and Popeye

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_e0_device::read_rom)
{
	// update banks
	if (!machine().side_effect_disabled())
	{
		if (offset >= 0xfe0 && offset <= 0xff8)
			m_base_banks[(offset >> 3) & 3] = offset & 7;
	}

	return m_rom[(offset & 0x3ff) + (m_base_banks[(offset >> 10) & 3] * 0x400)];
}

WRITE8_MEMBER(a26_rom_e0_device::write_bank)
{
	if (offset >= 0xfe0 && offset <= 0xff8)
		m_base_banks[(offset >> 3) & 3] = offset & 7;
}


/*-------------------------------------------------
 "E7 Bankswitch" Carts:
 this PCB can handle up to 16K of ROM and 2K of RAM,
 with the following layout
 1000-17ff is selectable bank
 1800-19ff is RAM
 1a00-1fff is fixed to the last 0x600 of ROM

 The selectable bank can be ROM (if selected by
 0x1fe0-0x1fe6 access) or a first 1K of RAM (if
 selected by 0x1fe7 access).
 The other 256byte RAM bank can be one of the
 four different chunks forming the other 1K of RAM
 (the bank is selected by accessing 0x1fe8-0x1feb)

 GAMES: M Network 16K games like Burgertime and
 Bump'n Jump

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_e7_device::read_rom)
{
	// update banks
	if (!machine().side_effect_disabled())
	{
		if (offset >= 0xfe0 && offset <= 0xfe7)
			m_base_bank = offset - 0xfe0;
		if (offset >= 0xfe8 && offset <= 0xfeb)
			m_ram_bank = offset - 0xfe8;
	}

	if (!m_ram.empty())
	{
		// 1K of RAM
		if (m_base_bank == 0x07 && offset >= 0x400 && offset < 0x800)
			return m_ram[0x400 + (offset & 0x3ff)];
		// the other 1K of RAM
		if (offset >= 0x900 && offset < 0xa00)
		{
			offset -= 0x900;
			return m_ram[offset + (m_ram_bank * 0x100)];
		}
	}

	if (offset > 0x800)
		return m_rom[(offset & 0x7ff) + 0x3800];
	else
		return m_rom[(offset & 0x7ff) + (m_base_bank * 0x800)];
}

WRITE8_MEMBER(a26_rom_e7_device::write_bank)
{
	if (offset >= 0xfe0 && offset <= 0xfe7)
		m_base_bank = offset - 0xfe0;
	if (offset >= 0xfe8 && offset <= 0xfeb)
		m_ram_bank = offset - 0xfe8;

	if (!m_ram.empty())
	{
		// 1K of RAM
		if (m_base_bank == 0x07 && offset < 0x400)
			m_ram[0x400 + (offset & 0x3ff)] = data;
		// the other 1K of RAM
		if (offset >= 0x800 && offset < 0x900)
		{
			offset -= 0x800;
			m_ram[offset + (m_ram_bank * 0x100)] = data;
		}
	}
}

/*-------------------------------------------------
 "UA Bankswitch" Carts:
 read/write access to 0x200-0x27f determines the
 4K ROM bank to be read (0x220-0x23f for low 4K,
 0x240-0x27f for high 4K)

 GAMES: UA Ltd. 8K games like Funky Flash and
 Pleaides

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_ua_device::read_rom)
{
	return m_rom[(offset + (m_base_bank * 0x1000)) & (m_rom_size - 1)];
}

READ8_MEMBER(a26_rom_ua_device::read_bank)
{
	if (!machine().side_effect_disabled())
		m_base_bank = offset >> 6;

	return 0;
}

WRITE8_MEMBER(a26_rom_ua_device::write_bank)
{
	m_base_bank = offset >> 6;
}


/*-------------------------------------------------
 Commavid Carts:
 It allows for both ROM and RAM on the cartridge,
 without using bankswitching.  There's 2K of ROM
 and 1K of RAM.

 GAMES: Magicard and Video Life by Commavid

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_cv_device::read_rom)
{
	if (!m_ram.empty() && offset < 0x400)
	{
		return m_ram[offset & (m_ram.size() - 1)];
	}

	// games shall not read from 0x1400-0x17ff (RAM write)
	// but we fall back to ROM just in case...
	return m_rom[offset & 0x7ff];
}

WRITE8_MEMBER(a26_rom_cv_device::write_bank)
{
	if (!m_ram.empty() && offset >= 0x400 && offset < 0x800)
	{
		m_ram[offset & (m_ram.size() - 1)] = data;
	}
}


/*-------------------------------------------------
 Dynacom Megaboy Carts (aka "F0 Banswitch"):
 read/write access to 0x1ff0 determines the 4K ROM
 bank to be read (each access increases the bank index
 up to 16, since the cart was 64K wide)

 GAMES: Megaboy by Dynacom

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_dc_device::read_rom)
{
	if (!machine().side_effect_disabled())
	{
		if (offset == 0xff0)
			m_base_bank = (m_base_bank + 1) & 0x0f;
	}

	if (offset == 0xfec)
		return m_base_bank;

	return m_rom[offset + (m_base_bank * 0x1000)];
}

WRITE8_MEMBER(a26_rom_dc_device::write_bank)
{
	if (offset == 0xff0)
		m_base_bank = (m_base_bank + 1) & 0x0f;
}


/*-------------------------------------------------
 "FV Bankswitch" Carts:
 The first access to 0x1fd0 switch the bank, but
 only if pc() & 0x1f00 == 0x1f00!

 GAMES: Challenge by HES

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_fv_device::read_rom)
{
	if (!machine().side_effect_disabled())
	{
		if (offset == 0xfd0)
		{
			if (!m_locked && (machine().device<cpu_device>("maincpu")->pc() & 0x1f00) == 0x1f00)
			{
				m_locked = 1;
				m_base_bank = m_base_bank ^ 0x01;
			}
		}
	}

	return m_rom[offset + (m_base_bank * 0x1000)];
}

WRITE8_MEMBER(a26_rom_fv_device::write_bank)
{
	if (offset == 0xfd0)
	{
		if (!m_locked && (machine().device<cpu_device>("maincpu")->pc() & 0x1f00) == 0x1f00)
		{
			m_locked = 1;
			m_base_bank = m_base_bank ^ 0x01;
		}
	}
}


/*-------------------------------------------------
 "JVP Bankswitch" Carts:
 read/write access to 0x0fa0-0x0fc0 determines the
 4K ROM bank to be read (notice that this overlaps
 the RIOT, currently handled in the main driver until
 I can better investigate the behavior)

 GAMES: No test case!?!

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_jvp_device::read_rom)
{
	return m_rom[offset + (m_base_bank * 0x1000)];
}

WRITE8_MEMBER(a26_rom_jvp_device::write_bank)
{
	switch (offset)
	{
		case 0x00:
		case 0x20:
			m_base_bank ^= 1;
			break;
		default:
			//printf("%04X: write to unknown mapper address %02X\n", m_maincpu->pc(), 0xfa0 + offset);
			break;
	}
}


/*-------------------------------------------------
 4 in 1 Carts (Reset based):
 the 4K bank changes at each reset

 GAMES: 4 in 1 carts

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_4in1_device::read_rom)
{
	return m_rom[offset + (m_base_bank * 0x1000)];
}


/*-------------------------------------------------
 8 in 1 Carts (Reset based):
 the 8K banks change at each reset, and internally
 each game runs as a F8-bankswitched cart

 GAMES: 8 in 1 cart

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_8in1_device::read_rom)
{
	if (!machine().side_effect_disabled())
	{
		switch (offset)
		{
			case 0x0ff8:
			case 0x0ff9:
				m_base_bank = offset - 0x0ff8;
				break;
		}
	}

	return m_rom[offset + (m_base_bank * 0x1000) + (m_reset_bank * 0x2000)];
}


/*-------------------------------------------------
 32 in 1 Carts (Reset based):
 the 2K banks change at each reset

 GAMES: 32 in 1 cart

 -------------------------------------------------*/

READ8_MEMBER(a26_rom_32in1_device::read_rom)
{
	return m_rom[(offset & 0x7ff) + (m_base_bank * 0x800)];
}


/*-------------------------------------------------
 "X07 Bankswitch" Carts:
 banking done with a PALC22V10B
 implementation based on information at
 http://blog.kevtris.org/blogfiles/Atari%202600%20Mappers.txt
 --------------------------------------------------*/

READ8_MEMBER(a26_rom_x07_device::read_rom)
{
	return m_rom[offset + (m_base_bank * 0x1000)];
}

WRITE8_MEMBER(a26_rom_x07_device::write_bank)
{
	/*
	A13           A0
	----------------
	0 1xxx nnnn 1101
	*/

	if ((offset & 0x180f) == 0x080d)
	{
		m_base_bank = (offset & 0x00f0) >> 4;
	}
	/*
	A13           A0
	----------------
	0 0xxx 0nxx xxxx
	*/

	if ((offset & 0x1880) == 0x0000)
	{
		// only has an effect if bank is already 14 or 15
		if (m_base_bank == 14 || m_base_bank == 15)
		{
			if (offset & 0x0040)
			{
				m_base_bank = 15;
			}
			else
			{
				m_base_bank = 14;
			}

		}
	}
}
